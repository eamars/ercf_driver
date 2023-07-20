from . import pulse_counter
from . import force_move
import toolhead
import configparser
import logging
import ast
import time
from contextlib import contextmanager
from itertools import product
from numpy import median
import traceback



class StopConditionException(Exception):
    pass


class FilamentSlipException(StopConditionException):
    pass


class FatalPrinterError(Exception):
    pass


class EncoderCounter:

    def __init__(self, printer, pin, sample_time, poll_time, encoder_steps):
        self._last_time = self._last_count = None
        self._counts = 0
        self._encoder_steps = encoder_steps
        self._counter = pulse_counter.MCU_counter(printer, pin, sample_time,
                                    poll_time)
        self._counter.setup_callback(self._counter_callback)

    def _counter_callback(self, time, count, count_time):
        if self._last_time is None:  # First sample
            self._last_time = time
        elif count_time > self._last_time:
            self._last_time = count_time
            self._counts += count - self._last_count
        else:  # No counts since last sample
            self._last_time = time
        self._last_count = count

    def get_counts(self):
        return self._counts

    def get_distance(self):
        return (self._counts/2.) * self._encoder_steps

    def set_encoder_steps(self, new_steps):
        self._encoder_steps = new_steps

    def set_distance(self, new_distance):
        self._counts = int( ( new_distance / self._encoder_steps ) * 2. )

    def reset_counts(self):
        self._counts = 0.


class ERCF(object):
    MACRO_PAUSE = 'ERCF_PAUSE'

    def __init__(self, config):
        self.config = config
        self.printer = config.get_printer()

        # Read config
        self.gear_stepper = None
        self.gear_stepper_name = config.get('gear_stepper')
        self.selector_stepper = None
        self.selector_stepper_name = config.get('selector_stepper')
        self.servo_name = config.get('servo')
        self.servo = None

        # Optionally read the toolhead sensor from the config. If the sensor is absent then the driver will consider
        # the ERCF is operating at sensorless mode.
        self.toolhead_sensor_name = config.get('toolhead_sensor', None)
        self.toolhead_sensor = None

        # Debug options
        # By enabling the `debug_mode` the Python exception won't cause Klipper to shutdown and print out more
        # debug information
        self.debug_mode = config.getboolean('debug_mode', False)

        self.encoder_pin_name = config.get('encoder_pin')
        self.encoder_sample_time = config.getfloat('encoder_sample_time', 0.1,
                                                   above=0.)
        self.encoder_poll_time = config.getfloat('encoder_poll_time', 0.0001,
                                                 above=0.)

        self.long_moves_speed = config.getfloat('long_moves_speed', 100.)
        self.long_moves_accel = config.getfloat('long_moves_accel', 400.)
        self.short_moves_speed = config.getfloat('short_moves_speed', 25.)
        self.short_moves_accel = config.getfloat('short_moves_accel', 400.)
        self.gear_stepper_long_move_threshold = config.getfloat('gear_stepper_long_move_threshold', 70)
        self.gear_stepper_accel = config.getfloat('gear_stepper_accel', 0)
        self.extruder_move_speed = config.getfloat('extruder_move_speed', None)
        self.extruder_move_accel = config.getfloat('extruder_move_accel', None)

        # Step distance
        self.extra_move_margin = config.getfloat('extra_move_margin', 100)
        self.long_move_distance = config.getfloat('long_move_distance', 30)
        self.short_move_distance = config.getfloat('short_move_distance', 10)
        self.minimum_step_distance = config.getfloat('minimum_step_distance', 5)
        self.maximum_move_distance = config.getfloat('maximum_move_distance', 1500)
        self.maximum_step_distance = config.getfloat('maximum_step_distance', 1500)
        self.calibrate_move_distance_per_step = config.getfloat('calibrate_move_distance_per_step', 3)

        self.servo_up_angle = config.getfloat('servo_up_angle')
        self.servo_down_angle = config.getfloat('servo_down_angle')
        self.extra_servo_dwell_up = config.getfloat('extra_servo_dwell_up', 0)
        self.extra_servo_dwell_down = config.getfloat('extra_servo_dwell_down', 0)
        self.servo_down_turn_off = config.getboolean('servo_down_turn_off', True)

        # Others
        self.selector_filament_engagement_retry = config.getint('selector_filament_engagement_retry', 2)
        self.auto_home_selector = config.getboolean('auto_home_selector', True)
        self.tip_forming_gcode_before_calibration = config.get('tip_forming_gcode_before_calibration', None)
        self.slip_detection_ratio_threshold = config.getint('slip_detection_ratio_threshold', 3)  # If the actual distance is less than 1/3 then it is considered as slip

        self.variable_path = config.get('variable_path')
        self.all_variables = {}
        self.load_variables()

        self.motion_counter = EncoderCounter(self.printer, self.encoder_pin_name,
                                             self.encoder_sample_time,
                                             self.encoder_poll_time,
                                             self.all_variables['calibrated_encoder_resolution'])

        # GCode commands
        self.gcode = self.printer.lookup_object('gcode')

        self.gcode.register_command('_ERCF_SERVO_UP',
                                    self.cmd_ERCF_SERVO_UP,
                                    desc='Lift the servo arm to release the gear')
        self.gcode.register_command('_ERCF_SERVO_DOWN',
                                    self.cmd_ERCF_SERVO_DOWN,
                                    desc='Press the servo arm to engage the gear')
        self.gcode.register_command('_ERCF_LOAD',
                                    self.cmd_ERCF_LOAD,
                                    desc='Load the filament to the nozzle')
        self.gcode.register_command('_ERCF_LOAD_FRESH',
                                    self.cmd_ERCF_LOAD_FRESH,
                                    desc='Load the filament to the nozzle')
        self.gcode.register_command('_ERCF_UNLOAD',
                                    self.cmd_ERCF_UNLOAD,
                                    desc='Unload the filament back to the selector')
        self.gcode.register_command('_ERCF_HOME_SELECTOR',
                                    self.cmd_ERCF_HOME_SELECTOR,
                                    desc='Home the selector cart')
        self.gcode.register_command('_ERCF_MOVE_SELECTOR_TO_TOOL',
                                    self.cmd_ERCF_MOVE_SELECTOR_TO_TOOL,
                                    desc='Move the selector cart to the corresponding tool')
        self.gcode.register_command('_ERCF_CHANGE_TOOL',
                                    self.cmd_ERCF_CHANGE_TOOL,
                                    desc='Tool change gcode')
        self.gcode.register_command('_ERCF_MOTORS_OFF',
                                    self.ercf_motors_off,
                                    desc='Turn off both the gear and selector stepper')

        # Calibration
        self.gcode.register_command('_ERCF_CALIBRATE_ENCODER_RESOLUTION',
                                    self.cmd_CALIBRATE_ENCODER_RESOLUTION,
                                    desc='Calibrate the resolution of the encoder')
        self.gcode.register_command('_ERCF_CALIBRATE_COMPONENT_LENGTH',
                                    self.cmd_ERCF_CALIBRATE_COMPONENT_LENGTH,
                                    desc='Execute the calibration routine on the current tool')
        self.gcode.register_command('_ERCF_CALIBRATE_GEAR_STEPPER_ROTATION_DISTANCE',
                                    self.cmd_CALIBRATE_GEAR_STEPPER_ROTATION_DISTANCE,
                                    desc='Calibrate the rotation distance')
        self.gcode.register_command('_ERCF_CALIBRATE_SELECTOR_LOCATION',
                                    self.cmd_ERCF_CALIBRATE_SELECTOR_LOCATION,
                                    desc='Locate the selector')
        self.gcode.register_command('_ERCF_CALIBRATE_EXTRUSION_FACTOR',
                                    self.cmd_ERCF_CALIBRATE_EXTRUSION_FACTOR,
                                    desc='Calibrate the extrusion factor against the reference channel')

        # Register event
        self.printer.register_event_handler('klippy:connect', self.handle_connect)
        self.printer.register_event_handler('stepper_enable:motor_off', self._on_motor_off)

    def handle_connect(self):
        self.toolhead = self.printer.lookup_object('toolhead')
        self.gear_stepper = self.printer.lookup_object(self.gear_stepper_name)
        self.selector_stepper = self.printer.lookup_object(self.selector_stepper_name)
        self.servo = self.printer.lookup_object(self.servo_name)
        if self.toolhead_sensor_name is not None:
            self.toolhead_sensor = self.printer.lookup_object(self.toolhead_sensor_name)

        self.reference_gear_stepper_rotation_distance = self.gear_stepper.get_steppers()[0].get_rotation_distance()[0]

        self.original_extruder_move_speed = self.toolhead.get_extruder().max_e_velocity
        self.original_extruder_move_accel = self.toolhead.get_extruder().max_e_accel

        # Read extruder move speed and acceleration
        if self.extruder_move_speed is None:
            self.extruder_move_speed = self.original_extruder_move_speed
        if self.extruder_move_accel is None:
            self.extruder_move_accel = self.original_extruder_move_accel

        # Initialize state machine status
        self._servo_status = None
        self._current_tool = None

    def load_variables(self):
        allvars = {}
        varfile = configparser.ConfigParser()
        try:
            varfile.read(self.variable_path)
            if varfile.has_section('Variables'):
                for name, val in varfile.items('Variables'):
                    allvars[name] = ast.literal_eval(val)
        except:
            msg = "Unable to parse existing variable file"
            logging.exception(msg)
            raise self.printer.command_error(msg)
        self.all_variables = allvars

    def save_variables(self):
        varfile = configparser.ConfigParser()
        varfile.add_section('Variables')
        for name, value in sorted(self.all_variables.items()):
            varfile.set('Variables', name, repr(value))

        try:
            with open(self.variable_path, 'w') as fp:
                varfile.write(fp)
        except:
            msg = "Unable to write to config file"
            logging.exception(msg)
            raise self.printer.command_error(msg)

    def get_status(self, eventtime):
        return {'current_tool': self._current_tool}

    def _on_motor_off(self, print_time=None):
        # Unset the current tool location when all motors are off
        self._current_tool = None

    def log_to_gcmd_respond(self, gcmd, text):
        gcmd.respond_info("ERCF:" + text)

    @contextmanager
    def _gear_stepper_move_guard(self, lift_servo=True):
        try:
            yield
        finally:
            if lift_servo:
                self.servo_up()

    @contextmanager
    def _command_exception_handler(self, gcmd):
        try:
            yield
        except FatalPrinterError as e:
            idle_timeout = self.printer.lookup_object('idle_timeout')
            if idle_timeout is None:
                raise self.printer.config_error("No idle timeout found")

            curtime = self.printer.get_reactor().monotonic()
            status = idle_timeout.get_status(curtime)
            if status['state'] == 'Printing':
                self.log_to_gcmd_respond(gcmd, 'Caught exception: {}, Calling {}'.format(e, self.MACRO_PAUSE))
                self.gcode.run_script_from_command(self.MACRO_PAUSE)
            else:
                if self.debug_mode:
                    self.log_to_gcmd_respond(gcmd, "Caught exception: {}, \nCallstack:\n---------------\n{}".format(e, traceback.format_exc()))
                else:
                    raise self.printer.command_error(e)
        # Catch other error
        except Exception as e:
            if self.debug_mode:
                self.log_to_gcmd_respond(gcmd, "Caught exception: {}, \nCallstack:\n---------------\n{}".format(e, traceback.format_exc()))
            else:
                raise

    def cmd_ERCF_SERVO_UP(self, gcmd):
        with self._command_exception_handler(gcmd):
            self.servo_up()

    def cmd_ERCF_SERVO_DOWN(self, gcmd):
        with self._command_exception_handler(gcmd):
            self.servo_down()

    def cmd_ERCF_CALIBRATE_COMPONENT_LENGTH(self, gcmd):
        with self._command_exception_handler(gcmd):
            self.calibrate_component_length(gcmd)

    def cmd_ERCF_LOAD(self, gcmd):
        with self._command_exception_handler(gcmd):
            self.ercf_load_from_unknown_location(gcmd)

    def cmd_ERCF_LOAD_FRESH(self, gcmd):
        with self._command_exception_handler(gcmd):
            self.ercf_load_fresh(gcmd)

    def cmd_ERCF_UNLOAD(self, gcmd):
        with self._command_exception_handler(gcmd):
            self.ercf_unload(gcmd)

    def cmd_ERCF_HOME_SELECTOR(self, gcmd):
        with self._command_exception_handler(gcmd):
            self.ercf_home_selector(gcmd)

    def cmd_ERCF_MOVE_SELECTOR_TO_TOOL(self, gcmd):
        with self._command_exception_handler(gcmd):
            tool_idx = int(gcmd.get_float("TOOL"))
            self.ercf_move_selector_to_tool(gcmd, tool_idx)

    def cmd_ERCF_CHANGE_TOOL(self, gcmd):
        with self._command_exception_handler(gcmd):
            tool_idx = int(gcmd.get_float("TOOL"))
            self.ercf_change_tool(gcmd, tool_idx)

    def cmd_ERCF_CALIBRATE_SELECTOR_LOCATION(self, gcmd):
        with self._command_exception_handler(gcmd):
            tool_idx = int(gcmd.get_float("TOOL"))
            self.calibrate_selector_location(gcmd, tool_idx)

    def cmd_ERCF_CALIBRATE_EXTRUSION_FACTOR(self, gcmd):
        with self._command_exception_handler(gcmd):
            tool_idx = int(gcmd.get_float("TOOL"))
            self.calibrate_extrusion_factor(gcmd, tool_idx)

    def cmd_CALIBRATE_GEAR_STEPPER_ROTATION_DISTANCE(self, gcmd):
        with self._command_exception_handler(gcmd):
            self.calibrate_gear_stepper_rotation_distance(gcmd)

    def cmd_CALIBRATE_ENCODER_RESOLUTION(self, gcmd):
        with self._command_exception_handler(gcmd):
            self.calibrate_encoder_resolution(gcmd)

    def servo_up(self):
        if self._servo_status != 'up':
            self._servo_status = 'up'
            servo_name = self.servo_name.split()[1]
            self.gcode.run_script_from_command('SET_SERVO SERVO={} ANGLE={}'.format(servo_name,
                                                                                    self.servo_up_angle))
            self.toolhead.dwell(0.25 + self.extra_servo_dwell_up)

            # Turn servo off
            self.gcode.run_script_from_command('SET_SERVO SERVO={} WIDTH=0.0'.format(servo_name))
            self.toolhead.wait_moves()

    def servo_down(self):
        if self._servo_status != 'down':
            self._servo_status = 'down'
            servo_name = self.servo_name.split()[1]

            # do the gear meshing to ensure the proper alignment of the selector gear
            self.gear_stepper.do_set_position(0)
            self.gear_stepper.do_move(0.5, speed=25, accel=self.gear_stepper_accel, sync=False)

            self.gcode.run_script_from_command('SET_SERVO SERVO={} ANGLE={}'.format(servo_name,
                                                                                    self.servo_down_angle))
            self.toolhead.wait_moves()
            self.toolhead.dwell(0.2)

            self.gear_stepper.do_move(0.0, speed=25, accel=self.gear_stepper_accel, sync=False)
            self.toolhead.dwell(0.1)
            self.gear_stepper.do_move(-0.5, speed=25, accel=self.gear_stepper_accel, sync=False)
            self.toolhead.dwell(0.1 + self.extra_servo_dwell_down)
            self.gear_stepper.do_move(0.0, speed=25, accel=self.gear_stepper_accel, sync=False)

            self.toolhead.wait_moves()

            # Turn off if required
            if self.servo_down_turn_off:
                self.gcode.run_script_from_command('SET_SERVO SERVO={} WIDTH=0.0'.format(servo_name))

            self.toolhead.wait_moves()

    def _gear_stepper_move_wait_legacy(self, dist, wait=True, speed=None, accel=None):
        # LEGACY METHOD. Pending removal
        self.gear_stepper.do_set_position(0.)
        is_long_move = abs(dist) > self.gear_stepper_long_move_threshold
        if speed is None:
            speed = self.long_moves_speed if is_long_move \
                    else self.short_moves_speed
        if accel is None:
            accel = self.long_moves_accel if is_long_move \
                    else self.short_moves_accel
        self.gear_stepper.do_move(dist, speed, accel, True)
        if wait:
            self.toolhead.wait_moves()

    def gear_stepper_move_wait(self, gcmd, target_move_distance,
                               step_distance=None, step_speed=None, step_accel=None,
                               raise_on_filament_slip=True, expect_partial_move=False,
                               lift_servo=True):
        self.servo_down()
        with self._gear_stepper_move_guard(lift_servo):
            accumulated_move_distance = self.stepper_move_wait(gcmd,
                                                               target_move_distance=target_move_distance,
                                                               step_distance=step_distance, step_speed=step_speed, step_accel=step_accel,
                                                               stepper_block_move_callback=self._gear_stepper_block_move,
                                                               raise_on_filament_slip=raise_on_filament_slip,
                                                               expect_partial_move=expect_partial_move)

        return accumulated_move_distance

    def toolhead_move_wait(self, gcmd, target_move_distance,
                           step_distance=None, step_speed=None,
                           initial_condition_callback=None, stop_condition_callback=None,
                           raise_on_filament_slip=True,
                           expect_partial_move=False):
        self.servo_up()
        accumulated_move_distance = self.stepper_move_wait(gcmd,
                                                           target_move_distance=target_move_distance,
                                                           step_distance=step_distance, step_speed=step_speed,
                                                           stepper_block_move_callback=self._toolhead_block_move,
                                                           stepper_init_callback=self._toolhead_move_init,
                                                           stepper_stop_callback=self._toolhead_move_stop,
                                                           initial_condition_callback=initial_condition_callback,
                                                           stop_condition_callback=stop_condition_callback,
                                                           raise_on_filament_slip=raise_on_filament_slip,
                                                           expect_partial_move=expect_partial_move)
        return accumulated_move_distance

    def stepper_move_wait(self, gcmd, target_move_distance,
                          stepper_block_move_callback,
                          stepper_init_callback=None,
                          stepper_stop_callback=None,
                          step_distance=None, step_speed=None, step_accel=None,
                          initial_condition_callback=None, stop_condition_callback=None,
                          raise_on_filament_slip=True,
                          expect_partial_move=False):
        if stop_condition_callback is None:
            stop_condition_callback = lambda x=None: x
        if initial_condition_callback is None:
            initial_condition_callback = lambda x=None: x

        if step_distance is None:
            step_distance = min(self.long_move_distance, abs(target_move_distance))

        # Calculate maximum single step distance based on the user config to avoid `Timer too close` issue.
        step_distance = min(step_distance, self.maximum_step_distance)

        if target_move_distance >= 0:
            direction = 1
        else:
            direction = -1

        if stepper_init_callback is None:
            stepper_init_callback = lambda x=None:x

        stepper_status = stepper_init_callback()
        self.motion_counter.reset_counts()

        accumulated_move_distance = 0
        self.log_to_gcmd_respond(gcmd, 'Requested stepper move distance: {} with step length: {}, accel: {}'.format(
            target_move_distance, step_distance, step_accel))

        try:
            prev_state = initial_condition_callback()
            remain_distance = abs(target_move_distance) - accumulated_move_distance

            while remain_distance > 0:
                # Calculate step
                step_distance = max(min(step_distance, remain_distance), self.minimum_step_distance)
                relative_step_distance = step_distance * direction

                # Update default step speed and acceleration
                if step_distance >= self.long_move_distance:
                    if step_speed is None:
                        step_speed = self.long_moves_speed
                    if step_accel is None:
                        step_accel = self.long_moves_accel
                else:
                    if step_speed is None:
                        step_speed = self.short_moves_speed
                    if step_accel is None:
                        step_accel = self.short_moves_accel

                # Move
                self.motion_counter.reset_counts()
                stepper_status = stepper_block_move_callback(stepper_status, relative_step_distance, step_speed, step_accel)
                # Check move distance
                filament_move_distance = self.motion_counter.get_distance()
                accumulated_move_distance += filament_move_distance
                remain_distance = abs(target_move_distance) - accumulated_move_distance

                if filament_move_distance < (step_distance / self.slip_detection_ratio_threshold):
                    msg = 'Filament is not moving. Requested: {}, filament measured move: {}'.format(step_distance, filament_move_distance)
                    raise FilamentSlipException(msg)

                # Check stop condition
                prev_state = stop_condition_callback(prev_state)

            else:
                # NO event
                if expect_partial_move:
                    raise self.printer.command_error('Full move is not expected. Actual moved distance: {}. Please check the calibration.'.format(accumulated_move_distance))

        except FilamentSlipException as e:
            msg = str(e)
            if raise_on_filament_slip:
                raise self.printer.command_error(msg)
            else:
                self.log_to_gcmd_respond(gcmd, msg)
        except StopConditionException:
            pass
        finally:
            if stepper_stop_callback:
                stepper_stop_callback(stepper_status)

        self.log_to_gcmd_respond(gcmd, 'Actual stepper move distance: {}'.format(accumulated_move_distance * direction))

        return accumulated_move_distance * direction

    def _toolhead_block_move(self, toolhead_position, relative_step_distance, speed, accel):
        toolhead_position[3] += relative_step_distance
        self.toolhead.manual_move(toolhead_position, speed)
        self.toolhead.wait_moves()

        return toolhead_position

    def _toolhead_move_init(self):
        self.gcode.run_script_from_command('G92 E0')
        toolhead_position = self.toolhead.get_position()

        # Set acceleration
        extruder = self.toolhead.get_extruder()
        extruder.max_e_velocity = self.extruder_move_speed
        extruder.max_e_accel = self.extruder_move_accel

        return toolhead_position

    def _toolhead_move_stop(self, stepper_status):
        # Restore speed and acceleration
        extruder = self.toolhead.get_extruder()
        extruder.max_e_velocity = self.original_extruder_move_speed
        extruder.max_e_accel = self.original_extruder_move_accel

    def _gear_stepper_block_move(self, stepper_status, relative_step_distance, speed, accel):
        self.gear_stepper.do_set_position(0)
        self.gear_stepper.do_move(relative_step_distance, speed, accel, sync=True)
        self.toolhead.wait_moves()  # WHY TOOLHEAD??
        return None

    def _toolhead_gear_stepper_synchronized_block_move(self, toolhead_position, relative_step_distance, speed, accel):
        # Setup the gear stepper first
        self.gear_stepper.do_set_position(0)
        self.gear_stepper.do_move(relative_step_distance, speed, self.toolhead.max_accel, sync=False)  # Do not use acceleration control in synchronous move

        toolhead_position[3] += relative_step_distance
        self.toolhead.manual_move(toolhead_position, speed)

        self.toolhead.wait_moves()

        return toolhead_position

    def _stop_on_filament_present(self, prev_condition=None):
        """
        The callback function to raise the exception when the filament sensor is triggered
        """
        if self.toolhead_sensor is None:
            raise self.printer.config_error("Unexpected call to toolhead filament sensor")

        if self.toolhead_sensor.runout_helper.filament_present:
            raise StopConditionException
        return prev_condition

    def ercf_unload_to_toolhead_sensor(self, gcmd):
        if self.toolhead_sensor is None:
            raise self.printer.config_error('Toolhead filament sensor is not defined')

        nozzle_to_sensor_length = self.all_variables.get('calibrated_nozzle_to_sensor_length')
        if nozzle_to_sensor_length is None:
            raise self.printer.command_error('Sensor before extruder setup is currently not supported')

        def stop_on_filament_not_present(prev_condition=None):
            if not self.toolhead_sensor.runout_helper.filament_present:
                raise StopConditionException
            return prev_condition

        nozzle_to_sensor_length = self.all_variables.get('calibrated_nozzle_to_sensor_length')
        accumulated_move_distance = self.toolhead_move_wait(gcmd, -nozzle_to_sensor_length - self.extra_move_margin, self.short_move_distance,
                                                            raise_on_filament_slip=True,
                                                            initial_condition_callback=stop_on_filament_not_present,
                                                            stop_condition_callback=stop_on_filament_not_present)

        self.log_to_gcmd_respond(
            gcmd, 'The filament tip is now parked right before the filament sensor. The total move distance: {}'.format(accumulated_move_distance))

        return accumulated_move_distance

    def ercf_load_from_toolhead_sensor(self, gcmd):
        if self.toolhead_sensor is None:
            raise self.printer.config_error('Toolhead filament sensor is not defined')

        # Extrude until the toolhead sensor (should be relative short)
        nozzle_to_sensor_length = self.all_variables.get('calibrated_nozzle_to_sensor_length')
        accumulated_move_distance = self.toolhead_move_wait(gcmd,
                                                            target_move_distance=nozzle_to_sensor_length,
                                                            step_distance=self.minimum_step_distance,
                                                            step_speed=self.short_moves_speed,
                                                            initial_condition_callback=self._stop_on_filament_present,
                                                            stop_condition_callback=self._stop_on_filament_present,
                                                            expect_partial_move=True)

        # Extrude to the toolhead (without feedback)
        nozzle_to_sensor_length = self.all_variables.get('calibrated_nozzle_to_sensor_length')
        accumulated_move_distance += self.toolhead_move_wait(gcmd,
                                                             target_move_distance=nozzle_to_sensor_length,
                                                             step_distance=nozzle_to_sensor_length,
                                                             step_speed=self.short_moves_speed,
                                                             raise_on_filament_slip=False)

        self.log_to_gcmd_respond(gcmd, 'The filament is loaded to the nozzle. The total move distance: {}'.format(accumulated_move_distance))

        return accumulated_move_distance

    def ercf_unload_from_toolhead_sensor_to_extruder(self, gcmd):
        # First stage is to unload the filament to a proximate position with long move distance and speed
        retract_distance = self.all_variables.get('calibrated_sensor_to_extruder_length') - self.long_move_distance
        accumulated_move_distance = self.toolhead_move_wait(gcmd, -retract_distance, raise_on_filament_slip=False,)

        self.log_to_gcmd_respond(gcmd,
            'The filament unloaded just passed the extruder. The total move distance: {}'.format(accumulated_move_distance))

        return accumulated_move_distance

    def ercf_unload_from_extruder_to_selector(self, gcmd):
        accumulated_move_distance = 0
        self.log_to_gcmd_respond(gcmd, 'Unloading from extruder to the selector')
        with self._gear_stepper_move_guard():
            self.servo_down()
            # Move a little by both toolhead and gear stepper to help pulling from the extruder
            target_move_distance = self.short_move_distance
            actual_move_distance = self.stepper_move_wait(gcmd,
                                                          target_move_distance=-target_move_distance,
                                                          stepper_block_move_callback=self._toolhead_gear_stepper_synchronized_block_move,
                                                          stepper_init_callback=self._toolhead_move_init,
                                                          stepper_stop_callback=self._toolhead_move_stop,
                                                          step_distance=self.short_move_distance,
                                                          step_speed=self.short_moves_speed,
                                                          step_accel=self.short_moves_accel,
                                                          raise_on_filament_slip=True)
            accumulated_move_distance += actual_move_distance

            # Pull purely by the gear stepper
            target_move_distance = max(0, self.all_variables['calibrated_extruder_to_selector_length'] - abs(actual_move_distance) - self.long_move_distance)
            actual_move_distance = self.gear_stepper_move_wait(gcmd,
                                                               target_move_distance=-target_move_distance,
                                                               step_distance=target_move_distance,
                                                               step_speed=self.long_moves_speed,
                                                               step_accel=self.long_moves_accel,
                                                               raise_on_filament_slip=True, lift_servo=False)
            accumulated_move_distance += actual_move_distance

            # Retract back to the selector with short moves
            target_move_distance = max(0, self.all_variables['calibrated_extruder_to_selector_length'] - abs(actual_move_distance)) + self.long_move_distance + self.extra_move_margin
            actual_move_distance = self.gear_stepper_move_wait(gcmd,
                                                               target_move_distance=-target_move_distance,
                                                               step_distance=self.short_move_distance,
                                                               step_speed=self.short_moves_speed,
                                                               step_accel=self.short_moves_accel,
                                                               raise_on_filament_slip=False, expect_partial_move=True,
                                                               lift_servo=False)
            accumulated_move_distance += actual_move_distance

            # Pull back a little
            target_move_distance = 7
            actual_move_distance = self.gear_stepper_move_wait(gcmd,
                                                               target_move_distance=-target_move_distance,
                                                               step_distance=self.short_move_distance,
                                                               step_speed=self.short_moves_speed,
                                                               step_accel=self.short_moves_accel,
                                                               raise_on_filament_slip=False, lift_servo=False)
            accumulated_move_distance += actual_move_distance

            self.servo_up()

        return accumulated_move_distance

    def ercf_unload(self, gcmd):
        # Lift the servo before attempting to move the toolhead
        self.servo_up()

        accumulated_move_distance = 0

        # Unload from toolhead
        if self.toolhead_sensor is None:
            #   1. Run tip forming
            #   2. Unload a known distance (calibrated_nozzle_to_extruder_length)
            #   3. If filament is not moving, then unload to selector
            self.gcode.run_script_from_command('_ERCF_FORM_TIP_STANDALONE')

            # Unload with single move
            self.log_to_gcmd_respond(gcmd, "Unloading from nozzle to extruder")
            target_move_distance = self.all_variables.get('calibrated_nozzle_to_extruder_length') + self.extra_move_margin
            accumulated_move_distance = self.toolhead_move_wait(gcmd, raise_on_filament_slip=False,
                                                                target_move_distance=-target_move_distance,
                                                                step_distance=target_move_distance,
                                                                step_speed=self.long_moves_speed,
                                                                expect_partial_move=True  # I'm expecting filament to slip as we moved with extra distance
                                                                )

            # Unload to selector
            accumulated_move_distance += self.ercf_unload_from_extruder_to_selector(gcmd)

        elif self.toolhead_sensor.runout_helper.filament_present:
            # TODO: Make it a function instead of running as the macro
            self.gcode.run_script_from_command('_ERCF_FORM_TIP_STANDALONE')

            self.log_to_gcmd_respond(gcmd, 'Unloading from nozzle to toolhead sensor')
            self.ercf_unload_to_toolhead_sensor(gcmd)

            # This is the clean retraction
            self.log_to_gcmd_respond(gcmd, 'Unloading from toolhead sensor to extruder')
            target_move_distance = self.all_variables['calibrated_sensor_to_extruder_length']
            actual_move_distance = self.toolhead_move_wait(gcmd,
                                                           target_move_distance=-target_move_distance,
                                                           step_distance=target_move_distance,
                                                           step_speed=self.long_moves_speed,
                                                           raise_on_filament_slip=False)
            accumulated_move_distance += actual_move_distance

            # Move a little to disengage with the extruder
            target_move_distance = max(0, self.all_variables['calibrated_sensor_to_extruder_length'] - abs(actual_move_distance)) + self.long_move_distance
            actual_move_distance = self.toolhead_move_wait(gcmd,
                                                           target_move_distance=-target_move_distance,
                                                           step_distance=self.short_move_distance,
                                                           step_speed=self.short_moves_speed,
                                                           raise_on_filament_slip=False,
                                                           expect_partial_move=True)
            accumulated_move_distance += actual_move_distance

            accumulated_move_distance += self.ercf_unload_from_extruder_to_selector(gcmd)

        # Unload from unknown location
        else:
            self.log_to_gcmd_respond(gcmd, 'Unloading from unknown location')

            # Do a short move to verify the filament is still engaged with the extruder
            actual_move_distance = self.toolhead_move_wait(gcmd,
                                                           target_move_distance=-self.short_move_distance,
                                                           step_distance=self.short_move_distance,
                                                           step_speed=self.short_move_distance,
                                                           raise_on_filament_slip=False)
            accumulated_move_distance += actual_move_distance
            if actual_move_distance != 0:
                # Do the short move until not moving anymore
                target_move_distance = self.all_variables['calibrated_sensor_to_extruder_length'] + self.long_move_distance
                actual_move_distance = self.toolhead_move_wait(gcmd,
                                                               target_move_distance=-target_move_distance,
                                                               step_distance=self.short_move_distance,
                                                               step_speed=self.short_move_distance,
                                                               expect_partial_move=True,
                                                               raise_on_filament_slip=False)
                accumulated_move_distance += actual_move_distance

                accumulated_move_distance += self.ercf_unload_from_extruder_to_selector(gcmd)
            else:
                # Between extruder and the selector
                with self._gear_stepper_move_guard():
                    self.servo_down()

                    # Move a little by both toolhead and gear stepper to help pulling from the extruder
                    target_move_distance = self.short_move_distance
                    actual_move_distance = self.stepper_move_wait(gcmd,
                                                                  target_move_distance=-target_move_distance,
                                                                  stepper_block_move_callback=self._toolhead_gear_stepper_synchronized_block_move,
                                                                  stepper_init_callback=self._toolhead_move_init,
                                                                  stepper_stop_callback=self._toolhead_move_stop,
                                                                  step_distance=self.short_move_distance,
                                                                  step_speed=self.short_moves_speed,
                                                                  step_accel=self.short_moves_accel,
                                                                  raise_on_filament_slip=True)
                    accumulated_move_distance += actual_move_distance

                    # Move slowly until the end of the selector
                    target_move_distance = self.all_variables['calibrated_extruder_to_selector_length'] + self.long_move_distance + self.extra_move_margin
                    actual_move_distance = self.gear_stepper_move_wait(gcmd,
                                                                       target_move_distance=-target_move_distance,
                                                                       step_distance=self.short_move_distance,
                                                                       step_speed=self.short_moves_speed,
                                                                       step_accel=self.short_moves_accel,
                                                                       raise_on_filament_slip=False,
                                                                       expect_partial_move=True,
                                                                       lift_servo=False)
                    accumulated_move_distance += actual_move_distance

                    # Pull back a little
                    target_move_distance = 5
                    actual_move_distance = self.gear_stepper_move_wait(gcmd,
                                                                       target_move_distance=-target_move_distance,
                                                                       step_distance=self.short_move_distance,
                                                                       step_speed=self.short_moves_speed,
                                                                       step_accel=self.short_moves_accel,
                                                                       raise_on_filament_slip=False, lift_servo=False)
                    accumulated_move_distance += actual_move_distance
                    self.servo_up()

        self.log_to_gcmd_respond(gcmd, 'Filament is unloaded to the selector with total move distance: {}'.format(accumulated_move_distance))

    def ercf_load_from_extruder_to_nozzle(self, gcmd):
        """
        A single move from extruder to nozzle without feedback
        """
        nozzle_to_extruder_length = self.all_variables.get("calibrated_nozzle_to_extruder_length")
        actual_move_distance = self.toolhead_move_wait(gcmd,
                                                       target_move_distance=nozzle_to_extruder_length,
                                                       step_distance=nozzle_to_extruder_length,
                                                       step_speed=self.short_moves_speed,
                                                       raise_on_filament_slip=False)
        return actual_move_distance

    def ercf_load_from_unknown_location(self, gcmd):
        accumulated_step_distance = 0

        if self.toolhead_sensor is None:
            # TODO:
            #   1. Check if the filament is engaged inside the extruder
            #       If true then move slowly with a certain distance (calibrated_nozzle_to_extruder_length) -> Finish
            #       If false then run the ercf_unload_from_extruder_to_selector
            #   2. Load fresh
            try:
                accumulated_step_distance += self.toolhead_move_wait(gcmd, raise_on_filament_slip=True,
                                                                     target_move_distance=self.short_move_distance,
                                                                     step_distance=self.short_move_distance,
                                                                     step_speed=self.short_moves_speed,
                                                                     expect_partial_move=False)
            except self.printer.command_error:
                # unload, the filament is between extruder and selector, or even before the selector
                self.ercf_unload_from_extruder_to_selector(gcmd)
                self.ercf_load_fresh(gcmd)
            else:
                # Load to the nozzle
                accumulated_step_distance += self.ercf_load_from_extruder_to_nozzle(gcmd)

        elif self.toolhead_sensor.runout_helper.filament_present:
            self.ercf_load_from_toolhead_sensor(gcmd)
        else:
            with self._gear_stepper_move_guard():
                # Check if the filament is engaged inside the selector
                self.log_to_gcmd_respond(gcmd, 'Check filament engagement inside selector')
                test_move_distance = 0
                for i in range(self.selector_filament_engagement_retry):
                    self.servo_down()
                    test_move_distance += self.stepper_move_wait(gcmd,
                                                                 target_move_distance=30,
                                                                 stepper_block_move_callback=self._toolhead_gear_stepper_synchronized_block_move,
                                                                 stepper_init_callback=self._toolhead_move_init,
                                                                 stepper_stop_callback=self._toolhead_move_stop,
                                                                 step_distance=30,
                                                                 step_speed=self.short_moves_speed,
                                                                 step_accel=self.short_moves_accel,
                                                                 raise_on_filament_slip=False)
                    if test_move_distance > 0:
                        break
                    else:
                        self.servo_up()
                    self.log_to_gcmd_respond(gcmd, 'Filament is not engaged. Will retry')
                else:
                    raise self.printer.command_error('Filament is not engaged inside the selector. Please insert the filament manually')

                # Load with long step until the filament sensor has triggered
                accumulated_step_distance = test_move_distance
                target_distance = self.all_variables['calibrated_extruder_to_selector_length'] + self.all_variables['calibrated_sensor_to_extruder_length']
                accumulated_step_distance += self.stepper_move_wait(gcmd,
                                                                    target_move_distance=target_distance,
                                                                    stepper_block_move_callback=self._toolhead_gear_stepper_synchronized_block_move,
                                                                    stepper_init_callback=self._toolhead_move_init,
                                                                    stepper_stop_callback=self._toolhead_move_stop,
                                                                    step_distance=self.long_move_distance,
                                                                    step_speed=self.short_moves_speed,
                                                                    step_accel=self.short_moves_accel,
                                                                    stop_condition_callback=self._stop_on_filament_present,
                                                                    raise_on_filament_slip=True)

            # When the filament sensor has triggered, then load from now on
            self.ercf_load_from_unknown_location(gcmd)

    def ercf_load_fresh(self, gcmd):
        if self.toolhead_sensor is not None and self.toolhead_sensor.runout_helper.filament_present:
            raise self.printer.command_error('Call ERCF_LOAD instead')

        # Check if the filament tip is somehow ended up in the selector block. If true then unload to get fresh start.
        if self.is_filament_in_selector(lift_servo=False, skip_filament_block_check=True):
            self.ercf_unload(gcmd)

        accumulated_step_distance = 0
        with self._gear_stepper_move_guard():
            # Check gear engagement
            test_move_distance = 0
            for i in range(self.selector_filament_engagement_retry):
                self.servo_down()
                test_move_distance += self.gear_stepper_move_wait(gcmd,
                                                                  target_move_distance=30,
                                                                  step_distance=30,
                                                                  step_speed=self.short_moves_speed,
                                                                  step_accel=self.short_moves_accel,
                                                                  raise_on_filament_slip=False,
                                                                  lift_servo=False)
                if test_move_distance > 0:
                    break
                else:
                    self.servo_up()
                self.log_to_gcmd_respond(gcmd, 'Filament is not engaged. Will retry')
            else:
                raise self.printer.command_error(
                    'Filament is not engaged inside the selector. Please insert the filament manually')

            # Do a single move to feed the filament to the extruder
            self.log_to_gcmd_respond(gcmd, 'Feeding from selector to just before the extruder -- long single move with just gear stepper')
            accumulated_step_distance = test_move_distance
            target_distance = max(0, self.all_variables['calibrated_extruder_to_selector_length'] - test_move_distance - self.long_move_distance)
            actual_distance = self.gear_stepper_move_wait(gcmd,
                                                          target_move_distance=target_distance,
                                                          step_distance=target_distance,
                                                          raise_on_filament_slip=True,
                                                          lift_servo=False)
            accumulated_step_distance += actual_distance

            # Feed to the extruder
            self.log_to_gcmd_respond(gcmd, 'Feeding to the extruder -- short pulse move using both extruders and stop on filament slip')
            target_distance = max(0, self.all_variables['calibrated_extruder_to_selector_length'] - actual_distance) + self.long_move_distance
            actual_distance = self.stepper_move_wait(gcmd,
                                                     target_move_distance=target_distance,
                                                     step_distance=target_distance,
                                                     step_speed=self.short_moves_speed,
                                                     step_accel=self.short_moves_accel,
                                                     stepper_block_move_callback=self._toolhead_gear_stepper_synchronized_block_move,
                                                     stepper_init_callback=self._toolhead_move_init,
                                                     stepper_stop_callback=self._toolhead_move_stop,
                                                     raise_on_filament_slip=True)
            accumulated_step_distance += actual_distance

            # Release the gear stepper and move to next
            self.servo_up()

        if self.toolhead_sensor is not None:
            # Move the toolhead single long move approaching the sensor
            self.log_to_gcmd_respond(gcmd, 'Feeding from the extruder to just before the sensor -- long single move with just the extruder, stop on filament sensor trigger')
            target_distance = max(0, self.all_variables['calibrated_sensor_to_extruder_length'] - actual_distance - self.long_move_distance)
            if target_distance < self.long_move_distance:
                step_distance = self.short_move_distance
            else:
                step_distance = target_distance
            actual_distance = self.toolhead_move_wait(gcmd,
                                                      target_move_distance=target_distance,
                                                      step_distance=step_distance,
                                                      initial_condition_callback=self._toolhead_move_init,
                                                      stop_condition_callback=self._stop_on_filament_present,
                                                      raise_on_filament_slip=True)
            accumulated_step_distance += actual_distance

            # Since we are closed to the toolhead sensor, we want do move slowly
            self.log_to_gcmd_respond(gcmd, 'Feeding to the sensor -- short pulse move and stop on filament sensor trigger')
            target_distance = max(0, self.all_variables['calibrated_sensor_to_extruder_length'] - actual_distance) + self.long_move_distance
            accumulated_step_distance += self.toolhead_move_wait(gcmd,
                                                                 target_move_distance=target_distance,
                                                                 step_distance=self.minimum_step_distance,
                                                                 step_speed=self.short_moves_speed,
                                                                 raise_on_filament_slip=True,
                                                                 initial_condition_callback=self._toolhead_move_init,
                                                                 stop_condition_callback=self._stop_on_filament_present)

            # we are on the filament sensor, move the final distance
            self.ercf_load_from_toolhead_sensor(gcmd)
        else:
            # Blind move from extruder to nozzle
            self.ercf_load_from_extruder_to_nozzle(gcmd)

    def is_filament_in_selector(self, lift_servo=True, skip_filament_block_check=False):
        with self._gear_stepper_move_guard(lift_servo):
            self.motion_counter.reset_counts()
            self.servo_down()
            move_distance = self.motion_counter.get_distance()

            # If the filament is moving while gear meshing then the filament is certainly inserted
            if move_distance > 0:
                return True

            # Secondary check (tiny moving distance)
            self.gear_stepper.do_set_position(0)
            self.motion_counter.reset_counts()
            self.gear_stepper.do_move(1, self.short_moves_speed, self.short_moves_accel, True)
            self.gear_stepper.do_move(0, self.short_moves_speed, self.short_moves_accel, True)
            self.toolhead.wait_moves()
            move_distance = self.motion_counter.get_distance()

            if move_distance > 0:
                return True

            # No need to check further, we can assume the filament is not present
            if skip_filament_block_check:
                return False

            # Third check (move 8mm forward to check if the filament is just extruded from the filament block)
            self.gear_stepper.do_set_position(0)
            self.gear_stepper.do_move(8, self.short_moves_speed, self.short_moves_accel, True)
            self.gear_stepper.do_move(0, self.short_moves_speed, self.short_moves_accel, True)
            self.toolhead.wait_moves()
            move_distance = self.motion_counter.get_distance()

            if move_distance > 0:
                return True

        return False

    def ercf_home_selector(self, gcmd):
        num_channels = len(self.all_variables['color_selector_positions'])
        homing_move_distance = 20 + num_channels * 21 + (num_channels/3.0) * 5

        # Check the filament status
        if self.is_filament_in_selector():
            self.log_to_gcmd_respond(gcmd, 'Filament is still in the selector cart. Will do the unload')

            # There is chance this will fail as the filament already retracted in the block but require 1-3mm retraction
            try:
                self.ercf_unload(gcmd)
            except Exception as e:
                # Check if the filament is in the selector again. If clear then we should ignore the unload error
                if self.is_filament_in_selector():
                    # Not good. Need some help from the user
                    self.log_to_gcmd_respond(gcmd, 'Unable to clear the filament from the selector. Requires user attention')
                    raise
                else:
                    self.log_to_gcmd_respond(gcmd, 'Filament is clear from the selector. Will continue homing')

        # TODO: Implement the sensorless homing
        self.selector_stepper.do_set_position(0)
        self.selector_stepper.do_homing_move(movepos=-homing_move_distance,
                                             speed=100,
                                             accel=self.short_moves_accel,
                                             triggered=True,
                                             check_trigger=True)
        self.selector_stepper.do_set_position(0)

        # Retract and do second homing
        self.selector_stepper.do_move(movepos=5,
                                      speed=100,
                                      accel=self.short_moves_accel)

        self.selector_stepper.do_homing_move(movepos=-10,
                                             speed=20,
                                             accel=self.short_moves_accel,
                                             triggered=True,
                                             check_trigger=True)
        self.selector_stepper.do_set_position(0)

        # Unset the selector
        self._current_tool = None
        self.ercf_move_selector_to_tool(gcmd, tool_idx=0, force=True)

        self.log_to_gcmd_respond(gcmd, 'Selector homed')

    def ercf_move_selector_to_tool(self, gcmd, tool_idx, force=False):
        color_selector_positions = self.all_variables['color_selector_positions']
        if tool_idx >= len(color_selector_positions):
            raise self.printer.command_error('Invalid tool index: {}'.format(tool_idx))

        if self._current_tool != tool_idx:
            if not force and self._current_tool is None:
                if self.auto_home_selector:
                    self.ercf_home_selector(gcmd)
                else:
                    raise self.printer.command_error('Selector must be homed before switching to the next tool')

            # Check the filament status
            elif not force and self.is_filament_in_selector():
                self.log_to_gcmd_respond(gcmd, 'Filament is still in the selector cart. Will do the unload')
                self.ercf_unload(gcmd)

            # Move the selector
            cart_move_distance = color_selector_positions[tool_idx]
            self.selector_stepper.do_move(movepos=cart_move_distance,
                                          speed=100,
                                          accel=self.short_moves_accel)

            self._current_tool = tool_idx
            self.log_to_gcmd_respond(gcmd, 'Tool {} is selected'.format(tool_idx))

        else:
            self.log_to_gcmd_respond(gcmd, 'Tool {} is already selected'.format(tool_idx))

    def ercf_change_tool(self, gcmd, tool_idx):
        self.ercf_move_selector_to_tool(gcmd, tool_idx)

        # Update extrusion factor
        extrusion_factor = self.all_variables['calibrated_tool_extrusion_factor'][tool_idx]
        new_rotation_distance = self.reference_gear_stepper_rotation_distance / extrusion_factor
        self.gear_stepper.get_steppers()[0].set_rotation_distance(new_rotation_distance)
        self.log_to_gcmd_respond(gcmd, 'Tool {} update rotation distance from {} ref to {}'.format(tool_idx,
                                                                                      self.reference_gear_stepper_rotation_distance,
                                                                                      new_rotation_distance))

        # Load to the toolhead
        self.ercf_load_fresh(gcmd)

    def ercf_motors_off(self, gcmd):
        self.gear_stepper.do_enable(False)
        self.selector_stepper.do_enable(False)
        self._on_motor_off()

    def calibrate_selector_location(self, gcmd, tool_idx):
        color_selector_positions = self.all_variables['color_selector_positions']

        if tool_idx >= len(color_selector_positions):
            raise self.printer.command_error('Invalid tool index: {}'.format(tool_idx))

        # Move the block + 1 length
        homing_move_distance = 20 + (tool_idx + 1) * 21 + ((tool_idx + 1) / 3.0) * 5

        self.selector_stepper.do_set_position(0)

        # Do the homing move
        initial_position = self.selector_stepper.get_steppers()[0].get_mcu_position()
        self.selector_stepper.do_homing_move(movepos=-homing_move_distance,
                                             speed=50,
                                             accel=self.short_moves_accel,
                                             triggered=True,
                                             check_trigger=True)
        self.toolhead.wait_moves()

        final_position = self.selector_stepper.get_steppers()[0].get_mcu_position()
        travel_distance = abs(final_position - initial_position) * self.selector_stepper.get_steppers()[0].get_step_dist()

        self.log_to_gcmd_respond(gcmd, 'Tool {} location: {}'.format(tool_idx, travel_distance))

        self.all_variables['color_selector_positions'][tool_idx] = travel_distance

        self.save_variables()

        # This is critical. The locate action is going to throw the homing invalid.
        self._current_tool = None

    def calibrate_encoder_resolution(self, gcmd):
        calibrate_move_distance = gcmd.get_float('DISTANCE', 500)
        repeats = gcmd.get_int('REPEATS', 1)

        speeds = [self.long_moves_speed, self.short_moves_speed]
        accels = [self.long_moves_accel, self.short_moves_accel]

        forward_count = []
        backward_count = []
        with self._gear_stepper_move_guard():
            self.servo_down()
            for speed, accel in product(speeds, accels):
                for _ in range(repeats):
                    self.log_to_gcmd_respond(gcmd, 'Speed: {}, Acceleration: {}'.format(speed, accel))
                    # Moving forwards
                    self.motion_counter.reset_counts()

                    self.gear_stepper.do_set_position(0)
                    self.gear_stepper.do_move(calibrate_move_distance, speed, accel, True)
                    self.toolhead.wait_moves()

                    count = self.motion_counter.get_counts()
                    self.log_to_gcmd_respond(gcmd, 'Forward Count: {}'.format(count))
                    forward_count.append(count)

                    # Moving backwards
                    self.motion_counter.reset_counts()
                    self.gear_stepper.do_move(0, speed, accel, True)
                    self.toolhead.wait_moves()
                    count = self.motion_counter.get_counts()
                    self.log_to_gcmd_respond(gcmd, 'Backward Count: {}'.format(count))
                    backward_count.append(count)

            self.servo_up()

        forward_median = median(forward_count)
        backward_median = median(backward_count)
        half_mean = (forward_median + backward_median) / 4

        # TODO: Why half median?
        resolution = calibrate_move_distance / half_mean

        self.log_to_gcmd_respond(gcmd, 'Old resolution: {}, New resolution: {}'.format(self.all_variables['calibrated_encoder_resolution'],
                                                                          resolution))
        # Apply result
        self.motion_counter.set_encoder_steps(resolution)
        self.all_variables['calibrated_encoder_resolution'] = resolution
        self.save_variables()

    def calibrate_gear_stepper_rotation_distance(self, gcmd):
        with self._gear_stepper_move_guard():
            self.servo_down()
            calibrate_move_distance = gcmd.get_float('DISTANCE', 100)
            self.gear_stepper.do_set_position(0)
            self.gear_stepper.do_move(calibrate_move_distance, self.short_moves_speed, self.short_moves_accel, True)
            self.toolhead.wait_moves()
            self.servo_up()

    def calibrate_extrusion_factor(self, gcmd, tool_idx):
        repeats = gcmd.get_int('REPEATS', 1)
        calibrate_move_distance = min(self.all_variables['calibrated_extruder_to_selector_length'] * 0.66, 200)

        self.log_to_gcmd_respond(gcmd, 'Tool {} requesting to move {} for extrusion factor calibration'.format(tool_idx, calibrate_move_distance))

        self.ercf_move_selector_to_tool(gcmd, tool_idx)

        speeds = [self.long_moves_speed, self.short_moves_speed]
        accels = [self.long_moves_accel, self.short_moves_accel]

        forward_count = []
        backward_count = []
        with self._gear_stepper_move_guard():
            self.servo_down()
            for speed, accel in product(speeds, accels):
                for _ in range(repeats):
                    self.log_to_gcmd_respond(gcmd, 'Speed: {}, Acceleration: {}'.format(speed, accel))
                    # Moving forwards
                    self.motion_counter.reset_counts()

                    self.gear_stepper.do_set_position(0)
                    self.gear_stepper.do_move(calibrate_move_distance, speed, accel, True)
                    self.toolhead.wait_moves()

                    distance = self.motion_counter.get_distance()
                    self.log_to_gcmd_respond(gcmd, 'Forward distance: {}'.format(distance))
                    forward_count.append(distance)

                    # Moving backwards
                    self.motion_counter.reset_counts()
                    self.gear_stepper.do_move(0, speed, accel, True)
                    self.toolhead.wait_moves()
                    distance = self.motion_counter.get_distance()
                    self.log_to_gcmd_respond(gcmd, 'Backward distance: {}'.format(distance))
                    backward_count.append(distance)

            self.servo_up()

        forward_median = median(forward_count)
        backward_median = median(backward_count)
        mean = (forward_median + backward_median) / 2.0

        self.log_to_gcmd_respond(gcmd, 'Tool {} moved {}'.format(tool_idx, mean))

        extrusion_factor = calibrate_move_distance / mean

        self.log_to_gcmd_respond(gcmd, 'Tool {} extrusion factor: {}'.format(tool_idx, extrusion_factor))

        # Apply
        self.all_variables['calibrated_tool_extrusion_factor'][tool_idx] = extrusion_factor
        self.save_variables()

    def calibrate_component_length(self, gcmd):
        """
        The calibration is designed to follow the filament path backwards:

        Without toolhead sensor:
        Nozzle -> Extruder -> Selector

        With filament sensor:
        Nozzle -> Filament Sensor -> Extruder -> Selector

        """
        self.log_to_gcmd_respond(gcmd, 'Going to calibrate the length of each component by unloading the '
                                       'filament from nozzle to the ERCF selector')

        # Optionally, run the tip forming gcode prior to the calibration
        if self.tip_forming_gcode_before_calibration:
            self.gcode.run_script_from_command(self.tip_forming_gcode_before_calibration)

        # Sanity check: the toolhead sensor should trigger
        if self.toolhead_sensor and not bool(self.toolhead_sensor.runout_helper.filament_present):
            raise self.printer.command_error(
                'Filament is not loaded to the toolhead, or the filament sensor is not triggering')
        elif self.toolhead_sensor is None:
            self.log_to_gcmd_respond(gcmd, 'Going to run the toolhead sensorless calibration')

        # Remove slack for filament in the bowden tube so the tiny toolhead movement can be detected
        self.log_to_gcmd_respond(gcmd, 'Remove slack by pushing in tiny steps. Will stop on filament slip')

        original_minimum_step_distance = self.minimum_step_distance
        self.minimum_step_distance = 1  # Temporarily override the minimum step distance
        original_slip_detection_ratio_threshold = self.slip_detection_ratio_threshold
        self.slip_detection_ratio_threshold = 1.5  # If move less than 2/3 then consider it slip
        try:
            with self._gear_stepper_move_guard():
                self.servo_down()
                actual_distance = self.gear_stepper_move_wait(gcmd,
                                                              target_move_distance=20,
                                                              step_distance=1,
                                                              step_speed=self.short_moves_speed,
                                                              step_accel=self.short_moves_accel,
                                                              raise_on_filament_slip=False)
                self.log_to_gcmd_respond(gcmd, 'There is about {}mm slack in the feeding system. This number will not be recorded'.format(actual_distance))
                self.servo_up()
        finally:
            self.minimum_step_distance = original_minimum_step_distance
            self.slip_detection_ratio_threshold = original_slip_detection_ratio_threshold

        # Will load the variable to begin with, then use the reference value as the maximum moving distance
        self.load_variables()

        # Variables to dump to Vars
        nozzle_to_sensor_length = None
        nozzle_to_extruder_length = None
        sensor_to_extruder_length = None
        extruder_to_selector_length = None

        self.toolhead.wait_moves()
        self.motion_counter.reset_counts()

        ###########
        # STAGE 1 #
        ###########
        if self.toolhead_sensor is None:
            ## For Sensorless toolhead configuration
            # At stage 1 the toolhead shall retract until the filament is not moving
            # This step will generate
            #   - calibrated_nozzle_to_extruder_length
            self.log_to_gcmd_respond(gcmd, "Stage 1: Calibrating nozzle to extruder length")
            try:
                actual_distance = self.toolhead_move_wait(gcmd, raise_on_filament_slip=False,
                                                          target_move_distance=-(self.all_variables.get('calibrated_nozzle_to_extruder_length') + self.extra_move_margin),
                                                          step_distance=self.long_move_distance,  # Use single long move to get filament out
                                                          step_speed=self.short_moves_speed,
                                                          expect_partial_move=True)
            except self.printer.command_error:
                raise FatalPrinterError(
                    'Calibration Stage 1 does not expect full move without slip. Please reset your config file '
                    '[calibrated_nozzle_to_extruder_length] according to '
                    'https://github.com/eamars/ercf_driver/blob/main/ercf_vars.cfg'
                )

            self.all_variables['calibrated_nozzle_to_extruder_length'] = abs(actual_distance)

            self.log_to_gcmd_respond(gcmd, "Stage 1 Done")

        else:
            ############
            # STAGE 1a #
            ############
            # The toolhead shall retract, until the toolhead filament sensor is not triggered
            # This step will generate
            #   - calibrated_nozzle_to_sensor_length
            self.log_to_gcmd_respond(gcmd, "Stage 1a: Calibrating nozzle to sensor length")
            try:
                actual_distance = self.toolhead_move_wait(gcmd, raise_on_filament_slip=True,
                                                          target_move_distance=-(self.all_variables.get('calibrated_nozzle_to_sensor_length') + self.extra_move_margin),
                                                          step_distance=self.calibrate_move_distance_per_step,
                                                          step_speed=self.short_moves_speed)
            except self.printer.command_error:
                raise FatalPrinterError(
                    'Calibration Stage 1a does not expect filament to slip. Please check your filament path to ensure '
                    'the filament can be ejected to the spool'
                )

            self.all_variables['calibrated_nozzle_to_sensor_length'] = abs(actual_distance)

            self.log_to_gcmd_respond(gcmd, "Stage 1a Done")

            ############
            # STAGE 1b #
            ############
            # The toolhead shall retract until the filament is not moving
            # This step will generate
            #   - calibrated_sensor_to_extruder_length
            self.log_to_gcmd_respond(gcmd, "Stage 1b: Calibrating sensor to extruder length")
            try:
                actual_distance = self.toolhead_move_wait(gcmd, raise_on_filament_slip=False,
                                                          target_move_distance=-(self.all_variables.get('calibrated_nozzle_to_extruder_length') + self.extra_move_margin),
                                                          step_distance=self.calibrate_move_distance_per_step,
                                                          step_speed=self.short_moves_speed,
                                                          expect_partial_move=True)
            except self.printer.command_error:
                raise FatalPrinterError(
                    'Calibration Stage 1b does not expect full move without slip. Please reset your config file '
                    '[calibrated_nozzle_to_extruder_length] according to '
                    'https://github.com/eamars/ercf_driver/blob/main/ercf_vars.cfg'
                )

            self.all_variables['calibrated_nozzle_to_extruder_length'] = abs(actual_distance)

            self.log_to_gcmd_respond(gcmd, "Stage 1b Done")

        ###########
        # STAGE 2 #
        ###########
        # The gear stepper shall retract until the filament is retracted to the selector
        # This step will generate
        #   - calibrated_extruder_to_selector_length

        # Move a little by both toolhead and gear stepper to help pulling from the extruder
        with self._gear_stepper_move_guard():
            self.servo_down()
            self.log_to_gcmd_respond(gcmd, "Stage 2a: Calibrating extruder to selector length (small sync move)")
            try:
                actual_distance = self.stepper_move_wait(gcmd,
                                                         target_move_distance=-self.short_move_distance,
                                                         stepper_block_move_callback=self._toolhead_gear_stepper_synchronized_block_move,
                                                         stepper_init_callback=self._toolhead_move_init,
                                                         stepper_stop_callback=self._toolhead_move_stop,
                                                         step_distance=self.short_move_distance,
                                                         step_speed=self.short_moves_speed,
                                                         step_accel=self.short_moves_accel,
                                                         raise_on_filament_slip=True)
            except self.printer.command_error:
                raise FatalPrinterError(
                    'Calibration Stage 2a does not expect filament to slip. Please check your filament path to ensure '
                    'the filament can be ejected to the spool and the filament is still engaged by the gear stepper.'
                )
            self.log_to_gcmd_respond(gcmd, "Stage 2a Done")

            # Continue to retract but with gear stepper only
            self.log_to_gcmd_respond(gcmd, "Stage 2b: Calibrating extruder to selector length")
            try:
                actual_distance += self.gear_stepper_move_wait(gcmd,
                                                               target_move_distance=-(self.all_variables['calibrated_extruder_to_selector_length'] + self.extra_move_margin),
                                                               step_distance=self.calibrate_move_distance_per_step,
                                                               step_speed=self.short_moves_speed,
                                                               step_accel=self.short_moves_accel,
                                                               raise_on_filament_slip=False,
                                                               expect_partial_move=True)
            except self.printer.command_error:
                raise FatalPrinterError(
                    'Calibration Stage 2b does not expect full move without slip. Please reset your config file '
                    '[calibrated_extruder_to_selector_length] according to '
                    'https://github.com/eamars/ercf_driver/blob/main/ercf_vars.cfg'
                )

            self.all_variables['calibrated_extruder_to_selector_length'] = abs(actual_distance)
            self.log_to_gcmd_respond(gcmd, "Stage 2b Done")

        ############
        # Finalize #
        ############
        self.log_to_gcmd_respond(gcmd,
                                 "Calibrated component lengths: " + str(self.all_variables))

        self.save_variables()


def load_config(config):
    return ERCF(config)
