from . import pulse_counter
from . import force_move
import toolhead
import configparser
import logging
import ast
import time
from contextlib import contextmanager


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

    def set_distance(self, new_distance):
        self._counts = int( ( new_distance / self._encoder_steps ) * 2. )

    def reset_counts(self):
        self._counts = 0.


class StopConditionException(Exception):
    pass


class ERCF(object):
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
        self.toolhead_sensor_name = config.get('toolhead_sensor', None)
        self.toolhead_sensor = None

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

        # Step distance
        self.extra_move_margin = config.getfloat('extra_move_margin', 100)
        self.long_move_distance = config.getfloat('long_move_distance', 10)
        self.short_move_distance = config.getfloat('short_move_distance', 3)
        self.minimum_step_distance = config.getfloat('minimum_step_distance', 2)
        self.calibrate_move_distance_per_step = config.getfloat('calibrate_move_distance_per_step', 3)

        self.servo_up_angle = config.getfloat('servo_up_angle')
        self.servo_down_angle = config.getfloat('servo_down_angle')
        self.extra_servo_dwell_up = config.getfloat('extra_servo_dwell_up', 0)
        self.extra_servo_dwell_down = config.getfloat('extra_servo_dwell_down', 0)

        self.variable_path = config.get('variable_path')
        self.all_variables = {}
        self.load_variables()

        self.motion_counter = EncoderCounter(self.printer, self.encoder_pin_name,
                                             self.encoder_sample_time,
                                             self.encoder_poll_time,
                                             self.all_variables['calibrated_encoder_resolution'])

        # GCode commands
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_command('_ERCF_CALIBRATE_COMPONENT_LENGTH',
                                    self.cmd_ERCF_CALIBRATE_COMPONENT_LENGTH,
                                    desc='Execute the calibration routine on the current tool')
        self.gcode.register_command('_ERCF_SERVO_UP',
                                    self.cmd_ERCF_SERVO_UP,
                                    desc='Lift the servo arm to release the gear')
        self.gcode.register_command('_ERCF_SERVO_DOWN',
                                    self.cmd_ERCF_SERVO_DOWN,
                                    desc='Press the servo arm to engage the gear')
        self.gcode.register_command('_ERCF_LOAD',
                                    self.cmd_ERCF_LOAD,
                                    desc='Load the filament to the nozzle')
        self.gcode.register_command('_ERCF_UNLOAD',
                                    self.cmd_ERCF_UNLOAD,
                                    desc='Unload the filament back to the selector')

        self.gcode.register_command('_ERCF_UNLOAD_TO_TOOLHEAD_SENSOR',
                                    self.ercf_unload_to_toolhead_sensor,
                                    desc='Unload the filament back to the toolhead sensor')
        self.gcode.register_command('_ERCF_LOAD_FROM_TOOLHEAD_SENSOR',
                                    self.ercf_load_from_toolhead_sensor,
                                    desc='Load the filament from the toolhead sensor to the nozzle')
        self.gcode.register_command('_ERCF_UNLOAD_FROM_TOOLHEAD_SENSOR_TO_EXTRUDER',
                                    self.ercf_unload_from_toolhead_sensor_to_extruder,
                                    desc='Unload the filament from toolhead sensor back to the extruder')

        # Register event
        self.printer.register_event_handler('klippy:connect', self.handle_connect)

    def handle_connect(self):
        self.toolhead = self.printer.lookup_object('toolhead')
        self.gear_stepper = self.printer.lookup_object(self.gear_stepper_name)
        self.selector_stepper = self.printer.lookup_object(self.selector_stepper_name)
        self.servo = self.printer.lookup_object(self.servo_name)
        if self.toolhead_sensor_name is not None:
            self.toolhead_sensor = self.printer.lookup_object(self.toolhead_sensor_name)

        # Initialize state machine status
        self._servo_status = None
        self._ercf_status = None

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

    @contextmanager
    def _ercf_move_guard(self, always_servo_up=True):
        try:
            yield
        finally:
            if always_servo_up:
                self.servo_up()

            # Reset the extruder
            self.gcode.run_script_from_command('G92 E0')

    def cmd_ERCF_SERVO_UP(self, gcmd):
        self.servo_up()

    def cmd_ERCF_SERVO_DOWN(self, gcmd):
        self.servo_down()

    def cmd_ERCF_CALIBRATE_COMPONENT_LENGTH(self, gcmd):
        with self._ercf_move_guard():
            self.calibrate_component_length(gcmd)

    def cmd_ERCF_LOAD(self, gcmd):
        with self._ercf_move_guard():
            self.ercf_load(gcmd)

    def cmd_ERCF_UNLOAD(self, gcmd):
        with self._ercf_move_guard():
            self.ercf_unload(gcmd)

    def servo_up(self):
        if self._servo_status != 'up':
            self._servo_status = 'up'
            servo_name = self.servo_name.split()[1]
            self.gcode.run_script_from_command('SET_SERVO SERVO={} ANGLE={}'.format(servo_name,
                                                                                    self.servo_up_angle))
            self.toolhead.wait_moves()
            time.sleep(0.25 + self.extra_servo_dwell_up)
            self.gcode.run_script_from_command('SET_SERVO SERVO={} WIDTH=0.0'.format(servo_name))
            self.toolhead.wait_moves()

    def servo_down(self):
        if self._servo_status != 'down':
            self._servo_status = 'down'
            servo_name = self.servo_name.split()[1]

            # do the gear meshing to ensure the proper alignment of the selector gear
            self.gear_stepper.do_set_position(0)
            self.gear_stepper.do_move(0.5, speed=25, accel=self.gear_stepper_accel)

            self.gcode.run_script_from_command('SET_SERVO SERVO={} ANGLE={}'.format(servo_name,
                                                                                    self.servo_down_angle))
            self.toolhead.wait_moves()
            time.sleep(0.2)

            self.gear_stepper.do_move(0.0, speed=25, accel=self.gear_stepper_accel)
            time.sleep(0.1)
            self.gear_stepper.do_move(-0.5, speed=25, accel=self.gear_stepper_accel)
            time.sleep(0.1 + self.extra_servo_dwell_down)
            self.gear_stepper.do_move(0.0, speed=25, accel=self.gear_stepper_accel)
            self.toolhead.wait_moves()
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

    def gear_stepper_move_wait(self, gcmd, target_move_distance, step_distance=None, raise_on_filament_slip=True, lift_servo=True):
        self.servo_down()
        try:
            accumulated_move_distance = self.stepper_move_wait(gcmd,
                                                               target_move_distance=target_move_distance,
                                                               step_distance=step_distance,
                                                               stepper_block_move_callback=self._gear_stepper_block_move,
                                                               raise_on_filament_slip=raise_on_filament_slip)
        finally:
            if lift_servo:
                self.servo_up()

        return accumulated_move_distance

    def toolhead_move_wait(self, gcmd, target_move_distance, step_distance=None, raise_on_filament_slip=True,
                           initial_condition_callback=None, stop_condition_callback=None):
        accumulated_move_distance = self.stepper_move_wait(gcmd,
                                                           target_move_distance=target_move_distance,
                                                           step_distance=step_distance,
                                                           stepper_block_move_callback=self._toolhead_block_move,
                                                           stepper_init_callback=self._toolhead_move_init,
                                                           initial_condition_callback=initial_condition_callback,
                                                           stop_condition_callback=stop_condition_callback,
                                                           raise_on_filament_slip=raise_on_filament_slip)
        return accumulated_move_distance

    def stepper_move_wait(self, gcmd, target_move_distance,
                          stepper_block_move_callback,
                          stepper_init_callback=None,
                          step_distance=None, step_speed=None, step_accel=None,
                          raise_on_filament_slip=True,
                          initial_condition_callback=None, stop_condition_callback=None):
        if stop_condition_callback is None:
            stop_condition_callback = lambda x=None: x
        if initial_condition_callback is None:
            initial_condition_callback = lambda x=None: x

        if step_distance is None:
            step_distance = min(self.long_move_distance, abs(target_move_distance))

        if target_move_distance >= 0:
            direction = 1
        else:
            direction = -1

        if step_distance >= self.long_move_distance:
            if step_speed is None:
                step_speed = self.long_moves_speed
            if step_accel is None:
                step_accel = accel = self.long_moves_accel
        else:
            if step_speed is None:
                step_speed = self.short_moves_speed
            if step_accel is None:
                step_accel = self.short_moves_accel

        if stepper_init_callback is None:
            stepper_init_callback = lambda x=None:x

        relative_step_distance = step_distance * direction

        stepper_status = stepper_init_callback()
        self.motion_counter.reset_counts()

        accumulated_move_distance = 0
        gcmd.respond_info('Requested stepper move distance: {}'.format(target_move_distance))

        try:
            prev_state = initial_condition_callback()

            while (abs(target_move_distance) - accumulated_move_distance) > step_distance:
                # Move
                self.motion_counter.reset_counts()
                stepper_status = stepper_block_move_callback(stepper_status, relative_step_distance, step_speed, step_accel)
                # Check move distance
                filament_move_distance = self.motion_counter.get_distance()
                accumulated_move_distance += filament_move_distance

                gcmd.respond_info('Measured move distance: {}, accumulated move distance: {}'
                                  .format(filament_move_distance, accumulated_move_distance))

                if filament_move_distance < step_distance / 3.0:
                    msg = 'Filament is not moving. Requested: {}, filament measured move: {}'.format(step_distance, filament_move_distance)
                    if raise_on_filament_slip:
                        raise self.printer.command_error(msg)
                    else:
                        gcmd.respond_info(msg)
                        raise StopConditionException

                # Check stop condition
                prev_state = stop_condition_callback(prev_state)

            # TODO: Handle the case where the step distance is still larger than the short_move_distance
            # Now move the remaining distance
            step_distance = abs(target_move_distance) - accumulated_move_distance
            relative_step_distance = step_distance * direction
            speed = self.short_moves_speed
            accel = self.short_moves_accel

            self.motion_counter.reset_counts()
            stepper_status = stepper_block_move_callback(stepper_status, relative_step_distance, speed, accel)
            self.toolhead.wait_moves()

            filament_move_distance = self.motion_counter.get_distance()
            accumulated_move_distance += filament_move_distance
        except StopConditionException:
            pass

        return accumulated_move_distance * direction

    def _toolhead_block_move(self, toolhead_position, relative_step_distance, speed, accel):
        toolhead_position[3] += relative_step_distance
        self.toolhead.manual_move(toolhead_position, speed)
        self.toolhead.wait_moves()

        return toolhead_position

    def _toolhead_move_init(self):
        self.gcode.run_script_from_command('G92 E0')
        toolhead_position = self.toolhead.get_position()

        return toolhead_position

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

    def ercf_unload_to_toolhead_sensor(self, gcmd):
        if not self.toolhead_sensor:
            raise self.printer.command_error('Filament sensor is not defined')

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

        gcmd.respond_info('The filament tip is now parked right before the filament sensor. The total move distance: {}'
                          .format(accumulated_move_distance))

        return accumulated_move_distance

    def ercf_load_from_toolhead_sensor(self, gcmd):
        if not self.toolhead_sensor:
            raise self.printer.command_error('Filament sensor is not defined')

        def stop_on_filament_present(prev_condition=None):
            if self.toolhead_sensor.runout_helper.filament_present:
                raise StopConditionException
            return prev_condition

        # Extrude until the toolhead sensor (should be relative short)
        nozzle_to_sensor_length = self.all_variables.get('calibrated_nozzle_to_sensor_length')
        accumulated_move_distance = self.toolhead_move_wait(gcmd, nozzle_to_sensor_length, self.short_move_distance,
                                                            initial_condition_callback=stop_on_filament_present,
                                                            stop_condition_callback=stop_on_filament_present)

        # Extrude to the toolhead (without feedback)
        nozzle_to_sensor_length = self.all_variables.get('calibrated_nozzle_to_sensor_length')
        accumulated_move_distance += self.toolhead_move_wait(gcmd, nozzle_to_sensor_length, self.short_move_distance)

        gcmd.respond_info('The filament is loaded to the nozzle. The total move distance: {}'.format(accumulated_move_distance))

        return accumulated_move_distance

    def ercf_unload_from_toolhead_sensor_to_extruder(self, gcmd):
        # First stage is to unload the filament to a proximate position with long move distance and speed
        retract_distance = self.all_variables.get('calibrated_sensor_to_extruder_length') - self.long_move_distance
        accumulated_move_distance = self.toolhead_move_wait(gcmd, -retract_distance, raise_on_filament_slip=False)

        gcmd.respond_info(
            'The filament unloaded just passed the extruder. The total move distance: {}'.format(accumulated_move_distance))

        return accumulated_move_distance

    def ercf_unload(self, gcmd):
        # TODO: Should ignore the first retraction to avoid tension from the entire bowden path?
        # Full unload routine
        accmulated_move_distance = 0
        gcmd.respond_info('Unloading from nozzle to toolhead sensor')
        accmulated_move_distance += self.ercf_unload_to_toolhead_sensor(gcmd)

        gcmd.respond_info('Unloading from toolhead sensor to extruder')
        accmulated_move_distance += self.ercf_unload_from_toolhead_sensor_to_extruder(gcmd)

        # Synchronize move the extruder and gear stepper a short distance
        gcmd.respond_info('Unloading from extruder to selector')
        accmulated_move_distance += self.stepper_move_wait(gcmd,
                                                           target_move_distance=-self.long_move_distance,
                                                           stepper_block_move_callback=self._toolhead_gear_stepper_synchronized_block_move,
                                                           stepper_init_callback=self._toolhead_move_init,
                                                           step_distance=self.short_move_distance,
                                                           step_speed=self.short_moves_speed,
                                                           step_accel=self.short_moves_accel,
                                                           raise_on_filament_slip=False)

        if not accmulated_move_distance == 0:
            # No slip move for the major calibrated distance
            major_move_distance = self.all_variables['calibrated_extruder_to_selector_length'] - self.long_move_distance
            self.gear_stepper_move_wait(gcmd, -major_move_distance, major_move_distance,
                                        raise_on_filament_slip=True, lift_servo=False)
            minor_move_distance = self.long_move_distance * 3
            # Stop on slip
            self.gear_stepper_move_wait(gcmd, -minor_move_distance, self.short_move_distance,
                                        raise_on_filament_slip=False, lift_servo=True)
        else:
            gcmd.respond_info('Filament tip is not in the extruder. Will skip the long move')
            major_move_distance = self.all_variables['calibrated_extruder_to_selector_length']

            # Stop on slip
            self.gear_stepper_move_wait(gcmd, -major_move_distance, self.short_move_distance,
                                        raise_on_filament_slip=False, lift_servo=True)

    def ercf_load(self, gcmd):
        """
        Determine current position:
         * If toolhead sensor is triggered, the filament tip is
            * Between the nozzle and extruder (if calibrated_extruder_to_sensor_length is positive)
            * Between the extruder and the selector (if calibrated_extruder_to_sensor_length is negative)
            -> Retract until the toolhead sensor is empty, then extrude from current position
         * If toolhead sensor is not triggered or not present, apply few mm of extrusion from the extruder
            * If moved, then the filament tip is between the nozzle to the extruder
                -> Retract until the filament doesn't move anymore, then extrude from current position
            * If not moved, then the filament tip is between the extruder to the selector
                -> Retract until the filament doesn't move anymore, then continue next step
         * Extrude few mm of filament from gear stepper
            * If moved, then retract until not moving anymore.
            * If not moved, then extrude from the gear stepper
        """
        if self.toolhead_sensor and self.toolhead_sensor.runout_helper.filament_present:
            self.servo_up()
            self.ercf_unload_to_toolhead_sensor(gcmd)
            self.ercf_load_from_toolhead_sensor(gcmd)
        else:
            pass
            #FIXME

    def calibrate_component_length(self, gcmd):
        gcmd.respond_info('Going to calibrate the length of each component by unloading the '
                          'filament from nozzle to the ERCF selector')

        self.servo_up()

        # Sanity check: the toolhead sensor should trigger
        if self.toolhead_sensor and not bool(self.toolhead_sensor.runout_helper.filament_present):
            raise self.printer.command_error('Filament is not loaded to the toolhead, or the filament sensor is not triggering')
        elif self.toolhead_sensor is None:
            gcmd.respond_info('Going to run the toolhead sensorless calibration')

        # Form the tip to begin with
        # self.gcode.run_script_from_command('_ERCF_FORM_TIP_STANDALONE')

        # Variables to dump to Vars
        nozzle_to_sensor_length = None
        nozzle_to_extruder_length = None
        sensor_to_extruder_length = None
        extruder_to_selector_length = None

        self.toolhead.wait_moves()
        time.sleep(3)
        self.motion_counter.reset_counts()

        ###########
        # STAGE 1 #
        ###########
        # At stage 1 the toolhead shall retract, until
        #   - The toolhead filament sensor is not triggered or
        #   - The filament is not moving (filament sensor is installed between the ERCF and the extruder)
        # This step will generate
        #   - nozzle_to_sensor_length: IF the sensor stage is changed
        #   - nozzle_to_extruder_length: IF the filament move passes the extruder (sensorless or the sensor is installed
        #       before the extruder).
        stage_1_move_distance = 0
        self.gcode.run_script_from_command('G92 E0')
        toolhead_position = self.toolhead.get_position()

        while True:
            # Move
            toolhead_position[3] -= self.calibrate_move_distance_per_step

            # We rely on the theoretical move distance as this is actuated by the extruder
            # stage_1_move_distance += self.calibrate_move_distance_per_step
            self.motion_counter.reset_counts()

            # Retract the move distance
            self.toolhead.manual_move(toolhead_position, self.short_moves_speed)
            self.toolhead.wait_moves()

            # Check the filament sensor status
            filament_move_distance = self.motion_counter.get_distance()
            stage_1_move_distance += filament_move_distance

            filament_moved = True
            # If filament moved less than a third of requested length then we consider not moving
            if filament_move_distance < self.calibrate_move_distance_per_step / 3.0:
                filament_moved = False

            gcmd.respond_info('Stage 1: Requested {}, filament measured move: {}, filament moved: {}'.format(
                self.calibrate_move_distance_per_step, filament_move_distance, filament_moved
            ))

            if self.toolhead_sensor:
                filament_present = bool(self.toolhead_sensor.runout_helper.filament_present)
                gcmd.respond_info('Stage 1: filament present: {}'.format(filament_present))
                if not filament_present:
                    nozzle_to_sensor_length = stage_1_move_distance
                    gcmd.respond_info('Stage 1: Filament is extracted passing the toolhead sensor')
                    break

            if not filament_moved:
                nozzle_to_extruder_length = stage_1_move_distance
                gcmd.respond_info('Stage 1: Filament passes the extruder')
                break

        ############
        # STAGE 2a #
        ############
        # At stage 2a the toolhead shall retract if nozzle_to_sensor_length is detected, until
        #   - The filament is not moving
        # This stage shall look for
        #   - sensor_to_extruder_length
        if nozzle_to_sensor_length is not None:
            stage_2_move_distance = 0
            self.gcode.run_script_from_command('G92 E0')
            toolhead_position = self.toolhead.get_position()
            while True:
                toolhead_position[3] -= self.calibrate_move_distance_per_step

                # stage_2_move_distance += self.calibrate_move_distance_per_step
                self.motion_counter.reset_counts()

                # Retract the move distance
                self.toolhead.manual_move(toolhead_position, self.short_moves_speed)
                self.toolhead.wait_moves()

                # Check the filament movement status
                filament_move_distance = self.motion_counter.get_distance()
                stage_2_move_distance += filament_move_distance

                filament_moved = True
                # If filament moved less than a third of requested length then we consider not moving
                if filament_move_distance < self.calibrate_move_distance_per_step / 3.0:
                    filament_moved = False

                gcmd.respond_info('Stage 2a: Requested {}, Filament measured move: {}'.format(
                    self.calibrate_move_distance_per_step, filament_move_distance
                ))

                if not filament_moved:
                    gcmd.respond_info('Stage 2a: Filament passes the extruder')
                    sensor_to_extruder_length = stage_2_move_distance
                    break

        ############
        # STAGE 2b #
        ############
        # At stage 2b the gear stepper shall retract if the nozzle to extruder length is found and will continue to
        # look for the sensor, until
        #   - The toolhead filament sensor is not triggered or
        #   - The filament is not moving (error condition)
        # This stage shall look for
        #   - sensor_to_extruder_length: The distance between the extruder and the sensor, represented as a negative number
        if self.toolhead_sensor and nozzle_to_extruder_length is not None:
            self.servo_down()
            stage_2_move_distance = 0
            while True:
                # Retract the move distance
                # stage_2_move_distance += self.calibrate_move_distance_per_step
                self.motion_counter.reset_counts()

                self._gear_stepper_move_wait_legacy(-self.calibrate_move_distance_per_step)

                # Check the filament status
                filament_present = bool(self.toolhead_sensor.runout_helper.filament_present)
                filament_move_distance = self.motion_counter.get_distance()
                stage_2_move_distance += filament_move_distance

                filament_moved = True
                # If filament moved less than a third of requested length then we consider not moving
                if filament_move_distance < self.calibrate_move_distance_per_step / 3.0:
                    filament_moved = False

                logging.debug('Stage 2b: Requested {}, Filament present: {}, Filament measured move: {}'.format(
                    self.calibrate_move_distance_per_step, filament_present, filament_move_distance
                ))

                if not filament_present:
                    # Yes we are using a negative value to represent the extruder to sensor length
                    # if the sensor is installed between the extruder and the gear stepper
                    gcmd.respond_info('Stage 2b: Filament passes the sensor')
                    sensor_to_extruder_length = -stage_2_move_distance
                    break

                if not filament_moved:
                    raise self.printer.command_error('Stage 2b: Filament is not moving')

        ###########
        # STAGE 3 #
        ###########
        # Sanity check
        if self.toolhead_sensor and bool(self.toolhead_sensor.runout_helper.filament_present):
            raise self.printer.command_error('Stage 3: Filament is still inside the toolhead while engaging the ERCF gear')

        # At stage 3 the gear stepper shall retract until the filament is retract below the selector
        stage_3_move_distance = 0
        self.servo_down()
        while True:
            # stage_3_move_distance += self.calibrate_move_distance_per_step
            self.motion_counter.reset_counts()

            self._gear_stepper_move_wait_legacy(-self.calibrate_move_distance_per_step)

            # Check filament status
            filament_move_distance = self.motion_counter.get_distance()
            stage_3_move_distance += filament_move_distance

            filament_moved = True
            # If filament moved less than a third of requested length then we consider not moving
            if filament_move_distance < self.calibrate_move_distance_per_step / 3.0:
                filament_moved = False

            gcmd.respond_info('Stage 3: Requested {}, Filament measured move: {}'.format(
                self.calibrate_move_distance_per_step, filament_move_distance
            ))

            if not filament_moved:
                extruder_to_selector_length = stage_3_move_distance
                gcmd.respond_info('Stage 3: Filament passes the ERCF gear')
                break

        ############
        # Finalize #
        ############
        gcmd.respond_info('Calibrated data:\n'
                          'Nozzle to sensor length: {}\n'
                          'Nozzle to extruder length: {}\n'
                          'Sensor to extruder length: {}\n'
                          'Extruder to selector_length: {}\n'
                          .format(nozzle_to_sensor_length,
                                  nozzle_to_extruder_length,
                                  sensor_to_extruder_length,
                                  extruder_to_selector_length))

        # Save to VARs
        self.all_variables['calibrated_nozzle_to_sensor_length'] = nozzle_to_sensor_length
        self.all_variables['calibrated_nozzle_to_extruder_length'] = nozzle_to_extruder_length
        self.all_variables['calibrated_sensor_to_extruder_length'] = sensor_to_extruder_length
        self.all_variables['calibrated_extruder_to_selector_length'] = extruder_to_selector_length

        self.save_variables()
        self.gcode.run_script_from_command('G92 E0')
        self.servo_up()


def load_config(config):
    return ERCF(config)
