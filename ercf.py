from . import pulse_counter
from . import force_move
import toolhead
import configparser
import logging
import ast
import time


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
        self.gcode.register_command('ERCF_CALIBRATE_COMPONENT_LENGTH',
                                    self.cmd_ERCF_CALIBRATE_COMPONENT_LENGTH,
                                    desc='Execute the calibration routine on the current tool')
        self.gcode.register_command('ERCF_SERVO_UP',
                                    self.cmd_ERCF_SERVO_UP,
                                    desc='Lift the servo arm to release the gear')
        self.gcode.register_command('ERCF_SERVO_DOWN',
                                    self.cmd_ERCF_SERVO_DOWN,
                                    desc='Press the servo arm to engage the gear')

        # Register event
        self.printer.register_event_handler('klippy:connect', self.handle_connect)

    def handle_connect(self):
        self.toolhead = self.printer.lookup_object('toolhead')
        self.gear_stepper = self.printer.lookup_object(self.gear_stepper_name)
        self.selector_stepper = self.printer.lookup_object(self.selector_stepper_name)
        self.servo = self.printer.lookup_object(self.servo_name)
        if self.toolhead_sensor is not None:
            self.toolhead_sensor = self.printer.lookup_object(self.toolhead_sensor_name)

        # Initialize status
        self._servo_status = None

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

    def cmd_ERCF_SERVO_UP(self, gcmd):
        self.servo_up()

    def cmd_ERCF_SERVO_DOWN(self, gcmd):
        self.servo_down()

    def servo_up(self):
        if self._servo_status != 'up':
            self._servo_status = 'up'
            servo_name = self.servo_name.split()[1]
            self.gcode.run_script_from_command('SET_SERVO SERVO={} ANGLE={}'.format(servo_name,
                                                                                    self.servo_up_angle))
            time.sleep(0.25 + self.extra_servo_dwell_up)
            self.gcode.run_script_from_command('SET_SERVO SERVO={} WIDTH=0.0'.format(servo_name))

    def servo_down(self):
        if self._servo_status != 'down':
            self._servo_status = 'down'
            servo_name = self.servo_name.split()[1]

            # do the gear meshing to ensure the proper alignment of the selector gear
            self.gear_stepper.do_set_position(0)
            self.gear_stepper.do_move(0.5, speed=25, accel=self.gear_stepper_accel)

            self.gcode.run_script_from_command('SET_SERVO SERVO={} ANGLE={}'.format(servo_name,
                                                                                    self.servo_down_angle))
            time.sleep(0.2)

            self.gear_stepper.do_move(0.0, speed=25, accel=self.gear_stepper_accel)
            time.sleep(0.1)
            self.gear_stepper.do_move(-0.5, speed=25, accel=self.gear_stepper_accel)
            time.sleep(0.1 + self.extra_servo_dwell_down)
            self.gear_stepper.do_move(0.0, speed=25, accel=self.gear_stepper_accel)
            self.gcode.run_script_from_command('SET_SERVO SERVO={} WIDTH=0.0'.format(servo_name))

    def gear_stepper_move_wait(self, dist, wait=True, speed=None, accel=None):
        self.gear_stepper.do_set_position(0.)
        is_long_move = abs(dist) > self.gear_stepper_long_move_threshold
        if speed is None:
            speed = self.long_moves_speed if is_long_move \
                    else self.short_moves_speed
        if accel is None:
            accel = self.long_moves_accel if is_long_move \
                    else self.short_moves_accel
        self.gear_stepper.do_move(dist, speed, accel, True)
        if wait :
            self.toolhead.wait_moves()

    def cmd_ERCF_CALIBRATE_COMPONENT_LENGTH(self, gcmd):
        try:
            self.calibrate_component_length(gcmd)
        except RuntimeError as e:
            self.gcode.respond_info('Failed to calibrate the system length: {}'.format(e))

    def calibrate_component_length(self, gcmd):
        gcmd.respond_info('Going to calibrate the length of each component by unloading the '
                          'filament from nozzle to the ERCF selector')

        self.servo_up()

        # Sanity check: the toolhead sensor should trigger
        if self.toolhead_sensor and not bool(self.toolhead_sensor.runout_helper.filament_present):
            raise RuntimeError('Filament is not loaded to the toolhead, or the filament sensor is not triggering')
        elif self.toolhead_sensor is None:
            gcmd.respond_info('Going to run the toolhead sensorless calibration')

        # Form the tip to begin with
        self.gcode.run_script_from_command('_ERCF_FORM_TIP_STANDALONE')

        # Initialize constant variables
        calibrate_move_distance_per_step = 1  # 1mm

        # Variables to dump to Vars
        nozzle_to_sensor_length = None
        nozzle_to_extruder_length = None
        sensor_to_extruder_length = None
        extruder_to_selector_length = None

        ###########
        # STAGE 1 #
        ###########
        # At stage 1 the toolhead shall retract, until
        #   - The toolhead filament sensor is not triggered or
        #   - The filament is not moving (filament sensor is installed between the ERCF and the extruder)
        stage_1_move_distance = 0
        self.gcode.run_script_from_command('G92 E0')
        toolhead_position = self.toolhead.get_position()

        while True:
            # Move
            toolhead_position[3] -= calibrate_move_distance_per_step

            # We rely on the theoretical move distance as this is actuated by the extruder
            stage_1_move_distance += calibrate_move_distance_per_step
            self.motion_counter.reset_counts()

            # Retract the move distance
            self.toolhead.move(toolhead_position, self.short_moves_speed)
            self.toolhead.wait_moves()

            # Check the filament sensor status
            filament_move_distance = self.motion_counter.get_distance()
            filament_move_diff = abs(filament_move_distance - calibrate_move_distance_per_step)
            filament_moved = True
            # If filament moved less than half of requested length then we consider not moving
            if filament_move_diff < calibrate_move_distance_per_step / 2.0:
                filament_moved = False

            logging.debug('Stage 1 Requested {},Filament measured move: {}'.format(
                calibrate_move_distance_per_step, filament_move_distance
            ))

            if self.toolhead_sensor and not bool(self.toolhead_sensor.runout_helper.filament_present):
                nozzle_to_sensor_length = stage_1_move_distance
                logging.debug('Filament is extracted passing the toolhead sensor')
                break
            elif not filament_moved:
                nozzle_to_extruder_length = stage_1_move_distance
                break

        ############
        # STAGE 2a #
        ############
        # At stage 2a the toolhead shall retract if nozzle_to_sensor_length is detected, until
        #   - The filament is not moving
        if nozzle_to_sensor_length is not None:
            stage_2_move_distance = 0
            self.gcode.run_script_from_command('G92 E0')
            toolhead_position = self.toolhead.get_position()
            while True:
                toolhead_position[3] -= calibrate_move_distance_per_step

                stage_2_move_distance += calibrate_move_distance_per_step
                self.motion_counter.reset_counts()

                # Retract the move distance
                self.toolhead.move(toolhead_position, self.short_moves_speed)
                self.toolhead.wait_moves()

                # Check the filament movement status
                filament_move_distance = self.motion_counter.get_distance()
                filament_move_diff = abs(filament_move_distance - calibrate_move_distance_per_step)
                filament_moved = True
                # If filament moved less than half of requested length then we consider not moving
                if filament_move_diff < calibrate_move_distance_per_step / 2.0:
                    filament_moved = False

                logging.debug('Stage 2a Requested {}, Filament measured move: {}'.format(
                    calibrate_move_distance_per_step, filament_move_distance
                ))

                if not filament_moved:
                    sensor_to_extruder_length = stage_2_move_distance
                    break

        ############
        # STAGE 2b #
        ############
        # At stage 2b the gear stepper shall retract if nozzle_to_extruder_length is detected for the system without
        # the toolhead sensor, until
        #   - The toolhead filament sensor is not triggered or
        #   - The filament is not moving (error condition)
        if self.toolhead_sensor and nozzle_to_extruder_length is not None:
            self.servo_down()
            stage_2_move_distance = 0
            while True:
                # Retract the move distance
                stage_2_move_distance += calibrate_move_distance_per_step
                self.motion_counter.reset_counts()

                self.gear_stepper_move_wait(-calibrate_move_distance_per_step)

                # Check the filament status
                filament_present = bool(self.toolhead_sensor.runout_helper.filament_present)
                filament_move_distance = self.motion_counter.get_distance()
                filament_move_diff = abs(filament_move_distance - calibrate_move_distance_per_step)
                filament_moved = True
                # If filament moved less than half of requested length then we consider not moving
                if filament_move_diff < calibrate_move_distance_per_step / 2.0:
                    filament_moved = False

                logging.debug('Stage 2a Requested {}, Filament present: {}, Filament measured move: {}'.format(
                    calibrate_move_distance_per_step, filament_present, filament_move_distance
                ))

                if not filament_present:
                    # Yes we are using a negative value to represent the extruder to sensor length
                    # if the sensor is installed between the extruder and the gear stepper
                    sensor_to_extruder_length = -stage_2_move_distance
                    break

                if not filament_moved:
                    raise RuntimeError("Filament ")

        ###########
        # STAGE 3 #
        ###########
        # At stage 3 the gear stepper shall retract until the filament is retract below the selector
        stage_3_move_distance = 0
        self.servo_down()
        while True:
            stage_3_move_distance += calibrate_move_distance_per_step
            self.motion_counter.reset_counts()

            self.gear_stepper_move_wait(-calibrate_move_distance_per_step)

            # Check filament status
            filament_move_distance = self.motion_counter.get_distance()
            filament_move_diff = abs(filament_move_distance - calibrate_move_distance_per_step)
            filament_moved = True
            # If filament moved less than half of requested length then we consider not moving
            if filament_move_diff < calibrate_move_distance_per_step / 2.0:
                filament_moved = False

            logging.debug('Stage 3 Requested {}, Filament measured move: {}'.format(
                calibrate_move_distance_per_step, filament_move_distance
            ))

            if not filament_moved:
                extruder_to_selector_length = stage_3_move_distance
                break

        ############
        # Finalize #
        ############
        gcmd.respond_info('Calibrated data:\n'
                          'Hotend to sensor length: {}\n'
                          'Hotend to extruder length: {}\n'
                          'Sensor to extruder length: {}\n'
                          'Extruder to selector_length: {}\n'
                          .format(nozzle_to_sensor_length,
                                  nozzle_to_extruder_length,
                                  sensor_to_extruder_length,
                                  extruder_to_selector_length))

        # Save to VARs
        self.all_variables['nozzle_to_sensor_length'] = nozzle_to_sensor_length
        self.all_variables['nozzle_to_extruder_length'] = nozzle_to_extruder_length
        self.all_variables['sensor_to_extruder_length'] = sensor_to_extruder_length
        self.all_variables['extruder_to_selector_length'] = extruder_to_selector_length

        self.save_variables()


def load_config(config):
    return ERCF(config)
