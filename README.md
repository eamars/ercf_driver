Alternative ERCF Driver Implementation - A Python implementation based on Klipper
====
[中文版](README_zh_cn.md)
# Goal
The goal of the alternative ERCF implementation is to overcome drawbacks from the current ERCF driver, as well as adding several cool features. 
The goal is to re-implement all feature that is provided by the ERCF driver in pure Python, with additional checks preventing load failures and jamming caused by failure to eject. 

# Install via Moonraker
Clone the repository to the home directory

    cd ~
    git clone git@github.com:eamars/ercf_driver.git
  
Then copy the below block into the moonraker.conf

    [update_manager client ercf_driver]
    type: git_repo
    path: ~/ercf_driver
    origin: git@github.com:eamars/ercf_driver.git
    install_script: install.sh
    managed_services: klipper

# GCode Macros
## Tool Changes
- `T1-T9`: Move the selector to the corresponding filament block then load the filament to the nozzle. 
## Action
- `ERCF_SERVO_UP`: Lift up the servo arm. 
- `ERCF_SERVO_DOWN`: Press down the servo arm. 
- `ERCF_LOAD`: Load the filament to the nozzle from the current filament block. 
- `ERCF_UNLOAD`: Unload the filament to the filament block and clear the selector moving path. 
- `ERCF_HOME_SELECTOR`: Home the selector cart and move to filament block 0. 
- `ERCF_CHANGE_TOOL`: Idential to T{tool} which moves the selector the corresponding filament block then load the filament to the nozzle. 

## Calibration
- `_ERCF_CALIBRATE_GEAR_STEPPER_ROTATION_DISTANCE`: Calibrate the gear stepper rotation distance by pushing 100mm filament and the user need to measure the actual distance that extuded. 
- `_ERCF_CALIBRATE_ENCODER_RESOLUTION`: Automate calibrate the ERCF encoder on the colour selector. 
- `_ERCF_CALIBRATE_COMPONENT_LENGTH`: Run the calibration routine to determine the length of each component in the system. The calibration procedure must start with filament already inserted at the nozzle. 
- `_ERCF_CALIBRATE_SELECTOR_LOCATION`: Calibrate the location of the selector by moving the selector to the filament block then running the homing routine. 
- `_ERCF_CALIBRATE_EXTRUSION_FACTOR`: Calibrate the extrusion factor of each filament at the block to compensate for difference in extrusion for multiple materials.

# Configurations
An example configuration file is supplied in "ercf_config.cfg". Please use the example file as the reference and modify as needed. Most variables are self explained. Otherwise referring the variable back to the context would help understanding the usage of the ERCF driver better. 

Example configuration

```ini
[ercf]
# Compulsory params
gear_stepper: manual_stepper gear_stepper                  # full name is required
selector_stepper: manual_stepper selector_stepper          # full name is required
servo: servo ercf_servo                                    # full name is required
toolhead_sensor: filament_switch_sensor toolhead_sensor    # full name is required
encoder_pin: ebb_ercf_02:PB3                               # The pin must be shared via [duplicate_pin_override]

servo_up_angle: 30                                         # The angle when the servo arm is in UP position. See the ERCF manual for more information.
servo_down_angle: 115                                      # The angle when the servo arm is in DOWN position. See the ERCF manual for more information.

variable_path: ...                                         # The path to the ERCF variable file. Usually [/home/pi/klipper_config/ercf_vars.cfg]

# Optional params. The default value is specified
extra_servo_dwell_up: 0                                    # The extra time in second that the servo need to dwell in up position (?)
extra_servo_dwell_down: 0                                  # The extra time in second that the servo need to dwell in down position (?)

encoder_sample_time: 0.1                                   # The ERCF encoder sampling time in second. 
encoder_poll_time: 0.0001                                  # The ERCF encoder poll time in seconds.
long_moves_speed: 100                                      # Long pulse move speed in mm/s. The speed is also been used by the single long move. 
long_moves_accel: 400                                      # Long pulse move acceleration in mm/s^2. Note the acceleration only apply to gear stepper motion.
short_moves_speed: 25                                      # Short pulse move speed in mm/s. 
short_moves_accel: 400                                     # Short pulse move acceleration in mm/s^2. Note the acceleration only apply to gear stepper motion.
gear_stepper_long_move_threshold: 70                       # (Deprecated) The threshold that determines the move speed/acceleration between short and long move. 

extra_move_margin: 100                                     # The extra margin that appends to the move where pending on the trigger condition (filament slip or toggle of the filament sensor. 
long_move_distance: 30                                     # The long pulse move distance in mm. 
short_move_distance: 10                                    # The short pulse move distance in mm.
minimum_step_distance: 5                                   # The minimum step distance in mm that can be detected by the ERCF encoder.
maximum_move_distance: 1500                                # The maximum continuous move distance (can include multiple steps) in mm.
maximum_step_distance: 1500                                # The maximum single step move distance in mm.
calibrate_move_distance_per_step: 3                        # The step distance in mm used during calibration. 

selector_filament_engagement_retry: 2                      # The maximum retry while the filament failes to engage. 
auto_home_selector: True                                   # Automatically home the selector if not homed previously when a selector move is requested.
tip_forming_gcode_before_calibration: None                 # The tip forming gcode to run before running the calibration routine (_ERCF_CALIBRATE_COMPONENT_LENGTH).
slip_detection_ratio_threshold: 3                          # If the actual move distance is less than [1/threshold * requested_distance] then the code will consider it slip
```


# ERCF Calibration
## Calibration Data Storage
Calibration data are stored as variables in Klipper. The calibration data will be populated automatically by the ERCF code therefore no user interaction is required. The user is not expected to manually update the "ercf_vars.cfg". The following is just the explaination of each field and serves only the informative purpose. 
- `calibrated_encoder_resolution`: The calibrated encoder resolution used by the ERCF driver. 
- `calibrated_extruder_to_selector_length`: The distance between the extruder and toolhead sensor measured in mm. Usually the length is less than 50mm for direct drive system and much longer for bowden and dual (multiple) inline extruders setup.
- `calibrated_nozzle_to_extruder_length`: The distance between the nozzle to the extruder. This distance is only used when the filament sensor is in front of the extruder (closed to the ERCF).
- `calibrated_nozzle_to_sensor_length`: The distance between the nozzle and the toolhead sensor. This is used in the conventional ERCF setup. 
- `calibrated_sensor_to_extruder_length` The distance between the toolhead sensor to the extruder. For dual (multiple) extruder setup he extruder refers to the one that is closest to the ERCF. 
- `calibrated_tool_extrusion_factor`: Relative extrusion factor for different material in each filament block. This variable shall match the number of filament blocks in the system. 
- `color_selector_positions`: The selector position for each filament block. This variable shall match the number of filament blocks in the system. 

## Calibration Procedure
My recommendation to the calibration procedure is to follow the current ERCF instruction for the `rotation_distance`, encoder resolution, extrusion factor, then run the `_ERCF_CALIBRATE_COMPONENT_LENGTH` when the filament is fully loaded to the nozzle. The calibration routine will then retract the filament from the nozzle all the way back to the filament block in short slow pulse. During that process the length of each component shall be recorded. 

