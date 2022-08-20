Alternative ERCF Driver Implementation - A Python implementation based on Klipper
====
# Goal
The goal of the alternative ERCF implementation is to overcome drawbacks from the current ERCF driver, as well as adding several cool features. 
The goal is to re-implement all feature that is provided by the ERCF driver in pure Python, with additional checks preventing load failures and jamming caused by failure to eject. 

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

# ERCF Calibration
## Calibration data storage
Calibration data are stored as variables in Klipper. The calibration data will be populated automatically by the ERCF code therefore no user interaction is required. The user is not expected to manually update the "ercf_vars.cfg". The following is just the explaination of each field and serves only the informative purpose. 
- `calibrated_encoder_resolution`: The calibrated encoder resolution used by the ERCF driver. 
- `calibrated_extruder_to_selector_length`: The distance between the extruder and toolhead sensor measured in mm. Usually the length is less than 50mm for direct drive system and much longer for bowden and dual (multiple) inline extruders setup.
- `calibrated_nozzle_to_extruder_length`: The distance between the nozzle to the extruder. This distance is only used when the filament sensor is in front of the extruder (closed to the ERCF).
- `calibrated_nozzle_to_sensor_length`: The distance between the nozzle and the toolhead sensor. This is used in the conventional ERCF setup. 
- `calibrated_sensor_to_extruder_length` The distance between the toolhead sensor to the extruder. For dual (multiple) extruder setup he extruder refers to the one that is closest to the ERCF. 
- `calibrated_tool_extrusion_factor`: Relative extrusion factor for different material in each filament block. This variable shall match the number of filament blocks in the system. 
- `color_selector_positions`: The selector position for each filament block. This variable shall match the number of filament blocks in the system. 

## Calibration Procedure


