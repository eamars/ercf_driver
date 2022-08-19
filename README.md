Alternative ERCF Driver Implementation - A Python implementation based on Klipper
====
Goal
---
The goal of the alternative ERCF implementation is to overcome drawbacks from the current ERCF driver, as well as adding several cool features. 
The goal is to re-implement all feature that is provided by the ERCF driver in pure Python, with additional checks preventing load failures and jamming caused by failure to eject. 


GCode Commands
---
- `_ERCF_CALIBRATE_GEAR_STEPPER_ROTATION_DISTANCE`: Calibrate the gear stepper rotation distance by pushing 100mm filament and the user need to measure the actual distance that extuded. 
- `_ERCF_CALIBRATE_ENCODER_RESOLUTION`: Automate calibrate the ERCF encoder on the colour selector. 
- `_ERCF_CALIBRATE_COMPONENT_LENGTH`: Run the calibration routine to determine the length of each component in the system. The calibration procedure must start with filament already inserted at the nozzle. 
- `_ERCF_CALIBRATE_SELECTOR_LOCATION`: Calibrate the location of the selector by moving the selector to the filament block then running the homing routine. 
- `_ERCF_CALIBRATE_EXTRUSION_FACTOR`: Calibrate the extrusion factor of each filament at the block to compensate for difference in extrusion for multiple materials. 
