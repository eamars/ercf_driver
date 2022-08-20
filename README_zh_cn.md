ERCF Python版驱动
===

# 目标
重写ERCF的目的是修复当前ERCF中的一些设计缺陷，bug，增加额外的料材状态检测，以及增加一部分很Cooool的功能。新版ERCF将主要由Python重写，配合少许Gcode宏实现，以达成增加换色稳定性的目的。

# Gcode宏
## 更换工具类
- `T1-T9`: 将选择器移动到对应的耗材仓并将耗材推送到喷嘴。

## 综合类ERCF宏
- `ERCF_SERVO_UP`: 抬起舵机（释放耗材）
- `ERCF_SERVO_DOWN`: 压下舵机（咬紧耗材）
- `ERCF_LOAD`: 将耗材从当前耗材仓推送到喷嘴处
- `ERCF_UNLOAD`: 将耗材从喷嘴抽回耗材仓。抽回后选择器即可左右移动
- `ERCF_HOME_SELECTOR`: 选择器归零并移动到0号耗材仓
- `ERCF_CHANGE_TOOL`: 与更换工具相同，将选择器移动到对应的耗材仓并将耗材推送到喷嘴。

## ERCF校准宏
- `_ERCF_CALIBRATE_GEAR_STEPPER_ROTATION_DISTANCE`: 送出100mm耗材，既由用户测量实际挤出长度来校准ERCF挤出轮旋转距离 (rotation_distance)
- `_ERCF_CALIBRATE_ENCODER_RESOLUTION`: 自动校准选择器中的编码器
- `_ERCF_CALIBRATE_COMPONENT_LENGTH`: 自动长度校准。需要用户在校准前将材料手动推送到喷嘴处
- `_ERCF_CALIBRATE_SELECTOR_LOCATION`: 通过将选择器移动到对应耗材仓来校准耗材仓绝对坐标
- `_ERCF_CALIBRATE_EXTRUSION_FACTOR`: 自动校准不同耗材对应的耗材仓的挤出长度系数

# 示例配置
"ercf_config.cfg" 已经包含了最低限度的需要配置的设置项，用户可以用其作为参考。大部分设置项通过文件名即可猜测出其用处。具体配置可以参考以下内容。


```ini
[ercf]
# 用户定义内容
gear_stepper: manual_stepper gear_stepper                  # ERCF挤出齿轮电机定义
selector_stepper: manual_stepper selector_stepper          # 选择器电机定义
servo: servo ercf_servo                                    # 舵机定义
toolhead_sensor: filament_switch_sensor toolhead_sensor    # 工具头上的断料检测定义
encoder_pin: ebb_ercf_02:PB3                               # 编码器输入针脚

servo_up_angle: 30                                         # 舵机抬起时角度。校准请参考官方ERCF手册
servo_down_angle: 115                                      # 舵机下压时角度。校准请参考官方ERCF手册

variable_path: ...                                         # 自动校准状态文件，通常为 [/home/pi/klipper_config/ercf_vars.cfg]

# 用户可选配置内容
extra_servo_dwell_up: 0                                    # 
extra_servo_dwell_down: 0                                  # 

encoder_sample_time: 0.1                                   # ERCF样本时间
encoder_poll_time: 0.0001                                  # 编码器轮询时间，单位为秒
long_moves_speed: 100                                      # 长脉冲移动速度，单位为mm/s
long_moves_accel: 400                                      # 长脉冲移动加速度，单位为mm/s^2。需要注意的是该设置只能配置ERCF挤出机的加速度
short_moves_speed: 25                                      # 短脉冲移动速度，单位为mm/s
short_moves_accel: 400                                     # 短脉冲移动加速度，单位为mm/s^2。需要注意的是该设置只能配置ERCF挤出机的加速度
gear_stepper_long_move_threshold: 70                       # (已过时) 区分长短脉冲的临界值

extra_move_margin: 100                                     # 等待触发时额外的耗材挤出距离。触发条件为耗材打滑或是耗材触发了工具头上的断料检测
long_move_distance: 30                                     # 长脉冲移动距离，单位为mm
short_move_distance: 10                                    # 短脉冲移动距离，单位为mm
minimum_step_distance: 5                                   # ERCF编码器最低可检测耗材移动距离，单位为mm
maximum_move_distance: 1500                                # 最大单次移动距离（可包含多个脉冲移动），单位为mm
maximum_step_distance: 1500                                # 最大单步移动距离，单位为mm
calibrate_move_distance_per_step: 3                        # 校准时耗材单步移动距离，单位为mm

selector_filament_engagement_retry: 2                      # 最大尝试次数（用于ERCF挤出机无法咬住耗材时重试）
auto_home_selector: True                                   # 若选择器未归零则任何选择器相关的移动将触发归零动作
tip_forming_gcode_before_calibration: None                 # 长度校准之前运行的耗材头抽插宏
slip_detection_ratio_threshold: 3                          # 耗材打滑比例 （若每步耗材实际移动距离小于[1/threshold * requested_distance]则认为是打滑）
```

# ERCF校准
## 校准数据
校准产生的数据将自动保存在 [variable_path] 指定的文件内。以下为各个项目的定义
- `calibrated_encoder_resolution`: 编码器分辨率（用于试剂耗材运动距离运算）
- `calibrated_extruder_to_selector_length`: 挤出机到选择器距离。对于双（或多）挤出机配置中该挤出机为距离ERCF最近的一个。
- `calibrated_nozzle_to_extruder_length`: 目前并未使用）喷嘴到挤出机距离。该项目适用于断料检测器安装在挤出机上方的配置（比如Orbiter2的断料检测）
- `calibrated_nozzle_to_sensor_length`: 喷嘴到断料检测器的距离
- `calibrated_sensor_to_extruder_length`: 断料检测到挤出机的距离。对于双（或多）挤出机配置中该挤出机为距离ERCF最近的一个。
- `calibrated_tool_extrusion_factor`: 耗材挤出系数
- `color_selector_positions`: 耗材仓绝对坐标

## 校准步骤
建议参考ERCF手册中的校准步骤。

