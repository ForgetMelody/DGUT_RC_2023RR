# DGUT_RC_2023RR
东莞理工 行者队 23年Robocon RR代码开源
![RR架构](https://github.com/ForgetMelody/DGUT_RC_2023RR/assets/28036853/11bd049d-a10b-4bbd-82f6-98f3d087a102)
以Python为主 仅包含小电脑ROS端开发代码，仅包含以下内容

| 包名        | 功能实现                        |
| ----------- | ------------------------------- |
| movement    | 底盘运动解算实现                |
| navigation  | move_base导航等相关配置         |
| selfaim     | 基于livox雷达点云实现的偏航自瞄 |
| serial_pkgs | 与stm32对接实现的通信接口       |

除此之外，需要额外配置 
 - Faster-LIO
 - Livox_ROS_Driver2
 - Joystick
等ros包
python需安装pyserial
