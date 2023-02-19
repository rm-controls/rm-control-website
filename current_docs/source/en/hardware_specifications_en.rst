Structural Composition
----------------------------------------------
The whole platform is modular-designed with quick disassembly and each part can be separately programmed and tested.

.. image:: ../images/en/hardware_specifications_en/hardware_en_1.png

.. image:: ../images/en/hardware_specifications_en/hardware_en_2.png

The platform is mainly composed of three parts.

1. chassis module

.. image:: ../images/en/hardware_specifications_en/hardware_en_3.png

- Use Mecanum wheels for omnidirectional movement

- Powered by the RoboMaster
  `M3508 P19 Brushless DC Gear Motor <https://store.dji.com/product/rm-m3508-p19-brushless-dc-gear-motor>`_
  and RoboMaster
  `C620 ESC <https://store.dji.com/product/rm-c620-brushless-dc-motor-speed-controller>`_


- Use RoboMaster `Development Board <https://store.dji.com/product/rm-development-board-type-a?from=menu_products>`_ `Type A <https://store.dji.com/product/rm-development-board-type-a?from=menu_products>`_ (STM32F427) as MCU

2. Gimbal module

.. image:: ../images/en/hardware_specifications_en/hardware_en_4.jpeg


- Use 2-axis gimbal for two-DOF rotation movement
- Provide the mechanism for supplying and launching 17mm TPU projectiles
- Powered by RoboMaster `GM6020 Brushless Motor <https://store.dji.com/cn/product/rm-gm6020-brushless-dc-motor>`_
  (with the ESC) for gimbal movement

- Powered by RoboMaster `M2006 P36 Brushless DC Gear Motor <https://store.dji.com/cn/product/rm-m2006-p36-brushless-motor>`_


for projectile supply
- Powered by `DJI Snail 2305 Racing Motor <https://store.dji.com/product/snail-racing-propulsion-system?from=menu_products>`_


for projectile launching
- Use RoboMaster `Development Board Type A <https://store.dji.com/product/rm-development-board-type-a?from=menu_products>`_
(STM32F427) as MCU


3. Referee system module

.. image:: ../images/en/hardware_specifications_en/hardware_en_5.png


- An electronic penalty system that integrates computation, communication, and control features into different submodules and is used for robotic competitions only. Developers can acquire the information from specific software interface about the progress of the competition and the status of robots
- The Referee System includes the onboard terminal installed on the robot, as well as the server and client software installed on the PC
- Submodules installed on the robot consists of **Armor Module**，**Main Control Module**，**Speed Monitor Module**，**RFID Interaction  Module** and **Power Management Module**
- For more information about the referee system, please refer to "the Referee System Specification Manuel" under [Related Resources](en/resources).

In addition, **DT7 Remote Controller** and smart Lipo 6s battery( `Matrice 100 TB47D Battery <https://store.dji.com/product/matrice-100-tb47d-battery?from=autocomplete&position=0>`_
or `TB48D <https://store.dji.com/product/matrice-100-tb48d-battery>`_ )  with related charger are included in the accessories of the robot plateform.
The platform accommodates an extensive variety of sensors and computing devices, customized to meet research needs easy for extended development. It provides sensor Installation holder compatible with different types of sensors including industrial mono-camera,  Lidar,  UWB locating kit, depth camera and so on.  And the platform officially supported DJI Manifold 2 as the onboard computing device, but it is compatible with intel NUC, Nvidia Jetson TX1, TX2 or Xavier with certain type of carrier board.


Hardware Parameters
----------------------------------------------

| Structure                  |                       |
| :-------------------- | :-------------------- |
| Overall size                      | 600 x 450 x 460 mm    |
| Weight (Including battery)        | 16.6 kg               |
| **Performance**                  |                       |
| Maximum forward Speed            | 3 m/s                 |
| Maximum Pan Speed                | 2 m/s                 |
| Gimbal Pitch axis rotating angle | -20° ~ 20°            |
| Gimbal Yaw axis rotating angle   | -90° ~ 90°            |
| Maximum launching frequency      | 10 projectiles per second              |
| Maximum launching speed         | 25 m/s                |
| Remote Controller              | 200 projetiles                |
| **Battery**                      |                       |
| Model                           | DJI TB47D / DJI TB48D |
| Type                            | LiPo 6s               |
| Voltage                         | 22.8v                 |
| Battery capacity              | 4500 mAH / 5700 mAH   |
| **Remote control**            |                       |
| Model                         | DJI DT7               |
| Firmware upgrade              | 2.4 GHz               |
| Charing port                    | Micro USB             |
| **Communicating port**          |                       |
| Port type                    | Micro USB             |
| Communication mode              | STM32????         |
| Baud rate                        | 921600                |


