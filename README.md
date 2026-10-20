# Controlling-UAV-Simulation
A simulation controlling UAV by keyboards and mission with PX4-AutoPilot, Gazebo Sim 8, ROS2 Humble 

## Table of Contents
- [INTRODUCTION](#introduction)
- [RUN SIMULATION](#run-simulation)
- [NOTE](#note)
- [CREDITS AND REFERENCES](#credits-and-references)
- [THANKS TO](#thanks-to)

## INTRODUCTION
  Thís project aims to control and simulation UAV in Gazebo Sim 8 by PX4-AutoPilot and ROS2 Humble. UAV can control by keyboards WASD and arrow keys. Moreover it can run mission by setpoint and adjust and create. Besides, it also has a position node to track the coordinate of UAV.

## RUN SIMULATION
  After download 2 programs and execute and build it, run 4 terminal simultaneously in 4 different terminals. 
  
  Terminal 1: PX4-AutoPilot 
```bash
cd PX4-AutoPilots/
make px4_sitl gz_x500
```

  Terminal 2: QGroundControl (this is in my laptop, you can change directory base on your folder position)
```bash
cd Downloads/
./QGroundControl
```

  Terminal 3: MicroXRCEAgent
```bash
MicroXRCEAgent udp4 -p 8888
```

  Terminal 4: Control Node (this is in my laptop, you can change directory base on your folder position)
```bash
cd ros2_ws/
source install/setup.bash
ros2 run my_robot_controller control_node
```

  Terminal 5: Position Node (this is in my laptop, you can change directory base on your folder position)
```bash
cd ros2_ws/
source install/setup.bash
ros2 run my_robot_controller position_node
```

## HOW TO CONTROL 
  Control node has 2 control mode 'Manual' or 'Mission'. Manual mode is the mode controling UAV by keyboards WASD and arrow keys to control the UAV. Mission mode is the mode UAV run automatically base on mission they're setting in the program. You can change the mission base on the setpoint in DEFAUT_MISSION in control_node file.
  
  With Manual mode this is the instruction to control UAV:
                T               :   TAKE OFF
                
                W / S           :   FLY FORWARD     (X)
                
                A / D           :   LEFT/RIGHT      (Y)
                
                UP/DOWN ARROW   :   INCREASE HEIGHT (Z)
                
                LEFT/RIGHT ARROW:   TURN LEFT/RIGHT (YAWN)
                
                Space           :   STOP AND HOLD POSITION
                
                M               :   FLY WITH MISSION
                
                L               :   LAND
                
                Q / Ctrl+C      :   EXIT
                
                    COORDINATE SYSTEMS NORTH-EAST-DOWN
  With Mission mode, you just press 'M' then UAV will fly to go through all setpoint.
  
## NOTE
  REMEMBER, NOT SHUTDOWN THE PROGRAM BY PRESS CTRL+C IN TERMINAL 
  
  Because the Gazebo's World will run in the background and next time you can't create world with UAV.

## CREDITS AND REFERENCES
This project is based on:

[ROS2 and PX4 Tutorial | Simulated Offboard Mode](https://www.youtube.com/watch?v=8gKIP0OqHdQ&t=19s)

[ROS2_PX4_Offboard_Example](https://github.com/ARK-Electronics/ROS2_PX4_Offboard_Example.git)

## THANKS TO
[Embedded Networking Labrotatory](https://www.facebook.com/lab411)

Tran Luong Duy 202113751 HUST
