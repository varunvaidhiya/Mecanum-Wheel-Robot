# OmniBot Mecanum Wheel Robot - System Workflow

## High-Level Block Diagram

```
+-----------------------------------------------------------------------------------------------------+
|                                        ROS2 Environment                                             |
|                                                                                                     |
|  +-------------------+      +---------------------+      +----------------------+                    |
|  |                   |      |                     |      |                      |                    |
|  |  User Interface   |----->|  Navigation Stack   |----->|  Path Planning      |                    |
|  |  (Teleop/Rviz)    |      |  (Optional)         |      |  (Optional)         |                    |
|  |                   |      |                     |      |                      |                    |
|  +--------+----------+      +---------+-----------+      +-----------+----------+                    |
|           |                           |                               |                             |
|           |                           |                               |                             |
|           v                           v                               v                             |
|  +--------+-------------------------------------------------------------------+                    |
|  |                                                                            |                    |
|  |                           /cmd_vel (Twist Messages)                         |                    |
|  |                                                                            |                    |
|  +--------+-------------------------------------------------------------------+                    |
|           |                                                                                        |
|           |                                                                                        |
|           v                                                                                        |
|  +--------+--------------------------------------+      +--------------------------------+         |
|  |                                               |      |                                |         |
|  |  MecanumController Node                       |      |  Serial Bridge Node            |         |
|  |  (C++ or Python Implementation)               |      |  (Alternative Implementation)  |         |
|  |                                               |      |                                |         |
|  |  +-------------------+                        |      |  +----------------------+      |         |
|  |  | Velocity Command  |                        |      |  | Command Formatting   |      |         |
|  |  | Subscriber        |                        |      |  | <CMD_VEL,x,y,z>      |      |         |
|  |  +--------+---------+                        |      |  +-----------+----------+      |         |
|  |           |                                   |      |              |                 |         |
|  |           v                                   |      |              v                 |         |
|  |  +--------+------------------------+         |      |  +-----------+----------+      |         |
|  |  |                                 |         |      |  |                      |      |         |
|  |  | Mecanum Wheel Kinematics        |         |      |  | Serial Communication  |      |         |
|  |  | Calculation                     |         |      |  | Interface             |      |         |
|  |  |                                 |         |      |  |                      |      |         |
|  |  | FL = Vx - Vy - (L+W)*omega     |         |      |  +-----------+----------+      |         |
|  |  | FR = Vx + Vy + (L+W)*omega     |         |      |              |                 |         |
|  |  | RL = Vx + Vy - (L+W)*omega     |         |      |              |                 |         |
|  |  | RR = Vx - Vy + (L+W)*omega     |         |      |              |                 |         |
|  |  |                                 |         |      |              |                 |         |
|  |  +--------+------------------------+         |      |              |                 |         |
|  |           |                                   |      |              |                 |         |
|  |           v                                   |      |              |                 |         |
|  |  +--------+------------------------+         |      |              |                 |         |
|  |  |                                 |         |      |              |                 |         |
|  |  | Serial Communication            |<--------+------+--------------+                 |         |
|  |  | <FL,FR,RL,RR>                   |         |      |                                |         |
|  |  |                                 |         |      |                                |         |
|  |  +--------+------------------------+         |      +--------------------------------+         |
|  |           |                                   |                                                 |
|  +-----------+-----------------------------------+                                                 |
|              |                                                                                    |
+--------------|-------------------------------------------------------------------------------------+
               |
               | Serial Communication (USB/UART)
               |
+--------------|-------------------------------------------------------------------------------------+
|              v                                                                                    |
|  +-----------+-----------------------------------+                                                 |
|  |                                               |                                                 |
|  |  STM32 Microcontroller                        |                                                 |
|  |                                               |                                                 |
|  |  +-------------------+    +----------------+  |                                                 |
|  |  | Command Parser    |    | Motor Control  |  |                                                 |
|  |  | <FL,FR,RL,RR>     |--->| PWM Generation |  |                                                 |
|  |  +-------------------+    +--------+-------+  |                                                 |
|  |                                    |          |                                                 |
|  |                                    v          |                                                 |
|  |  +-------------------+    +--------+-------+  |                                                 |
|  |  | Encoder Feedback  |<---| Motor Drivers   |  |                                                 |
|  |  | Processing        |    |                |  |                                                 |
|  |  +--------+----------+    +----------------+  |                                                 |
|  |           |                                   |                                                 |
|  |           v                                   |                                                 |
|  |  +--------+------------------------+         |                                                 |
|  |  |                                 |         |                                                 |
|  |  | Status & Odometry Data          |         |                                                 |
|  |  | Formatting                      |         |                                                 |
|  |  |                                 |         |                                                 |
|  |  +--------+------------------------+         |                                                 |
|  |           |                                   |                                                 |
|  +-----------+-----------------------------------+                                                 |
|              |                                                                                    |
+--------------|-------------------------------------------------------------------------------------+
               |
               | Serial Communication (USB/UART)
               |
+--------------|-------------------------------------------------------------------------------------+
|              v                                                                                    |
|  +-----------+-----------------------------------+                                                 |
|  |                                               |                                                 |
|  |  ROS2 Environment                             |                                                 |
|  |                                               |                                                 |
|  |  +-------------------+                        |                                                 |
|  |  | Serial Data       |                        |                                                 |
|  |  | Processing        |                        |                                                 |
|  |  +--------+---------+                        |                                                 |
|  |           |                                   |                                                 |
|  |           v                                   |                                                 |
|  |  +--------+------------------------+         |                                                 |
|  |  |                                 |         |                                                 |
|  |  | Odometry Publisher              |         |                                                 |
|  |  | /odom                           |         |                                                 |
|  |  |                                 |         |                                                 |
|  |  +--------+------------------------+         |                                                 |
|  |           |                                   |                                                 |
|  |           v                                   |                                                 |
|  |  +--------+------------------------+         |                                                 |
|  |  |                                 |         |                                                 |
|  |  | TF Broadcaster                  |         |                                                 |
|  |  | odom → base_link                |         |                                                 |
|  |  |                                 |         |                                                 |
|  |  +-----------------------------------+      |                                                 |
|  |                                               |                                                 |
|  +-----------------------------------------------+                                                 |
|                                                                                                    |
+----------------------------------------------------------------------------------------------------+
```

## System Components Description

### ROS2 Environment (Raspberry Pi 5)

1. **User Interface**
   - Teleop keyboard/joystick for manual control
   - RViz for visualization

2. **Navigation Stack (Optional)**
   - For autonomous navigation capabilities
   - Path planning and obstacle avoidance

3. **MecanumController Node**
   - Subscribes to `/cmd_vel` topic (Twist messages)
   - Implements mecanum wheel kinematics
   - Converts velocity commands to individual wheel velocities
   - Communicates with STM32 via serial port
   - Publishes odometry data
   - Broadcasts TF transforms

4. **Serial Bridge Node (Alternative Implementation)**
   - Alternative implementation for serial communication
   - Formats commands for STM32 microcontroller
   - Processes feedback from STM32

### STM32 Microcontroller

1. **Command Parser**
   - Parses commands received from Raspberry Pi
   - Extracts wheel velocity values

2. **Motor Control**
   - Generates PWM signals for motor drivers
   - Controls individual wheel speeds

3. **Encoder Feedback**
   - Reads encoder values from motors
   - Calculates actual wheel speeds

4. **Status & Odometry**
   - Formats encoder data and status information
   - Sends data back to Raspberry Pi

### Communication Protocol

1. **Raspberry Pi to STM32**
   - Format: `<FL,FR,RL,RR>\n`
   - FL: Front Left wheel velocity
   - FR: Front Right wheel velocity
   - RL: Rear Left wheel velocity
   - RR: Rear Right wheel velocity

2. **STM32 to Raspberry Pi**
   - Format: `<ENCODERS,FL,FR,RL,RR>\n`
   - Encoder values for each wheel
   - Status messages: `<STATUS,...>\n`

## Data Flow

1. **Command Flow**
   - User/Navigation Stack → `/cmd_vel` → MecanumController → Wheel Velocity Calculation → Serial Communication → STM32 → Motor Control

2. **Feedback Flow**
   - Motor Encoders → STM32 → Serial Communication → Raspberry Pi → Odometry Publisher → TF Broadcaster → Navigation Stack/Visualization

## Mecanum Wheel Kinematics

The system uses the following equations to convert robot velocity commands to individual wheel velocities:

```
FL = (Vx - Vy - (L+W)*omega) / wheel_radius
FR = (Vx + Vy + (L+W)*omega) / wheel_radius
RL = (Vx + Vy - (L+W)*omega) / wheel_radius
RR = (Vx - Vy + (L+W)*omega) / wheel_radius
```

Where:
- Vx: Linear velocity in x direction (forward/backward)
- Vy: Linear velocity in y direction (left/right)
- omega: Angular velocity (rotation)
- L: Half of wheel separation length
- W: Half of wheel separation width