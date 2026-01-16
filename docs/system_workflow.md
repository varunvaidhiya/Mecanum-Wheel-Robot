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
|  |  Yahboom ROS Expansion Board                  |                                                 |
|  |                                               |                                                 |
|  |  +-------------------+    +----------------+  |                                                 |
|  |  | Command Parser    |    | Motor Control  |  |                                                 |
|  |  | (USB Protocol)    |--->| PWM Generation |  |                                                 |
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
|  |  | UDP/USB Packet                  |         |                                                 |
|  |  |                                 |         |                                                 |
|  |  +--------+------------------------+         |                                                 |
|  |           |                                   |                                                 |
|  +-----------+-----------------------------------+                                                 |
|              |                                                                                    |
+--------------|-------------------------------------------------------------------------------------+
               |
               | USB/Serial Communication
               |
+--------------|-------------------------------------------------------------------------------------+
               |              v                                                                                    |
|  +-----------+-----------------------------------+                                                 |
|  |                                               |                                                 |
|  |  ROS2 Environment (Jazzy)                     |                                                 |
|  |                                               |                                                 |
|  |  +-------------------+                        |                                                 |
|  |  | Yahboom Driver    |                        |                                                 |
|  |  | Node              |                        |                                                 |
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

3. **MecanumController / Yahboom Driver Node**
   - Subscribes to `/cmd_vel` topic (Twist messages)
   - Implements mecanum wheel kinematics or delegates to board
   - Communicates with Yahboom Board via USB
   - Publishes odometry data
   - Broadcasts TF transforms

### Yahboom ROS Robot Expansion Board

1. **Command Parser**
   - Parses USB packet commands from Raspberry Pi
   - Extracts control values

2. **Motor Control**
   - Integrated motor drivers handle PWM
   - Controls individual wheel speeds

3. **Encoder Feedback**
   - Reads onboard encoder values
   - Calculates actual wheel speeds

4. **Status & Odometry**
   - Sends formatted data packets back to Raspberry Pi

### Communication Protocol

1. **Raspberry Pi to Yahboom Board**
   - USB Serial / Binary Protocol

2. **Yahboom Board to Raspberry Pi**
   - Encoder feedback and IMU data (if applicable)

## Data Flow

1. **Command Flow**
   - User/Navigation Stack → `/cmd_vel` → Driver Node → USB Serial → Yahboom Board → Motor Control

2. **Feedback Flow**
   - Motor Encoders → Yahboom Board → USB Serial → Driver Node → Odometry Publisher → TF Broadcaster → Navigation Stack/Visualization

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