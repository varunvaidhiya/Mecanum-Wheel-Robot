package com.varunvaidhiya.robotcontrol.data.models

/**
 * Represents the overall status of the robot system
 */
data class RobotStatus(
    val isConnected: Boolean = false,
    val ipAddress: String = "",
    val signalStrength: Int = 0, // 0-100
    val timestamp: Long = System.currentTimeMillis(),
    val rosNodesActive: Int = 0,
    val emergencyStopActive: Boolean = false,
    val currentMode: RobotMode = RobotMode.MANUAL
)

enum class RobotMode {
    TELEOP,      // Manual joystick control
    AUTONOMOUS,  // Autonomous navigation
    MANUAL       // Direct motor control
}
