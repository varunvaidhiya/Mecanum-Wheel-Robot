package com.varunvaidhiya.robotcontrol.utils

/**
 * Application-wide constants
 */
object Constants {
    // Network
    const val DEFAULT_ROBOT_IP = "192.168.1.100"
    const val DEFAULT_ROSBRIDGE_PORT = 9090
    const val CONNECTION_TIMEOUT_MS = 10000L
    
    // Control
    const val JOYSTICK_DEADZONE = 0.1f
    const val MAX_LINEAR_VELOCITY = 1.5f  // m/s
    const val MAX_ANGULAR_VELOCITY = 2.0f // rad/s
    const val CMD_VEL_PUBLISH_RATE_HZ = 20
    
    // ROS Topics
    const val TOPIC_CMD_VEL = "/cmd_vel"
    const val TOPIC_ROBOT_STATUS = "/robot_status"
    const val TOPIC_WHEEL_SPEEDS = "/wheel_speeds"
    const val TOPIC_MOTOR_PWM = "/motor_pwm"
    const val TOPIC_DIAGNOSTICS = "/diagnostics"
    const val TOPIC_EMERGENCY_STOP = "/emergency_stop"
    const val TOPIC_ROBOT_MODE = "/robot_mode"
    
    // UI
    const val CHART_MAX_DATA_POINTS = 300
    const val DASHBOARD_UPDATE_RATE_HZ = 10
    
    // Logging
    const val LOG_FILE_MAX_SIZE_MB = 50
    const val LOG_RETENTION_DAYS = 7
}
