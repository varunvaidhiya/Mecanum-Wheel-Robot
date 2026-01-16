package com.varunvaidhiya.robotcontrol.data.models

/**
 * Motor telemetry data for all four mecanum wheels
 */
data class MotorData(
    val frontLeft: MotorStatus,
    val frontRight: MotorStatus,
    val backLeft: MotorStatus,
    val backRight: MotorStatus,
    val timestamp: Long = System.currentTimeMillis()
)

data class MotorStatus(
    val pwmValue: Int = 0,        // -255 to 255
    val rpm: Float = 0f,
    val current: Float = 0f,       // Amps
    val temperature: Float = 0f,   // Celsius
    val health: MotorHealth = MotorHealth.NORMAL
)

enum class MotorHealth {
    NORMAL,    // All good
    WARNING,   // Approaching limits
    CRITICAL   // Immediate attention needed
}
