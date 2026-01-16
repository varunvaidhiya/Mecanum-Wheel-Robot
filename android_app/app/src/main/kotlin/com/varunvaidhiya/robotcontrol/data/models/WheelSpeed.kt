package com.varunvaidhiya.robotcontrol.data.models

/**
 * Individual wheel speeds for mecanum drive
 */
data class WheelSpeed(
    val frontLeft: Float = 0f,   // m/s
    val frontRight: Float = 0f,
    val backLeft: Float = 0f,
    val backRight: Float = 0f,
    val timestamp: Long = System.currentTimeMillis()
)
