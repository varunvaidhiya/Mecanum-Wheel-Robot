package com.varunvaidhiya.robotcontrol.data.models

/**
 * Velocity command for mecanum wheel robot
 */
data class VelocityCommand(
    val linearX: Float = 0f,   // Forward/backward velocity (m/s)
    val linearY: Float = 0f,   // Left/right strafe velocity (m/s)
    val angular: Float = 0f,   // Rotation velocity (rad/s)
    val timestamp: Long = System.currentTimeMillis()
) {
    /**
     * Convert to ROS Twist message format
     */
    fun toROSTwist(): Map<String, Any> {
        return mapOf(
            "linear" to mapOf(
                "x" to linearX.toDouble(),
                "y" to linearY.toDouble(),
                "z" to 0.0
            ),
            "angular" to mapOf(
                "x" to 0.0,
                "y" to 0.0,
                "z" to angular.toDouble()
            )
        )
    }
}
