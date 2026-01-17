package com.varunvaidhiya.robotcontrol.data.repository

import com.varunvaidhiya.robotcontrol.data.models.*
import com.varunvaidhiya.robotcontrol.network.ROSBridgeListener
import com.varunvaidhiya.robotcontrol.network.ROSBridgeManager
import com.varunvaidhiya.robotcontrol.utils.Constants
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import timber.log.Timber
import javax.inject.Inject
import javax.inject.Singleton

/**
 * Single source of truth for robot data.
 * Manages ROSBridge connection and exposes data as StateFlows.
 */
@Singleton
class RobotRepository @Inject constructor() {

    // Network Manager
    private var rosManager: ROSBridgeManager? = null

    // StateFlows for UI to observe
    private val _connectionState = MutableStateFlow(ROSBridgeManager.ConnectionState.DISCONNECTED)
    val connectionState: StateFlow<ROSBridgeManager.ConnectionState> = _connectionState.asStateFlow()

    private val _robotStatus = MutableStateFlow(RobotStatus())
    val robotStatus: StateFlow<RobotStatus> = _robotStatus.asStateFlow()

    private val _motorData = MutableStateFlow(
        MotorData(MotorStatus(), MotorStatus(), MotorStatus(), MotorStatus())
    )
    val motorData: StateFlow<MotorData> = _motorData.asStateFlow()

    private val _wheelSpeeds = MutableStateFlow(WheelSpeed())
    val wheelSpeeds: StateFlow<WheelSpeed> = _wheelSpeeds.asStateFlow()

    // Listener for ROS events
    private val rosListener = object : ROSBridgeListener {
        override fun onConnected() {
            _connectionState.value = ROSBridgeManager.ConnectionState.CONNECTED
            subscribeToTopics()
        }

        override fun onDisconnected() {
            _connectionState.value = ROSBridgeManager.ConnectionState.DISCONNECTED
            _robotStatus.value = _robotStatus.value.copy(isConnected = false)
        }

        override fun onError(error: String) {
            _connectionState.value = ROSBridgeManager.ConnectionState.ERROR
            Timber.e("ROS Error: $error")
        }

        override fun onMessageReceived(topic: String, message: Map<String, Any>) {
            handleMessage(topic, message)
        }
    }

    /**
     * Initialize connection
     */
    fun connect(ip: String = Constants.DEFAULT_ROBOT_IP, port: Int = Constants.DEFAULT_ROSBRIDGE_PORT) {
        val url = "ws://$ip:$port"
        rosManager = ROSBridgeManager(url, rosListener)
        rosManager?.connect()
        _connectionState.value = ROSBridgeManager.ConnectionState.CONNECTING
    }

    fun disconnect() {
        rosManager?.disconnect()
    }

    /**
     * Send velocity command
     */
    fun sendVelocity(command: VelocityCommand) {
        rosManager?.publish(Constants.TOPIC_CMD_VEL, command.toROSTwist())
    }

    /**
     * Send emergency stop
     */
    fun sendEmergencyStop() {
        rosManager?.publish(Constants.TOPIC_EMERGENCY_STOP, mapOf("data" to true))
    }

    private fun subscribeToTopics() {
        rosManager?.apply {
            subscribe(Constants.TOPIC_ROBOT_STATUS, "robot_msgs/RobotStatus")
            subscribe(Constants.TOPIC_WHEEL_SPEEDS, "robot_msgs/WheelSpeed")
            subscribe(Constants.TOPIC_MOTOR_PWM, "robot_msgs/MotorData")
            // Add other subscriptions here
        }
    }

    private fun handleMessage(topic: String, message: Map<String, Any>) {
        try {
            when (topic) {
                Constants.TOPIC_ROBOT_STATUS -> parseRobotStatus(message)
                Constants.TOPIC_WHEEL_SPEEDS -> parseWheelSpeeds(message)
                Constants.TOPIC_MOTOR_PWM -> parseMotorData(message)
            }
        } catch (e: Exception) {
            Timber.e(e, "Error parsing message for topic: $topic")
        }
    }

    // Parsing logic (Simplified for Phase 2, expand in Phase 3/4)
    private fun parseRobotStatus(msg: Map<String, Any>) {
        // TODO: Implement full parsing based on JSON structure
        val isConnected = msg["connected"] as? Boolean ?: true
        _robotStatus.value = _robotStatus.value.copy(
            isConnected = isConnected,
            timestamp = System.currentTimeMillis()
        )
    }

    private fun parseWheelSpeeds(msg: Map<String, Any>) {
        val fl = (msg["front_left"] as? Number)?.toFloat() ?: 0f
        val fr = (msg["front_right"] as? Number)?.toFloat() ?: 0f
        val bl = (msg["back_left"] as? Number)?.toFloat() ?: 0f
        val br = (msg["back_right"] as? Number)?.toFloat() ?: 0f
        
        _wheelSpeeds.value = WheelSpeed(frontLeft = fl, frontRight = fr, backLeft = bl, backRight = br)
    }

    private fun parseMotorData(msg: Map<String, Any>) {
        // TODO: Implement full parsing
    }
}
