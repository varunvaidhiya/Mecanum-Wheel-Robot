package com.varunvaidhiya.robotcontrol.network

/**
 * Callback interface for ROSBridge events
 */
interface ROSBridgeListener {
    fun onConnected()
    fun onDisconnected()
    fun onError(error: String)
    fun onMessageReceived(topic: String, message: Map<String, Any>)
}
