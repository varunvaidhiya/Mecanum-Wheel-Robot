package com.varunvaidhiya.robotcontrol.utils

import android.content.Context
import android.os.Environment
import com.varunvaidhiya.robotcontrol.data.models.MotorData
import com.varunvaidhiya.robotcontrol.data.models.WheelSpeed
import timber.log.Timber
import java.io.File
import java.io.FileWriter
import java.io.IOException
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale

class DataLogger(private val context: Context) {

    private var currentLogFile: File? = null
    private var writer: FileWriter? = null
    private val dateFormat = SimpleDateFormat("yyyy-MM-dd HH:mm:ss.SSS", Locale.US)
    private val fileDateFormat = SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US)

    // Check if logging is enabled (can be controlled by settings)
    var isLoggingEnabled = false

    fun startNewSession() {
        if (!isLoggingEnabled) return
        
        try {
            val fileName = "robot_log_${fileDateFormat.format(Date())}.csv"
            val dir = context.getExternalFilesDir(Environment.DIRECTORY_DOCUMENTS)
            currentLogFile = File(dir, fileName)
            
            writer = FileWriter(currentLogFile, true)
            // Header
            writer?.append("Timestamp,Type,FL,FR,RL,RR\n")
            writer?.flush()
            Timber.i("Started logging to: ${currentLogFile?.absolutePath}")
            
        } catch (e: IOException) {
            Timber.e(e, "Failed to start logging")
        }
    }

    fun stopSession() {
        try {
            writer?.close()
            writer = null
            Timber.i("Stopped logging")
        } catch (e: IOException) {
            Timber.e(e, "Failed to stop logging")
        }
    }

    fun logWheelSpeeds(speeds: WheelSpeed) {
        if (!isLoggingEnabled || writer == null) return
        
        try {
            val timestamp = dateFormat.format(Date())
            val line = "$timestamp,SPEED,${speeds.frontLeft},${speeds.frontRight},${speeds.backLeft},${speeds.backRight}\n"
            writer?.append(line)
            writer?.flush()
        } catch (e: Exception) {
            Timber.e(e, "Error logging speeds")
        }
    }

    // TODO: Add log methods for MotorData and RobotStatus
    
    fun getLogFiles(): List<File> {
        val dir = context.getExternalFilesDir(Environment.DIRECTORY_DOCUMENTS)
        return dir?.listFiles()?.sortedDescending()?.toList() ?: emptyList()
    }
}
