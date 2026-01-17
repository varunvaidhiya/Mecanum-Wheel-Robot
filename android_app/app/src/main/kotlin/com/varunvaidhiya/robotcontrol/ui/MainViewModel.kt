package com.varunvaidhiya.robotcontrol.ui

import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.varunvaidhiya.robotcontrol.data.repository.RobotRepository
import com.varunvaidhiya.robotcontrol.network.ROSBridgeManager.ConnectionState
import dagger.hilt.android.lifecycle.HiltViewModel
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.launch
import javax.inject.Inject

/**
 * Global app ViewModel, tied to MainActivity lifecycle.
 * Handles top-level connection state.
 */
@HiltViewModel
class MainViewModel @Inject constructor(private val repository: RobotRepository) : ViewModel() {

    val connectionState: StateFlow<ConnectionState> = repository.connectionState
    val robotStatus = repository.robotStatus

    fun connectToRobot(ip: String) {
        viewModelScope.launch {
            repository.connect(ip)
        }
    }

    fun disconnect() {
        repository.disconnect()
    }
}
