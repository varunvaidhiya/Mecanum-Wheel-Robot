package com.varunvaidhiya.robotcontrol.ui.controls

import androidx.lifecycle.ViewModel
import com.varunvaidhiya.robotcontrol.data.models.VelocityCommand
import com.varunvaidhiya.robotcontrol.data.repository.RobotRepository
import dagger.hilt.android.lifecycle.HiltViewModel
import javax.inject.Inject

@HiltViewModel
class ControlsViewModel @Inject constructor(
    private val repository: RobotRepository
) : ViewModel() {

    fun sendVelocity(linearX: Float, linearY: Float, angular: Float) {
        val cmd = VelocityCommand(linearX, linearY, angular)
        repository.sendVelocity(cmd)
    }

    fun triggerEmergencyStop() {
        repository.sendEmergencyStop()
    }
}
