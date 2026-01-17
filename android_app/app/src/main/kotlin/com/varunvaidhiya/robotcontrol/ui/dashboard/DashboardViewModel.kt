package com.varunvaidhiya.robotcontrol.ui.dashboard

import androidx.lifecycle.ViewModel
import com.varunvaidhiya.robotcontrol.data.repository.RobotRepository
import dagger.hilt.android.lifecycle.HiltViewModel
import javax.inject.Inject

@HiltViewModel
class DashboardViewModel @Inject constructor(
    private val repository: RobotRepository
) : ViewModel() {
    
    val wheelSpeeds = repository.wheelSpeeds
    val motorData = repository.motorData
    val robotStatus = repository.robotStatus
    val connectionState = repository.connectionState

}
