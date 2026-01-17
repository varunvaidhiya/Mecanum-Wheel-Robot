package com.varunvaidhiya.robotcontrol.ui.slam

import androidx.lifecycle.ViewModel
import com.varunvaidhiya.robotcontrol.data.repository.RobotRepository
import dagger.hilt.android.lifecycle.HiltViewModel
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import javax.inject.Inject

@HiltViewModel
class SlamViewModel @Inject constructor(
    private val repository: RobotRepository
) : ViewModel() {
    
    val connectionState = repository.connectionState
    
    private val _mapData = MutableStateFlow<IntArray?>(null)
    val mapData: StateFlow<IntArray?> = _mapData.asStateFlow()
    
    private val _mapWidth = MutableStateFlow(0)
    val mapWidth: StateFlow<Int> = _mapWidth.asStateFlow()
    
    private val _mapHeight = MutableStateFlow(0)
    val mapHeight: StateFlow<Int> = _mapHeight.asStateFlow()
    
    // Robot pose in map coordinates (pixels)
    private val _robotPoseX = MutableStateFlow(0f)
    val robotPoseX: StateFlow<Float> = _robotPoseX.asStateFlow()
    
    private val _robotPoseY = MutableStateFlow(0f)
    val robotPoseY: StateFlow<Float> = _robotPoseY.asStateFlow()
    
    private val _robotYaw = MutableStateFlow(0f)
    val robotYaw: StateFlow<Float> = _robotYaw.asStateFlow()

    // TODO: Subscribe to /map and /tf topics in Repository and expose here
    // For Phase 3, we'll simulate map updates or use a placeholder
}
