package com.varunvaidhiya.robotcontrol.ui.slam

import android.os.Bundle
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import androidx.fragment.app.viewModels
import androidx.lifecycle.Lifecycle
import androidx.lifecycle.lifecycleScope
import androidx.lifecycle.repeatOnLifecycle
import com.varunvaidhiya.robotcontrol.databinding.FragmentSlamMapBinding
import com.varunvaidhiya.robotcontrol.ui.common.BaseFragment
import dagger.hilt.android.AndroidEntryPoint
import kotlinx.coroutines.flow.collectLatest
import kotlinx.coroutines.launch

@AndroidEntryPoint
class SlamMapFragment : BaseFragment<FragmentSlamMapBinding>() {

    private val slamViewModel: SlamViewModel by viewModels()

    override fun inflateBinding(
        inflater: LayoutInflater,
        container: ViewGroup?
    ): FragmentSlamMapBinding {
        return FragmentSlamMapBinding.inflate(inflater, container, false)
    }

    override fun onViewCreated(view: View, savedInstanceState: Bundle?) {
        super.onViewCreated(view, savedInstanceState)
        setupObservers()
    }

    private fun setupObservers() {
        viewLifecycleOwner.lifecycleScope.launch {
            viewLifecycleOwner.repeatOnLifecycle(Lifecycle.State.STARTED) {
                
                // Collect Map Data
                launch {
                    slamViewModel.mapData.collectLatest { data ->
                        if (data != null) {
                            val width = slamViewModel.mapWidth.value
                            val height = slamViewModel.mapHeight.value
                            binding.slamMapView.updateMap(width, height, data)
                        }
                    }
                }
                
                // Collect Robot Pose
                launch {
                    slamViewModel.robotPoseX.collectLatest { x ->
                        // Combine with Y and Yaw if possible, or trigger update here
                        val y = slamViewModel.robotPoseY.value
                        val yaw = slamViewModel.robotYaw.value
                        binding.slamMapView.updateRobotPose(x, y, yaw)
                    }
                }
            }
        }
    }
}
