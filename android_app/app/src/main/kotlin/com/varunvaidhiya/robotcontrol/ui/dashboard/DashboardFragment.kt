package com.varunvaidhiya.robotcontrol.ui.dashboard

import android.os.Bundle
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import androidx.fragment.app.viewModels
import androidx.lifecycle.Lifecycle
import androidx.lifecycle.lifecycleScope
import androidx.lifecycle.repeatOnLifecycle
import com.varunvaidhiya.robotcontrol.R
import com.varunvaidhiya.robotcontrol.databinding.FragmentDashboardBinding
import com.varunvaidhiya.robotcontrol.network.ROSBridgeManager
import com.varunvaidhiya.robotcontrol.ui.camera.CameraViewModel
import com.varunvaidhiya.robotcontrol.ui.common.BaseFragment
import dagger.hilt.android.AndroidEntryPoint
import kotlinx.coroutines.flow.collectLatest
import kotlinx.coroutines.launch

@AndroidEntryPoint
class DashboardFragment : BaseFragment<FragmentDashboardBinding>() {

    private val dashboardViewModel: DashboardViewModel by viewModels()
    private val cameraViewModel: CameraViewModel by viewModels()

    override fun inflateBinding(
        inflater: LayoutInflater,
        container: ViewGroup?
    ): FragmentDashboardBinding {
        return FragmentDashboardBinding.inflate(inflater, container, false)
    }

    override fun onViewCreated(view: View, savedInstanceState: Bundle?) {
        super.onViewCreated(view, savedInstanceState)
        setupObservers()
    }

    private fun setupObservers() {
        // Observe Connection State
        viewLifecycleOwner.lifecycleScope.launch {
            viewLifecycleOwner.repeatOnLifecycle(Lifecycle.State.STARTED) {
                dashboardViewModel.connectionState.collectLatest { state ->
                    updateConnectionStatus(state)
                }
            }
        }
        
        // Observe Robot Status (IP)
        viewLifecycleOwner.lifecycleScope.launch {
            viewLifecycleOwner.repeatOnLifecycle(Lifecycle.State.STARTED) {
                dashboardViewModel.robotStatus.collectLatest { status ->
                    binding.textIpAddress.text = "IP: ${if (status.ipAddress.isEmpty()) "--" else status.ipAddress}"
                }
            }
        }
        
        // Observe Camera URL
        viewLifecycleOwner.lifecycleScope.launch {
            viewLifecycleOwner.repeatOnLifecycle(Lifecycle.State.STARTED) {
                cameraViewModel.streamUrl.collectLatest { url ->
                    if (url.isNotEmpty()) {
                        binding.mjpegView.startStream(url)
                    } else {
                        binding.mjpegView.stopStream()
                    }
                }
            }
        }
        
        // Setup Chart
        setupChart()

        // Observe Wheel Speeds (to update chart)
        viewLifecycleOwner.lifecycleScope.launch {
            viewLifecycleOwner.repeatOnLifecycle(Lifecycle.State.STARTED) {
                dashboardViewModel.wheelSpeeds.collectLatest { speeds ->
                    updateChart(speeds)
                }
            }
        }
    }
    
    private fun setupChart() {
        binding.chartVelocity.apply {
            description.isEnabled = false
            setTouchEnabled(false)
            setDrawGridBackground(false)
            axisRight.isEnabled = false
            xAxis.setDrawLabels(false)
            legend.isEnabled = true
        }
    }
    
    private fun updateChart(speeds: com.varunvaidhiya.robotcontrol.data.models.WheelSpeed) {
        // TODO: Implement real-time data plotting
        // For now, just invalidate to show we touched it
        binding.chartVelocity.invalidate()
    }
    
    private fun updateConnectionStatus(state: ROSBridgeManager.ConnectionState) {
        val (text, colorRes) = when (state) {
            ROSBridgeManager.ConnectionState.CONNECTED -> "Connected" to R.drawable.shape_circle_green
            ROSBridgeManager.ConnectionState.CONNECTING -> "Connecting..." to R.drawable.shape_circle_orange
            ROSBridgeManager.ConnectionState.DISCONNECTED -> "Disconnected" to R.drawable.shape_circle_red
            ROSBridgeManager.ConnectionState.ERROR -> "Error" to R.drawable.shape_circle_red
        }
        
        binding.textConnectionStatus.text = text
        binding.statusIndicator.setBackgroundResource(colorRes)
    }
}
