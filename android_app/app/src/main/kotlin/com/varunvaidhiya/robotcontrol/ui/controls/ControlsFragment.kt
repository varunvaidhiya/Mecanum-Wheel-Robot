package com.varunvaidhiya.robotcontrol.ui.controls

import android.os.Bundle
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import androidx.appcompat.app.AlertDialog
import androidx.fragment.app.viewModels
import com.varunvaidhiya.robotcontrol.R
import com.varunvaidhiya.robotcontrol.databinding.FragmentControlsBinding
import com.varunvaidhiya.robotcontrol.ui.common.BaseFragment
import dagger.hilt.android.AndroidEntryPoint
import java.util.Locale

@AndroidEntryPoint
class ControlsFragment : BaseFragment<FragmentControlsBinding>() {

    private val viewModel: ControlsViewModel by viewModels()

    override fun inflateBinding(
        inflater: LayoutInflater,
        container: ViewGroup?
    ): FragmentControlsBinding {
        return FragmentControlsBinding.inflate(inflater, container, false)
    }

    override fun onViewCreated(view: View, savedInstanceState: Bundle?) {
        super.onViewCreated(view, savedInstanceState)
        
        setupJoystick()
        setupEmergencyStop()
        setupModeToggle()
    }

    private fun setupJoystick() {
        binding.virtualJoystick.onJoystickMoved = { x, y ->
            // x is normalized (-1 to 1) -> corresponds to Angular Z (Left/Right)
            // y is normalized (-1 to 1) -> corresponds to Linear X (Forward/Backward)
            
            // Mecanum conversion: 
            // In basic teleop, left/right stick usually means rotate or strafe.
            // Let's assume standard differential drive style for now: Y=LinearX, X=AngularZ
            
            // Scaling factors (should come from settings)
            val maxLinear = 1.0f 
            val maxAngular = 1.5f
            
            val linearX = y * maxLinear
            val angularz = -x * maxAngular // Invert x for correct rotation direction
            
            viewModel.sendVelocity(linearX, 0f, angularz) // Strafe (Linear Y) is 0 for standard joystick
            
            binding.textValues.text = String.format(
                Locale.US, "Linear: %.2f | Angular: %.2f", linearX, angularz
            )
        }
    }

    private fun setupEmergencyStop() {
        binding.btnEmergencyStop.setOnClickListener {
            showEmergencyStopConfirmation()
        }
    }

    private fun showEmergencyStopConfirmation() {
        AlertDialog.Builder(requireContext())
            .setTitle("EMERGENCY STOP")
            .setMessage(getString(R.string.emergency_stop_confirm))
            .setIcon(android.R.drawable.ic_dialog_alert) // Use a standard warning icon
            .setPositiveButton("STOP") { _, _ ->
                viewModel.triggerEmergencyStop()
            }
            .setNegativeButton("Cancel", null)
            .show()
    }
    
    private fun setupModeToggle() {
        // TODO: Implement mode switching logic publishing to /robot_mode
    }
}
