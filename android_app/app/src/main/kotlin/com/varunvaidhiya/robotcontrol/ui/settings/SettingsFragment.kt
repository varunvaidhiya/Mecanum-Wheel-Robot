package com.varunvaidhiya.robotcontrol.ui.settings

import android.os.Bundle
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.Toast
import androidx.fragment.app.viewModels
import androidx.lifecycle.Lifecycle
import androidx.lifecycle.lifecycleScope
import androidx.lifecycle.repeatOnLifecycle
import com.varunvaidhiya.robotcontrol.databinding.FragmentSettingsBinding
import com.varunvaidhiya.robotcontrol.ui.common.BaseFragment
import dagger.hilt.android.AndroidEntryPoint
import kotlinx.coroutines.flow.collectLatest
import kotlinx.coroutines.launch

@AndroidEntryPoint
class SettingsFragment : BaseFragment<FragmentSettingsBinding>() {

    private val viewModel: SettingsViewModel by viewModels()

    override fun inflateBinding(
        inflater: LayoutInflater,
        container: ViewGroup?
    ): FragmentSettingsBinding {
        return FragmentSettingsBinding.inflate(inflater, container, false)
    }

    override fun onViewCreated(view: View, savedInstanceState: Bundle?) {
        super.onViewCreated(view, savedInstanceState)
        
        setupObservers()
        setupListeners()
    }

    private fun setupObservers() {
        viewLifecycleOwner.lifecycleScope.launch {
            viewLifecycleOwner.repeatOnLifecycle(Lifecycle.State.STARTED) {
                launch {
                    viewModel.robotIp.collectLatest { ip ->
                        if (binding.editIp.text.isNullOrEmpty()) {
                           binding.editIp.setText(ip)
                        }
                    }
                }
                launch {
                    viewModel.robotPort.collectLatest { port ->
                        if (binding.editPort.text.isNullOrEmpty()) {
                           binding.editPort.setText(port.toString())
                        }
                    }
                }
            }
        }
    }

    private fun setupListeners() {
        binding.btnSave.setOnClickListener {
            val ip = binding.editIp.text.toString()
            val portStr = binding.editPort.text.toString()
            
            if (ip.isNotBlank() && portStr.isNotBlank()) {
                val port = portStr.toIntOrNull()
                if (port != null) {
                    viewModel.saveConnectionSettings(ip, port)
                    Toast.makeText(requireContext(), "Settings Saved", Toast.LENGTH_SHORT).show()
                } else {
                    binding.layoutPort.error = "Invalid Port"
                }
            } else {
                Toast.makeText(requireContext(), "Please fill all fields", Toast.LENGTH_SHORT).show()
            }
        }
    }
}
