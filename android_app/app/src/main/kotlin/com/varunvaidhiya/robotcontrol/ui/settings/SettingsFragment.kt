package com.varunvaidhiya.robotcontrol.ui.settings

import android.view.LayoutInflater
import android.view.ViewGroup
import com.varunvaidhiya.robotcontrol.databinding.FragmentSettingsBinding
import com.varunvaidhiya.robotcontrol.ui.common.BaseFragment

class SettingsFragment : BaseFragment<FragmentSettingsBinding>() {
    override fun inflateBinding(
        inflater: LayoutInflater,
        container: ViewGroup?
    ): FragmentSettingsBinding {
        return FragmentSettingsBinding.inflate(inflater, container, false)
    }
}
