package com.varunvaidhiya.robotcontrol.ui.logs

import android.view.LayoutInflater
import android.view.ViewGroup
import com.varunvaidhiya.robotcontrol.databinding.FragmentLogsBinding
import com.varunvaidhiya.robotcontrol.ui.common.BaseFragment

class LogsFragment : BaseFragment<FragmentLogsBinding>() {
    override fun inflateBinding(
        inflater: LayoutInflater,
        container: ViewGroup?
    ): FragmentLogsBinding {
        return FragmentLogsBinding.inflate(inflater, container, false)
    }
}
