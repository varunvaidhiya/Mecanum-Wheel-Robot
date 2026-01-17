package com.varunvaidhiya.robotcontrol.ui.logs

import android.os.Bundle
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.TextView
import androidx.recyclerview.widget.LinearLayoutManager
import androidx.recyclerview.widget.RecyclerView
import com.varunvaidhiya.robotcontrol.R
import com.varunvaidhiya.robotcontrol.RobotApplication
import com.varunvaidhiya.robotcontrol.databinding.FragmentLogsBinding
import com.varunvaidhiya.robotcontrol.ui.common.BaseFragment
import com.varunvaidhiya.robotcontrol.utils.DataLogger
import java.io.File

class LogsFragment : BaseFragment<FragmentLogsBinding>() {

    // We can instantiate DataLogger here or in Application.
    // For simplicitly in this phase:
    private val dataLogger by lazy { DataLogger(requireContext()) }
    
    override fun inflateBinding(
        inflater: LayoutInflater,
        container: ViewGroup?
    ): FragmentLogsBinding {
        return FragmentLogsBinding.inflate(inflater, container, false)
    }

    override fun onViewCreated(view: View, savedInstanceState: Bundle?) {
        super.onViewCreated(view, savedInstanceState)
        
        setupRecyclerView()
        setupListeners()
        loadLogs()
    }
    
    private fun setupRecyclerView() {
        binding.recyclerLogs.layoutManager = LinearLayoutManager(requireContext())
        // Adapter would go here. For now, we'll keep it simple or assume an adapter
        // Let's create a simple adapter inline or separately.
        
        binding.recyclerLogs.adapter = LogAdapter(emptyList())
    }
    
    private fun setupListeners() {
        binding.switchLogging.setOnCheckedChangeListener { _, isChecked ->
            dataLogger.isLoggingEnabled = isChecked
            if (isChecked) {
                dataLogger.startNewSession()
            } else {
                dataLogger.stopSession()
            }
            loadLogs() // Refresh list
        }
    }
    
    private fun loadLogs() {
        val files = dataLogger.getLogFiles()
        if (files.isEmpty()) {
            binding.textEmpty.visibility = View.VISIBLE
            binding.recyclerLogs.visibility = View.GONE
        } else {
            binding.textEmpty.visibility = View.GONE
            binding.recyclerLogs.visibility = View.VISIBLE
            (binding.recyclerLogs.adapter as LogAdapter).updateData(files)
        }
    }
    
    // Simple Adapter Class
    class LogAdapter(private var files: List<File>) : RecyclerView.Adapter<LogAdapter.ViewHolder>() {
        
        class ViewHolder(view: View) : RecyclerView.ViewHolder(view) {
            val textView: TextView = view.findViewById(android.R.id.text1)
        }

        override fun onCreateViewHolder(parent: ViewGroup, viewType: Int): ViewHolder {
            val view = LayoutInflater.from(parent.context)
                .inflate(android.R.layout.simple_list_item_1, parent, false)
            return ViewHolder(view)
        }

        override fun onBindViewHolder(holder: ViewHolder, position: Int) {
            val file = files[position]
            holder.textView.text = "${file.name} (${file.length() / 1024} KB)"
            // click listener to share?
        }

        override fun getItemCount() = files.size
        
        fun updateData(newFiles: List<File>) {
            files = newFiles
            notifyDataSetChanged()
        }
    }
}
