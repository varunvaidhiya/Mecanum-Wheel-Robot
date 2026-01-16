package com.varunvaidhiya.robotcontrol

import android.os.Bundle
import androidx.appcompat.app.AppCompatActivity
import androidx.navigation.findNavController
import androidx.navigation.ui.AppBarConfiguration
import androidx.navigation.ui.setupActionBarWithNavController
import androidx.navigation.ui.setupWithNavController
import com.google.android.material.bottomnavigation.BottomNavigationView
import timber.log.Timber

/**
 * Main activity - entry point for the Robot Control app
 */
class MainActivity : AppCompatActivity() {
    
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        
        // Initialize Timber logging
        if (BuildConfig.DEBUG) {
            Timber.plant(Timber.DebugTree())
        }
        
        // TODO: Set content view and navigation after layouts are created
        // setContentView(R.layout.activity_main)
        // setupNavigation()
        
        Timber.i("MainActivity created")
    }
    
    private fun setupNavigation() {
        // TODO: Implement navigation setup
        // val navView: BottomNavigationView = findViewById(R.id.nav_view)
        // val navController = findNavController(R.id.nav_host_fragment)
        // val appBarConfiguration = AppBarConfiguration(setOf(
        //     R.id.navigation_dashboard,
        //     R.id.navigation_controls,
        //     R.id.navigation_logs,
        //     R.id.navigation_settings
        // ))
        // setupActionBarWithNavController(navController, appBarConfiguration)
        // navView.setupWithNavController(navController)
    }
}
