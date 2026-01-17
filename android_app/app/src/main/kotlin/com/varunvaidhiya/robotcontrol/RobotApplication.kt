package com.varunvaidhiya.robotcontrol

import android.app.Application
import com.varunvaidhiya.robotcontrol.data.repository.RobotRepository
import com.varunvaidhiya.robotcontrol.data.preferences.AppPreferences
import timber.log.Timber

class RobotApplication : Application() {

    // Global singleton repository
    val repository: RobotRepository by lazy { RobotRepository() }
    
    // Global singleton preferences
    val preferences: AppPreferences by lazy { AppPreferences(this) }

    override fun onCreate() {
        super.onCreate()
        if (BuildConfig.DEBUG) {
            Timber.plant(Timber.DebugTree())
        }
    }
}
