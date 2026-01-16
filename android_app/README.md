# Robot Control Android App

Native Android application for controlling and monitoring the OmniBot mecanum wheel robot via ROSBridge WebSocket protocol.

## Project Status
**Phase 1: Foundation** - ✅ Complete

## Features (Implemented)
- ✅ Project structure with MVVM architecture
- ✅ Gradle build configuration with all dependencies
- ✅ Data models for robot telemetry
- ✅ ROSBridge WebSocket client with auto-reconnection
- ✅ Network layer skeleton
- ✅ Android manifest with permissions

## Features (To Do)
- ⏳ Dashboard UI with real-time charts
- ⏳ Virtual joystick control
- ⏳ Emergency stop button
- ⏳ Settings screen
- ⏳ Data logging system

## Requirements
- **Android Studio**: Hedgehog (2023.1.1) or later
- **Minimum SDK**: 24 (Android 7.0)
- **Target SDK**: 34 (Android 14)
- **Kotlin**: 1.9.20+
- **Gradle**: 8.2.0+

## Setup Instructions

### 1. Open in Android Studio
1. Launch Android Studio
2. Select **File > Open**
3. Navigate to `Mecanum-Wheel-Robot/android_app/`
4. Click **OK**
5. Wait for Gradle sync to complete

### 2. Configure SDK
If prompted:
- Install Android SDK 34
- Install Kotlin plugin (should be pre-installed)
- Accept licenses

### 3. Build the Project
```bash
# Via Android Studio: Build > Make Project
# Or via command line:
./gradlew build
```

### 4. Run on Device/Emulator
1. Connect Android device or start emulator
2. Click **Run** ▶ button
3. Select target device

## Project Structure
```
app/
├── src/main/
│   ├── kotlin/com/varunvaidhiya/robotcontrol/
│   │   ├── MainActivity.kt           # Entry point
│   │   ├── data/models/              # Data models
│   │   │   ├── RobotStatus.kt
│   │   │   ├── MotorData.kt
│   │   │   ├── VelocityCommand.kt
│   │   │   └── WheelSpeed.kt
│   │   ├── network/                  # ROSBridge client
│   │   │   ├── ROSBridgeManager.kt
│   │   │   └── ROSBridgeListener.kt
│   │   ├── ui/                       # UI components (TODO)
│   │   └── utils/                    # Utilities
│   │       └── Constants.kt
│   ├── res/                          # Resources
│   │   ├── layout/                   # XML layouts (TODO)
│   │   ├── values/                   # Strings, colors, themes
│   │   └── navigation/               # Navigation graph (TODO)
│   └── AndroidManifest.xml
└── build.gradle.kts
```

## Key Dependencies
- **AndroidX**: Core, AppCompat, Material3, Navigation
- **Coroutines**: Async operations
- **OkHttp**: WebSocket communication
- **Gson**: JSON parsing
- **MPAndroidChart**: Data visualization
- **Timber**: Logging

## ROSBridge Configuration
The app connects to ROSBridge server at:
- **Default IP**: 192.168.1.100
- **Default Port**: 9090
- **URL**: `ws://ROBOT_IP:9090`

Configurable in Settings (Phase 5).

## Next Steps (Development Phases)
1. ✅ **Phase 1**: Foundation (DONE)
2. **Phase 2**: Core Communication
   - Implement Repository pattern
   - Create ViewModels
   - Test bidirectional ROS communication
3. **Phase 3**: Dashboard UI
   - Design layouts with Material3
   - Implement charts (MPAndroidChart)
   - Motor status indicators
4. **Phase 4**: Controls
   - Custom VirtualJoystickView
   - Emergency stop
   - Mode selection
5. **Phase 5**: Advanced Features
   - Data logging to CSV
   - Log viewer
   - Settings persistence
6. **Phase 6**: Polish & Testing
   - Dark theme
   - Animations
   - Tablet optimization

## Testing
```bash
# Unit tests
./gradlew test

# Instrumented tests (requires device/emulator running)
./gradlew connectedAndroidTest
```

## Architecture
- **Pattern**: MVVM (Model-View-ViewModel)
- **UI**: Material Design 3
- **Networking**: OkHttp WebSocket
- **Concurrency**: Kotlin Coroutines
- **Dependency Injection**: Manual (Hilt optional for Phase 2+)

## Robot Requirements
Your robot must run:
- `rosbridge_server` package
- Publish topics: `/robot_status`, `/wheel_speeds`, `/motor_pwm`
- Subscribe to: `/cmd_vel`, `/emergency_stop`

### Install ROSBridge on Robot
```bash
sudo apt install ros-jazzy-rosbridge-suite
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

## Troubleshooting

### Gradle Sync Failed
- Check internet connection (downloads dependencies)
- Invalidate caches: **File > Invalidate Caches > Restart**

### Build Errors
- Ensure JDK 17 is configured
- Clean build: **Build > Clean Project**

### WebSocket Connection Failed
- Verify robot is on same network
- Check ROSBridge is running: `ros2 node list`
- Ping robot IP from phone

## Contributing
This is Phase 1. UI components and business logic are TODO.

## License
MIT License

## Contact
Varun Vaidhiya
