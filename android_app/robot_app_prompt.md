# Mecanum Wheel Robot Control Android App - Complete Development Prompt

## Project Overview
Build a native Android application in Kotlin for monitoring and controlling a mecanum wheel robot. The app communicates with ROS (Robot Operating System) via ROSBridge WebSocket protocol, providing real-time telemetry visualization, robot control, and system diagnostics.

---

## Technical Stack

### Core Technologies
- **Language**: Kotlin
- **Min SDK**: 24 (Android 7.0)
- **Target SDK**: 34 (Android 14)
- **Build System**: Gradle with Kotlin DSL
- **Architecture**: MVVM (Model-View-ViewModel)

### Key Dependencies
```gradle
dependencies {
    // AndroidX Core
    implementation("androidx.core:core-ktx:1.12.0")
    implementation("androidx.appcompat:appcompat:1.6.1")
    implementation("com.google.android.material:material:1.11.0")
    implementation("androidx.constraintlayout:constraintlayout:2.1.4")
    
    // Navigation Component
    implementation("androidx.navigation:navigation-fragment-ktx:2.7.6")
    implementation("androidx.navigation:navigation-ui-ktx:2.7.6")
    
    // Lifecycle & ViewModel
    implementation("androidx.lifecycle:lifecycle-viewmodel-ktx:2.7.0")
    implementation("androidx.lifecycle:lifecycle-livedata-ktx:2.7.0")
    
    // Coroutines
    implementation("org.jetbrains.kotlinx:kotlinx-coroutines-android:1.7.3")
    
    // WebSocket (OkHttp)
    implementation("com.squareup.okhttp3:okhttp:4.12.0")
    
    // JSON Parsing
    implementation("com.google.code.gson:gson:2.10.1")
    
    // Charts for data visualization
    implementation("com.github.PhilJay:MPAndroidChart:v3.1.0")
    
    // Logging
    implementation("com.jakewharton.timber:timber:5.0.1")
    
    // Preferences DataStore
    implementation("androidx.datastore:datastore-preferences:1.0.0")
}
```

---

## Application Architecture

### Package Structure
```
com.yourname.robotcontrol/
├── ui/
│   ├── MainActivity.kt
│   ├── dashboard/
│   │   ├── DashboardFragment.kt
│   │   └── DashboardViewModel.kt
│   ├── controls/
│   │   ├── ControlsFragment.kt
│   │   ├── ControlsViewModel.kt
│   │   └── VirtualJoystickView.kt
│   ├── logs/
│   │   ├── LogsFragment.kt
│   │   └── LogsViewModel.kt
│   └── settings/
│       ├── SettingsFragment.kt
│       └── SettingsViewModel.kt
├── data/
│   ├── models/
│   │   ├── RobotStatus.kt
│   │   ├── MotorData.kt
│   │   ├── VelocityCommand.kt
│   │   └── WheelSpeed.kt
│   ├── repository/
│   │   └── RobotRepository.kt
│   └── preferences/
│       └── AppPreferences.kt
├── network/
│   ├── ROSBridgeManager.kt
│   ├── ROSBridgeListener.kt
│   └── WebSocketClient.kt
└── utils/
    ├── Constants.kt
    ├── Extensions.kt
    └── DataLogger.kt
```

---

## Implementation Details

### 1. ROSBridge WebSocket Manager

**Requirements:**
- Connect to ROSBridge server at `ws://ROBOT_IP:9090`
- Subscribe to ROS topics: `/robot_status`, `/wheel_speeds`, `/motor_pwm`, `/diagnostics`
- Publish to: `/cmd_vel`, `/emergency_stop`
- Handle reconnection with exponential backoff
- Parse JSON messages from ROSBridge protocol

**ROSBridge Message Format:**
```json
// Subscribe to topic
{
  "op": "subscribe",
  "topic": "/robot_status",
  "type": "robot_msgs/RobotStatus"
}

// Publish message
{
  "op": "publish",
  "topic": "/cmd_vel",
  "msg": {
    "linear": {"x": 0.5, "y": 0.0, "z": 0.0},
    "angular": {"x": 0.0, "y": 0.0, "z": 0.3}
  }
}
```

**Key Features:**
- Connection state management (DISCONNECTED, CONNECTING, CONNECTED, ERROR)
- Automatic reconnection with 1s, 2s, 4s, 8s retry intervals
- Thread-safe message queue
- Callback-based message handling
- Heartbeat/ping mechanism every 5 seconds

---

### 2. Data Models

**RobotStatus.kt**
```kotlin
data class RobotStatus(
    val isConnected: Boolean,
    val ipAddress: String,
    val signalStrength: Int, // 0-100
    val timestamp: Long,
    val rosNodesActive: Int,
    val emergencyStopActive: Boolean,
    val currentMode: RobotMode // TELEOP, AUTONOMOUS, MANUAL
)

enum class RobotMode {
    TELEOP, AUTONOMOUS, MANUAL
}
```

**MotorData.kt**
```kotlin
data class MotorData(
    val frontLeft: MotorStatus,
    val frontRight: MotorStatus,
    val backLeft: MotorStatus,
    val backRight: MotorStatus,
    val timestamp: Long
)

data class MotorStatus(
    val pwmValue: Int, // -255 to 255
    val rpm: Float,
    val current: Float, // Amps
    val temperature: Float, // Celsius
    val health: MotorHealth // NORMAL, WARNING, CRITICAL
)

enum class MotorHealth {
    NORMAL, WARNING, CRITICAL
}
```

**VelocityCommand.kt**
```kotlin
data class VelocityCommand(
    val linearX: Float, // m/s
    val linearY: Float, // m/s (for mecanum)
    val angular: Float, // rad/s
    val timestamp: Long
)
```

**WheelSpeed.kt**
```kotlin
data class WheelSpeed(
    val frontLeft: Float,  // m/s
    val frontRight: Float,
    val backLeft: Float,
    val backRight: Float,
    val timestamp: Long
)
```

---

### 3. Dashboard Fragment UI Components

#### Connection Status Card
- **Green indicator**: Connected to ROS
- **Red indicator**: Disconnected
- **Yellow indicator**: Connecting
- Display: Robot IP, Signal strength (0-100%), Last update timestamp
- **Reconnect Button**: Manual reconnection trigger

#### Velocity Command Card
- Real-time line chart (MPAndroidChart) showing last 30 seconds
- Three lines: Linear X (blue), Linear Y (green), Angular Z (red)
- Current values displayed as text: "Linear X: 0.50 m/s"
- Update rate: 10 Hz

#### Motor Status Card
- 4 circular progress indicators (Material3 CircularProgressIndicator)
- Layout: 2x2 grid (FL, FR, BL, BR)
- Each shows: PWM value, RPM, Temperature
- Color coding:
  - Green: PWM < 170, Temp < 50°C
  - Yellow: PWM 170-220, Temp 50-70°C
  - Red: PWM > 220, Temp > 70°C

#### Wheel Speeds Card
- Top-down robot visualization (custom Canvas drawing)
- Draw 4 wheels as rectangles
- Arrow vectors showing speed direction and magnitude
- Display RPM or m/s for each wheel

#### System Health Card
- CPU usage progress bar (if available)
- Temperature gauge (if available)
- Active ROS nodes count
- Emergency stop status (BIG indicator)

---

### 4. Controls Fragment

#### Virtual Joystick (Custom View)
- Center-return joystick with circular boundary
- Output: X and Y values from -1.0 to 1.0
- Deadzone: 0.1 (ignore values < 0.1)
- Visual feedback: Outer circle (boundary), Inner circle (handle)
- Publish `/cmd_vel` at 20 Hz while active

**Touch Handling:**
- On touch down: Capture position
- On touch move: Calculate delta from center, constrain to circle
- On touch up: Return to center, publish zero velocity

#### Emergency Stop Button
- Large red circular button (diameter: 120dp)
- Publishes to `/emergency_stop` topic
- Shows confirmation dialog before activating
- Locks all controls when active
- Requires physical robot reset to clear

#### Mode Selection
- Three toggle buttons: TELEOP / AUTONOMOUS / MANUAL
- Publishes to `/robot_mode` topic
- Disables joystick in AUTONOMOUS mode

---

### 5. Logs Fragment

#### ROS Log Viewer
- RecyclerView with log entries
- Color-coded by severity: DEBUG (gray), INFO (blue), WARN (yellow), ERROR (red)
- Timestamp for each log
- Auto-scroll to latest
- Filter by severity level
- Clear logs button
- Export logs to file button

#### Diagnostics Panel
- Subscribe to `/diagnostics` topic
- Display diagnostic messages from robot
- Show warning/error counts
- Expandable sections for detailed diagnostics

---

### 6. Settings Fragment

#### Connection Settings
- Robot IP Address input (EditText with IP validation)
- Port number (default: 9090)
- Auto-reconnect toggle
- Connection timeout (5s, 10s, 30s)

#### Control Settings
- Joystick sensitivity slider (0.5x - 2.0x)
- Max linear velocity (0.5 - 2.0 m/s)
- Max angular velocity (0.5 - 3.0 rad/s)
- Joystick deadzone (0.0 - 0.3)

#### Display Settings
- Chart update rate (5 Hz, 10 Hz, 20 Hz)
- Chart history duration (10s, 30s, 60s)
- Theme selection (Light, Dark, Auto)

#### Data Logging
- Enable/disable logging toggle
- Log location display
- Export logs button
- Clear logs button

---

### 7. Data Logging System

**Requirements:**
- Log all telemetry data to local CSV files
- File format: `robot_log_YYYYMMDD_HHMMSS.csv`
- Location: `Android/data/com.yourname.robotcontrol/files/logs/`
- Columns: timestamp, linear_x, linear_y, angular_z, motor_fl_pwm, motor_fr_pwm, motor_bl_pwm, motor_br_pwm, motor_fl_rpm, etc.
- Max file size: 50 MB (create new file after)
- Background logging using Kotlin Coroutines

**Logger Features:**
- Start/stop logging from settings
- Show current log file size
- Export via share intent
- Auto-cleanup logs older than 7 days

---

### 8. Network Configuration

#### WiFi Connection Helper
- Scan for robot WiFi network (prefix: "ROBOT_")
- Auto-connect to robot network
- Display WiFi strength
- Handle network callbacks for connection status

#### IP Discovery
- Option 1: Manual IP entry
- Option 2: mDNS/Bonjour discovery (if robot broadcasts)
- Option 3: Network scan (ping sweep on local subnet)

---

## UI/UX Requirements

### Material Design 3
- Use Material3 components throughout
- Color scheme: 
  - Primary: #2196F3 (Blue)
  - Secondary: #4CAF50 (Green)
  - Error: #F44336 (Red)
  - Warning: #FF9800 (Orange)
- Dark theme support

### Responsive Layout
- Optimize for tablets (10" screens)
- Support both portrait and landscape orientations
- Use ConstraintLayout for flexible layouts
- Fragment navigation with bottom navigation bar

### Animations
- Smooth transitions between fragments
- Animated connection status indicator
- Pulsing effect on emergency stop button
- Chart animations for data updates

### Accessibility
- Content descriptions for all interactive elements
- Minimum touch target size: 48dp
- High contrast mode support
- Screen reader compatibility

---

## Error Handling & Edge Cases

### Connection Errors
- **WebSocket fails to connect**: Show error message, trigger auto-reconnect
- **Connection drops mid-operation**: Freeze UI, show reconnecting spinner
- **Invalid IP address**: Validate input, show error hint
- **Robot not responding**: Timeout after 10 seconds, show error dialog

### Data Validation
- Validate all incoming JSON messages
- Handle missing fields gracefully (use default values)
- Catch malformed messages, log error, continue operation
- Clamp velocity values to safe ranges

### Safety Features
- Emergency stop always functional (local button, no network dependency)
- Watchdog timer: If no velocity command for 1 second, robot stops
- Connection loss: Automatically send stop command
- Confirmation dialog for critical actions

---

## Testing Requirements

### Unit Tests
- Test ROSBridge message parsing
- Test velocity calculations from joystick input
- Test data model validation
- Test connection state machine

### Integration Tests
- Test WebSocket connection lifecycle
- Test pub/sub with mock ROSBridge server
- Test data flow from network to UI

### Manual Testing Checklist
- [ ] Connect to robot WiFi
- [ ] Establish WebSocket connection
- [ ] Verify all dashboard cards display data
- [ ] Test virtual joystick control
- [ ] Test emergency stop
- [ ] Switch between modes (TELEOP/AUTONOMOUS/MANUAL)
- [ ] Verify charts update in real-time
- [ ] Test connection loss and recovery
- [ ] Verify data logging works
- [ ] Export logs successfully
- [ ] Test on different screen sizes

---

## Development Phases

### Phase 1: Foundation (Week 1)
- [ ] Create Android Studio project
- [ ] Set up project structure and dependencies
- [ ] Implement ROSBridgeManager with WebSocket client
- [ ] Create basic data models
- [ ] Test connection to ROSBridge server using hardcoded IP

### Phase 2: Core Communication (Week 2)
- [ ] Implement Repository pattern
- [ ] Create ViewModels for each fragment
- [ ] Set up LiveData/StateFlow for reactive updates
- [ ] Implement topic subscription and publishing
- [ ] Build connection status UI
- [ ] Test bidirectional communication

### Phase 3: Dashboard UI (Week 3)
- [ ] Design main dashboard layout with Material3
- [ ] Implement all dashboard cards
- [ ] Integrate MPAndroidChart for velocity graphs
- [ ] Create custom motor status circular indicators
- [ ] Build wheel speed visualization
- [ ] Test with mock data

### Phase 4: Controls (Week 4)
- [ ] Implement custom VirtualJoystickView
- [ ] Add emergency stop button with confirmation
- [ ] Create mode selection UI
- [ ] Implement settings screen
- [ ] Add auto-reconnection logic
- [ ] Test control responsiveness

### Phase 5: Advanced Features (Week 5)
- [ ] Implement data logging system
- [ ] Create logs viewer with filtering
- [ ] Add export functionality
- [ ] Optimize chart performance (limit data points)
- [ ] Add WiFi helper features
- [ ] Implement preferences persistence

### Phase 6: Polish & Testing (Week 6)
- [ ] Add animations and transitions
- [ ] Implement dark theme
- [ ] Optimize for tablets
- [ ] Comprehensive testing with actual robot
- [ ] Fix bugs and edge cases
- [ ] Write user documentation

---

## Performance Optimization

### Memory Management
- Limit chart data points to 300 (30 seconds at 10 Hz)
- Use RecyclerView with ViewHolder pattern for logs
- Clear old log entries from memory (keep last 1000)
- Properly close WebSocket connections

### Battery Optimization
- Reduce update frequency when app in background
- Use WorkManager for background logging if needed
- Disconnect WebSocket when app paused (optional setting)

### Network Optimization
- Batch multiple small messages if possible
- Compress large payloads
- Use binary protocol if ROSBridge supports it
- Implement message priority queue

---

## Documentation Requirements

### README.md
- Project overview
- Features list
- Setup instructions (Android Studio, dependencies)
- ROS node requirements on robot side
- Connection setup guide
- Screenshots of all screens

### Code Documentation
- KDoc comments for all public classes and functions
- Explain complex algorithms (joystick math, reconnection logic)
- Document ROSBridge message formats used

### User Guide
- How to connect to robot
- How to use each feature
- Troubleshooting common issues
- Safety guidelines

---

## Security Considerations

- Store robot IP in encrypted SharedPreferences
- Don't log sensitive information
- Validate all user inputs
- Sanitize data before logging to file
- No hardcoded credentials

---

## Future Enhancements (Optional)

- Camera feed integration (MJPEG stream)
- Xbox controller support via Bluetooth
- Multi-robot support (switch between robots)
- Path recording and playback
- Voice commands integration
- Cloud data backup
- Firmware update capability

---

## Deliverables

1. Complete Android Studio project with all source code
2. APK file for installation
3. README with setup and usage instructions
4. Architecture documentation
5. Testing report
6. Screenshots of all app screens

---

## Success Criteria

- ✅ App connects reliably to ROSBridge server
- ✅ All dashboard metrics display real-time data
- ✅ Virtual joystick controls robot smoothly (< 50ms latency)
- ✅ Emergency stop works instantly
- ✅ Charts update without lag or stuttering
- ✅ App recovers gracefully from connection loss
- ✅ No crashes during normal operation
- ✅ Data logging works correctly
- ✅ UI is responsive and intuitive

---

## Notes for AI Assistant

- Use modern Kotlin idioms (coroutines, flows, sealed classes)
- Follow SOLID principles and clean architecture
- Use dependency injection (manual or Hilt) for better testability
- Write null-safe code with proper null handling
- Use ViewBinding (not findViewById)
- Implement proper lifecycle awareness (no memory leaks)
- Follow Material Design guidelines strictly
- Add TODO comments for complex sections
- Use Timber for logging (not Log.d)
- Handle configuration changes (screen rotation) properly

---

## Getting Started Command

To begin implementation:
1. Create new Android Studio project: "Robot Control"
2. Package name: `com.yourname.robotcontrol`
3. Language: Kotlin
4. Minimum SDK: 24
5. Use ViewBinding: Yes
6. Add all dependencies from the Technical Stack section
7. Create the package structure as outlined
8. Start with Phase 1: Foundation

Let's build an amazing robot control app! 🤖📱