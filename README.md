# 🤖 ESP32 Battle Bot Extraordinaire! 

Welcome to the most exciting DIY combat robot project you'll ever build! This isn't just any robot - it's a fully autonomous warrior with real-time vision, smart sensing, and a mighty battle hammer! 

## ✨ Features That Make This Bot Amazing

### 🎥 Vision System
- Real-time video streaming via ESP32-CAM
- Web-based remote viewing
- VGA quality battle footage

### 🧠 Smart Combat Systems
- 🔨 Servo-powered battle hammer
- 🎯 Autonomous attack mode (triggers within 30cm)
- 🛡️ Defensive color detection system
- 💥 Touch-sensitive "armor" with visual feedback

### 🌈 Visual Feedback System
- 8x8 LED Matrix with dynamic rainbow patterns
- Red alert system when damaged
- Status indicator lighting

### 🎮 Control Features
- WebSocket-based remote control
- WASD keyboard navigation
- Space bar weapon activation
- Autonomous defensive maneuvers

## 🛠️ Hardware Requirements

### Core Components
- ESP32 Development Board
- ESP32-CAM Module
- Servo Motor for Battle Hammer
- Dual DC Motors for Movement
- Color Sensor (TCS3200 or similar)
- Ultrasonic Distance Sensor
- 8x8 NeoMatrix Display
- Single NeoPixel LED
- Limit Switch
- Power Supply (Battery Pack)

### 📌 Pin Configuration
```
Color Sensor:
- S0: 14
- S1: 27
- S2: 26
- S3: 25
- OUT: 33
- LED: 32

Motors:
- IN1: 23
- IN2: 22
- ENA: 4
- IN3: 19
- IN4: 18
- ENB: 5

Other:
- Servo: Pin 2
- NeoPixel Strip: Pin 16
- NeoMatrix: Pin 21
- Limit Switch: Pin 13
- Ultrasonic TRIG: 15
- Ultrasonic ECHO: 34
```

## 🚀 Getting Started

1. **Hardware Assembly**
   - Mount all components securely
   - Connect according to pin configuration
   - Ensure proper power distribution

2. **Software Setup**
   ```bash
   # Clone the repository
   git clone [your-repo-url]
   
   # Install Python dependencies for control server
   pip install websockets keyboard

   # Install Arduino Libraries
   - WebSocketsClient
   - Adafruit_NeoPixel
   - Adafruit_NeoMatrix
   - ESP32Servo
   - ezButton
   ```

3. **Network Configuration**
   - Update WiFi credentials in both ESP32 files
   - Set WebSocket server IP in robot code

4. **Launch Sequence**
   ```bash
   # Start WebSocket control server
   python PythonWebsocket.py

   # Access camera stream
   http://[ESP32-IP-ADDRESS]/stream
   ```

## 🎮 Controls

- **W** - Move Forward
- **S** - Move Backward
- **A** - Turn Left
- **D** - Turn Right
- **SPACE** - Activate Battle Hammer

## 🛡️ Battle Features

### Autonomous Defense
- **Blue Detection**: Robot ignores commands for 5 seconds when blue detected
- **Collision Detection**: 30-second lockdown when hit
- **Auto-Attack**: Triggers hammer when opponents within 30cm

### Visual Alerts
- Rainbow pattern during normal operation
- Full red alert when damaged
- Status indicator for battle conditions

## 🔧 Maintenance Tips

1. Regular battery check before battles
2. Secure all mounting points after matches
3. Test limit switch and color sensor calibration
4. Check servo alignment for optimal hammer strike

## 🏆 Battle Tips

1. Use the autonomous features to your advantage
2. Keep an eye on the visual feedback for damage status
3. Time your hammer strikes strategically
4. Use the color detection system for defensive maneuvers

## ⚠️ Safety First!

- Always operate in a proper battle arena
- Keep spectators at a safe distance
- Wear proper eye protection
- Have an emergency stop procedure ready
- Check all mechanical connections before battles

## 🤝 Contributing

Found a way to make this battle bot even more awesome? Contributions are welcome!

1. Fork the repository
2. Create your feature branch
3. Commit your changes
4. Push to the branch
5. Open a Pull Request

## 📝 License

This project is licensed under the MIT License - see the LICENSE file for details.

## 🌟 Acknowledgments

- The amazing ESP32 community
- Combat robotics enthusiasts worldwide
- Everyone who contributed to the testing phase

Happy Building and Battle On! 🚀
