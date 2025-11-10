# Pypilot Autopilot System Diagram

The diagram below summarizes the electrical and data connections for the pypilot installation described in the recent discussion. It combines the high-current drive path, Arduino Nano servo controller, Raspberry Pi 5 running pypilot, the remote Signal K server, and the supported sensors/actuators.

```mermaid
flowchart TD
    subgraph Power_Stage["Power & Motor Drive"]
        Battery["12V Battery Bank"] -->|Fuse or Breaker| Fuse["Fuse / Breaker"]
        Fuse -->|B+| IBT2["IBT-2 (BTS7960) Dual 43A H-Bridge"]
        Battery -->|B-| IBT2
        IBT2 -->|M+ / M-| Motor["Drive Motor"]
        IBT2 <-->|High Current Return| Battery
    end

    subgraph Servo_Controller["Arduino Nano Motor Controller"]
        Nano["Arduino Nano running motor.ino"]
        Nano -->|PWM D9→RPWM, D10→LPWM| IBT2
        Nano -->|5V and GND| IBT2
        Nano -. "Optional D4/D5 current range select" .-> ShuntConfig["Current Shunt Relays"]
        Nano -->|A0 voltage sense divider| Battery
        Nano -->|A1 current sense| IBT2_IS["IBT-2 R_IS / L_IS"]
        Nano -->|A2 H-bridge thermistor| HBTemp["Bridge Thermistor"]
        Nano -->|A3 motor thermistor| MotorTemp["Motor Thermistor"]
        Nano -->|A4 rudder feedback pot| RudderSensor["Rudder Angle Sensor"]
        Nano -->|D7/D8 limit switch inputs| LimitSwitches["Port / Starboard Limit Switches"]
        Nano -->|D11 clutch MOSFET| Clutch["Optional Clutch Solenoid"]
        Nano -->|D13 status LED| StatusLED["Drive Status LED"]
    end

    subgraph Pi5["Pypilot Host"]
        Pi["Raspberry Pi 5 running pypilot"]
        Pi -->|UART: TX/RX plus 5V and GND| Nano
        Pi -->|I2C, SPI, or USB| IMU["9-DOF IMU (gyro + accelerometer + magnetometer)"]
        Pi -->|Ethernet or Wi-Fi| Network["Vessel Network"]
    end

    subgraph Remote_SignalK["Remote Sensor Sources"]
        SignalK["Separate Raspberry Pi running Signal K"]
        SignalK -->|Publishes wind, GPS, water-speed, APB| Network
    end

    Pi -->|WebSocket subscription| SignalK

    classDef hw fill:#f9f9f9,stroke:#555;
    class Battery,Fuse,IBT2,Motor,Nano,Pi,SignalK,IMU,RudderSensor,HBTemp,MotorTemp,LimitSwitches,Clutch,StatusLED,IBT2_IS,ShuntConfig hw;
```

## Notes

- **Grounding:** The battery negative (B-), IBT-2 ground, Arduino ground, and Raspberry Pi ground must all be common to ensure valid logic-level signals and telemetry measurements. If opto-isolation is used between the Pi and Nano, supply isolated 5V/GND on the isolated side.
- **Power protection:** Place a suitably rated fuse or breaker close to the battery to protect the high-current wiring feeding the IBT-2 module.
- **Sensor calibration:** The 9-DOF IMU connects directly to the Raspberry Pi 5 and is calibrated via the `pypilot_calibration` tool to provide gyro, accelerometer, and compass data to pypilot.
- **Telemetry:** The Arduino forwards voltage, current, thermistor temperatures, rudder angle, and limit switch states to the Pi via the 4-byte serial protocol built into `motor.ino`.
- **Remote data:** Wind, GPS, water-speed, and route/APB data can be consumed from the remote Signal K server once pypilot authenticates and subscribes to its WebSocket feed.
