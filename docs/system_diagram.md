# Pypilot Autopilot System Diagram

The diagram below summarizes the electrical and data connections for the pypilot installation described in the recent discussion. It combines the high-current drive path, Arduino Nano servo controller, Raspberry Pi 5 running pypilot, the remote Signal K server, and the supported sensors/actuators.

```mermaid
graph TD
    subgraph Power_Stage[Power & Motor Drive]
        Battery[12 V Battery Bank] -->|Fuse / Breaker| Fuse[Fuse / Breaker]
        Fuse -->|B+| IBT2[IBT-2 (BTS7960) Dual 43 A H-Bridge]
        Battery -->|B-| IBT2
        IBT2 -->|M+ / M-| Motor[Drive Motor]
        IBT2 <--> |High Current Return| Battery
    end

    subgraph Servo_Controller[Arduino Nano Motor Controller]
        Nano[Arduino Nano running motor.ino]
        Nano -->|PWM: D9→RPWM, D10→LPWM| IBT2
        Nano -->|5 V / GND| IBT2
        Nano -. optional .->|D4/D5 current range select| ShuntConfig[Current Shunt Relays]
        Nano -->|A0 Voltage Sense Divider| Battery
        Nano -->|A1 Current Sense| IBT2_IS[IBT-2 R_IS/L_IS]
        Nano -->|A2 H-Bridge Thermistor| HBTemp[Bridge Thermistor]
        Nano -->|A3 Motor Thermistor| MotorTemp[Motor Thermistor]
        Nano -->|A4 Rudder Feedback Pot| RudderSensor[Rudder Angle Sensor]
        Nano -->|D7/D8 Limit Switch Inputs| LimitSwitches[Port/Starboard Limit Switches]
        Nano -->|D11 Clutch MOSFET| Clutch[Optional Clutch Solenoid]
        Nano -->|D13 Status LED| StatusLED[Drive Status LED]
    end

    subgraph Pi5[Pypilot Host]
        Pi[Raspberry Pi 5 running pypilot]
        Pi -->|UART (TX/RX + 5 V/GND)| Nano
        Pi -->|I²C/SPI/USB| IMU[9-DOF IMU (gyro+accelerometer+magnetometer)]
        Pi -->|Ethernet/Wi-Fi| Network[(Vessel Network)]
    end

    subgraph Remote_SignalK[Remote Sensor Sources]
        SignalK[Separate Raspberry Pi running Signal K]
        SignalK -->|Publishes wind, GPS, water-speed, APB| Network
    end

    Pi -->|WebSocket Subscription| SignalK

    classDef hw fill:#f9f9f9,stroke:#555;
    class Battery,Fuse,IBT2,Motor,Nano,Pi,SignalK,IMU,RudderSensor,HBTemp,MotorTemp,LimitSwitches,Clutch,StatusLED,IBT2_IS,ShuntConfig hw;
```

## Notes

- **Grounding:** The battery negative (B−), IBT-2 ground, Arduino ground, and Raspberry Pi ground must all be common to ensure valid logic-level signals and telemetry measurements. If opto-isolation is used between the Pi and Nano, supply isolated 5 V/GND on the isolated side.
- **Power protection:** Place a suitably rated fuse or breaker close to the battery to protect the high-current wiring feeding the IBT-2 module.
- **Sensor calibration:** The 9-DOF IMU connects directly to the Raspberry Pi 5 and is calibrated via the `pypilot_calibration` tool to provide gyro, accelerometer, and compass data to pypilot.
- **Telemetry:** The Arduino forwards voltage, current, thermistor temperatures, rudder angle, and limit switch states to the Pi via the 4-byte serial protocol built into `motor.ino`.
- **Remote data:** Wind, GPS, water-speed, and route/APB data can be consumed from the remote Signal K server once pypilot authenticates and subscribes to its WebSocket feed.
