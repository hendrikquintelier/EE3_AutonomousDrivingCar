| Supported Targets | ESP32 | ESP32-C2 | ESP32-C3 | ESP32-C5 | ESP32-C6 | ESP32-C61 | ESP32-H2 | ESP32-P4 | ESP32-S2 | ESP32-S3 |
| ----------------- | ----- | -------- | -------- | -------- | -------- | --------- | -------- | -------- | -------- | -------- |

# Blink Example

(See the README.md file in the upper level 'examples' directory for more information about examples.)

This example demonstrates how to blink a LED by using the GPIO driver or using the [led_strip](https://components.espressif.com/component/espressif/led_strip) library if the LED is addressable e.g. [WS2812](https://cdn-shop.adafruit.com/datasheets/WS2812B.pdf). The `led_strip` library is installed via [component manager](main/idf_component.yml).

## How to Use Example

Before project configuration and build, be sure to set the correct chip target using `idf.py set-target <chip_name>`.

### Hardware Required

* A development board with normal LED or addressable LED on-board (e.g., ESP32-S3-DevKitC, ESP32-C6-DevKitC etc.)
* A USB cable for Power supply and programming

See [Development Boards](https://www.espressif.com/en/products/devkits) for more information about it.

### Configure the Project

Open the project configuration menu (`idf.py menuconfig`).

In the `Example Configuration` menu:

* Select the LED type in the `Blink LED type` option.
  * Use `GPIO` for regular LED
  * Use `LED strip` for addressable LED
* If the LED type is `LED strip`, select the backend peripheral
  * `RMT` is only available for ESP targets with RMT peripheral supported
  * `SPI` is available for all ESP targets
* Set the GPIO number used for the signal in the `Blink GPIO number` option.
* Set the blinking period in the `Blink period in ms` option.

### Build and Flash

Run `idf.py -p PORT flash monitor` to build, flash and monitor the project.

(To exit the serial monitor, type ``Ctrl-]``.)

See the [Getting Started Guide](https://docs.espressif.com/projects/esp-idf/en/latest/get-started/index.html) for full steps to configure and use ESP-IDF to build projects.

## Example Output

As you run the example, you will see the LED blinking, according to the previously defined period. For the addressable LED, you can also change the LED color by setting the `led_strip_set_pixel(led_strip, 0, 16, 16, 16);` (LED Strip, Pixel Number, Red, Green, Blue) with values from 0 to 255 in the [source file](main/blink_example_main.c).

```text
I (315) example: Example configured to blink addressable LED!
I (325) example: Turning the LED OFF!
I (1325) example: Turning the LED ON!
I (2325) example: Turning the LED OFF!
I (3325) example: Turning the LED ON!
I (4325) example: Turning the LED OFF!
I (5325) example: Turning the LED ON!
I (6325) example: Turning the LED OFF!
I (7325) example: Turning the LED ON!
I (8325) example: Turning the LED OFF!
```

Note: The color order could be different according to the LED model.

The pixel number indicates the pixel position in the LED strip. For a single LED, use 0.

## Troubleshooting

* If the LED isn't blinking, check the GPIO or the LED type selection in the `Example Configuration` menu.

For any technical queries, please open an [issue](https://github.com/espressif/esp-idf/issues) on GitHub. We will get back to you soon.

# EE3 Autonomous Driving Car

This project implements an autonomous driving car using an ESP32-S3 microcontroller. The car features obstacle detection, heading control, and real-time debugging through WiFi logging.

## WiFi Logging System

The car includes a comprehensive WiFi-based logging system that allows real-time monitoring of the car's state and behavior. This is particularly useful for debugging and tuning the control systems.

### Network Configuration

1. **Current Network Setup**
   - ESP32 IP: 192.168.75.179
   - Laptop IP: 192.168.75.87
   - Both devices are on the same subnet (192.168.75.x)
   - WiFi Network: "Boenks"

2. **Checking Network Configuration**
   ```bash
   # On Windows laptop
   ipconfig
   
   # On ESP32 (via serial monitor)
   # Look for "Local IP:" in the output
   ```

3. **Verifying Connection**
   ```bash
   # On Windows laptop
   # Test if ESP32 is reachable
   ping 192.168.75.179
   
   # Check if UDP port is listening
   netstat -an | findstr "1234"
   ```

4. **Common Network Issues**
   - If devices are on different subnets:
     - Solution: Connect both devices to the same WiFi network
     - Check router settings
     - Try connecting to a different WiFi network
     - Verify DHCP settings
   - If devices are on same subnet but not communicating:
     - Check firewall settings
     - Verify UDP port 1234 is not blocked
     - Try disabling Windows Defender temporarily for testing

### Setup

1. Connect your laptop to the same WiFi network as the car (SSID: "Boenks")
2. Run the UDP listener on your laptop:
```bash
python udp_listener.py
```

### Common Connection Issues

1. **Socket Buffer Errors**
   - If you see "Failed to set receive buffer size: errno 109" or "Failed to set send buffer size: errno 109"
   - This is a non-critical warning and the logger should still function
   - The error occurs because the ESP32's socket buffer size is limited

2. **Connection Timeouts**
   - If the ESP32 repeatedly tries to recreate the socket
   - Check that:
     - The UDP listener is running on your laptop
     - No firewall is blocking UDP port 1234
     - Both devices can ping each other

3. **Permission Errors**
   - If you see "ClearCommError failed (PermissionError(13))"
   - This indicates a serial port access issue
   - Solution: Close any other programs using the serial port

### Verifying Connection

1. **On the Laptop**
   ```bash
   # Check your IP address
   ipconfig
   
   # Test UDP port
   netstat -an | findstr "1234"
   ```

2. **On the ESP32**
   - Watch the serial monitor for connection messages
   - Look for "Local IP:" and "Attempting to connect to:" messages
   - Verify the IP addresses match your network configuration

### Logged Information

The WiFi logger provides real-time information about:

1. **Heading Control**
   - Current heading angle
   - Heading error
   - PID control terms (Proportional, Integral, Derivative)
   - Left and right motor speeds

2. **Motor Control**
   - Motor duties (PWM values)
   - Speed settings
   - Braking status

3. **System Status**
   - Initialization sequence
   - WiFi connection status
   - Error conditions

### Example Log Output

```
[HH:MM:SS.mmm] Starting initialization sequence...
[HH:MM:SS.mmm] Initializing MPU...
[HH:MM:SS.mmm] Initializing encoders...
[HH:MM:SS.mmm] Initializing motors...
[HH:MM:SS.mmm] Initializing ultrasonic sensors...
[HH:MM:SS.mmm] Setting initial heading to 0 degrees...
[HH:MM:SS.mmm] Heading=0.00°, Error=0.00, I=0.00, D=0.00 => L=0.60, R=0.60
[HH:MM:SS.mmm] Front distance: 45.2 cm
```

### Debugging Tips

1. **Heading Control Issues**
   - Watch for large heading errors
   - Monitor the PID terms to identify if the system is over/under-correcting
   - Check if the integral term is growing too large (windup)

2. **Motor Control Issues**
   - Compare left and right motor speeds for asymmetry
   - Monitor duty cycles for proper PWM operation
   - Check for sudden changes in motor speeds

3. **Connection Issues**
   - Ensure both devices are on the same network
   - Check the UDP listener's IP address matches the car's configuration
   - Monitor for connection timeouts or dropped messages

### UDP Listener Features

The Python UDP listener provides:
- Real-time message display
- Message rate monitoring
- System information display
- Error handling and automatic reconnection
- Timestamp-based message ordering

## Hardware Requirements

* ESP32-S3 development board
* L293D motor driver
* MPU6050 gyroscope/accelerometer
* Ultrasonic sensors
* DC motors with encoders
* WiFi network for debugging

## Software Requirements

* ESP-IDF framework
* Python 3.x for UDP listener
* Required Python packages:
  - socket
  - time
  - platform

## Building and Flashing

1. Set the target to ESP32-S3:
```bash
idf.py set-target esp32s3
```

2. Configure the project:
```bash
idf.py menuconfig
```

3. Build and flash:
```bash
idf.py -p PORT flash monitor
```

## Troubleshooting

* If no logs are received, check:
  - WiFi connection status
  - IP address configuration
  - UDP port availability
  - Firewall settings

* For heading control issues:
  - Verify MPU6050 calibration
  - Check PID gains
  - Monitor heading drift

* For motor control issues:
  - Verify motor connections
  - Check PWM configuration
  - Monitor encoder readings
