# RC Controller PPM to BLE Gamepad

This project converts PPM signals from a RC Controller into a Bluetooth Low Energy (BLE) gamepad using an ESP32-C3. The gamepad can be used to control various applications that support BLE gamepad input.

## Repository

- **GitHub URL:** [PPM2BuetoothController](https://github.com/mevenG/PPM2BuetoothController)
- **Description:** A very simple esp32-C3 converter from PPM (which you can get on the trainer port of your controller to Bluetooth controller for your computer.

## Features

- Converts up to 6 PPM channels to BLE gamepad axes
- Supports linear and centered axis modes
- Configurable channel inversion
- Smoothing of PPM signals for stable input
- BLE connection status feedback
- Built-in debug visualization when serial monitor is connected
- **Calibration data saving and loading**: Automatically saves calibration data to EEPROM and loads it on startup.

## Hardware Requirements

- ESP32-C3 development board
- Any controller with PPM output (I used a FlySky FS-i6 for testing)
- USB cable for programming and power

## Software Requirements

- Arduino IDE with ESP32 board support
- [BleGamepad](https://github.com/lemmingDev/ESP32-BLE-Gamepad) library
- A drone sim, or any tool to see Bluetooth controller inputs like [HTML5 Gamepad Tester](https://greggman.github.io/html5-gamepad-test/)

## Installation

1. **Set up Arduino IDE:**
   - Install the ESP32 board support in Arduino IDE.
   - Install the BleGamepad library via the Library Manager.

2. **Clone the repository:**
   ```bash
   git clone https://github.com/mevenG/PPM2BuetoothController.git
   cd PPM2BuetoothController
   ```

3. **Open the project:**
   - Open `main.ino` in Arduino IDE.

4. **Configure the project:**
   - Adjust `channelSettings` in `main.ino` to match your PPM channel configuration. Users can modify the parameters to adapt it to almost any controller with PPM output.

   Example configuration:
   ```cpp
   #define PPM_PIN 4                // PPM input pin
   #define NUM_CHANNELS 6           // Total PPM channels
   #define FRAME_GAP_THRESHOLD 5000 // Frame gap threshold in microseconds
   #define SMOOTHING_FRAMES 5       // Number of frames for smoothing

   ChannelConfig channelSettings[NUM_CHANNELS] = { // Each channel has to be referenced here
     {1000, 2000, false, AXIS_LINEAR}, // CH1
     {1000, 2000, false, AXIS_LINEAR}, // CH2
     {1000, 2000, false, AXIS_LINEAR}, // CH3
     {1000, 2000, false, AXIS_LINEAR}, // CH4
     {1000, 2000, false, AXIS_LINEAR}, // CH5
     {1000, 2000, false, AXIS_LINEAR}  // CH6
     // {pulseMin, pulseMax, inverted, mode}
   };
   ```
   - `pulseMin` and `pulseMax` define the expected PPM pulse width range.
   - `inverted` can be set to `true` to invert the axis.
   - `mode` can be `AXIS_LINEAR` or `AXIS_CENTERED`.

5. **Upload the code:**
   - Connect your ESP32-C3 to your computer.
   - Select the correct board and port in Arduino IDE.
   - Upload the code to the ESP32-C3.

## Usage

1. **Connect the PPM output:**
   - Connect the PPM output from your receiver to the specified PPM pin on the ESP32-C3.

2. **Power the ESP32-C3:**
   - Use a USB cable to power the ESP32-C3.

3. **Pair with a device:**
   - Search for "BLE-RCController" on your BLE-enabled device and pair with it.

4. **Start using the gamepad:**
   - Once connected, the gamepad will send axis data based on the PPM input.

## Debug Visualization

The project includes a built-in debug visualization that automatically activates when a serial monitor is connected:

1. **Activating debug mode:**
   - Connect the ESP32-C3 to your computer via USB
   - Open the Arduino IDE Serial Monitor (set to 115200 baud)
   - The debug visualization will automatically start

2. **Debug display features:**
   - Displays a real-time chart of all channel values every 50ms
   - Shows raw PPM values in microseconds and mapped percentages
   - Visual bar graphs represent the position of each channel
   - Displays actual output values sent to the gamepad
   - Indicates mode (LINEAR/CENTERED) and inversion status for each channel
   - Shows BLE connection status
   - **Displays loaded configuration**: Prints the loaded calibration and optimal parameters at startup.

3. **Example debug output:**
   ```
   --- BLE Status: Connected ---
   
   --- Channel Values ---
   CH1 [1500 µs]  50% [███████████████               ] Out:  16383 (LINEAR)
   CH2 [1750 µs]  75% [█████████████████████         ] Out:  24575 (LINEAR)
   CH3 [1250 µs]  25% [███████                       ] Out:   8191 (LINEAR)
   CH4 [2000 µs] 100% [██████████████████████████████] Out:  32767 (LINEAR)
   CH5 [1000 µs]   0% [                              ] Out:      0 (LINEAR)
   CH6 [1500 µs]  50% [███████████████               ] Out:      0 (CENTERED)
   ```

## Troubleshooting

- **No BLE connection:**
  - Ensure the ESP32-C3 is powered and the BLE is enabled on your device.
  - Check the serial monitor for connection status messages.

- **Unexpected axis behavior:**
  - Verify the PPM channel configuration and adjust `channelSettings` as needed.
  - Use the debug visualization to see actual channel values and outputs.

- **Poor responsiveness:**
  - Adjust the `SMOOTHING_FRAMES` value. Lower values give more responsive but potentially jittery control.

## Contributing

Contributions are welcome! Please fork the repository and submit a pull request for any improvements or bug fixes.

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

## Acknowledgments

- [ESP32-BLE-Gamepad](https://github.com/lemmingDev/ESP32-BLE-Gamepad) for the BLE gamepad library.

## Calibration

To calibrate the PPM channels, follow these steps:

1. **Calibration Button Pin**:
   - The calibration button is connected to pin `BUTTON_PIN` (default is pin 5).

2. **Calibration Process**:
   - **Start Calibration**: Press the calibration button to enter calibration mode. The built-in LED will turn on to indicate that calibration mode is active.
   - **Move Controls**: Move all controls to their extremes to record the minimum and maximum pulse widths for each channel. Do it slowly, you have all of your time.
   - **End Calibration**: Press the button again to end calibration. The LED will turn off, the smoothing, noise threshold and EMA calibration will take half a second. Then the calibration data will be saved to EEPROM.
   - **Confirmation**: The calibration results will be displayed in the serial monitor, showing the min and max values for each channel.