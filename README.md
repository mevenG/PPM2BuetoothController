# FlySky FS-i6 PPM to BLE Gamepad

This project converts PPM signals from a FlySky FS-i6 transmitter into a Bluetooth Low Energy (BLE) gamepad using an ESP32-C3. The gamepad can be used to control various applications that support BLE gamepad input.

## Repository

- **GitHub URL:** [PPM2BuetoothController](https://github.com/mevenG/PPM2BuetoothController)
- **Description:** A very simple esp32-C3 converter from PPM (which you can get on the trainer port of your controller to Bluetooth controller for your computer.

## Features

- Converts up to 6 PPM channels to BLE gamepad axes
- Supports linear and centered axis modes
- Configurable channel inversion
- Smoothing of PPM signals for stable input
- BLE connection status feedback

## Hardware Requirements

- ESP32-C3 development board
- Any controller with PPM output (I used a FlySky fs-i6 for testing)
- USB cable for programming and power

## Software Requirements

- Arduino IDE with ESP32 board support
- [BleGamepad](https://github.com/lemmingDev/ESP32-BLE-Gamepad) library
- A drone sim, or any tool to see Bluetooth controller inputs like [Gamepad Tester](https://gamepadtester.net/)

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
   - Search for "FlySky-BLE" on your BLE-enabled device and pair with it.

4. **Start using the gamepad:**
   - Once connected, the gamepad will send axis data based on the PPM input.

## Troubleshooting

- **No BLE connection:**
  - Ensure the ESP32-C3 is powered and the BLE is enabled on your device.
  - Check the serial monitor for connection status messages.

- **Unexpected axis behavior:**
  - Verify the PPM channel configuration and adjust `channelSettings` as needed.

## Contributing

Contributions are welcome! Please fork the repository and submit a pull request for any improvements or bug fixes.

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

## Acknowledgments

- [ESP32-BLE-Gamepad](https://github.com/lemmingDev/ESP32-BLE-Gamepad) for the BLE gamepad library.