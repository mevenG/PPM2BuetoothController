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
- FlySky FS-i6 transmitter
- PPM receiver compatible with FlySky FS-i6
- USB cable for programming and power

## Software Requirements

- Arduino IDE with ESP32 board support
- [BleGamepad](https://github.com/lemmingDev/ESP32-BLE-Gamepad) library

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