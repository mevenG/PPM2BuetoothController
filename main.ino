/**
 * PPM to BLE Gamepad Converter
 * 
 * This sketch converts PPM signals from RC controllers into a Bluetooth Low Energy gamepad.
 * It reads PPM signals on a specified pin, processes them with smoothing,
 * and transmits them as BLE HID gamepad axes.
 * 
 * Hardware: ESP32-C3
 * Author: mevenG
 * Repository: https://github.com/mevenG/PPM2BuetoothController
 */

#include <BleGamepad.h>

// ===== Configuration =====
#define PPM_PIN 4                // Pin where PPM signal is connected
#define NUM_CHANNELS 6           // Number of PPM channels to process
#define FRAME_GAP_THRESHOLD 5000 // Microseconds threshold to detect frame gap
#define SMOOTHING_FRAMES 5       // Number of frames to average for smoothing (higher = smoother but more latency)

// Define axis modes
enum AxisMode {
  AXIS_LINEAR,    // 0-100% maps to 0-32767
  AXIS_CENTERED   // 0-100% maps to -32767 to 32767 (centered joystick)
};

// Configuration structure for each channel
struct ChannelConfig {
  uint16_t pulseMin;  // Minimum pulse width in microseconds (typically ~1000)
  uint16_t pulseMax;  // Maximum pulse width in microseconds (typically ~2000)
  bool inverted;      // Whether to invert the axis (false = normal, true = inverted)
  AxisMode mode;      // Axis mode (LINEAR or CENTERED)
};

// Channel configuration - customize these values for your controller
ChannelConfig channelSettings[NUM_CHANNELS] = {
  {1000, 2000, false, AXIS_LINEAR}, // CH1
  {1000, 2000, false, AXIS_LINEAR}, // CH2
  {1000, 2000, false, AXIS_LINEAR}, // CH3
  {1000, 2000, false, AXIS_LINEAR}, // CH4
  {1000, 2000, false, AXIS_LINEAR}, // CH5
  {1000, 2000, false, AXIS_LINEAR}  // CH6
};

// ===== Global Variables =====
// Values updated by the interrupt
volatile uint16_t rawPpmValues[NUM_CHANNELS] = {0};  // Raw PPM pulse widths
volatile unsigned long lastRise = 0;                 // Timestamp of last rising edge
volatile uint8_t currentChannel = 0;                 // Current channel being processed

// Values used by the main loop
uint16_t ppmValues[NUM_CHANNELS] = {0};              // Smoothed PPM values
uint16_t ppmBuffer[NUM_CHANNELS][SMOOTHING_FRAMES] = {0}; // Buffer for smoothing
uint8_t bufferIndex = 0;                             // Current position in the smoothing buffer

// Create BLE gamepad instance
BleGamepad bleGamepad("BLE-RCController", "ESP32-C3", 100);

/**
 * Draws a visualization bar for the serial monitor
 * 
 * @param percent The percentage (0-100) to visualize
 * @param barWidth The width of the bar in characters
 */
void printBar(int percent, int barWidth = 30) {
  String bar = "[";
  int filledWidth = map(percent, 0, 100, 0, barWidth);
  
  for (int i = 0; i < barWidth; i++) {
    if (i < filledWidth) {
      bar += "█";
    } else {
      bar += " ";
    }
  }
  bar += "]";
  Serial.print(bar);
}

/**
 * Interrupt Service Routine for PPM signal processing
 * Triggered on rising edge of PPM signal
 */
void IRAM_ATTR ppmISR() {
  unsigned long now = micros();
  unsigned long pulseWidth = now - lastRise;
  lastRise = now;

  // Check for frame gap (space between frames)
  if (pulseWidth > FRAME_GAP_THRESHOLD) {
    currentChannel = 0;  // Reset to first channel when frame gap detected
  }

  // Store pulse width if it's within valid range
  if (currentChannel < NUM_CHANNELS && pulseWidth >= 800 && pulseWidth <= 2200) {
    rawPpmValues[currentChannel++] = pulseWidth;
  }
}

/**
 * Setup function - runs once at startup
 */
void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  
  // Set up PPM input pin with interrupt
  pinMode(PPM_PIN, INPUT);
  attachInterrupt(PPM_PIN, ppmISR, RISING);

  // Initialize BLE gamepad
  bleGamepad.begin();
  Serial.println("✅ BLE RC Controller + PPM Debug Started");

  // Initialize smoothing buffer with center values
  for (int ch = 0; ch < NUM_CHANNELS; ch++) {
    for (int i = 0; i < SMOOTHING_FRAMES; i++) {
      ppmBuffer[ch][i] = 1500;  // Default center position
    }
    ppmValues[ch] = 1500;
  }
}

/**
 * Main loop - runs continuously
 */
void loop() {
  if (bleGamepad.isConnected()) {
    // Copy volatile data to local array to avoid problems with interrupts
    uint16_t localRaw[NUM_CHANNELS];
    noInterrupts();
    memcpy((void*)localRaw, (const void*)rawPpmValues, sizeof(localRaw));
    interrupts();

    // Process each channel
    for (int ch = 0; ch < NUM_CHANNELS; ch++) {
      uint16_t val = localRaw[ch];

      // Only add valid values to the smoothing buffer
      if (val >= 800 && val <= 2200) {
        ppmBuffer[ch][bufferIndex] = val;
      }

      // Calculate average (smoothed) value
      uint32_t sum = 0;
      for (int i = 0; i < SMOOTHING_FRAMES; i++) {
        sum += ppmBuffer[ch][i];
      }
      ppmValues[ch] = sum / SMOOTHING_FRAMES;
    }

    // Move to next position in the smoothing buffer
    bufferIndex = (bufferIndex + 1) % SMOOTHING_FRAMES;

    // Map PPM values to gamepad axes values
    int16_t mapped[NUM_CHANNELS];
    for (int ch = 0; ch < NUM_CHANNELS; ch++) {
      int val = ppmValues[ch];
      
      // Map to 0-100% range based on min/max settings
      int percent = map(
        val,
        channelSettings[ch].pulseMin,
        channelSettings[ch].pulseMax,
        0, 100
      );

      // Constrain to valid range and apply inversion if configured
      percent = constrain(percent, 0, 100);
      if (channelSettings[ch].inverted) percent = 100 - percent;

      // Map percentage to appropriate output range based on axis mode
      if (channelSettings[ch].mode == AXIS_CENTERED) {
        mapped[ch] = map(percent, 0, 100, -32767, 32767);
      } else {
        mapped[ch] = map(percent, 0, 100, 0, 32767);
      }
    }

    // Send all axes to the BLE gamepad
    bleGamepad.setAxes(
      mapped[0],
      mapped[1],
      mapped[2],
      mapped[3],
      mapped[4],
      mapped[5]
    );

    // Optional debug visualization when serial monitor is connected
    static unsigned long lastDebugTime = 0;
    if (Serial && millis() - lastDebugTime > 50) {  // Only debug if Serial is connected and every 50ms
      Serial.println("\n--- Channel Values ---");
      for (int ch = 0; ch < NUM_CHANNELS; ch++) {
        int val = ppmValues[ch];
        int percent = map(val, channelSettings[ch].pulseMin, channelSettings[ch].pulseMax, 0, 100);
        percent = constrain(percent, 0, 100);
        if (channelSettings[ch].inverted) percent = 100 - percent;
        
        // Display channel info with visualization bar
        Serial.printf("CH%d [%4d µs] %3d%% ", ch+1, val, percent);
        printBar(percent);
        
        // Show output value and mode
        if (channelSettings[ch].mode == AXIS_CENTERED) {
          int16_t output = map(percent, 0, 100, -32767, 32767);
          Serial.printf(" Out: %6d (CENTERED)", output);
        } else {
          int16_t output = map(percent, 0, 100, 0, 32767);
          Serial.printf(" Out: %6d (LINEAR)", output);
        }
        
        if (channelSettings[ch].inverted) {
          Serial.print(" [INV]");
        }
        
        Serial.println();
      }
      
      // Show BLE status
      Serial.println("--- BLE Status: Connected ---");
      lastDebugTime = millis();
    }

  } else {
    // Not connected to BLE - waiting for connection
    Serial.println("🔌 Waiting for BLE connection...");
    delay(1000); // Longer delay when not connected to reduce log spam
  }

  // Short delay to prevent overwhelming the system
  delay(2);
}
