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
#include <EEPROM.h>

// ===== Configuration =====
#define PPM_PIN 4                  // Pin where PPM signal is connected
#define NUM_CHANNELS 6             // Number of PPM channels to process
#define FRAME_GAP_THRESHOLD 5000   // Microseconds threshold to detect frame gap
#define SMOOTHING_FRAMES 5         // Number of frames to average for smoothing (higher = smoother but more latency)
#define NOISE_THRESHOLD 10         // Minimum change required to update value
#define EMA_ALPHA 0.1              // Smoothing factor (0 < EMA_ALPHA <= 1)
#define CHANGE_THRESHOLD 100        // Minimum change required to send update
#define MAX_NO_CHANGE_DELAY 250UL    // Maximum delay in milliseconds when no change is detected
#define BUTTON_PIN 5               // Pin where the button is connected
#define LED_PIN LED_BUILTIN        // Use the built-in LED for feedback
#define EEPROM_SIZE 64             // Define the size of EEPROM to use

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
int16_t lastSentValues[NUM_CHANNELS] = {0};          // Last sent values to BLE gamepad
unsigned long lastSendTime = 0;                      // Last time values were sent
unsigned long noChangeDelay = 2;                     // Current delay when no change is detected
unsigned long lastChangeTime = 0;                      // Last time a change was detected
unsigned long lastDebugTime = 0;  // Initialize lastDebugTime for debugging

bool isCalibrating = false;
uint16_t calibrationMin[NUM_CHANNELS];
uint16_t calibrationMax[NUM_CHANNELS];

// New variables for adaptive parameters
uint16_t optimalSmoothingFrames = SMOOTHING_FRAMES;
uint16_t optimalNoiseThreshold = NOISE_THRESHOLD;
float optimalEmaAlpha = EMA_ALPHA;

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
 * Function to initialize the PPM input pin and attach interrupt
 *
 * Sets the PPM pin as input and attaches an interrupt service routine
 * to handle rising edges of the PPM signal.
 */
void initializePPM() {
  pinMode(PPM_PIN, INPUT);
  attachInterrupt(PPM_PIN, ppmISR, RISING);
}

/**
 * Function to initialize the smoothing buffer with center values
 *
 * Fills the smoothing buffer with default center position values
 * for each channel to ensure smooth startup behavior.
 */
void initializeSmoothingBuffer() {
  for (int ch = 0; ch < NUM_CHANNELS; ch++) {
    for (int i = 0; i < SMOOTHING_FRAMES; i++) {
      ppmBuffer[ch][i] = 1500;  // Default center position
    }
    ppmValues[ch] = 1500;
  }
}

/**
 * Function to process PPM signals and update smoothed PPM values
 *
 * Copies raw PPM values from the interrupt context, applies noise
 * thresholding, and updates the smoothed PPM values using an
 * Exponential Moving Average (EMA) for each channel.
 */
void processPPMSignals() {
  uint16_t localRaw[NUM_CHANNELS];
  noInterrupts();
  memcpy((void*)localRaw, (const void*)rawPpmValues, sizeof(localRaw));
  interrupts();

  for (int ch = 0; ch < NUM_CHANNELS; ch++) {
    uint16_t val = localRaw[ch];
    if (val >= 800 && val <= 2200) {
      if (abs(val - ppmValues[ch]) > NOISE_THRESHOLD) {
        ppmValues[ch] = (EMA_ALPHA * val) + ((1 - EMA_ALPHA) * ppmValues[ch]);
      }
    }
  }
}

/**
 * Function to map PPM values to gamepad axes values
 *
 * Converts smoothed PPM values to percentage values based on channel
 * configuration, applies inversion if necessary, and maps them to
 * the appropriate gamepad axes range (linear or centered).
 *
 * @param mapped Array to store the mapped gamepad axes values.
 */
void mapPPMToGamepadAxes(int16_t* mapped) {
  for (int ch = 0; ch < NUM_CHANNELS; ch++) {
    int val = ppmValues[ch];
    int percent = map(val, channelSettings[ch].pulseMin, channelSettings[ch].pulseMax, 0, 100);
    percent = constrain(percent, 0, 100);
    if (channelSettings[ch].inverted) percent = 100 - percent;

    if (channelSettings[ch].mode == AXIS_CENTERED) {
      mapped[ch] = map(percent, 0, 100, -32767, 32767);
    } else {
      mapped[ch] = map(percent, 0, 100, 0, 32767);
    }
  }
}

/**
 * Function to determine if gamepad update is needed based on change threshold
 *
 * Compares current mapped gamepad axes values with the last sent
 * values to decide if a significant change has occurred that warrants
 * sending an update to the BLE gamepad.
 *
 * @param mapped Array of current mapped gamepad axes values.
 * @return True if an update is needed, false otherwise.
 */
bool shouldSendGamepadUpdate(int16_t* mapped) {
  for (int ch = 0; ch < NUM_CHANNELS; ch++) {
    if (abs(mapped[ch] - lastSentValues[ch]) > CHANGE_THRESHOLD) {
      return true;
    }
  }
  return false;
}

/**
 * Function to update BLE gamepad axes with new values
 *
 * Sends the current mapped gamepad axes values to the BLE gamepad,
 * updates the last sent values, and resets the no-change delay.
 *
 * @param mapped Array of current mapped gamepad axes values.
 */
void updateGamepadAxes(int16_t* mapped) {
  bleGamepad.setAxes(mapped[0], mapped[1], mapped[2], mapped[3], mapped[4], mapped[5]);
  for (int ch = 0; ch < NUM_CHANNELS; ch++) {
    lastSentValues[ch] = mapped[ch];
  }
  lastChangeTime = millis();
  noChangeDelay = 2;
}

/**
 * Function to print debug information to the serial monitor
 *
 * Outputs the current channel values, percentage, and mapped output
 * values to the serial monitor for debugging purposes. Also displays
 * BLE connection status and time since the last change.
 */
void printDebugInfo() {
  Serial.println("\n--- Channel Values ---");
  for (int ch = 0; ch < NUM_CHANNELS; ch++) {
    int val = ppmValues[ch];
    int percent = map(val, channelSettings[ch].pulseMin, channelSettings[ch].pulseMax, 0, 100);
    percent = constrain(percent, 0, 100);
    if (channelSettings[ch].inverted) percent = 100 - percent;

    Serial.printf("CH%d [%4d µs] %3d%% ", ch+1, val, percent);
    printBar(percent);

    if (channelSettings[ch].mode == AXIS_CENTERED) {
      int16_t output = map(percent, 0, 100, -32767, 32767);
      Serial.printf(" Out: %6d (CENTERED)", output);
    } else {
      int16_t output = map(percent, 0, 100, 0, 65534);
      Serial.printf(" Out: %6d (LINEAR)", output);
    }

    if (channelSettings[ch].inverted) {
      Serial.print(" [INV]");
    }

    Serial.println();
  }

  Serial.printf("--- BLE Status: Connected ---");
  Serial.printf("Time since last change: %lu ms\n", millis() - lastChangeTime);
}

/**
 * Function to start calibration
 *
 * Sets the calibration flag and turns on the LED to indicate calibration mode.
 * Initializes the calibration range with max and min possible values.
 */
void startCalibration() {
  isCalibrating = true;
  digitalWrite(LED_PIN, HIGH);  // Turn on LED to indicate calibration mode
  for (int ch = 0; ch < NUM_CHANNELS; ch++) {
    calibrationMin[ch] = 2200; // Start with max possible value
    calibrationMax[ch] = 800;  // Start with min possible value
  }
  Serial.println("🔧 Calibration started. Move all controls to their extremes.");
}

/**
 * Function to process calibration data
 *
 * Updates the calibration range with the current values from the smoothed PPM values.
 */
void processCalibration() {
  for (int ch = 0; ch < NUM_CHANNELS; ch++) {
    uint16_t val = ppmValues[ch];
    if (val < calibrationMin[ch]) calibrationMin[ch] = val;
    if (val > calibrationMax[ch]) calibrationMax[ch] = val;
  }
}

/**
 * Function to determine optimal parameters
 *
 * Calculates the mean and standard deviation of pulse widths during calibration.
 * Determines the optimal smoothing frames, noise threshold, and EMA alpha based on these statistics.
 */
void determineOptimalParameters() {
  // Calculate mean and standard deviation of pulse widths during calibration
  float mean[NUM_CHANNELS] = {0};
  float variance[NUM_CHANNELS] = {0};
  for (int ch = 0; ch < NUM_CHANNELS; ch++) {
    mean[ch] = (calibrationMin[ch] + calibrationMax[ch]) / 2.0;
    for (int i = 0; i < SMOOTHING_FRAMES; i++) {
      float diff = ppmBuffer[ch][i] - mean[ch];
      variance[ch] += diff * diff;
    }
    variance[ch] /= SMOOTHING_FRAMES;
  }

  // Calculate average standard deviation across all channels
  float avgStdDev = 0;
  for (int ch = 0; ch < NUM_CHANNELS; ch++) {
    avgStdDev += sqrt(variance[ch]);
  }
  avgStdDev /= NUM_CHANNELS;

  // Determine optimal smoothing frames based on standard deviation
  optimalSmoothingFrames = map(avgStdDev, 0, 100, 5, 15);  // Map std dev to a range of smoothing frames

  // Determine optimal noise threshold based on calibration range
  uint16_t minRange = 2200, maxRange = 800;
  for (int ch = 0; ch < NUM_CHANNELS; ch++) {
    if (calibrationMin[ch] < minRange) minRange = calibrationMin[ch];
    if (calibrationMax[ch] > maxRange) maxRange = calibrationMax[ch];
  }
  optimalNoiseThreshold = (maxRange - minRange) * 0.05;  // 5% of the range

  // Determine optimal EMA alpha based on standard deviation
  // Map std dev to a range of EMA alpha between 0.05 and 0.2
  optimalEmaAlpha = map(avgStdDev, 0, 100, 5, 20) / 100.0;
}

/**
 * Function to end calibration
 *
 * Resets the calibration flag and turns off the LED to indicate calibration end.
 * Outputs the calibration results and saves them to EEPROM.
 */
void endCalibration() {
  isCalibrating = false;
  digitalWrite(LED_PIN, LOW);  // Turn off LED to indicate calibration end

  Serial.println("Calibration Results:");

  for (int ch = 0; ch < NUM_CHANNELS; ch++) {
    uint16_t minVal = calibrationMin[ch];
    uint16_t maxVal = calibrationMax[ch];

    // Ensure min < max
    if (minVal >= maxVal) {
      Serial.printf("⚠️ Channel %d: Invalid calibration (min >= max). Resetting to default.\n", ch + 1);
      minVal = 1000;
      maxVal = 2000;
    }

    // Enforce minimum spread to avoid flat mapping
    if ((maxVal - minVal) < 100) {
      int center = (minVal + maxVal) / 2;
      minVal = center - 50;
      maxVal = center + 50;
      Serial.printf("⚠️ Channel %d: Narrow range. Adjusted to [%d, %d]\n", ch + 1, minVal, maxVal);
    }

    if ((maxVal - minVal) < 150) {
      Serial.printf("⚠️ Channel %d: Suggest recalibration. Limited stick movement.\n", ch + 1);
    }

    channelSettings[ch].pulseMin = minVal;
    channelSettings[ch].pulseMax = maxVal;

    Serial.printf("CH%d: pulseMin = %d, pulseMax = %d, range = %d µs\n",
                  ch + 1, minVal, maxVal, maxVal - minVal);
  }

  determineOptimalParameters();  // Optional fine-tuning

  Serial.printf("Optimal Smoothing Frames: %d\n", optimalSmoothingFrames);
  Serial.printf("Optimal Noise Threshold: %d\n", optimalNoiseThreshold);
  Serial.printf("Optimal EMA Alpha: %.2f\n", optimalEmaAlpha);

  // Save calibration settings to EEPROM
  EEPROM.write(1, optimalSmoothingFrames);
  EEPROM.write(2, optimalNoiseThreshold);
  EEPROM.write(3, (int)(optimalEmaAlpha * 100));
  EEPROM.write(0, 1);  // Mark calibration as valid

  for (int ch = 0; ch < NUM_CHANNELS; ch++) {
    EEPROM.write(4 + ch * 2, calibrationMin[ch] & 0xFF); // Save lower byte
    EEPROM.write(5 + ch * 2, (calibrationMin[ch] >> 8) & 0xFF); // Save upper byte
    EEPROM.write(16 + ch * 2, calibrationMax[ch] & 0xFF); // Save lower byte
    EEPROM.write(17 + ch * 2, (calibrationMax[ch] >> 8) & 0xFF); // Save upper byte
  }
  EEPROM.commit();

  Serial.println("✅ Calibration complete. Settings saved.");
}

/**
 * Function to setup the initial configuration
 *
 * Initializes the serial communication, button pin, LED pin, EEPROM, and BLE gamepad.
 * Checks if calibration data exists in EEPROM and loads it if available.
 */
void setup() {
  Serial.begin(115200);
  pinMode(BUTTON_PIN, INPUT_PULLUP);  // Configure button pin with internal pull-up
  pinMode(LED_PIN, OUTPUT);           // Configure LED pin as output
  digitalWrite(LED_PIN, LOW);         // Ensure LED is off initially

  EEPROM.begin(EEPROM_SIZE);  // Initialize EEPROM with the defined size

  // Check if calibration data exists
  if (EEPROM.read(0) == 1) {
    // Load calibration data
    optimalSmoothingFrames = EEPROM.read(1);
    optimalNoiseThreshold = EEPROM.read(2);
    optimalEmaAlpha = EEPROM.read(3) / 100.0;

    for (int ch = 0; ch < NUM_CHANNELS; ch++) {
      calibrationMin[ch] = EEPROM.read(4 + ch * 2) | (EEPROM.read(5 + ch * 2) << 8);
      calibrationMax[ch] = EEPROM.read(16 + ch * 2) | (EEPROM.read(17 + ch * 2) << 8);
    }

    Serial.println("Calibration data loaded.");
    Serial.println("Loaded Configuration:");
    Serial.printf("Optimal Smoothing Frames: %d\n", optimalSmoothingFrames);
    Serial.printf("Optimal Noise Threshold: %d\n", optimalNoiseThreshold);
    Serial.printf("Optimal EMA Alpha: %.2f\n", optimalEmaAlpha);

    for (int ch = 0; ch < NUM_CHANNELS; ch++) {
      Serial.printf("Channel %d: Min = %d, Max = %d\n", ch + 1, calibrationMin[ch], calibrationMax[ch]);
    }
  } else {
    Serial.println("No calibration data found. Using default settings.");
  }

  initializePPM();
  bleGamepad.begin();
  Serial.println("✅ BLE RC Controller + PPM Debug Started");
  initializeSmoothingBuffer();
}

/**
 * Main loop function
 *
 * Continuously checks for button press to start or end calibration.
 * If a BLE connection is established, processes PPM signals and updates gamepad axes.
 */
void loop() {
  if (digitalRead(BUTTON_PIN) == LOW) {
    if (!isCalibrating) {
      startCalibration();
    } else {
      endCalibration();
    }
    delay(1000);  // Debounce delay
  }

  if (bleGamepad.isConnected()) {
    processPPMSignals();
    if (isCalibrating) {
      processCalibration();
    } else {
      int16_t mapped[NUM_CHANNELS];
      mapPPMToGamepadAxes(mapped);

      if (shouldSendGamepadUpdate(mapped)) {
        updateGamepadAxes(mapped);
      } else {
        noChangeDelay = min(noChangeDelay + 2, MAX_NO_CHANGE_DELAY);
      }

      if (Serial && millis() - lastDebugTime > 50) {
        printDebugInfo();
        lastDebugTime = millis();
      }
      delay(noChangeDelay);
    }
  } else {
    Serial.println("🔌 Waiting for BLE connection...");
    delay(1000);
  }
}
