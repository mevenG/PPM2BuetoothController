#include <BleGamepad.h>

#define PPM_PIN 4
#define NUM_CHANNELS 6
#define FRAME_GAP_THRESHOLD 5000
#define SMOOTHING_FRAMES 5

enum AxisMode {
  AXIS_LINEAR,
  AXIS_CENTERED
};

struct ChannelConfig {
  uint16_t pulseMin;
  uint16_t pulseMax;
  bool inverted;
  AxisMode mode;
};

ChannelConfig channelSettings[NUM_CHANNELS] = {
  {1000, 2000, false, AXIS_LINEAR}, // CH1
  {1000, 2000, false, AXIS_LINEAR}, // CH2
  {1000, 2000, false, AXIS_LINEAR}, // CH3
  {1000, 2000, false, AXIS_LINEAR}, // CH4
  {1000, 2000, false, AXIS_LINEAR}, // CH5
  {1000, 2000, false, AXIS_LINEAR}  // CH6
};

volatile uint16_t rawPpmValues[NUM_CHANNELS] = {0};
volatile unsigned long lastRise = 0;
volatile uint8_t currentChannel = 0;

uint16_t ppmValues[NUM_CHANNELS] = {0}; 
uint16_t ppmBuffer[NUM_CHANNELS][SMOOTHING_FRAMES] = {0};
uint8_t bufferIndex = 0;

BleGamepad bleGamepad("FlySky-BLE", "ESP32-C3", 100);

// Function to draw visualization bar
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

void IRAM_ATTR ppmISR() {
  unsigned long now = micros();
  unsigned long pulseWidth = now - lastRise;
  lastRise = now;

  if (pulseWidth > FRAME_GAP_THRESHOLD) {
    currentChannel = 0;
  }

  if (currentChannel < NUM_CHANNELS && pulseWidth >= 800 && pulseWidth <= 2200) {
    rawPpmValues[currentChannel++] = pulseWidth;
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(PPM_PIN, INPUT);
  attachInterrupt(PPM_PIN, ppmISR, RISING);

  bleGamepad.begin();
  Serial.println("✅ BLE Gamepad + PPM Debug Started");

  for (int ch = 0; ch < NUM_CHANNELS; ch++) {
    for (int i = 0; i < SMOOTHING_FRAMES; i++) {
      ppmBuffer[ch][i] = 1500;
    }
    ppmValues[ch] = 1500;
  }
}

void loop() {
  if (bleGamepad.isConnected()) {
    uint16_t localRaw[NUM_CHANNELS];

    noInterrupts();
    memcpy((void*)localRaw, (const void*)rawPpmValues, sizeof(localRaw));
    interrupts();

    for (int ch = 0; ch < NUM_CHANNELS; ch++) {
      uint16_t val = localRaw[ch];

      if (val >= 800 && val <= 2200) {
        ppmBuffer[ch][bufferIndex] = val;
      }

      uint32_t sum = 0;
      for (int i = 0; i < SMOOTHING_FRAMES; i++) {
        sum += ppmBuffer[ch][i];
      }
      ppmValues[ch] = sum / SMOOTHING_FRAMES;
    }

    bufferIndex = (bufferIndex + 1) % SMOOTHING_FRAMES;

    int16_t mapped[NUM_CHANNELS];
    for (int ch = 0; ch < NUM_CHANNELS; ch++) {
      int val = ppmValues[ch];
      int percent = map(
        val,
        channelSettings[ch].pulseMin,
        channelSettings[ch].pulseMax,
        0, 100
      );

      percent = constrain(percent, 0, 100);
      if (channelSettings[ch].inverted) percent = 100 - percent;

      if (channelSettings[ch].mode == AXIS_CENTERED) {
        mapped[ch] = map(percent, 0, 100, -32767, 32767);
      } else {
        mapped[ch] = map(percent, 0, 100, 0, 32767);
      }
    }

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
    Serial.println("🔌 Waiting for BLE connection...");
    delay(1000); // Longer delay when not connected to reduce log spam
  }

  // Short delay to prevent overwhelming the system
  delay(2);
}
