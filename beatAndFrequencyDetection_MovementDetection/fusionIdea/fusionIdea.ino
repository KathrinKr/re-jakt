/*
  This project powers an interactive jacket called RE:Jakt.
  The goal is to create an immersive and playful experience for the wearer 
  by transforming music into dynamic LED colors 
  that change in sync with the beat, 
  while motion‑detection triggers additional light patterns.

  Hardware used:  
  • ESP32: main controller
  • INMP441: digital microphone for audio input
  • MPU6050: motion sensor for gesture and movement detection
  • WS2812B: LEDs for visual feedback

  Status:
  This project is a concept and wip
  
*/

#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include <arduinoFFT.h>
#include <driver/i2s.h>
#include <Adafruit_NeoPixel.h>

// =========================
// --- KONFIG: AUDIO / FFT
// =========================

#define USE_MICROPHONE 1
#define SAMPLE_RATE    44100
#define WINDOW_SIZE    1024

#define I2S_SD_PIN   32
#define I2S_WS_PIN   15
#define I2S_SCK_PIN  14

#define ENERGY_HISTORY 16
#define FREQ_HISTORY   3
#define MAX_BEATS      200

float energyHistory[ENERGY_HISTORY];
int   energyIndex = 0;

float freqHistory[FREQ_HISTORY];
int   freqIndex = 0;

float beatsFiltered[MAX_BEATS];
int   beatCount = 0;

const float minimalInterval = 0.25f;
float lastBeat = -minimalInterval;

ArduinoFFT<float> FFT = ArduinoFFT<float>();
float vRealToneDataInWindow[WINDOW_SIZE];
float vTempToneDataInWindow[WINDOW_SIZE];

float samplerate = SAMPLE_RATE;
unsigned long startMillis = 0;

struct AudioState {
  bool beat;
  float pitch;
  uint8_t r, g, b;
  String colorName;
};

AudioState audioState;

// =========================
// --- KONFIG: LEDS
// =========================

#define LED_PIN   18
#define NUM_LEDS  300

Adafruit_NeoPixel ledStrip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// Effektzustände
float pulsePhase = 0;
int waveOffset = 0;
int laserPos = 0;
int blockOffset = 0;
int twistPhase = 0;
int armLaserPosLeft = 0;
int armLaserPosRight = 0;

int effectTimer = 0;
int effectType = 0;
// 0 = none
// 1 = armFlash
// 2 = horizontal
// 3 = vertical
// 4 = twist
// 5 = circle


// ===============================
// --- KONFIG: MOTION / MPU6050
// ===============================

MPU6050 mpuArm;

// Threshold
#define FAST_ON_THRESHOLD   10000
#define FAST_OFF_THRESHOLD  30000
#define STILL_THRESHOLD     2000

#define ARM_UP_THRESHOLD    10
#define ARM_DOWN_THRESHOLD  6

// states
bool armUpRight = false;

bool isFastMove = false;

int intensityMovement = 0;

// =========================
// --- HILFSFUNKTIONEN: I2S
// =========================

void i2sSetup() {
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags = 0,
    .dma_buf_count = 4,
    .dma_buf_len = 1024,
    .use_apll = false
  };
  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK_PIN,
    .ws_io_num = I2S_WS_PIN,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_SD_PIN
  };
  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);
}

void readAudioData(float* buffer, int windowSize) {
  int32_t samples[windowSize];
  size_t bytesRead;
  i2s_read(I2S_NUM_0, &samples, sizeof(samples), &bytesRead, portMAX_DELAY);
  for (int i = 0; i < windowSize; i++) {
    buffer[i] = (samples[i] / 2147483648.0f) * 25.0f;
  }
}

// =========================
// --- AUDIO
// =========================

float computeEnergy(const float *buffer, int len) {
  float s = 0.0;
  for (int i = 0; i < len; i++) s += buffer[i] * buffer[i];
  return s;
}

float smoothFrequency(float newFreq) {
  freqHistory[freqIndex] = newFreq;
  freqIndex = (freqIndex + 1) % FREQ_HISTORY;

  float sum = 0;
  for (int i = 0; i < FREQ_HISTORY; i++) sum += freqHistory[i];
  return sum / FREQ_HISTORY;
}

float detectPitchFFT(float* magnitudes, int windowSize, float sampleRate) {
  int lowIndex  = (int)(80.0  * windowSize / sampleRate);
  int highIndex = (int)(3000.0 * windowSize / sampleRate);

  int   maxIndex = lowIndex;
  float maxVal   = 0;

  for (int i = lowIndex; i <= highIndex; i++) {
    if (magnitudes[i] > maxVal) {
      maxVal   = magnitudes[i];
      maxIndex = i;
    }
  }

  return (float)maxIndex * sampleRate / windowSize;
}

String mapFrequencyToColorName(float f) {

  if (f < 150)   return "Red";
  else if (f < 300)   return "Bright Red";
  else if (f < 450)   return "Deep Orange";
  else if (f < 600)   return "Orange";
  else if (f < 750)   return "Yellow";
  else if (f < 900)   return "Lime";
  else if (f < 1050)  return "Green";
  else if (f < 1200)  return "Turquoise";
  else if (f < 1350)  return "Purple";       
  else if (f < 1500)  return "Deep Purple";  
  else if (f < 1650)  return "Indigo";      
  else if (f < 1800)  return "Magenta";    
  else if (f < 1950)  return "Pink";    
  else if (f < 2100)  return "Brown";
  else if (f < 2250)  return "Red";   
  else if (f < 2400)  return "Green"; 
  else if (f < 2550)  return "Yellow";  
  else if (f < 2700)  return "Purple";     
  else                return "Bright Red";  
}




void colorNameToRGB(const String &name, uint8_t &r, uint8_t &g, uint8_t &b) {
  if (name == "Deep Purple")      { r=40;  g=0;   b=60;  }
  else if (name == "Purple")      { r=70;  g=0;   b=90;  }
  else if (name == "Indigo")      { r=40;  g=0;   b=120; }
  else if (name == "Dark Blue")   { r=0;   g=0;   b=120; }
  else if (name == "Blue")        { r=0;   g=60;  b=180; }
  else if (name == "Sky Blue")    { r=0;   g=130; b=230; }
  else if (name == "Turquoise")   { r=0;   g=200; b=180; }
  else if (name == "Green")       { r=0;   g=255; b=80;  }
  else if (name == "Lime")        { r=160; g=255; b=0;   }
  else if (name == "Yellow")      { r=255; g=230; b=0;   }
  else if (name == "Orange")      { r=255; g=150; b=0;   }
  else if (name == "Deep Orange") { r=255; g=100; b=0;   }
  else if (name == "Red")         { r=255; g=60;  b=60;  }
  else if (name == "Bright Red")  { r=255; g=120; b=120; }
  else if (name == "Brown")       { r=60;  g=30;  b=10;  }
  else                            { r=120; g=120; b=120; }
}

// =========================
// --- LED 
// =========================

void setColorAll(uint8_t r, uint8_t g, uint8_t b) {
  for (int i = 0; i < NUM_LEDS; i++)
    ledStrip.setPixelColor(i, ledStrip.Color(r, g, b));
}

void boostBrightnessOnce() {
  for (int i = 0; i < NUM_LEDS; i++) {
    uint32_t c = ledStrip.getPixelColor(i);
    uint8_t r = (c >> 16) & 0xFF;
    uint8_t g = (c >> 8) & 0xFF;
    uint8_t b = c & 0xFF;
    b = min(255, (int)b + 80);
    ledStrip.setPixelColor(i, ledStrip.Color(r, g, b));
  }
}

uint32_t colorWheel(byte pos) {
  if (pos < 85) {
    return ledStrip.Color(pos * 3, 255 - pos * 3, 0);
  } else if (pos < 170) {
    pos -= 85;
    return ledStrip.Color(255 - pos * 3, 0, pos * 3);
  } else {
    pos -= 170;
    return ledStrip.Color(0, pos * 3, 255 - pos * 3);
  }
}

byte fastRainbowOffset = 0;

void stepFastRainbow() {
  fastRainbowOffset += 15; // speed

  for (int i = 0; i < NUM_LEDS; i++) {
    byte c = fastRainbowOffset + (i * 5);
    ledStrip.setPixelColor(i, colorWheel(c));
  }
}

void rainbowFlashStrong() {
  static byte offset = 0;
  offset += 40;

  for (int i = 0; i < NUM_LEDS; i++) {
    byte c = offset + (i * 3);

    uint32_t col;
    if (c < 85) col = ledStrip.Color(c * 3, 255 - c * 3, 0);
    else if (c < 170) { c -= 85; col = ledStrip.Color(255 - c * 3, 0, c * 3); }
    else { c -= 170; col = ledStrip.Color(0, c * 3, 255 - c * 3); }

    ledStrip.setPixelColor(i, col);
  }
}


void stepSlowWave() {
  static float t1 = 0;
  static float t2 = 0;

  t1 += 0.015;
  t2 += 0.008;

  for (int i = 0; i < NUM_LEDS; i++) {
    float w1 = sin((i * 0.05) + t1);
    float w2 = sin((i * 0.11) + t2);

    float mix = (w1 * 0.6) + (w2 * 0.4);
    float brightness = 0.2 + (mix + 1.0) * 0.4;

    uint8_t r = audioState.r * brightness;
    uint8_t g = audioState.g * brightness;
    uint8_t b = audioState.b * brightness;

    ledStrip.setPixelColor(i, ledStrip.Color(r, g, b));
  }
}




void stepFastFlash() {
  static int t = 0;
  t += 20;

  for (int i = 0; i < NUM_LEDS; i++) {
    float flicker = (sin((i * 0.3) + t * 0.3) + 1.0) * 0.5; 

    // colors shifting
    int shift = (int)(sin((i * 0.05) + t * 0.2) * 20);

    uint8_t r = constrain(audioState.r + shift, 0, 255) * flicker;
    uint8_t g = constrain(audioState.g + shift, 0, 255) * flicker;
    uint8_t b = constrain(audioState.b + shift, 0, 255) * flicker;

    ledStrip.setPixelColor(i, ledStrip.Color(r, g, b));
  }
}

void effectHorizontalSweep() {
  static int pos = 45;
  pos += 6;
  if (pos > 259) pos = 45;

  for (int i = 45; i < 260; i++) {
    int dist = abs(i - pos);
    float fade = max(0.0f, 1.0f - dist / 40.0f);

    uint8_t r = audioState.r * (0.6 + fade * 0.4);
    uint8_t g = audioState.g * (0.6 + fade * 0.4);
    uint8_t b = audioState.b * (0.6 + fade * 0.4);

    ledStrip.setPixelColor(i, ledStrip.Color(r, g, b));
  }
}

void effectVerticalPulse() {
  static float phase = 0;
  phase += 0.25;

  float pulse = (sin(phase) + 1.0) * 0.5; // 0..1

  uint8_t r = audioState.r * (0.5 + pulse * 0.5);
  uint8_t g = audioState.g * (0.5 + pulse * 0.5);
  uint8_t b = audioState.b * (0.5 + pulse * 0.5);

  // left arm 0–44 LED
  for (int i = 0; i < 45; i++)
    ledStrip.setPixelColor(i, ledStrip.Color(r, g, b));

  // right arm 260–299 LED
  for (int i = 260; i < 300; i++)
    ledStrip.setPixelColor(i, ledStrip.Color(r, g, b));
}

void effectTwistSpiral() {
  static float t = 0;
  t += 0.15;

  for (int i = 0; i < NUM_LEDS; i++) {
    float swirl = sin((i * 0.15) + t);
    float bright = 0.6 + swirl * 0.4;

    uint8_t r = audioState.r * bright;
    uint8_t g = audioState.g * bright;
    uint8_t b = audioState.b * bright;

    ledStrip.setPixelColor(i, ledStrip.Color(r, g, b));
  }
}

void effectCircleOrbit() {
  static int pos = 0;
  pos = (pos + 4) % NUM_LEDS;

  for (int i = 0; i < NUM_LEDS; i++) {
    int dist = abs(i - pos);
    float fade = max(0.0f, 1.0f - dist / 30.0f);

    uint8_t r = audioState.r * (0.5 + fade * 0.5);
    uint8_t g = audioState.g * (0.5 + fade * 0.5);
    uint8_t b = audioState.b * (0.5 + fade * 0.5);

    ledStrip.setPixelColor(i, ledStrip.Color(r, g, b));
  }
}

// =========================
// --- LED EFFECTS
// =========================

void stepCalmPulse(int level) {
  pulsePhase += 0.1;
  uint8_t base = 40 + level * 10;
  uint8_t brightness = base + sin(pulsePhase) * 20;

  uint8_t r = (audioState.r * brightness) / 255;
  uint8_t g = (audioState.g * brightness) / 255;
  uint8_t b = (audioState.b * brightness) / 255;

  setColorAll(r, g, b);
}

void stepLavaWave() {
  waveOffset++;

  for (int i = 0; i < NUM_LEDS; i++) {
    float wave = sin((i * 0.15) + waveOffset * 0.4);
    uint8_t brightness = 120 + wave * 120;  // viel stärker

    uint8_t r = (audioState.r * brightness) / 255;
    uint8_t g = (audioState.g * brightness) / 255;
    uint8_t b = (audioState.b * brightness) / 255;

    ledStrip.setPixelColor(i, ledStrip.Color(r, g, b));
  }
}


void stepLaser() {
  laserPos = (laserPos + 3) % NUM_LEDS;

  // background slightly darker
  setColorAll(audioState.r / 15, audioState.g / 15, audioState.b / 15);

  // lazer like
  for (int i = -5; i <= 5; i++) {
    int pos = (laserPos + i + NUM_LEDS) % NUM_LEDS;
    ledStrip.setPixelColor(pos, ledStrip.Color(audioState.r, audioState.g, audioState.b));
  }
}


void stepBlocks() {
  blockOffset++;
  for (int i = 0; i < NUM_LEDS; i++) {
    if (((i + blockOffset) % 20) < 10)
      ledStrip.setPixelColor(i, ledStrip.Color(audioState.r, audioState.g, audioState.b));
    else
      ledStrip.setPixelColor(i, ledStrip.Color(audioState.r/10, audioState.g/10, audioState.b/10));
  }
}

void stepTwist() {
  twistPhase++;

  for (int i = 0; i < NUM_LEDS; i++) {
    float swirl = sin((i * 0.2) + twistPhase * 0.5);
    uint8_t brightness = 150 + swirl * 100; 

    uint8_t r = (audioState.r * brightness) / 255;
    uint8_t g = (audioState.g * brightness) / 255;
    uint8_t b = (audioState.b * brightness) / 255;

    ledStrip.setPixelColor(i, ledStrip.Color(r, g, b));
  }
}


void stepArmLaser(bool armUp, bool isLeft) {
  int start = isLeft ? 0 : 25;
  int end   = start + 25;
  int &pos  = isLeft ? armLaserPosLeft : armLaserPosRight;

  if (armUp) pos++; else pos--;
  if (pos >= end) pos = start;
  if (pos < start) pos = end - 1;

  //lazer like
  for (int i = -3; i <= 3; i++) {
    int p = pos + i;
    if (p >= start && p < end) {
      ledStrip.setPixelColor(p, ledStrip.Color(audioState.r, audioState.g, audioState.b));
    }
  }
}

float noise(float x) {
  int i = (int)x;
  float f = x - i;
  float u = f * f * (3 - 2 * f);
  return (1 - u) * random(i) / (float)RAND_MAX + u * random(i + 1) / (float)RAND_MAX;
}

void rainbowFlash() {
  static byte offset = 0;
  offset += 25;

  for (int i = 0; i < NUM_LEDS; i++) {
    byte c = offset + (i * 5);
    uint32_t col;

    if (c < 85) {
      col = ledStrip.Color(c * 3, 255 - c * 3, 0);
    } else if (c < 170) {
      c -= 85;
      col = ledStrip.Color(255 - c * 3, 0, c * 3);
    } else {
      c -= 170;
      col = ledStrip.Color(0, c * 3, 255 - c * 3);
    }

    ledStrip.setPixelColor(i, col);
  }
}




// =========================
// --- UPDATE AUDIO
// =========================

void updateAudio() {
  audioState.beat = false;

  readAudioData(vRealToneDataInWindow, WINDOW_SIZE);

  for (int i = 0; i < WINDOW_SIZE; i++) vTempToneDataInWindow[i] = 0.0;

  float energy = computeEnergy(vRealToneDataInWindow, WINDOW_SIZE);
  energyHistory[energyIndex] = energy;
  energyIndex = (energyIndex + 1) % ENERGY_HISTORY;

  float meanEnergy = 0;
  for (int i = 0; i < ENERGY_HISTORY; i++) meanEnergy += energyHistory[i];
  meanEnergy /= ENERGY_HISTORY;

  float threshold = meanEnergy * 2.5f; //CHANGE IF NEEDED

  float currentTimeSec = (millis() - startMillis) / 1000.0;

  if (energy > threshold && energy > meanEnergy * 1.1f && (currentTimeSec - lastBeat) >= minimalInterval) {
    if (beatCount < MAX_BEATS) beatsFiltered[beatCount++] = currentTimeSec;
    lastBeat = currentTimeSec;
    audioState.beat = true;

    FFT.windowing(vRealToneDataInWindow, WINDOW_SIZE, FFT_WIN_TYP_HAMMING, FFT_FORWARD, vTempToneDataInWindow);
    FFT.compute(vRealToneDataInWindow, vTempToneDataInWindow, WINDOW_SIZE, FFT_FORWARD);
    FFT.complexToMagnitude(vRealToneDataInWindow, vTempToneDataInWindow, WINDOW_SIZE);

    float pitch = smoothFrequency(
      detectPitchFFT(vRealToneDataInWindow, WINDOW_SIZE, SAMPLE_RATE)
    );
    // Serial.println("Pitch");
    // Serial.println(pitch);

    audioState.pitch = pitch;

    audioState.colorName = mapFrequencyToColorName(pitch);
    colorNameToRGB(audioState.colorName, audioState.r, audioState.g, audioState.b);

    if (audioState.beat) {
    Serial.println("------BEAT ------");
    Serial.print("Pitch: ");
    Serial.println(audioState.pitch);
    // Serial.print("Color: ");
    // Serial.println(audioState.colorName);
}

  }
}

// =========================
// --- UPDATE MOTION
// =========================

float getPitch(int16_t ax, int16_t ay, int16_t az) {
  return atan2(ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
}

float getRotationSpeed(int16_t gx, int16_t gy, int16_t gz) {
  float gx_dps = gx / 131.0;
  float gy_dps = gy / 131.0;
  float gz_dps = gz / 131.0;
  return sqrt(gx_dps*gx_dps + gy_dps*gy_dps + gz_dps*gz_dps);
}

int getIntensityLevel(float rot) {
  if (rot < 5) return 0;
  if (rot < 20) return 1;
  if (rot < 40) return 2;
  if (rot < 80) return 3;
  if (rot < 150) return 4;
  if (rot < 250) return 5;
  if (rot < 400) return 6;
  if (rot < 700) return 7;
  if (rot < 1200) return 8;
  if (rot < 2000) return 9;
  return 10;
}

void updateMotion() {
  int16_t axR, ayR, azR, gxR, gyR, gzR;

  // Sensor auslesen
  mpuArm.getMotion6(&axR, &ayR, &azR, &gxR, &gyR, &gzR);

  // Sensor laggy → Re-Init
  if (gxR == 0 && gyR == 0 && gzR == 0 && axR == 0 && ayR == 0 && azR == 0) {
    Serial.println("MPU hang → reinit");
    mpuArm.initialize();
    mpuArm.setSleepEnabled(false);
    delay(50);
    return;
}


  Serial.print("RAW gx gy gz = ");
  Serial.print(gxR); Serial.print("  ");
  Serial.print(gyR); Serial.print("  ");
  Serial.println(gzR);

  float pitchMotion = getPitch(axR, ayR, azR);
  float rotationMotion = getRotationSpeed(gxR, gyR, gzR);

  intensityMovement = getIntensityLevel(rotationMotion);

  String patternCreatedMovement = "none";
  if (rotationMotion > 40) {
    float ax = abs(gxR);
    float ay = abs(gyR);
    float az = abs(gzR);

    if (ax > ay && ax > az) patternCreatedMovement = "vertical";
    else if (ay > ax && ay > az) patternCreatedMovement = "horizontal";
    else if (az > ax && az > ay) patternCreatedMovement = "twist";
    else patternCreatedMovement = "circle";
  }

  if (effectTimer == 0) {
    if (patternCreatedMovement == "horizontal") { effectType = 2; effectTimer = 12; }
    if (patternCreatedMovement == "vertical")   { effectType = 3; effectTimer = 12; }
    if (patternCreatedMovement == "twist")      { effectType = 4; effectTimer = 12; }
    if (patternCreatedMovement == "circle")     { effectType = 5; effectTimer = 12; }
  }

  if (!isFastMove && rotationMotion > 300) {
    isFastMove = true;
    Serial.println("FAST___________________________________________________________");
  }

  if (isFastMove && rotationMotion < 150) {
    isFastMove = false;
    Serial.println("SLOW~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.");
  }

  if (rotationMotion < 10) {
    if (!armUpRight && pitchMotion > ARM_UP_THRESHOLD) {
      armUpRight = true;
      Serial.println("right arm UP");
      effectType = 1;
      effectTimer = 12;
    }
    if (armUpRight && pitchMotion < ARM_DOWN_THRESHOLD) {
      armUpRight = false;
      Serial.println("right arm DOWN");
      effectType = 1;
      effectTimer = 12;
    }
  }

  
  // Debug
  // Serial.print("Intensity L: ");
  // Serial.print(intensityLeft);
  // Serial.print("  Pattern L: ");
  // Serial.print(patternLeft);
  // Serial.print("  Intensity R: ");
  // Serial.print(intensityMovement);
  // Serial.print("  Pattern R: ");
  // Serial.println(patternCreatedMovement);
}

// =========================
// --- UPDATE LED EFFECTS
// =========================

void updateLedEffects() {
  //Effects
  // 0 = none
  // 1 = arm raise or down
  // 2 = horizontal
  // 3 = vertical
  // 4 = twist
  // 5 = circle
  if (effectTimer > 0) {
    effectTimer--;

    switch (effectType) {
      case 1: rainbowFlashStrong(); break;
      case 2: effectHorizontalSweep(); break;
      case 3: effectVerticalPulse(); break;
      case 4: effectTwistSpiral(); break;
      case 5: effectCircleOrbit(); break;
    }
    return;
  }

  // FAST
  if (isFastMove) {
    stepFastFlash();
    return;
  }

  // SLOW default
  if (intensityLeft < 3 && intensityMovement < 3) {
    stepSlowWave();
    return;
  }

  // AUDIO
  if (audioState.beat) {
    int level = intensityMovement;

    if (level <= 3)       stepCalmPulse(level);
    else if (level <= 6)  stepLavaWave();
    else if (level <= 8)  stepLaser();
    else                  stepBlocks();

    stepArmLaser(armUpRight, false);
    // Serial.println("level");
    // Serial.print(level);

    Serial.println("------BEAT ------"); 
    Serial.print("Pitch: "); 
    Serial.println(audioState.pitch);

    boostBrightnessOnce();
  }
}




// =========================
// --- SETUP & LOOP
// =========================

void setup() {
  Serial.begin(115200);
  delay(200);

  startMillis = millis();

  for (int i = 0; i < ENERGY_HISTORY; i++){
    energyHistory[i] = 0.0;
  } 

  Wire.begin(21, 22);
  Wire.setClock(50000);

  //check for movementsensores and initialize
  mpuArm.initialize();
  mpuArm.setSleepEnabled(false);
  mpuArm.setFIFOEnabled(false); 
  mpuArm.setFullScaleGyroRange(MPU6050_GYRO_FS_2000); 
  mpuArm.setDLPFMode(3);
  delay(100);

  // Serial.println("Sensor Kanal 4:");
  // Serial.println(mpuLeft.testConnection() ? "OK" : "FAIL");

  // Serial.println("Sensor Kanal 5:");
  // Serial.println(mpuRight.testConnection() ? "OK" : "FAIL");

  i2sSetup();  //microphone setup
  Serial.println("Microphone mode enabled (I2S).");

  //led setup
  ledStrip.begin();
  ledStrip.show();
  ledStrip.setBrightness(80);

}

void loop() {
  updateMotion();
  delay(3);
  updateAudio();
  updateLedEffects();

  ledStrip.show();
}
