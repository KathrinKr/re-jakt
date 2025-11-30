// Übersetzung/Portierung deines Python-Codes auf ESP32 (Arduino C++)
// Variablennamen und Kommentare weitestgehend beibehalten.

// Benötigte Bibliotheken: ArduinoFFT (für FFT), I2S (falls Mikro genutzt)
// In Arduino IDE: Sketch -> Bibliotheken einbinden -> Bibliotheken verwalten

//----------------Begriffserklärungen
/*
Sample: Ein Mikro wandelt Schwingungen in Zahlen um. das nennt man Samples. 
SampleRate= wie viele Zahlen pro Sekunde gemessen wurden
FFT= sortiert Töne nach Frequenz (hoch, tief) und zeigt an wie stark jeder Ton ist
Magnitudes = Liste aus FFTs (?)
Magnitude = Lautstärke der jeweiligen Frequenz

I2S = digitales Audio-Protokoll


*/

#include <Arduino.h>
#include <arduinoFFT.h>   //https://github.com/kosme/arduinoFFT -> von Ton zu frequenz

#define STOP_RUNNING false //zum stoppen der endlosschleife von ESP32. True setzen, wenn man die anwendung beenden will, ohne den ESP32 auszustecken

// ----- Konfiguration -----
#define USE_MICROPHONE 0   // 0 = Simulationsmodus (kein Mikro), 1 = INMP441 I2S benutzen
#define SAMPLE_RATE 44100  //wie oft pro Sekunde das Mikro Werte liefern soll
#define WINDOW_SIZE 1024   //Anzahl Samples pro Fenster (je groesser desto bessere Frequenzauflösung desto mehr Rechenpower)

// I2S Pins - IF USE_MICROPHONE=1
//Die Zahlen hier sind jeweils die D[Zahl] Werte des ESPs an denen das Mikro angeschlossen ist
#define I2S_SD_PIN 22 //Audiodaten des Mikros (Messwerte) 
#define I2S_WS_PIN 25 //Wann neuer Sample beginnt
#define I2S_SCK_PIN 26 //Takt der Datenübertragung

// Minimalzeit zwischen Beats
const float minimalInterval = 0.3; // in Sekunden

// FFT Setup
ArduinoFFT<float> FFT = ArduinoFFT<float>(); // Macht aus Ton Frequenz
float vRealToneDataInWindow[WINDOW_SIZE];
float vTempToneDataInWindow[WINDOW_SIZE]; //hilfsvariabel

// Hilfsvariablen
float samplerate = SAMPLE_RATE;
unsigned long startMillis = 0; //merkt wann ESP gestartet wurde

// Für Beat-Detection
#define ENERGY_HISTORY 16 //Energy=Lautstärke
float energyHistory[ENERGY_HISTORY]; //für Durchschnittsberechnung (if energie>durchschnitt*faktor, dann Beat)
int energyIndex = 0;

// Pseudo-"Zeitstempel" für Beats (Sekunden seit Start)
#define MAX_BEATS 200
float beatsFiltered[MAX_BEATS];
int beatCount = 0;
float lastBeat = -minimalInterval;

// ----------------- Funktionen -----------------

void sendColorToLEDs(uint8_t r, uint8_t g, uint8_t b) {
  Serial.printf("SEND LED RGB: %u,%u,%u\n", r, g, b);
}

String mapFrequencyToColorName(float dominantFrequency) {
  String color;
  if (dominantFrequency <= 120) color = "Purple";
  else if (dominantFrequency <= 160) color = "Dark Blue";
  else if (dominantFrequency <= 200) color = "Deep Blue";
  else if (dominantFrequency <= 275) color = "Sky Blue";
  else if (dominantFrequency <= 350) color = "bright blue";
  else if (dominantFrequency <= 500) color = "Turquoise";
  else if (dominantFrequency <= 1000) color = "Green";
  else if (dominantFrequency <= 2000) color = "Yellow";
  else if (dominantFrequency <= 3000) color = "Occher";
  else if (dominantFrequency <= 4500) color = "Orange";
  else if (dominantFrequency <= 6000) color = "Ginger";
  else color = "Red";
  return color;
}

void colorNameToRGB(const String &name, uint8_t &r, uint8_t &g, uint8_t &b) {
  if (name == "Purple") { r=128; g=0; b=128; }
  else if (name == "Dark Blue") { r=0; g=0; b=100; }
  else if (name == "Deep Blue") { r=0; g=50; b=200; }
  else if (name == "Sky Blue") { r=0; g=150; b=255; }
  else if (name == "bright blue") { r=50; g=180; b=255; }
  else if (name == "Turquoise") { r=0; g=255; b=200; }
  else if (name == "Green") { r=0; g=255; b=0; }
  else if (name == "Yellow") { r=255; g=200; b=0; }
  else if (name == "Occher") { r=200; g=130; b=0; }
  else if (name == "Orange") { r=255; g=120; b=0; }
  else if (name == "Ginger") { r=255; g=180; b=50; }
  else if (name == "Red") { r=255; g=0; b=0; }
  else { r=128; g=128; b=128; }
}

float computeEnergy(const float *buffer, int len) {
  float s = 0.0;
  for (int i=0; i<len; i++) {
    s += buffer[i]*buffer[i];
  }
  return s;
}

float findDominantFrequency(float *magnitudes, int len, float sampleRate, int samples) {
  int maxIndex = 1;
  float maxVal = 0;
  int half = samples/2;
  for (int i=1; i<half; i++) {
    if (magnitudes[i] > maxVal) {
      maxVal = magnitudes[i];
      maxIndex = i;
    }
  }
  float freq = (float)maxIndex * (sampleRate / (float)samples);
  return freq;
}

void fillAudioBuffer(float *buffer, int windowSize) {
  #if USE_MICROPHONE == 1 //if no microphone 
    for (int i=0 ; i<windowSize; i++) 
    {
      buffer[i] = 0; // Platzhalter für Mikrofon
    }

  #else //if micro
    static unsigned long t = 0;

      for (int i=0; i<windowSize; i++) 
      {
        float timeSec = (t + i) / samplerate;
        float s = 0.6 * sin(2.0*PI*100.0*timeSec)
                + 0.3 * sin(2.0*PI*800.0*timeSec)
                + 0.1 * sin(2.0*PI*3000.0*timeSec);
        float beatEnv = (sin(2.0*PI*2.0*timeSec) > 0) ? 1.0 : 0.4;
        buffer[i] = s * beatEnv;
      }
      t += windowSize;

    

  #endif
}

//Bass meist im 20-250 Hz Bereich
  float findBassFrequency(float *magnitudes, int len, float sampleRate) {
    int lowIndex = (int)(20.0 * len / sampleRate);
    int highIndex = (int)(250.0 * len / sampleRate);
    int maxIndex = lowIndex;
    float maxVal = 0;
    for (int i = lowIndex; i <= highIndex; i++) {
        if (magnitudes[i] > maxVal) {
            maxVal = magnitudes[i];
            maxIndex = i;
        }
    }
    return (float)maxIndex * sampleRate / len;
  }

//Lead meist im 300-3000 Hz Bereich
float findVocalFrequency(float *magnitudes, int len, float sampleRate) {
    int lowIndex = (int)(300.0 * len / sampleRate);
    int highIndex = (int)(3000.0 * len / sampleRate);
    int maxIndex = lowIndex;
    float maxVal = 0;
    for (int i = lowIndex; i <= highIndex; i++) {
        if (magnitudes[i] > maxVal) {
            maxVal = magnitudes[i];
            maxIndex = i;
        }
    }
    return (float)maxIndex * sampleRate / len;
  }



  

// ----------------- Setup & Loop -----------------

void setup() {
  Serial.begin(115200); //Datenleitung von USB zu PC mit geschwindigkeit 115200
  delay(200);
  Serial.println("Connection established. \nESP is gonna send you some sick beats to your PC~~ :D");
  startMillis = millis(); //time stamp(?) wann esp gestartet wurde

  for (int i = 0; i < ENERGY_HISTORY; i++) 
  {
    energyHistory[i]=0.0; //jede lautstärke soll auf 0 gesetzt werden (als clean setup)
  }

  #if USE_MICROPHONE
    Serial.println("Microphone mode enabled (I2S). Implement I2S config.");
  #else
    Serial.println("Simulation mode enabled. No microphone is connected.");
  #endif
}

void loop() 
{
  //in case I want to stop the loop
  if(STOP_RUNNING){return;}

  //fill buffer with either microphone data or sample data
  fillAudioBuffer(vRealToneDataInWindow, WINDOW_SIZE);

  // reset
  for (int i=0; i<WINDOW_SIZE; i++) 
  {
    vTempToneDataInWindow[i] = 0.0;
  }

  //berechnen von Lautstärke
  float energy = computeEnergy(vRealToneDataInWindow, WINDOW_SIZE);
  energyHistory[energyIndex] = energy;
  energyIndex = (energyIndex + 1) % ENERGY_HISTORY;

  float meanEnergy = 0;
  for (int i=0; i<ENERGY_HISTORY; i++) meanEnergy += energyHistory[i];
  meanEnergy /= ENERGY_HISTORY;
  float threshold = meanEnergy * 1.2; //Definition eines Beats: Durchschnitt der Energie/Lautstärke*1.2=Beat

  float currentTimeSec = (millis() - startMillis) / 1000.0;
  if (energy > threshold && (currentTimeSec - lastBeat) >= minimalInterval) {
    if (beatCount < MAX_BEATS) beatsFiltered[beatCount++] = currentTimeSec;
    lastBeat = currentTimeSec;
    Serial.printf("Beat erkannt bei t=%.3f s (Energy=%.2f, thr=%.2f)\n", currentTimeSec, energy, threshold);
  }

  FFT.windowing(vRealToneDataInWindow, WINDOW_SIZE, FFT_WIN_TYP_HAMMING, FFT_FORWARD, vTempToneDataInWindow);
  FFT.compute(vRealToneDataInWindow, vTempToneDataInWindow, WINDOW_SIZE, FFT_FORWARD);
  FFT.complexToMagnitude(vRealToneDataInWindow, vTempToneDataInWindow, WINDOW_SIZE);

  // float dominantFrequency = findDominantFrequency(vRealToneDataInWindow, WINDOW_SIZE, samplerate, WINDOW_SIZE);

  float leadVocalFrequency = findVocalFrequency(vRealToneDataInWindow, WINDOW_SIZE, samplerate);
  float bassFrequency = findBassFrequency(vRealToneDataInWindow, WINDOW_SIZE, samplerate);

  // Pruefen wie laut der Bass ist. Falls zu leise, dann fallback auf vocal
  float bassMagnitude = vRealToneDataInWindow[(int)(bassFrequency * WINDOW_SIZE / SAMPLE_RATE)];
  float bassThreshold = 10.0; //change maybe

  float frequenceToUse; //safes which frequence should be used (bass or lead)
  if(bassMagnitude>bassThreshold){
    frequenceToUse = bassFrequency;
  }
  else {
    frequenceToUse= leadVocalFrequency; //lead as fallback
  }

  // String colorName = mapFrequencyToColorName(dominantFrequency);
  String colorName = mapFrequencyToColorName(frequenceToUse);


  uint8_t r,g,b;
  colorNameToRGB(colorName, r, g, b);
  sendColorToLEDs(r,g,b);
  Serial.println(colorName + " is currently showing");

  float waitSec = (float)WINDOW_SIZE / samplerate;
  delay(max(10UL, (unsigned long)(waitSec*1000.0)));
}
