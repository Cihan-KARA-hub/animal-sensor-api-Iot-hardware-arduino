#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <Adafruit_MLX90614.h>
#include <arduinoFFT.h>
#include <PubSubClient.h>
#include <WiFi.h>


const char* ssid = "cihan";
const char* password = "123aa1267";

const char* mqtt_server = "192.168.43.69"; 
const int mqtt_port = 1883;
const char* mqtt_topic = "sensor_data_1";

// MQTT Client nesnesi
WiFiClient espClient;
PubSubClient client(espClient);


// --- I2C Pinleri ---
#define SDA_PIN 21
#define SCL_PIN 22

// --- Nabız Sensörü ---
#define PULSE_PIN 34
#define SAMPLE_SIZE 20
#define SAMPLE_INTERVAL 50
#define ANALOG_THRESHOLD 500

// --- FFT için tanımlar ---
#define FFT_SAMPLE_SIZE 128
#define FFT_SAMPLING_FREQUENCY 20 // Hz

// --- FFT Veri ---
double vReal[FFT_SAMPLE_SIZE];
double vImag[FFT_SAMPLE_SIZE];

// --- Sensör Nesneleri ---
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
ArduinoFFT<double> FFT(vReal, vImag, FFT_SAMPLE_SIZE, FFT_SAMPLING_FREQUENCY);

// --- Değişkenler ---
int pulseReadings[SAMPLE_SIZE];
int sampleIndex = 0;
int beatCount = 0;
bool beatDetected = false;
unsigned long lastSampleTime = 0;

int fftIndex = 0;
unsigned long lastFFTTime = 0;

unsigned long measurementStartTime = 0;
const unsigned long measurementDuration = 60000;
//Geviş getirme durumu
unsigned long ruminationDuration = 0;
bool ruminationActive = false;
//BPM
int lastBPM = 0;
float lastAmbientTemp = 0;
float lastObjectTemp = 0;

// led and buzzer pin
const int ledPin = 26;       
const int buzzerPin = 27;

// --- Veri yapısı: JSON formatında string hazırlamak için buffer ---
#define JSON_BUFFER_SIZE 256
char jsonBuffer[JSON_BUFFER_SIZE];

// wifi setup
void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("WiFi’ye bağlanılıyor: ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi bağlandı");
  Serial.print("IP adresi: ");
  Serial.println(WiFi.localIP());
}
void reconnect() {
  // MQTT broker’a yeniden bağlan
  while (!client.connected()) {
    Serial.print("MQTT bağlantısı deneniyor...");
    if (client.connect("ESP32Client")) {
      Serial.println("Bağlandı!");
client.subscribe("sensor_topic_2"); 
    } else {
        Serial.print("Bağlantı hatası, kod: ");
  Serial.println(client.state());
      Serial.print("Bağlantı hatası, tekrar denenecek...");
      delay(2000);
    }
  }
}
// -------------------------
// Sensör Başlatma Fonksiyonları
// -------------------------
void initADXL345() {
  if (!accel.begin()) {
    Serial.println("ADXL345 sensörü başlatılamadı!");
    while (1);
  }
  accel.setRange(ADXL345_RANGE_16_G);
  Serial.println("ADXL345 başlatıldı.");
}

void initMLX90614() {
  if (!mlx.begin()) {
    Serial.println("MLX90614 sensörü başlatılamadı!");
    while (1);
  }
  Serial.println("MLX90614 başlatıldı.");
}

void initPulseSensor() {
  pinMode(PULSE_PIN, INPUT);
  Serial.println("Nabız sensörü başlatıldı.");
}

// -------------------------
// Sensör Okuma Fonksiyonları
// -------------------------
void readAccelerometerAndCollectFFT() {
  unsigned long currentTime = millis();
  if (currentTime - lastFFTTime >= (1000 / FFT_SAMPLING_FREQUENCY)) {
    lastFFTTime = currentTime;

    sensors_event_t event;
    accel.getEvent(&event);

    vReal[fftIndex] = event.acceleration.y;
    vImag[fftIndex] = 0;
    fftIndex++;

    if (fftIndex >= FFT_SAMPLE_SIZE) {
      fftIndex = 0;

      FFT.windowing(vReal, FFT_SAMPLE_SIZE, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
      FFT.compute(vReal, vImag, FFT_SAMPLE_SIZE, FFT_FORWARD);
      FFT.complexToMagnitude(vReal, vImag, FFT_SAMPLE_SIZE);

      double peakFrequency = FFT.majorPeak(vReal, FFT_SAMPLE_SIZE, FFT_SAMPLING_FREQUENCY);

      bool currentlyRumination = (peakFrequency >= 0.4 && peakFrequency <= 1.2);
      ruminationActive = currentlyRumination;
    }
  }
}
// -------------------------
// MQTT Protocol Subs topic 5
// -------------------------
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Mesaj geldi [");
  Serial.print(topic);
  Serial.print("]: ");

  char message[length + 1];
  memcpy(message, payload, length);
  message[length] = '\0';
  Serial.println(message);

  if (strcmp(topic, "sensor_topic_2") == 0) {
    Serial.println("Komut topic'inden mesaj alındı.");
    ledAndBuzzer();
  }
}


void updateRuminationDuration() {
  static unsigned long lastCheck = 0;
  unsigned long now = millis();

  if (ruminationActive && now > lastCheck) {
    ruminationDuration += (now - lastCheck);
  }
  lastCheck = now;
}

void readTemperature() {
  lastAmbientTemp = mlx.readAmbientTempC();
  lastObjectTemp = mlx.readObjectTempC();
}

void readPulse() {
  unsigned long currentTime = millis();
  if (currentTime - lastSampleTime >= SAMPLE_INTERVAL) {
    lastSampleTime = currentTime;

    int pulseValue = analogRead(PULSE_PIN);
    pulseReadings[sampleIndex] = pulseValue;
    sampleIndex++;

    if (sampleIndex >= SAMPLE_SIZE) {
      sampleIndex = 0;

      for (int i = 1; i < SAMPLE_SIZE - 1; i++) {
        if (pulseReadings[i] > ANALOG_THRESHOLD &&
            pulseReadings[i] > pulseReadings[i - 1] &&
            pulseReadings[i] > pulseReadings[i + 1]) {
          if (!beatDetected) {
            beatCount++;
            beatDetected = true;
          }
        }
        else if (pulseReadings[i] < ANALOG_THRESHOLD) {
          beatDetected = false;
        }
      }

      unsigned long elapsedSeconds = (millis() - measurementStartTime) / 1000;
      if (elapsedSeconds > 0) {
        lastBPM = (beatCount * 60) / elapsedSeconds;
      }
    }
  }
}

// -------------------------
// JSON Veri Hazırlama Fonksiyonu
// -------------------------
void prepareJson() {
  float chewingActivity = ruminationDuration / 1000.0;  // örneğin saniyeye çevrildi
  snprintf(jsonBuffer, JSON_BUFFER_SIZE,
    "{"
    "\"animalId\": 1,"
    "\"chewingActivity\": %.2f,"
    "\"heartBeat\": %d,"
    "\"temperature\": %.2f,"
    "\"humidity\": %d"
    "}",
    chewingActivity, beatCount, lastAmbientTemp, 70);
  
  Serial.print("JSON Veri: ");
  Serial.println(jsonBuffer);
}
// -------------------------
// Mqtt 'den kesme geldi
// -------------------------
void ledAndBuzzer(){
  digitalWrite(ledPin, HIGH);
  digitalWrite(buzzerPin, HIGH);
  delay(500);
  digitalWrite(ledPin, LOW);
  digitalWrite(buzzerPin, LOW);
  delay(500); 
}
// -------------------------
// Setup ve Loop
// -------------------------
void setup() {
  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, LOW);  
    pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);  
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);  


  initADXL345();
  initMLX90614();
  initPulseSensor();

  measurementStartTime = millis();
  Serial.println("Sistem başlatıldı .");


}
void loop() {
  if (!client.connected()) {
    reconnect(); 
  }
  client.loop();  

  readAccelerometerAndCollectFFT();
  updateRuminationDuration();

  unsigned long elapsed = millis() - measurementStartTime;

  if (elapsed < 60100) {
    readPulse();
  }

  if (elapsed >= measurementDuration) {
    readTemperature();
    prepareJson(); 

    // MQTT üzerinden JSON veriyi gönder
    if (client.publish(mqtt_topic, jsonBuffer,true)) {
      Serial.println("MQTT: Veri başarıyla gönderildi.");
    } else {
      Serial.println("MQTT: Veri gönderilemedi!");
    }
    beatCount = 0;
    ruminationDuration = 0;
    measurementStartTime = millis();
  }

  delay(100);
}
