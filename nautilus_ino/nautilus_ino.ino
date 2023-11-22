// Basic demo for accelerometer readings from Adafruit MPU6050

// ESP32 Guide: https://RandomNerdTutorials.com/esp32-mpu-6050-accelerometer-gyroscope-arduino/
// ESP8266 Guide: https://RandomNerdTutorials.com/esp8266-nodemcu-mpu-6050-accelerometer-gyroscope-arduino/
// Arduino Guide: https://RandomNerdTutorials.com/arduino-mpu-6050-accelerometer-gyroscope/

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

#define alimentacao 17

// Configurações do LED RGB
#define red 15  // Pino do canal vermelho
#define green 2   // Pino do canal verde
#define blue 4   // Pino do canal azul

const unsigned long duration = 60000;  // 60 seconds in milliseconds
unsigned long startTime;

// WiFi
const char *ssid = "iPhone Luz"; // Enter your WiFi name
const char *password = "analuz2110";  // Enter WiFi password

// MQTT Broker
const char *mqtt_broker = "broker.emqx.io"; // broker address
const char *topic = "ufabc/nautilusguard"; // define topic 
const char *mqtt_username = "emqx"; // username for authentication
const char *mqtt_password = "public"; // password for authentication
const int mqtt_port = 1883; // port of MQTT over TCP

WiFiClient espClient;
PubSubClient client(espClient);

Adafruit_MPU6050 mpu;

void setup(void) {

  // Set the GPIO pin to OUTPUT mode
  pinMode(alimentacao, OUTPUT);
  pinMode(red, OUTPUT);//Define a variável azul como saída
  pinMode(green, OUTPUT);//Define a variável verde como saída
  pinMode (blue, OUTPUT);//Define a variável vermelho como saída
  
  // Turn on the GPIO pin by setting it to HIGH
  digitalWrite(alimentacao, HIGH);
  acenderLedVermelho();

  delay(500);

  Serial.begin(115200);
  startTime = millis();

  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }

  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  setup_wifi();

  //connecting to a mqtt broker
  client.setServer(mqtt_broker, mqtt_port);
  client.setCallback(callback);
  while (!client.connected()) {
      String client_id = "esp8266-client-";
      client_id += String(WiFi.macAddress());
      Serial.printf("The client %s connects to the public mqtt broker\n", client_id.c_str());
      if (client.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
          Serial.println("Public emqx mqtt broker connected");
      } else {
          Serial.print("failed with state ");
          Serial.print(client.state());
          delay(2000);
      }
  }

  acenderLedVerde();

  // publish and subscribe
  client.publish(topic, "hello emqx"); // publish to the topic
  client.subscribe(topic); // subscribe from the topic
 
}


void loop() {

  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);  

  String result = String(g.gyro.x) + ";" + String(g.gyro.y) + ";" + String(g.gyro.z);
  Serial.println(result);

  client.publish(topic, result.c_str());

  // Esperar por uma mensagem
  if (client.loop() && client.connected()) {
    acenderLedVerde();
  } else {
    acenderLedVermelho();;
  }

  client.loop();
  delay(500);  
}

void setup_wifi() {
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println("\"" + String(ssid) + "\"");

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char *topic, byte *payload, unsigned int length) {
    Serial.print("Message arrived in topic: ");
    Serial.println(topic);
    Serial.print("Message:");
    for (int i = 0; i < length; i++) {
        Serial.print((char) payload[i]);
    }
    Serial.println();
    Serial.println("-----------------------");
}

void acenderLedAmarelo() {
  digitalWrite(blue, LOW);
  digitalWrite(green, HIGH);
  digitalWrite(red, HIGH);
}

void acenderLedVermelho() {
  digitalWrite(blue, LOW);
  digitalWrite(green, LOW);
  digitalWrite(red, HIGH);
}

void acenderLedVerde() {
  digitalWrite(blue, LOW);
  digitalWrite(green, HIGH);
  digitalWrite(red, LOW);
}