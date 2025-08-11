#include <HardwareSerial.h>
#include <DHT.h>
#include <ModbusMaster.h>
 
#define DHTPIN 5
#define DHTTYPE DHT11
#define SOIL_PIN 34
#define RE_DE 18
#define RX1_PIN 16    // For NPK Sensor (RS485) — UART2 RX
#define TX1_PIN 17    // UART2 TX
 
#define RX2_PIN 27    // For SIM Module — UART1 RX
#define TX2_PIN 26    // UART1 TX
 
DHT dht(DHTPIN, DHTTYPE);
ModbusMaster node;
 
HardwareSerial& SIMSerial = Serial1;  // SIM A7670C on UART2
HardwareSerial& NPKSerial = Serial2; // NPK Sensor on UART1
 
// Calibration values for soil sensor
#define DRY_VALUE 3000
#define WET_VALUE 1300
 
const char* URL_NPK = "http://46.202.160.176:2020/agrifield/npk/save";
const char* URL_SOIL = "http://46.202.160.176:2020/agrifield/soil/save";
const char* URL_WEATHER = "http://46.202.160.176:2020/agrifield/weather/save";
 
void preTransmission() {
  digitalWrite(RE_DE, HIGH);
}
void postTransmission() {
  digitalWrite(RE_DE, LOW);
}
 
void setup() {
  Serial.begin(115200);
  delay(1000);
 
  pinMode(RE_DE, OUTPUT);
  digitalWrite(RE_DE, LOW);
 
  // Init UART1 for NPK RS485 (GPIO17 TX, GPIO16 RX)
  NPKSerial.begin(9600, SERIAL_8N1, RX1_PIN, TX1_PIN);
  node.begin(37, NPKSerial);
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);
 
  // Init UART2 for SIM A7670C (GPIO26 TX, GPIO27 RX)
  SIMSerial.begin(115200, SERIAL_8N1, RX2_PIN, TX2_PIN);
 
  dht.begin();
 
  Serial.println("Initializing SIM Module...");
  initializeGPRS();
}
 
void loop() {
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();
 
  int soilRaw = analogRead(SOIL_PIN);
  float soilPercent = map(soilRaw, WET_VALUE, DRY_VALUE, 100, 0);
  soilPercent = constrain(soilPercent, 0, 100);
 
  uint16_t nitrogen = 0, phosphorus = 0, potassium = 0;
  uint8_t result = node.readHoldingRegisters(3, 3);
 
  if (result == node.ku8MBSuccess) {
    nitrogen = node.getResponseBuffer(0);
    phosphorus = node.getResponseBuffer(1);
    potassium = node.getResponseBuffer(2);
 
 
    Serial.printf("🌡 Temp: %.2f°C  💧 Hum: %.2f%%  🌱 Soil: %.1f%%\n", temperature, humidity, soilPercent);
    Serial.printf("🧪 N: %d, P: %d, K: %d\n", nitrogen, phosphorus, potassium);
  } else {
    Serial.print("❌ Modbus Error: 0x");
    Serial.println(result, HEX);
  }
 
  sendHTTP(URL_WEATHER, "{\"temperature\":" + String(temperature, 1) + ",\"humidity\":" + String(humidity, 1) + "}");
  sendHTTP(URL_SOIL, "{\"moistureLevel\":" + String(soilPercent, 1) + "}");
  sendHTTP(URL_NPK, "{\"nitrogen\":" + String(nitrogen) + ",\"phosphorus\":" + String(phosphorus) + ",\"potassium\":" + String(potassium) + "}");
 
  delay(15000);
}
 
void sendAT(String command, int waitTime = 1000) {
  Serial.println("📡 " + command);
  SIMSerial.println(command);
  unsigned long startTime = millis();
  while (millis() - startTime < waitTime) {
    while (SIMSerial.available()) {
      String line = SIMSerial.readStringUntil('\n');
      line.trim();
      if (line.length() > 0) {
        Serial.println("📥 " + line);
      }
    }
  }
}
 
void initializeGPRS() {
  sendAT("AT");
  sendAT("AT+CFUN=1", 2000);
  sendAT("AT+CGATT=1", 2000);
 
  sendAT("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"");
  sendAT("AT+SAPBR=3,1,\"APN\",\"airtelgprs.com\"");
  sendAT("AT+SAPBR=1,1", 5000);  // Open bearer
  sendAT("AT+SAPBR=2,1");        // Check bearer
 
  sendAT("AT+HTTPTERM");         // Reset HTTP
  sendAT("AT+HTTPINIT");
  sendAT("AT+HTTPPARA=\"CID\",1");
 
  // Optional network check
  sendAT("AT+CGATT?");
  sendAT("AT+CREG?");
}
 
void sendHTTP(String url, String payload) {
  Serial.println("📤 Posting to: " + url);
  sendAT("AT+HTTPPARA=\"URL\",\"" + url + "\"");
  sendAT("AT+HTTPPARA=\"CONTENT\",\"application/json\"");
 
  sendAT("AT+HTTPDATA=" + String(payload.length()) + ",10000", 1000);
  SIMSerial.print(payload);
  delay(3000);
 
  sendAT("AT+HTTPACTION=1", 8000);
  sendAT("AT+HTTPREAD", 3000);
}