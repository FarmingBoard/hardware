#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>
#include <ArduinoJson.h>
#include "MQ135.h"
#include <SPI.h>
#include <MFRC522.h>
#include <Servo.h>

#define SS_PIN D8
#define RST_PIN D0
#define SERVO_PIN D1 // Chân D1 (GPIO5) dùng cho Servo

#define EEPROM_SIZE 512
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define RESET_BUTTON_PIN D2

String rfid_uids[10]; // Giả sử tối đa 10 UID

MFRC522 rfid(SS_PIN, RST_PIN);
Servo myServo;

WiFiClient espClient;
PubSubClient client(espClient);

/*
Status device:
Không load được dữ liệu từ EEPROM: "no_data"
Không kết nối được WiFi: "wifi_error"
Không kết nối được MQTT: "mqtt_error"
Kết nối WiFi và MQTT thành công: "connected"
*/
String statusDevice = "";

char wifiSSID[32] = "hoang";
char wifiPass[64] = "1234567899";
char mqttUser[32] = "UGWFFT06CjCTqTDGtEMp";
char mqttServer[32] = "192.168.218.181";
char *mqttPass = "";
char *mqttTopic = "v1/devices/me/telemetry";
int mqttPort = 1883;

bool wifiConnected = false;
bool mqttConnected = false;

void setWarnLed(const int type);
void setupBLE();
bool connectToWiFi(const char *ssid, const char *password);
bool connectToMQTT();
void sendDataMqtt();
void sendDHT11Data();
void onBLEReceive(String jsonData);
void saveCredentialsToEEPROM();
void loadCredentialsFromEEPROM();
void handleResetButton();
void resetDevice();
void callback(char *topic, byte *payload, unsigned int length);
void readRFID();

class MyBLECallbacks : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *pCharacteristic) override
    {
        String rxData = pCharacteristic->getValue().c_str();
        if (rxData.length() > 0)
        {
            Serial.println("Received credentials over BLE");
            rxValue += rxData;
            Serial.println("Received Data: " + rxValue);
            if (rxValue.indexOf(';') != -1)
            {
                onBLEReceive(rxValue);
                Serial.println("BLECALLBACK.LOG -> " + rxValue);
                rxValue = "";
            }
        }
        else
        {
            Serial.println("Received empty data");
        }
    }
};

void setup()
{
    Serial.begin(115200);
    pinMode(RESET_BUTTON_PIN, INPUT_PULLUP);
    SPI.begin();
    rfid.PCD_Init();

    myServo.attach(SERVO_PIN);
    myServo.write(0); // Mở ở góc 0 độ

    client.setCallback(callback);
    EEPROM.begin(EEPROM_SIZE);
    loadCredentialsFromEEPROM();
    setWarnLed(0);
    if (statusDevice == "no_data")
    {
        setWarnLed(2);
        Serial.println("No data found in EEPROM");
        setupBLE();
        while (statusDevice != "connected")
        {
            if (isChangedData)
            {
                connectToWiFi(wifiSSID, wifiPass);
                if (wifiConnected)
                {
                    connectToMQTT();
                }
                if (mqttConnected)
                {
                    String message = "SUCCESS";
                    pCharacteristic->setValue(message.c_str());
                    pCharacteristic->notify();
                    Serial.println("BLE notification sent: " + message);
                    saveCredentialsToEEPROM();
                    statusDevice = "connected";
                    isChangedData = false;
                }
            }
            delay(2000);
        }
        BLEDevice::deinit();         // Tắt BLE stack
        esp_bt_controller_disable(); // Tắt Bluetooth controller
        setWarnLed(1);
    }
    connectToWiFi(wifiSSID, wifiPass);
}

unsigned long pressStartTime = 0;
bool isLongPressHandled = false;

void loop()
{
    if (digitalRead(BUTTON_PIN) == LOW)
    { // Nút nhấn được bấm (mức LOW)
        if (pressStartTime == 0)
        {
            pressStartTime = millis(); // Lưu thời gian bắt đầu nhấn
        }

        if (millis() - pressStartTime >= 3000 && !isLongPressHandled)
        {
            Serial.println("Button held for more than 3s!");
            isLongPressHandled = true; // Đảm bảo chỉ chạy một lần khi giữ lâu
        }
    }
    else
    { // Khi nút được thả ra
        pressStartTime = 0;
        isLongPressHandled = false;
    }

    if (WiFi.status() != WL_CONNECTED)
    {
        WiFi.disconnect();
        WiFi.reconnect();
        delay(1000);
        return;
    }
    if (!mqttConnected)
    {
        connectToMQTT();
        if (!mqttConnected)
            return;
    }
    if (WiFi.status() == WL_CONNECTED && mqttConnected)
    {
        client.loop();
        readRFID();
        delay(1000);
    }
}

void setWarnLed(const int type)
{
    bool red = false, green = false, blue = false;
    if (type == 1)
    {
        red = true;
    }
    if (type == 2)
    {
        green = true;
    }
    if (type == 3)
    {
        blue = true;
    }
}

bool connectToWiFi(const char *ssid, const char *password)
{
    Serial.println("Connecting to WiFi...");
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    unsigned long startTime = millis();
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
        if (millis() - startTime > 30000)
        {
            Serial.println("\nFailed to connect to WiFi");
            wifiConnected = false;
            return false;
        }
    }
    Serial.println("\nWiFi connected");
    wifiConnected = true;
    return true;
}

bool connectToMQTT()
{
    client.setServer(mqttServer, mqttPort);
    client.setCallback(callback);
    Serial.print("Connecting to MQTT...");
    if (client.connect("ESP32Client", mqttUser, mqttPass))
    {
        Serial.println("MQTT connected");
        mqttConnected = true;
        client.subscribe("v1/devices/me/attributes");
        String request = "{\"sharedKeys\":\"lightState,RFID_UIDs\"}";
        client.publish("v1/devices/me/attributes/request/1", request.c_str());
        return true;
    }
    else
    {
        Serial.println("Failed to connect to MQTT");
        mqttConnected = false;
        return false;
    }
}

void setupBLE()
{
    BLEDevice::init("ESP32_BLE");
    pServer = BLEDevice::createServer();
    BLEService *pService = pServer->createService(SERVICE_UUID);
    pCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_WRITE |
            BLECharacteristic::PROPERTY_NOTIFY);
    pCharacteristic->setCallbacks(new MyBLECallbacks());
    pService->start();
    pServer->getAdvertising()->start();
    Serial.println("BLE setup complete, waiting for credentials...");
}

void onBLEReceive(String jsonData)
{
    DynamicJsonDocument doc(512);
    DeserializationError error = deserializeJson(doc, jsonData);
    if (error)
    {
        Serial.print("Failed to parse JSON: ");
        Serial.println(error.c_str());
        return;
    }
    if (doc.containsKey("reset") && doc["reset"] == true)
    {
        Serial.println("Reset command received via BLE. Resetting device...");
        resetDevice();
        return;
    }
    strcpy(wifiSSID, doc["ssid"]);
    strcpy(wifiPass, doc["password"]);
    strcpy(mqttUser, doc["mqttUser"]);
    strcpy(mqttServer, doc["mqttServer"]);
    mqttPort = doc["mqttPort"];
    Serial.printf("onBLEReceive.LOG -> SSI: %s\n", wifiSSID);
    Serial.printf("onBLEReceive.LOG -> PASSWORD: %s\n", wifiPass);
    Serial.printf("onBLEReceive.LOG -> MQTTUSER: %s\n", mqttUser);
    Serial.printf("onBLEReceive.LOG -> MQTTSERVER: %s\n", mqttServer);
    Serial.printf("onBLERecevie.LOG -> MQTTPORT: $d\n", mqttPort);
    isChangedData = true;
}

void saveCredentialsToEEPROM()
{
    EEPROM.writeString(0, wifiSSID);
    EEPROM.writeString(32, wifiPass);
    EEPROM.writeString(96, mqttUser);
    EEPROM.writeString(128, mqttServer);
    // mqttPort
    EEPROM.write(160, (uint8_t)(mqttPort >> 8));   // Byte cao
    EEPROM.write(161, (uint8_t)(mqttPort & 0xFF)); // Byte thấp

    EEPROM.commit();
    Serial.printf("SAVE_EROM.LOG -> SSI: %s\n", wifiSSID);
    Serial.printf("SAVE_EROM.LOG -> PASSWORD: %s\n", wifiPass);
    Serial.printf("SAVE_EROM.LOG -> MQTTUSER: %s\n", mqttUser);
    Serial.printf("SAVE_EROM.LOG -> MQTTSERVER: %s\n", mqttServer);
    Serial.printf("SAVE_EROM.LOAD.LOG -> MQTTPORT: %d\n", mqttPort);
    Serial.println("Credentials saved to EEPROM.");
}

void loadCredentialsFromEEPROM()
{
    EEPROM.readString(0, wifiSSID, 32);
    EEPROM.readString(32, wifiPass, 64);
    EEPROM.readString(96, mqttUser, 32);
    EEPROM.readString(128, mqttServer, 32);
    // read mqttPort
    mqttPort = (EEPROM.read(160) << 8) | EEPROM.read(161);

    Serial.printf("SAVE_EROM.LOAD.LOG -> SSI: %s\n", wifiSSID);
    Serial.printf("SAVE_EROM.LOAD.LOG -> PASSWORD: %s\n", wifiPass);
    Serial.printf("SAVE_EROM.LOAD.LOG -> MQTTUSER: %s\n", mqttUser);
    Serial.printf("SAVE_EROM.LOAD.LOG -> MQTTSERVER: %s\n", mqttServer);
    Serial.printf("SAVE_EROM.LOAD.LOG -> MQTTPORT: %d\n", mqttPort);

    if (strlen(wifiSSID) == 0 || strlen(wifiPass) == 0 || strlen(mqttUser) == 0 || strlen(mqttServer) == 0)
    {
        statusDevice = "no_data";
        Serial.println("No data found in EEPROM");
    }
    else
    {
        statusDevice = "data_found";
        Serial.println("Credentials loaded from EEPROM.");
    }
}

void resetDevice()
{
    memset(wifiSSID, 0, sizeof(wifiSSID));
    memset(wifiPass, 0, sizeof(wifiPass));
    memset(mqttUser, 0, sizeof(mqttUser));
    memset(mqttServer, 0, sizeof(mqttServer));
    mqttPort = 0;
    saveCredentialsToEEPROM();
    ESP.restart();
}

void callback(char *topic, byte *payload, unsigned int length)
{
    String message;
    for (unsigned int i = 0; i < length; i++)
    {
        message += (char)payload[i];
    }
    Serial.print("Message received: ");
    Serial.println(message);

    DynamicJsonDocument doc(1024);
    DeserializationError error = deserializeJson(doc, message);
    if (!error)
    {
        if (doc["shared"]["RFID_UIDs"])
        {
            JsonArray array = doc["shared"]["RFID_UIDs"].as<JsonArray>();
            int i = 0;
            for (String uid : array)
            {
                if (i < 10) // Giới hạn 10 UID
                {
                    rfid_uids[i] = uid;
                    i++;
                }
            }
        }

        //  servo
        if (doc["share"]["servoState"])
        {
            int servoState = doc["shared"]["servoState"];
            if (servoState == 1)
            {
                myServo.write(90); // Quay servo đến 90 độ
                delay(3000);       // Giữ trạng thái mở trong 3 giây
                myServo.write(0);  // Quay lại vị trí ban đầu
            }
        }

        // neu nhan duoc message moi servoState
        if (doc["servoState"])
        {
            int servoState = doc["servoState"];
            if (servoState == 1)
            {
                myServo.write(90); // Quay servo đến 90 độ
                delay(3000);       // Giữ trạng thái mở trong 3 giây
                myServo.write(0);  // Quay lại vị trí ban đầu
            }
        }

        // neu nhan duoc message moi rfid_uids
        if (doc["RFID_UIDs"])
        {
            JsonArray array = doc["RFID_UIDs"].as<JsonArray>();
            int i = 0;
            for (String uid : array)
            {
                if (i < 10) // Giới hạn 10 UID
                {
                    rfid_uids[i] = uid;
                    i++;
                }
            }
        }
    }
}

void readRFID()
{
    if (!rfid.PICC_IsNewCardPresent() || !rfid.PICC_ReadCardSerial())
        return;

    String uid = "";
    for (byte i = 0; i < rfid.uid.size; i++)
    {
        uid += String(rfid.uid.uidByte[i], HEX);
    }

    Serial.print("RFID UID detected: ");
    Serial.println(uid);

    for (int i = 0; i < 10; i++)
    {
        Serial.println(rfid_uids[i]);
        if (rfid_uids[i] == uid)
        {
            Serial.println("Access granted! Moving servo...");
            myServo.write(90); // Quay servo đến 90 độ
            client.publish(mqttTopic, String("{\"RFID_OPEN\":\"" + uid + "\"}").c_str());
            delay(3000);      // Giữ trạng thái mở trong 3 giây
            myServo.write(0); // Quay lại vị trí ban đầu
            client.publish(mqttTopic, String("{\"RFID_CLOSE\":\"" + uid + "\"}").c_str());
            return;
        }
    }
    Serial.println("Access denied!");
}
530