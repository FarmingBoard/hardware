#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266httpUpdate.h>
#include <PubSubClient.h>
#include <DHT.h>
#include <ArduinoJson.h>
#include "MQ135.h"
#include <SPI.h>
#include <MFRC522.h>

#define SS_PIN D8
#define RST_PIN D0
#define SERVO_PIN D1 // Chân D1 (GPIO5) dùng cho Servo

String rfid_uids[10]; // Giả sử tối đa 10 UID

MFRC522 rfid(SS_PIN, RST_PIN);

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

char wifiSSID[32] = "zunoiot";
char wifiPass[64] = "1234567899";
char mqttUser[32] = "ZiMrNROOWph6pTLfIXJt";
char mqttServer[32] = "139.59.97.249";
char *mqttPass = "";
char *mqttTopic = "v1/devices/me/telemetry";
int mqttPort = 1883;

String fw_title = "";
String fw_version = "v0.0.2";
String fw_tag = "";
int fw_size = 0;
String fw_checksum_algorithm = "";
String fw_checksum = "";

bool wifiConnected = false;
bool mqttConnected = false;

void setWarnLed(const int type);
bool connectToWiFi(const char *ssid, const char *password);
bool connectToMQTT();
void sendDataMqtt();
void sendDHT11Data();
void onBLEReceive(String jsonData);
void handleResetButton();
void callback(char *topic, byte *payload, unsigned int length);
void readRFID();
bool downloadFirmware(const char *deviceToken, const char *fwTitle, const char *fwVersion, const char *savePath = NULL);
void checkAndUpdateFirmware(const char *deviceToken, String fwTitle, String fwVersion);
void performUpdate(const char *filePath);

void setup()
{
    Serial.begin(115200);
    if (!SPIFFS.begin())
    {
        Serial.println("Failed to mount file system");
        return;
    }
    pinMode(SERVO_PIN, OUTPUT);
    SPI.begin();
    rfid.PCD_Init();
    digitalWrite(SERVO_PIN, LOW);
    client.setCallback(callback);
    setWarnLed(0);
    connectToWiFi(wifiSSID, wifiPass);
}

unsigned long pressStartTime = 0;
bool isLongPressHandled = false;

void loop()
{
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
        if (!client.connected())
            connectToMQTT();
        client.loop();
        readRFID();
        delay(100);
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
        String request = "{\"sharedKeys\":\"servoState,RFID_UIDs,fw_title,fw_version\"}";
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
        if (doc["shared"]["servoState"])
        {
            int servoState = doc["shared"]["servoState"];
            if (servoState == 1)
            {
                client.publish(mqttTopic, String("{\"RFID\":\"APP\", \"open\": true}").c_str());
                digitalWrite(SERVO_PIN, HIGH);
            }
        }

        // neu nhan duoc message moi servoState
        if (doc.containsKey("servoState"))
        {
            int servoState = doc["servoState"];
            if (servoState == 1)
            {
                digitalWrite(SERVO_PIN, HIGH);
                client.publish(mqttTopic, String("{\"RFID\":\"APP\", \"open\": true}").c_str());
            }
            else
            {
                digitalWrite(SERVO_PIN, LOW);
                client.publish(mqttTopic, String("{\"RFID\":\"APP\", \"open\": false}").c_str());
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

        if (doc["helloWorld"] == "ota")
        {
            Serial.println("TEST OTA");
        }

        // if(doc["shared"].containsKey("fw_version")){
        //   Serial.println("Phiên bản MQTT gửi về: " + String(doc["shared"]["fw_version"]));
        //   Serial.println("Phiên bản HIỆN TẠI : " + String(fw_version));
        //   if(!strcmp(fw_version, doc["shared"]["fw_version"])){
        //     Serial.println("BẮT ĐẦU UPDATE");
        //     checkAndUpdateFirmware(mqttUser, doc["shared"]["fw_title"].as<const char*>(), doc["shared"]["fw_version"].as<const char*>());
        //     fw_version = strdup(doc["shared"]["fw_version"].as<const char*>());
        //   }
        // }

        // Then modify your version check logic:
        if (doc["shared"].containsKey("fw_version"))
        {
            String new_version = doc["shared"]["fw_version"].as<String>();
            Serial.println("Phiên bản MQTT gửi về: " + new_version);
            Serial.println("Phiên bản HIỆN TẠI : " + fw_version);

            if (fw_version != new_version)
            {
                Serial.println("BẮT ĐẦU UPDATE");
                String new_title = doc["shared"]["fw_title"].as<String>();
                checkAndUpdateFirmware(mqttUser, new_title.c_str(), new_version.c_str());
                fw_version = new_version;
            }
        }

        if (doc.containsKey("fw_version"))
        {
            String new_version = doc["fw_version"].as<String>();
            Serial.println("Phiên bản MQTT gửi về: " + new_version);
            Serial.println("Phiên bản HIỆN TẠI : " + fw_version);

            if (fw_version != new_version)
            {
                Serial.println("BẮT ĐẦU UPDATE");
                String new_title = doc["fw_title"].as<String>();
                checkAndUpdateFirmware(mqttUser, new_title.c_str(), new_version.c_str());
                fw_version = new_version;
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
        client.loop(); // Đảm bảo MQTT luôn hoạt động
        Serial.println(rfid_uids[i]);

        if (rfid_uids[i] == uid)
        {
            Serial.println("Access granted! Moving servo...");
            digitalWrite(SERVO_PIN, HIGH);
            client.publish(mqttTopic, String("{\"RFID\":\"" + uid + "\", \"open\": true}").c_str());

            // Sử dụng while thay vì delay
            unsigned long startTime = millis();
            while (millis() - startTime < 3000)
            {
                client.loop(); // Đảm bảo MQTT vẫn chạy khi chờ

                yield(); // Cho phép ESP8266 xử lý tác vụ nền (WiFi, MQTT, watchdog)
            }

            digitalWrite(SERVO_PIN, LOW);
            client.publish(mqttTopic, String("{\"RFID\":\"" + uid + "\", \"open\": false}").c_str());
            return;
        }
    }
    Serial.println("Access denied!");
}

// Hàm tải firmware từ ThingsBoard
bool downloadFirmware(const char *deviceToken, const char *fwTitle, const char *fwVersion, const char *savePath)
{
    HTTPClient http;

    // Xây dựng URL request
    String url = "http://" + String(mqttServer) + ":8080/api/v1/";
    url += String(deviceToken);
    url += "/firmware?title=";
    url += String(fwTitle);
    url += "&version=";
    url += String(fwVersion);

    // if (chunkSize > 0) {
    //   url += "&size=";
    //   url += chunkSize;
    //   url += "&chunk=";
    //   url += chunk;
    // }

    Serial.print("Requesting firmware from: ");
    Serial.println(url);

    if (!http.begin(espClient, url))
    {
        Serial.println("Failed to begin HTTP connection");
        return false;
    }

    int httpCode = http.GET();

    if (httpCode == HTTP_CODE_OK)
    {
        Serial.println("Firmware download started");

        // Nếu không chỉ định savePath, trả về true nếu request thành công
        if (savePath == NULL)
        {
            http.end();
            return true;
        }

        // Mở file để ghi firmware
        File file = SPIFFS.open(savePath, "w");
        if (!file)
        {
            Serial.println("Failed to open file for writing");
            http.end();
            return false;
        }

        // Nhận dữ liệu và ghi vào file
        int len = http.getSize();
        uint8_t buff[128] = {0};
        WiFiClient *stream = http.getStreamPtr();

        while (http.connected() && (len > 0 || len == -1))
        {
            size_t size = stream->available();
            if (size)
            {
                int c = stream->readBytes(buff, ((size > sizeof(buff)) ? sizeof(buff) : size));
                file.write(buff, c);
                if (len > 0)
                {
                    len -= c;
                }
                Serial.print(".");
            }
            delay(1);
        }

        file.close();
        Serial.println("\nFirmware download complete");
        http.end();
        return true;
    }
    else
    {
        Serial.printf("HTTP request failed, error: %s\n", http.errorToString(httpCode).c_str());
        http.end();
        return false;
    }
}

// Hàm kiểm tra và cập nhật firmware
void checkAndUpdateFirmware(const char *deviceToken, const char *fwTitle, const char *fwVersion)
{
    // Tải firmware
    const char *binFile = "/firmware.bin";
    if (downloadFirmware(deviceToken, fwTitle, fwVersion, binFile))
    {
        // Tiến hành cập nhật firmware
        performUpdate(binFile);
    }
}

// Hàm cập nhật firmware
void performUpdate(const char *filePath)
{
    File updateFile = SPIFFS.open(filePath, "r");
    if (!updateFile)
    {
        Serial.println("Failed to open firmware file");
        return;
    }

    size_t updateSize = updateFile.size();
    if (updateSize == 0)
    {
        Serial.println("Firmware file is empty");
        updateFile.close();
        return;
    }

    Serial.println("Starting firmware update...");
    if (Update.begin(updateSize))
    {
        size_t written = Update.writeStream(updateFile);
        if (written == updateSize)
        {
            Serial.println("Firmware written successfully");
        }
        else
        {
            Serial.printf("Firmware written %d/%d bytes\n", written, updateSize);
        }

        if (Update.end())
        {
            Serial.println("OTA done!");
            if (Update.isFinished())
            {
                Serial.println("Update successfully completed. Rebooting...");
                updateFile.close();
                ESP.restart();
            }
            else
            {
                Serial.println("Update not finished? Something went wrong!");
            }
        }
        else
        {
            Serial.println("Error Occurred. Error #: " + String(Update.getError()));
        }
    }
    else
    {
        Serial.println("Not enough space to begin OTA");
    }

    updateFile.close();
}