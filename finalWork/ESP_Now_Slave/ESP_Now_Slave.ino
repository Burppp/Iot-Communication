/**
    Flow: Slave
    Step 1 : ESPNow Init on Slave
    Step 2 : Update the SSID of Slave with a prefix of `slave`
    Step 3 : Set Slave in AP mode
    Step 4 : Register for receive callback and wait for data
    Step 5 : Once data arrives, print it in the serial monitor

    Application layer: Slave
    Step 1 : Using ESP_NOW to get cmd_vel
    Step 2 : Serial output to stm32
*/

#include <esp_now.h>
#include <WiFi.h>

#define CHANNEL 1
#define UART_BUFFER_SIZE 8

// Init ESP Now with fallback
void InitESPNow() 
{
    WiFi.disconnect();
    if (esp_now_init() == ESP_OK) 
    {
        // Serial.println("ESPNow Init Success");
    }
    else 
    {
        // Serial.println("ESPNow Init Failed");
        // Retry InitESPNow, add a counte and then restart?
        // InitESPNow();
        // or Simply Restart
        ESP.restart();
    }
}

// config AP SSID
void configDeviceAP() 
{
    const char *SSID = "Slave_1";
    bool result = WiFi.softAP(SSID, "Slave_1_Password", CHANNEL, 0);
    if (!result) 
    {
        // Serial.println("AP Config failed.");
    } 
    else 
    {
        // Serial.println("AP Config Success. Broadcasting with AP: " + String(SSID));
        // Serial.print("AP CHANNEL "); Serial.println(WiFi.channel());
    }
}

void setup() 
{
    Serial.begin(115200);
    Serial1.begin(115200, SERIAL_8N1, 6, 7);
    // Serial.println("ESPNow Slave");
    //Set device in AP mode to begin with
    WiFi.mode(WIFI_AP);
    // configure device AP mode
    configDeviceAP();
    // This is the mac address of the Slave in AP Mode
    // Serial.print("AP MAC: "); Serial.println(WiFi.softAPmacAddress());
    // Init ESPNow with a fallback logic
    InitESPNow();
    // Once ESPNow is successfully Init, we will register for recv CB to
    // get recv packer info.
    esp_now_register_recv_cb(OnDataRecv);
}

// callback when data is recv from Master
uint8_t recvBuffer[UART_BUFFER_SIZE] = {0};
uint8_t bufferIndex = 0;
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) 
{
    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
            mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);   
    neopixelWrite(RGB_BUILTIN, 0x40, 0x40, 0x40);
    memcpy_P(recvBuffer, data, data_len);
    Serial.write(recvBuffer, data_len);
    Serial1.write(recvBuffer, data_len);
    memset(recvBuffer, 0, data_len);
    // recvBuffer[bufferIndex++] = *data;
    // if(*data == 'j')                   
    // {
    //   String str = String(recvBuffer, 8);
    //   Serial.print(str);
    //   Serial1.print(str);
    //   memset(recvBuffer, 0, sizeof(recvBuffer));
    //   bufferIndex = 0;
    // }
}

uint8_t RGB_val[3] = {0x00, 0x00, 0x00};
bool RGB_reverse = false;
void loop() 
{
    // Chill
    // if(!RGB_reverse)
    //   RGB_val[0] += 5;
    // else
    //   RGB_val[0] -= 5;
    // if(RGB_val[0] >= 0x40)
    //   RGB_reverse = true;
    // if(RGB_val[0] <= 0x0A)
    //   RGB_reverse = false;
    // neopixelWrite(RGB_BUILTIN, RGB_val[0], RGB_val[1], RGB_val[2]);
    // delay(1);
}
