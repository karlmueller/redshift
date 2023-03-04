/****************************************************************************
http://retro.moe/unijoysticle2

Copyright 2021 Ricardo Quesada

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
****************************************************************************/

/*
BLUEPAD32 USING CPU0. Can use CPU1 to do processing and send over esp now 
*/

#include "sdkconfig.h"
#ifndef CONFIG_BLUEPAD32_PLATFORM_ARDUINO
#error "Must only be compiled when using Bluepad32 Arduino platform"
#endif  // !CONFIG_BLUEPAD32_PLATFORM_ARDUINO

#include <Arduino.h>
#include <Bluepad32.h>
//#include "enow.h"
#include <WiFi.h>
#include <esp_now.h>

#define STICK_ULIM 511 //check on this value
#define STICK_LLIM -512
#define STICK_RANGE (STICK_ULIM-STICK_LLIM)

float change_threshold = 0.075; //proportion of stick range to change before and acutal command is either sent or interpreted by the joint node

const uint8_t broadcast_address[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

//message structure for sending. Must be identical in sender and recipient
typedef struct message_object{
    int xStick_left = 0;
    int yStick_left = 0;

    int xStick_right = 0;
    int yStick_right = 0;

    int r_trigger = 0;
    int r_bumper = 0;

    int l_trigger = 0;
    int l_bumper = 0;


} struct_sendmsg;

struct_sendmsg myData;  

esp_now_peer_info_t peerInfo;

int prev_ry;
int prev_rx;
int prev_ly;
int prev_lx;
int prev_l_trigger;
int prev_r_trigger;
int prev_l_bumper;
int prev_r_bumper;

bool isNew(int prev_controller, int controller_input) { //testing a function to set a threshold for the stick value change
    if (abs(controller_input - prev_controller) < change_threshold*STICK_RANGE){
        return true;
    }
    return false;
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status){
    //Serial.print("\r\nLast Packet Send Status:\t");
    //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Failure");
    Serial.print("\r[Packet_status] >>  ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "SUCC" : "FAIL");
}

void enow_setup(){
    WiFi.mode(WIFI_STA);
    Serial.print("Wifi DEBUG Mac Address: ");
    Serial.println(WiFi.macAddress());

    if (esp_now_init() != ESP_OK){
        Serial.println("Error in ESP-now initialization");
        return;
    }

    esp_now_register_send_cb(OnDataSent);
    
    memcpy(peerInfo.peer_addr, broadcast_address, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK){
        Serial.println("Failed to add peer:");
        return;
    }
}

GamepadPtr myGamepads[1]; //BP32_MAX_GAMEPADS];

// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.
void onConnectedGamepad(GamepadPtr gp) {
    bool foundEmptySlot = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myGamepads[i] == nullptr) {
            Serial.print("CALLBACK: Gamepad is connected, index=");
            Serial.println(i);
            // Additionally, you can get certain gamepad properties like:
            // Model, VID, PID, BTAddr, flags, etc.
            GamepadProperties properties = gp->getProperties();
            Serial.print("Gamepad model: ");
            Serial.print(gp->getModelName());
            Serial.print(", VID/PID: ");
            Serial.print(properties.vendor_id, HEX);
            Serial.print(":");
            Serial.print(properties.product_id, HEX);
            Serial.println();
            myGamepads[i] = gp;
            foundEmptySlot = true;
            break;
        }
    }
    if (!foundEmptySlot) {
        Serial.println("CALLBACK: Gamepad connected, but could not found empty slot");
    }
}

void onDisconnectedGamepad(GamepadPtr gp) {
    bool foundGamepad = false;

    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myGamepads[i] == gp) {
            Serial.print("CALLBACK: Gamepad is disconnected from index=");
            Serial.println(i);
            myGamepads[i] = nullptr;
            foundGamepad = true;
            break;
        }
    }

    if (!foundGamepad) {
        Serial.println("CALLBACK: Gamepad disconnected, but not found in myGamepads");
    }
}

// Arduino setup function. Runs in CPU 1
void setup() {
    Serial.begin(115200);

    enow_setup(); //initialize exp_now in a single command according to function above
    String fv = BP32.firmwareVersion();
    Serial.print("Firmware: ");
    Serial.println(fv);

    // Setup the Bluepad32 callbacks
    BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad);

    // "forgetBluetoothKeys()" should be called when the user performs
    // a "device factory reset", or similar.
    // Calling "forgetBluetoothKeys" in setup() just as an example.
    // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
    // But might also fix some connection / re-connection issues.
    BP32.forgetBluetoothKeys();
}

void command_controller()
{
    if (abs(myData.xStick_left - prev_lx) >= 5 || abs(myData.yStick_left - prev_ly) >= 5 || abs(myData.xStick_right  - prev_rx) >= 5 || abs(myData.yStick_right - prev_ry) >= 5) {

        esp_err_t result = esp_now_send(broadcast_address, (uint8_t *) &myData, sizeof(myData));
        //if (result == ESP_OK) {
        //Serial.println("s");
        //}
        //else {
        //Serial.println("e");
        //}

        //vTaskDelay(10 / portTICK_PERIOD_MS);
    }

}

// Arduino loop function. Runs in CPU 1
void loop() {
    // This call fetches all the gamepad info from the NINA (ESP32) module.
    // Just call this function in your main loop.
    // The gamepads pointer (the ones received in the callbacks) gets updated
    // automatically.
    BP32.update();

    // It is safe to always do this before using the gamepad API.
    // This guarantees that the gamepad is valid and connected.
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        GamepadPtr myGamepad = myGamepads[i];

        if (myGamepad && myGamepad->isConnected()) {
            
            myData.xStick_left = myGamepad->axisX();
            myData.yStick_left = myGamepad->axisY();

            myData.xStick_right = myGamepad->axisRX();
            myData.yStick_right = myGamepad->axisRY();

            myData.l_trigger = myGamepad->brake();
            myData.r_trigger = myGamepad->throttle();

            Serial.println(myData.xStick_left);

            command_controller();
            
            // You can query the axis and other properties as well. See Gamepad.h
            // For all the available functions.
        }
    }

    delay(150); //vTaskDelay(75 / portTICK_PERIOD_MS);
}
