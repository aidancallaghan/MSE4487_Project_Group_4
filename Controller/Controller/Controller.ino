// 
// MME 4487 Lab 4 Controller
// 
//  Language: Arduino (C++)
//  Target:   ESP32
//  Author:   Michael Naish
//  Date:     2024 10 07
//

// #define PRINT_SEND_STATUS                             // uncomment to turn on output packet send status
// #define PRINT_INCOMING                                // uncomment to turn on output of incoming data

#include <Arduino.h>
#include "ESP32_NOW.h"
#include "WiFi.h"
#include <esp_mac.h>                                  // For the MAC2STR and MACSTR macros

// Definitions
#define ESPNOW_WIFI_IFACE WIFI_IF_STA                 // Wi-Fi interface to be used by the ESP-NOW protocol
#define ESPNOW_WIFI_CHANNEL 4                         // Channel to be used by the ESP-NOW protocol

// Structs

// Control data packet structure
// The attribute "packed" ensures that the struct is not padded (all data is contiguous in the memory and without gaps).
// The maximum size of the complete message is 250 bytes (ESP_NOW_MAX_DATA_LEN).
typedef struct {
  int dir;                                            // drive direction: 1 = forward, -1 = reverse, 0 = stop
  uint32_t time;                                      // time packet sent
  int speed;                                          //pot value from 0-4095
  bool left;                                          //is left button pressed?
  bool right;                                         //is right button pressed?
  bool bot;
  bool mid;
  bool top;

} __attribute__((packed)) esp_now_control_data_t;

// Drive data packet structure
// The attribute "packed" ensures that the struct is not padded (all data is contiguous in the memory and without gaps).
// The maximum size of the complete message is 250 bytes (ESP_NOW_MAX_DATA_LEN).
typedef struct {
  uint32_t time;                                      // time packet sent
  int colourTemp;                                     //Number which represents the colour score
} __attribute__((packed)) esp_now_drive_data_t;

// Button structure
struct Button {
  const int pin;                                      // GPIO pin for button
  unsigned int numberPresses;                         // counter for number of button presses
  unsigned int lastPressTime;                         // time of last button press in ms
  bool pressed;                                       // flag for button press event
  bool state;                                         // current state of button; 0 = pressed; 1 = unpressed
  bool lastState;                                     // last state of button
};

// Function declarations
void doHeartbeat();
void failReboot();
void ARDUINO_ISR_ATTR buttonISR(void* arg);

// Constants
const int cHeartbeatLED = 2;                          // GPIO pin of built-in LED for heartbeat
const int cHeartbeatInterval = 500;                   // heartbeat blink interval, in milliseconds
const int cStatusLED = 26;                            // GPIO pin of communication status LED
const int cDebounceDelay = 20;                        // button debounce delay in milliseconds
const int cMaxDroppedPackets = 20;                    // maximum number of packets allowed to drop

// Variables
uint32_t lastHeartbeat = 0;                           // time of last heartbeat state change
uint32_t lastTime = 0;                                // last time of motor control was updated
uint32_t commsLossCount = 0;                          // number of sequential sent packets have dropped

Button buttonFwd = {14, 0, 0, false, true, true};     // forward, NO pushbutton on GPIO 14, low state when pressed
Button buttonRev = {13, 0, 0, false, true, true};     // reverse, NO pushbutton on GPIO 12, low state when pressed
Button buttonLeft = {12, 0, 0, false, true, true};     // left, NO pushbutton on GPIO 27, low state when pressed
Button buttonRight = {27, 0, 0, false, true, true};     // right, NO pushbutton on GPIO 13, low state when pressed
Button buttonBot = {32, 0, 0, false, true, true};     // reverse, NO pushbutton on GPIO 12, low state when pressed
Button buttonMid = {33, 0, 0, false, true, true};     // left, NO pushbutton on GPIO 27, low state when pressed
Button buttonTop = {25, 0, 0, false, true, true};     // right, NO pushbutton on GPIO 13, low state when pressed

// REPLACE WITH MAC ADDRESS OF YOUR DRIVE ESP32
uint8_t receiverMacAddress[] = {0xAC,0x15,0x18,0xD6,0x81,0x34};  // MAC address of drive 00:01:02:03:04:05 
esp_now_control_data_t controlData;                   // data packet to send to drive system
esp_now_drive_data_t inData;                          // data packet from drive system

// Classes
//
// The ESP_NOW_NetworkPeer class inherits from ESP_NOW_Peer and implements the _onReceive and _onSent methods.
// For more information about the ESP_NOW_Peer class, see the ESP_NOW_Peer class in the ESP32_NOW.h file.
//Copied from Lab4
class ESP_NOW_Network_Peer : public ESP_NOW_Peer {
public:
  ESP_NOW_Network_Peer(const uint8_t *mac_addr, const uint8_t *lmk = NULL)
    : ESP_NOW_Peer(mac_addr, ESPNOW_WIFI_CHANNEL, ESPNOW_WIFI_IFACE, lmk) {}

  ~ESP_NOW_Network_Peer() {}

  //Copied from Lab4
  bool begin() {
    // Assumes that the ESP-NOW protocol is already initialized
    if (!add()) {
      log_e("Failed to initialize ESP-NOW or register the peer");
      return false;
    }
    return true;
  }

  //Copied from Lab4
  bool send_message(const uint8_t *data, size_t len) {
    if (data == NULL || len == 0) {
      log_e("Data to be sent is NULL or has a length of 0");
      return false;
    }
    // Call the parent class method to send the data
    return send(data, len);
  }

  // callback function for when data is received
  //Copied from Lab4
  void onReceive(const uint8_t *data, size_t len, bool broadcast) {
    if (len == 0)                                       // if empty packet
    {
      return;                                           // return
    }
    memcpy(&inData, data, sizeof(inData));              // store drive data from controller
    #ifdef PRINT_INCOMING
    Serial.printf("%d\n", inData.time);
    #endif
  }
  
  // callback function for when data is sent
  //Copied from Lab4
  void onSent(bool success) {
    if (success) {
      #ifdef PRINT_SEND_STATUS
        log_i("Unicast message reported as sent %s to peer " MACSTR, success ? "successfully" : "unsuccessfully", MAC2STR(addr()));
      #endif
      commsLossCount = 0;
    }
    else {
      digitalWrite(cStatusLED, 1);                      // turn on communication status LED
      commsLossCount++;
    }
  }
};

// Peers
ESP_NOW_Network_Peer *peer;

void setup() {

  //Setup Serial
  Serial.begin(9600);                               // standard baud rate for ESP32 serial monitor
  while (!Serial) {                                   // wait for Serial to start
    delay(10);                                        // okay to delay during setup
  }

  //Setup WIFI
  WiFi.mode(WIFI_STA);                                // use WiFi in station mode
  WiFi.setChannel(ESPNOW_WIFI_CHANNEL);               // set WiFi channel to use with peer
  while (!WiFi.STA.started()) {                       // wait for WiFi to start
    delay(100);                                       // okay to delay during setup
  }

  //Connection Debug
  Serial.print("MAC address for controller "); 
  Serial.println(WiFi.macAddress());                  // print MAC address of ESP32
  
  // Configure GPIO
  pinMode(cHeartbeatLED, OUTPUT);                     // configure built-in LED for heartbeat as output
  pinMode(cStatusLED, OUTPUT);                        // configure GPIO for communication status LED as output

  //FWD
  pinMode(buttonFwd.pin, INPUT_PULLUP);                             // configure GPIO for forward button pin as an input with pullup resistor
  attachInterruptArg(buttonFwd.pin, buttonISR, &buttonFwd, CHANGE); // Configure forward pushbutton ISR to trigger on change

  //REV
  pinMode(buttonRev.pin, INPUT_PULLUP);                             // configure GPIO for reverse button pin as an input with pullup resistor
  attachInterruptArg(buttonRev.pin, buttonISR, &buttonRev, CHANGE); // Configure reverse pushbutton ISR to trigger on change

  //LEFT
  pinMode(buttonLeft.pin, INPUT_PULLUP);                              // configure GPIO for reverse button pin as an input with pullup resistor
  attachInterruptArg(buttonLeft.pin, buttonISR, &buttonLeft, CHANGE); // Configure reverse pushbutton ISR to trigger on change

  //RIGHT
  pinMode(buttonRight.pin, INPUT_PULLUP);                               // configure GPIO for reverse button pin as an input with pullup resistor
  attachInterruptArg(buttonRight.pin, buttonISR, &buttonRight, CHANGE); // Configure reverse pushbutton ISR to trigger on change
  
  //BOTTOM
  pinMode(buttonBot.pin, INPUT_PULLUP);                               // configure GPIO for reverse button pin as an input with pullup resistor
  attachInterruptArg(buttonBot.pin, buttonISR, &buttonBot, CHANGE); // Configure reverse pushbutton ISR to trigger on change
  
  //MID
  pinMode(buttonMid.pin, INPUT_PULLUP);                               // configure GPIO for reverse button pin as an input with pullup resistor
  attachInterruptArg(buttonMid.pin, buttonISR, &buttonMid, CHANGE); // Configure reverse pushbutton ISR to trigger on change

  //TOP
  pinMode(buttonTop.pin, INPUT_PULLUP);                               // configure GPIO for reverse button pin as an input with pullup resistor
  attachInterruptArg(buttonTop.pin, buttonISR, &buttonTop, CHANGE); // Configure reverse pushbutton ISR to trigger on change

  // Initialize the ESP-NOW protocol
  if (!ESP_NOW.begin()) {
    Serial.printf("Failed to initialize ESP-NOW\n");
    failReboot();
  }

  //add drive as peer
  //Copied from Lab4
  peer = new ESP_NOW_Network_Peer(receiverMacAddress);
  if (peer == nullptr || !peer->begin()) {
    Serial.printf("Failed to create or register the drive peer\n");
    failReboot();
  }
  else {
    Serial.printf("Successfully added peer %x:%x:%x:%x:%x:%x\n", receiverMacAddress[0], receiverMacAddress[1], 
                                                                 receiverMacAddress[2], receiverMacAddress[3], 
                                                                 receiverMacAddress[4], receiverMacAddress[5]);
  }

  memset(&inData, 0, sizeof(inData));                 // clear drive data
  memset(&controlData, 0, sizeof(controlData));       // clear controller data

}

void loop() {

  uint32_t curTime = micros();                        // capture current time in microseconds
  if (curTime - lastTime > 10000) {                   // wait ~10 ms
    lastTime = curTime;
    controlData.time = curTime;                       // update transmission time
  

    if (!buttonFwd.state) {                           // forward pushbutton pressed
      controlData.dir = 1;
    }
    else if (!buttonRev.state) {                      // reverse pushbutton pressed
      controlData.dir = -1;
    }
    else {                                            // no input, stop
      controlData.dir = 0;
    }

    if (!buttonBot.state){

      controlData.mid = false;
      controlData.top = false;
      controlData.bot = true;
    }
    else if (!buttonMid.state){

      controlData.bot = false;
      controlData.top = false;
      controlData.mid = true;

    }
    else if (!buttonTop.state){

      controlData.bot = false;
      controlData.mid = false;
      controlData.top = true;
      
    }
    else{

    }


    controlData.left = !buttonLeft.state;              //set the struct variables
    controlData.right = !buttonRight.state;            //set the struct variables

    controlData.speed = analogRead(34);               //Pot value sent as a variable in the structure
    

    // if drive appears disconnected, update control signal to stop before sending
    if (commsLossCount > cMaxDroppedPackets) {
      //controlData.dir = 0;
    }
    // send control signal to drive
    if (peer->send_message((const uint8_t *) &controlData, sizeof(controlData))) {
      digitalWrite(cStatusLED, 0);                    // if successful, turn off communucation status LED
    }
    else {
      digitalWrite(cStatusLED, 1);                    // otherwise, turn on communication status LED
    }

    
    Serial.print(controlData.bot);
    Serial.print("    ");
    Serial.print(controlData.mid);
    Serial.print("    ");
    Serial.print(controlData.top);
    Serial.print("    ");
    Serial.print(controlData.left);
    Serial.print("    ");
    Serial.print(controlData.right);
    Serial.print("    ");
    Serial.print(controlData.speed);
    Serial.print("    ");
    Serial.println(control