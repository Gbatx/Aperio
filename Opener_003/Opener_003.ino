/*
 * Opener Ver 3 
 * 
 * (1) NodeMCU ESP8266
 * (2) Fujitsu JV3S-KT 3V SPST Relays (with flyback diode)
 * (1) SSD1306 OLED 0.91" I2C OLED Display
 * (2) 12mm Momentary Push Button Switches
 * (3) 10k Ohm Resistors
 * (1) External Reed Switch and wire (optional)
 * (1) Micro-USB cable and 5V Charger or 3.3v/5V battery pack
 * (1) Breadboard and hook-up wires
 * 
*/

#include <ESP8266WiFi.h>          //https://github.com/esp8266/Arduino

//needed for library
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager
#include <Arduino.h>
#include <U8g2lib.h>              //for I2C OLED
#include <Wire.h>

//for LED status during startup
#include <Ticker.h>
Ticker ticker;

// Constants
#define SD2 9
#define SD3 10

const int button1Pin = D7;        // pushbutton 1 pin
const int button2Pin = D8;        // pushbutton 2 pin
const int reedPin = SD2;          // reed switch pin
const int relay1Pin =  D1;        // relay 1 pin - Note: pins D1 and D2 (GPIO5 and GPIO4) are the only high impedance pins
const int relay2Pin =  D2;        // relay 2 pin -       during startup on the ESP8266
const int SCApin = D5;            // OLED pins
const int SCLpin = D6;            // 
const int dataInterval = 30 * 1000;       // timer to post data
const unsigned long period = 30 * 1000;   // wait period = number of seconds * 1000 milliseconds
const char *ssidAP = "Aperio 0001";         // default access point ssid
const char *passwordAP = "passw0rd";      // default access point password
// Variables

byte mac[6];                        // MAC address
unsigned long startMillis;
unsigned long currentMillis;

int relay1Count = 0;          // keeps count of relay changes since program start
int relay2Count = 0;          // will eventually store in file system to keep running total
int connectStatus = 0;
char* doorStatus = "";        // variable for door Open or door Closed
String ipString = "";         // ip address string
long rssi = -50;              // signal strength
int bars = 0;                 // signal strength indicator
String rssiMessage = "";
  
// U8g2 Contructor List (Frame Buffer) for OLED 128x32
U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ SCLpin, /* data=*/ SCApin);   // pin remapping with ESP8266 HW I2C

WiFiServer server(80);        // set web server port to 80
String header;                // variable to store the HTTP request

void tick()
{
  //toggle state
  int state = digitalRead(BUILTIN_LED);  // get the current state of GPIO1 pin
  digitalWrite(BUILTIN_LED, !state);     // set pin to the opposite state
}

//gets called when WiFiManager enters configuration mode
void configModeCallback (WiFiManager *myWiFiManager) {
  
  Serial.println("");
  Serial.println("Entered config mode");
  WiFi.macAddress(mac);
  Serial.print("MAC: ");
  Serial.print(mac[5],HEX);
  Serial.print(":");
  Serial.print(mac[4],HEX);
  Serial.print(":");
  Serial.print(mac[3],HEX);
  Serial.print(":");
  Serial.print(mac[2],HEX);
  Serial.print(":");
  Serial.print(mac[1],HEX);
  Serial.print(":");
  Serial.println(mac[0],HEX);
  Serial.println("");
  
  Serial.println(WiFi.softAPIP());
  //if you used auto generated SSID, print it
  Serial.println(myWiFiManager->getConfigPortalSSID());
  //entered config mode, make led toggle faster
  ticker.attach(0.2, tick);
  
  // displays the access point ssid and default ip address
  u8g2.clearBuffer();                     // clear the internal memory
  u8g2.setFont(u8g2_font_helvR12_tr);     // choose a suitable font
  u8g2.drawStr(0, 12, ssidAP);            // write AP ssid to the internal memory
  u8g2.drawStr(1, 28, "192.168.4.1");     // write AP IP Address to the internal memory
  drawAPsymbol();
  u8g2.sendBuffer();                      // transfer internal memory to the display
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  u8g2.begin();   // for OLED display
  /* u8g2 fonts
   *  u8g2_font_ncenB08_tr - 8 point
   *  u8g2_font_helvB12_tf - 12 point
   *  u8g2_font_helvR12_tr - 12 point
   *  u8g2_font_helvR14_tf - 14 point
   *  
   *  More at https://github.com/olikraus/u8g2/wiki/fntlistall
   */
   
  u8g2.clearBuffer();                   // clear the internal memory
  u8g2.setFont(u8g2_font_helvR12_tr);   // choose a suitable font
  u8g2.drawStr(0,12,"Connecting...");   // write something to the internal memory
  drawNoSignal();
  u8g2.sendBuffer();                    // transfer internal memory to the display
  delay(1000);  
  
  pinMode(BUILTIN_LED, OUTPUT);       // set LED pins as output
  pinMode(relay1Pin, OUTPUT);         // set relay pins as output
  pinMode(relay2Pin, OUTPUT);         //
  pinMode(button1Pin, INPUT);         // initialize the pushbutton pins
  pinMode(button2Pin, INPUT);         //  and
  pinMode(reedPin, INPUT);            // door sensor pin as input

  // Set button and reed pins as interrupts, assign interrupt function and set CHANGE mode
  attachInterrupt(digitalPinToInterrupt(button1Pin), button1Changed, CHANGE); 
  attachInterrupt(digitalPinToInterrupt(button2Pin), button2Changed, CHANGE); 
  attachInterrupt(digitalPinToInterrupt(reedPin), reedChanged, CHANGE); 
        
  // start ticker with 0.5 because we start in AP mode and try to connect
  ticker.attach(0.5, tick);

  // WiFiManager
  // Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;
  // reset settings - for testing
  // wifiManager.resetSettings();

  // set callback that gets called when connecting to previous WiFi fails, and enters Access Point mode
  wifiManager.setAPCallback(configModeCallback);

  // fetches ssid and pass and tries to connect
  // if it does not connect it starts an access point with the specified name
  // and goes into a blocking loop awaiting configuration
  if (!wifiManager.autoConnect(ssidAP, passwordAP)) {
    Serial.println("failed to connect and hit timeout");
    //reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(1000);
  }

  //if you get here you have connected to the WiFi

  ticker.detach();    // stop built-in LED blink timer
  //Turn LED OFF
  digitalWrite(BUILTIN_LED, HIGH);
  
  u8g2.clearBuffer();                   // clear the internal memory
  u8g2.setFont(u8g2_font_helvR12_tr);   // choose a suitable font
  u8g2.drawStr(0,12,"Connected");       // write something to the internal memory
  u8g2.sendBuffer();                    // transfer internal memory to the display
  delay(1000);  
  
  // Done with Setup   
  Serial.println("");

  getStatus();                // displays door status and rssi strength
  startMillis = millis();     // start timer for rssi signal strength
  
  server.begin();     // begin web server
}

void loop() {
  WiFiClient client = server.available();   // Listen for incoming clients
  long uptime;
  
  currentMillis = millis();                       // gets current time
  if (currentMillis - startMillis >= period) {    // compares elapsed time to wait period
    getStatus();                                  // displays door status and rssi strength
    startMillis = currentMillis;                  // resets start time
  }
  rssi = WiFi.RSSI();

  // web based control
  if (client) {                             // If a new client connects,
    Serial.println("New Client.");          // print a message out in the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected()) {            // loop while the client's connected
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        header += c;
        if (c == '\n') {                    // if the byte is a newline character
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();
            
            // turns the Relay on and then off
            if (header.indexOf("GET /1/pressed") >= 0) {
              Serial.println("");
              digitalWrite(BUILTIN_LED, LOW);
              digitalWrite(relay1Pin, HIGH);         // activate relay
              delay(1500);
              digitalWrite(relay1Pin, LOW);          // deactivate relay
              relay1Count += 1;
              Serial.print("Relay count: ");
              Serial.println(relay1Count);
              if (doorStatus == "Door Closed") {
                digitalWrite(BUILTIN_LED, HIGH);
              } 
            } else if (header.indexOf("GET /2/pressed") >= 0) {
              Serial.println("");
              digitalWrite(BUILTIN_LED, LOW);
              digitalWrite(relay2Pin, HIGH);         // activate relay
              delay(1500);
              digitalWrite(relay2Pin, LOW);          // deactivate relay
              relay2Count += 1;
              Serial.print("Relay count: ");
              Serial.println(relay2Count);
              if (doorStatus == "Door Closed") {
                digitalWrite(BUILTIN_LED, HIGH);
              } 
            }
            
            // Display the HTML web page
            client.println("<!DOCTYPE html><html>");
            client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
            client.println("<meta http-equiv=\"refresh\" content=\"8; url=http://" + ipString + "/\">");
            client.println("<link rel=\"icon\" href=\"data:,\">");
            // CSS to style the on/off buttons 
            // Feel free to change the background-color and font-size attributes to fit your preferences
            client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}");
            client.println(".button { background-color: #195B6A; border: none; color: white; padding: 16px 40px;");
            client.println("text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}");
            client.println(".button2 {background-color: #003865;}</style></head>");
            
            // Web Page Heading
            client.println("<body><h1>Door Controller</h1>");
            client.println("<p><b>" + ipString + "</b><br>");
            client.print("Signal strength: ");
            client.print(rssi);
            client.print(rssiMessage);
            client.println("</p>");
            
            // Display current state for door
            client.print("<h2>Door Status: ");
            client.print(doorStatus);
            client.println("</h2>");

            // Display buttons
            client.println("<p><a href=\"/1/pressed\"><button class=\"button\">Button 1</button></a></p>");
            client.println("<p><a href=\"/2/pressed\"><button class=\"button\">Button 2</button></a></p>");

            // Display Uptime
            uptime = millis();                    // # of milliseconds since boot
            uptime = uptime / 1000;               // uptime total in seconds
            uptime = uptime / 60;                 // uptime total in minutes
            uptime = uptime / 60;                 // uptime total in hours
            uptime = uptime / 24;                 // uptime total in days
            
            client.print("<p>Uptime: ");
            client.print(uptime);
            client.println(" days</p>");

            // end of HTML
            client.println("</body></html>");                      

            // The HTTP response ends with another blank line
            client.println();
            // Break out of the while loop
            break;
          } else { // if you got a newline, then clear currentLine
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }
      }
    }
    // Clear the header variable
    header = "";
    // Close the connection
    client.stop();
    Serial.println("Client disconnected.");
    Serial.println("");
    }
}

void button1Changed() {
  if (digitalRead(button1Pin) == HIGH) {
    relay1Count += 1;
    Serial.print("Button 1 pressed (");
    Serial.print(relay1Count);
    Serial.println(")");    
    digitalWrite(relay1Pin, HIGH);         // activate relay    
    digitalWrite(BUILTIN_LED, LOW);
  } else {
    Serial.println("Button 1 released");
    digitalWrite(relay1Pin, LOW);          // deactivate relay
    digitalWrite(BUILTIN_LED, HIGH);
    if (doorStatus == "Door Closed") {
      digitalWrite(BUILTIN_LED, HIGH);
    } else {
      digitalWrite(BUILTIN_LED, LOW);
    }
  }

}

void button2Changed() {
  if (digitalRead(button2Pin) == HIGH) {
    relay2Count += 1;
    Serial.print("Button 2 pressed (");
    Serial.print(relay2Count);
    Serial.println(")");
    digitalWrite(relay2Pin, HIGH);         // activate relay    
    digitalWrite(BUILTIN_LED, LOW);
  } else {
    Serial.println("Button 2 released");
    digitalWrite(relay2Pin, LOW);          // deactivate relay
    digitalWrite(BUILTIN_LED, HIGH);
    if (doorStatus == "Door Closed") {
      digitalWrite(BUILTIN_LED, HIGH);
    } else {
      digitalWrite(BUILTIN_LED, LOW);
    }
  }
}

void reedChanged() {
  getStatus();
}

void getStatus() {
  if (digitalRead(reedPin) == HIGH) {
    doorStatus = "Door Closed";
    digitalWrite(BUILTIN_LED, HIGH);
  } else {
    doorStatus = "Door Open";
    digitalWrite(BUILTIN_LED, LOW);
  }
  Serial.println(doorStatus);

  rssi = WiFi.RSSI();
  bars = getBarsSignal(rssi);
  Serial.print("RSSI: ");
  Serial.println(rssi);   
  Serial.print("Signal bars: ");
  Serial.println(bars);
  Serial.println("");
      
  u8g2.clearBuffer();                     // clear the internal memory
  u8g2.setFont(u8g2_font_helvR12_tr);     // choose a suitable font
  ipString = WiFi.localIP().toString();
  u8g2.drawStr(0, 12, ipString.c_str());  // write IP address to the internal memory
  u8g2.drawStr(1, 28, doorStatus);        // write status to the internal memory
  drawBars();
  u8g2.sendBuffer();                      // transfer internal memory to the display  
}

void drawBars() {
  switch (bars) {
    case 0:
      drawNoSignal();
      rssiMessage = "  Messed up";
      break;
    case 1:
      u8g2.drawBox(104,27,4,3);
      u8g2.drawFrame(109,24,4,6);
      u8g2.drawFrame(114,21,4,9);
      u8g2.drawFrame(119,18,4,12);
      u8g2.drawFrame(124,15,4,15); 
      rssiMessage = "  Wrecked";
      break;
    case 2:
      u8g2.drawBox(104,27,4,3);
      u8g2.drawBox(109,24,4,6);
      u8g2.drawFrame(114,21,4,9);
      u8g2.drawFrame(119,18,4,12);
      u8g2.drawFrame(124,15,4,15); 
      rssiMessage = "  Sad";
      break;
    case 3:
      u8g2.drawBox(104,27,4,3);
      u8g2.drawBox(109,24,4,6);
      u8g2.drawBox(114,21,4,9);
      u8g2.drawFrame(119,18,4,12);
      u8g2.drawFrame(124,15,4,15); 
      rssiMessage = "  Not bad";
      break;
    case 4:
      u8g2.drawBox(104,27,4,3);
      u8g2.drawBox(109,24,4,6);
      u8g2.drawBox(114,21,4,9);
      u8g2.drawBox(119,18,4,12);
      u8g2.drawFrame(124,15,4,15);
      rssiMessage = "  Sweet"; 
      break;
    case 5:
      u8g2.drawBox(104,27,4,3);
      u8g2.drawBox(109,24,4,6);
      u8g2.drawBox(114,21,4,9);
      u8g2.drawBox(119,18,4,12);
      u8g2.drawBox(124,15,4,15); 
      rssiMessage = "  Wicked";
      break;
  } 
}

void drawNoSignal() {
      u8g2.drawFrame(104,29,4,3);
      u8g2.drawFrame(109,29,4,3);
      u8g2.drawFrame(114,29,4,3);
      u8g2.drawFrame(119,29,4,3);
      u8g2.drawFrame(124,29,4,3);  
}

void drawAPsymbol() {
      u8g2.drawBox(105,22,3,12);
      u8g2.drawBox(110,26,3,6);
      u8g2.drawBox(115,29,3,3);
      u8g2.drawBox(120,26,3,6);
      u8g2.drawBox(125,22,3,12);
      u8g2.drawDisc(116,22,3,U8G2_DRAW_ALL);
}

int getBarsSignal(long rssi){
  // 5. High quality: 90% ~= -55db
  // 4. Good quality: 75% ~= -65db
  // 3. Medium quality: 50% ~= -75db
  // 2. Low quality: 30% ~= -85db
  // 1. Unusable quality: 8% ~= -96db
  // 0. No signal
  int bars;
  
  if (rssi > -55) { 
    bars = 5;
  } else if (rssi < -55 & rssi > -65) {
    bars = 4;
  } else if (rssi < -65 & rssi > -75) {
    bars = 3;
  } else if (rssi < -75 & rssi > -85) {
    bars = 2;
  } else if (rssi < -85 & rssi > -96) {
    bars = 1;
  } else {
    bars = 0;
  }
  return bars;
}
