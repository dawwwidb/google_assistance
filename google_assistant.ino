/***************************************************
  Adafruit MQTT Library ESP8266 Example

  Must use ESP8266 Arduino from:
    https://github.com/esp8266/Arduino

  Works great with Adafruit's Huzzah ESP board & Feather
  ----> https://www.adafruit.com/product/2471
  ----> https://www.adafruit.com/products/2821

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Tony DiCola for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/
#include <ESP8266WiFi.h>

//adafruit
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

//LCD 2x16
#include <Wire.h>
#include <LiquidCrystal_I2C.h>


#define Port_Button D7      //Button       
#define Port_Sound D8       //spiker
#define Port_LED D0         // Analog output pin that the LED is attached to
#define Port_analog_in_pin A0      // Analog input pin that the potentiometer is attached to

//RGB wspólny +
#define Port_LEDR D3  //R
#define Port_LEDG D4  //G + dioda esp8266
#define Port_LEDB D5  //B


int Value_analog_in = 0;        // value read from the pot
int Value_analog_out = 0;        // value output to the PWM (analog out) - przeskalowany
int State_Button = 0;           //Stan przycisku

// Set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27, 16, 2);

#include "passwd.h" //Paswords
/*
  //************************* WiFi Access Point *********************************

  #define WLAN_SSID       "...your SSID..."
  #define WLAN_PASS       "...your password..."

  //************************* Adafruit.io Setup *********************************

  #define AIO_SERVER      "io.adafruit.com"
  #define AIO_SERVERPORT  1883                   // use 8883 for SSL
  #define AIO_USERNAME    "...your AIO username (see https://accounts.adafruit.com)..."
  #define AIO_KEY         "...your AIO key..."
*/


/************ Global State (you don't need to change this!) ******************/

// Create an ESP8266 WiFiClient class to connect to the MQTT server.
WiFiClient client;
// or... use WiFiFlientSecure for SSL
//WiFiClientSecure client;

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

/****************************** Feeds ***************************************/

// Setup a feed called 'photocell' for publishing.
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>
//Adafruit_MQTT_Publish photocell = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/photocell");
Adafruit_MQTT_Publish Pub_Light = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/esp8266.light");  //Publikacja stanu

// Setup a feed called 'onoff' for subscribing to changes.
//Adafruit_MQTT_Subscribe light = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/light");
Adafruit_MQTT_Subscribe Sub_Light = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/esp8266.light"); //Pobranie stanu


/*************************** Sketch Code ************************************/

// Bug workaround for Arduino 1.6.6, it seems to need a function declaration
// for some reason (only affects ESP8266, likely an arduino-builder bug).
void MQTT_connect();

void setup() {
  Serial.begin(115200);

  pinMode( Port_Sound, OUTPUT);
  pinMode( Port_LED, OUTPUT);

  //RGB
  pinMode( Port_LEDR, OUTPUT);
  pinMode( Port_LEDG, OUTPUT);
  pinMode( Port_LEDB, OUTPUT);
  digitalWrite(Port_LEDR, LOW);Serial.println(" R");delay(100);digitalWrite(Port_LEDR, HIGH);delay(100);
  digitalWrite(Port_LEDG, LOW);Serial.println(" G");delay(100);digitalWrite(Port_LEDG, HIGH);delay(100);
  digitalWrite(Port_LEDB, LOW);Serial.println(" B");delay(100);digitalWrite(Port_LEDB, HIGH);delay(100);
  //delay(500);

  //Przycisk
  pinMode(Port_Button, INPUT);

  lcd.begin();// initialize the LCD
  lcd.clear();// - czyści ekran
  lcd.backlight();// Turn on the blacklight and print a message.
  lcd.print("Witaj Swiecie!");


  // Connect to WiFi access point.
  Serial.println(); Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WLAN_SSID);

  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  //Wifi connected
  Serial.println("WiFi connected");
  Serial.println("IP address: "); Serial.println(WiFi.localIP());


  lcd.clear();// - czyści ekran//lcd.print("                ");
  lcd.setCursor(0, 0);
  lcd.print(WiFi.localIP());

  tone(Port_Sound, 2000); delay(500); noTone(Port_Sound);



  // Setup MQTT subscription for onoff feed.
  mqtt.subscribe(&Sub_Light); //pobranie stanu Light
}

uint32_t x = 0;

void loop() {


  //Value_analog_in = 0;        // value read from the pot
  //Value_analog_out = 0;        // value output to the PWM (analog out) - przeskalowany

  //odczyt wartości 0-1023
  Value_analog_in = analogRead(Port_analog_in_pin);
  // mappowanie zakresu
  Value_analog_out = map(Value_analog_in, 0, 1023, 0, 255);
  Serial.print("sensor = ");
  Serial.print(Value_analog_in);
  Serial.print("\t output = ");
  Serial.println(Value_analog_out);


State_Button = digitalRead(Port_Button);
    // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
  if (State_Button == HIGH) {
    // turn LED_RED on:
    digitalWrite(Port_LEDG, HIGH);
  } else {
    // turn LED off:
    digitalWrite(Port_LEDG, LOW);
  }



  //lcd.clear();// - czyści ekran
  // Ensure the connection to the MQTT server is alive (this will make the first
  // connection and automatically reconnect when disconnected).  See the MQTT_connect
  // function definition further below.
  MQTT_connect();
  // this is our 'wait for incoming subscription packets' busy subloop
  // try to spend your time here

  //lcd.clear();// - czyści ekran
  lcd.setCursor(0, 1);
  lcd.print("Przelacznik:");

  if (Value_analog_out<10) {lcd.setCursor(13, 0);lcd.print("  "); lcd.print(Value_analog_out);digitalWrite(Port_LEDB, LOW);digitalWrite(Port_LEDR, HIGH);}
  else if (Value_analog_out<100) {lcd.setCursor(13, 0); lcd.print(" ");lcd.print(Value_analog_out);digitalWrite(Port_LEDB, HIGH);digitalWrite(Port_LEDR, LOW);}
  else {lcd.setCursor(13, 0);lcd.print(Value_analog_out);digitalWrite(Port_LEDB, HIGH);digitalWrite(Port_LEDR, HIGH);}


  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(500))) {
    if (subscription == &Sub_Light) {
      Serial.print(F("Got: "));
      Serial.print((char *)Sub_Light.lastread);
      Serial.println(" LIGHT");

      if (strcmp((char *)Sub_Light.lastread, "ON") == 0) {
        digitalWrite(Port_LED, HIGH);
        lcd.setCursor(12, 1); lcd.print("ON ");
        tone(Port_Sound, 1000); delay(250); noTone(Port_Sound);
      }
      if (strcmp((char *)Sub_Light.lastread, "OFF") == 0) {
        digitalWrite(Port_LED, LOW);
        lcd.setCursor(12, 1); lcd.print("OFF");
        tone(Port_Sound, 3000); delay(100); noTone(Port_Sound); delay(50); tone(Port_Sound, 3000); delay(100); noTone(Port_Sound);
      }
    }

  }

  // Now we can publish stuff!
  /*Serial.print(F("\nSending photocell val "));
    Serial.print(x);
    Serial.print("...");
    if (! photocell.publish(x++)) {
    Serial.println(F("Failed"));
    } else {
    Serial.println(F("OK!"));
    }*/

  // ping the server to keep the mqtt connection alive
  // NOT required if you are publishing once every KEEPALIVE seconds
  /*
    if(! mqtt.ping()) {
    mqtt.disconnect();
    }
  */
}

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3;

  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Retrying MQTT connection in 5 seconds...");
    mqtt.disconnect();
    delay(5000);  // wait 5 seconds
    retries--;
    if (retries == 0) {
      // basically die and wait for WDT to reset me
      while (1);
    }
  }
  Serial.println("MQTT Connected!");
  tone(Port_Sound, 2000); //Wygeneruj sygnał o częstotliwości 2000Hz
  delay(500);
  tone(Port_Sound, 800); //Wygeneruj sygnał o częstotliwości 2000Hz
  delay(500);
  noTone(Port_Sound);
}
