
// Load Wi-Fi library
#include <ESP8266WiFi.h>
#include <OneWire.h> 
#include <DallasTemperature.h>
#include <DHT.h>
#include <Adafruit_BMP280.h>

Adafruit_BMP280 bmp; // I2C Interface

//water temperature
#define ONE_WIRE_BUS D6
OneWire oneWire(ONE_WIRE_BUS); 
DallasTemperature sensors(&oneWire);

//air temp + humidity
#define DHTPIN D3
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

const int sensorPin = 0;const int sensorPin = 0;


// Set network credentials
const char* ssid     = ":) 2.4G";
const char* password = "123654789";

// Set web server port number to 80
WiFiServer server(80);

// Variable to store the HTTP request
String header;


// Current time
unsigned long currentTime = millis();
// Previous time
unsigned long previousTime = 0; 
// Define timeout time in milliseconds (example: 2000ms = 2s)
const long timeoutTime = 5000;








// set up global variables for the light level a calibration value and
//and a raw light value
int lightCal;
int lightVal;
long atmosphericPressure;


void setup()
{
  Serial.begin(9600);
  Serial.print("Hello");
  if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  pinMode(0, INPUT); //light
  pinMode(D3, INPUT); //air temp + humidity
  lightCal = analogRead(sensorPin);
  //take a single reading from the light sensor and store it in the lightCal
  //variable. This will give us a preliminary value to compare against in the loop

  //setup server
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  // Print local IP address and start web server
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  server.begin();

  //temperature
   sensors.begin();

   //air temp  + humidity
   dht.begin();
}

void loop(){
  delay(200);
  WiFiClient client = server.available();   // Listen for incoming clients

  if (client) {                             // If a new client connects,
    Serial.println("New Client.");          // print a message out in the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    currentTime = millis();
    previousTime = currentTime;
    while (client.connected() && currentTime - previousTime <= timeoutTime) { // loop while the client's connected
      currentTime = millis();         
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
            
            // turns the GPIOs on and off
            if (header.indexOf("GET /data") >= 0) {
            delay(20);
            lightVal = analogRead(sensorPin);
            delay(20);
            client.println(lightVal);

            atmosphericPressure = bmp.readPressure()/100;

            client.println(atmosphericPressure);
            
             

             delay(20);

              //temperature
              sensors.requestTemperatures();
              delay(20);
              client.println(sensors.getTempCByIndex(0));
              
              client.println(digitalRead(0));

              //air temp + humidity
              
             float h = dht.readHumidity();  
             float t = dht.readTemperature(); 
             float hic = dht.computeHeatIndex(t, h, false);

             
              client.println(h);
              client.println(t);
              client.println(hic);
            }
            
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
