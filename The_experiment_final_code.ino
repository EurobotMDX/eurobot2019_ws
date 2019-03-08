#include <WS2812FX.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

#define LED_COUNT 5
#define LED_PIN D5
#define MOTOR_PIN D2

char* ssid = "eurobot2019";
char* password = "eurobot2019";

const int port = 8000;
const char* host = "192.168.100.103";

WiFiClient client;
ESP8266WebServer server;
WS2812FX ws2812fx = WS2812FX(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

volatile bool activation_status = false;

void setup()
{
  ws2812fx.init();
  ws2812fx.setBrightness(255);
  ws2812fx.setSpeed(1000);
  ws2812fx.setColor(0x007BFF);
  ws2812fx.setMode(FX_MODE_STATIC);
  ws2812fx.start();
  ws2812fx.service();
  
  pinMode(MOTOR_PIN, OUTPUT);
  digitalWrite(MOTOR_PIN, LOW);

  Serial.begin(115200);
  WiFi.begin(ssid, password);
  Serial.print("Connecting");

  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(500);
  }

  Serial.println("");
  Serial.print("I am connected to: ");
  Serial.println(ssid);
  Serial.println("");
  Serial.println("IP Address: ");
  Serial.print(WiFi.localIP());

  server.on("/", [] ()
  {
    server.send(200, "text/plain", "You're at the Experiment");
  });
  
  server.on("/activate", activate);
  server.on("/deactivate", deactivate);
  server.begin();
}

void loop()
{
  server.handleClient();
  delay(300);
  toggle_neopixel();
}

void toggle_neopixel()
{
  static bool state = false;

  if (state)
  {
    if (activation_status)
    {
      ws2812fx.setColor(0x00FF00);
    }
    else
    {
      ws2812fx.setColor(0xFF0000);
    }
  }
  else
  {
    ws2812fx.setColor(0x000000);
  }

  state = !state;
  ws2812fx.service();
}

void activate()
{
  activation_status = true;
  digitalWrite(MOTOR_PIN, HIGH);
  server.send(200, "OK");
}

void deactivate()
{
  activation_status = false;
  digitalWrite(MOTOR_PIN, LOW);
  server.send(200, "OK");
}
