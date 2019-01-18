#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

WiFiClient client;
ESP8266WebServer server;

uint8_t pin_led = 16;
const int motor_pin = D2;

char* ssid = "eurobot2019";
char* password = "eurobot2019";

const int port = 8000;
const char* host = "192.168.100.103";

void setup()
{
  pinMode(pin_led, OUTPUT);
  pinMode(motor_pin, OUTPUT);

  digitalWrite(motor_pin, LOW);
  
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

  server.on("/toggle", toggleLED);
  server.begin();
}

void loop()
{
  server.handleClient();

  if (millis() % 100 == 0)
  {
    if (check_status())
    {
      Serial.println("Experiment Enabled!");
      digitalWrite(motor_pin, HIGH);
    }
  }
}

void toggleLED()                 
{
  digitalWrite(pin_led, digitalRead(pin_led));
  server.send(204, "");
}

bool check_status()
{
  if (client.connect(host, port))
  {
    client.print(String("GET /start_experiment") + " HTTP/1.1\r\n" +
                 "Host: " + host + "\r\n" +
                 "Connection: close\r\n" +
                 "\r\n"
                );

    Serial.println("[Response:]");

    String response = "";

    while (client.connected() || client.available())
    {
      if (client.available())
      {
        client.find('{');client.find('{');
        response += client.readStringUntil('}');
      }
    }
    client.stop();
    Serial.println("\n[Disconnected]");

    Serial.println("");
    Serial.println(response);
    Serial.println("");
    
    return response == "True";
  }
  else
  {
    Serial.println("connection failed!");
    client.stop();
    return false;
  }
}
