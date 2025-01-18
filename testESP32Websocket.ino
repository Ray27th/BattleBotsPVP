#include <WiFi.h>
#include <WebSocketsClient.h>

// WiFi credentials
const char* ssid = "Hacker";
const char* password = "mo123456";

// Websocket server details
const char* websocket_host = "192.168.36.113"; // Replace with the Python server's IP
const uint16_t websocket_port = 8765;

WebSocketsClient webSocket;

void webSocketEvent(WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {
    case WStype_TEXT:
      Serial.print("Received Command: ");
      Serial.println((char*)payload);
      break;
    case WStype_DISCONNECTED:
      Serial.println("WebSocket disconnected");
      break;
    case WStype_CONNECTED:
      Serial.println("WebSocket connected");
      break;
  }
}

void setup() {
  Serial.begin(115200);

  // Connect to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  // Initialize WebSocket
  webSocket.begin(websocket_host, websocket_port, "/");
  webSocket.onEvent(webSocketEvent);
}

void loop() {
  webSocket.loop();
}
