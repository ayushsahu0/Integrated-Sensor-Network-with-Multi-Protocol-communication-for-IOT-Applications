#include <ESP8266WiFi.h>
#include <DHT.h>

// Wi-Fi credentials
const char* ssid = "M42";                // Replace with your Wi-Fi SSID
const char* password = "keepgoing19";    // Replace with your Wi-Fi password

// Server configuration
const char* serverIP = "192.168.172.31"; // Replace with your server's IP address
const int serverPort = 5005;             // Replace with your server's port

// DHT11 sensor configuration
#define DHTPIN D2        // GPIO pin connected to the DHT11 sensor
#define DHTTYPE DHT11    // Define the DHT type (DHT11)
DHT dht(DHTPIN, DHTTYPE);

WiFiClient client;

void setup() {
    // Initialize serial communication
    Serial.begin(9600);
    Serial.println("Starting...");

    // Initialize DHT sensor
    dht.begin();

    // Connect to Wi-Fi
    Serial.print("Connecting to Wi-Fi");
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.print(".");
    }
    Serial.println("\nConnected to Wi-Fi");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
}

void sendDataToServer() {
    // Read temperature and humidity
    float humidity = dht.readHumidity();
    float temperature = dht.readTemperature();

    // Check if readings are valid
    if (isnan(humidity) || isnan(temperature)) {
        Serial.println("Failed to read from DHT sensor!");
        return;
    }

    // Prepare the payload
    String payload = "Temperature: " + String(temperature) + "°C, Humidity: " + String(humidity) + "%";
    Serial.println("Sending: " + payload);

    // Send data to the server
    if (client.connect(serverIP, serverPort)) {
        client.println(payload);
        client.stop();
        Serial.println("Data sent to server");
    } else {
        Serial.println("Failed to connect to server");
    }
}

void loop() {
    sendDataToServer();
    delay(10000); // Send data every 10 seconds
}
