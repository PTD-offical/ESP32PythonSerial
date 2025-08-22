#include <Arduino.h>

String receivedMessage = "";

void setup() {
  Serial.begin(115200); // Initialize serial communication at 115200 baud rate
  pinMode(LED_BUILTIN, OUTPUT); // Assuming LED_BUILTIN is connected to an LED
  Serial.println("ESP32 Ready. Waiting for commands.");
}

void loop() {
  while (Serial.available()) {
    char incomingChar = Serial.read();
    if (incomingChar == '\n') { // Check for newline character as command terminator
      if (receivedMessage == "LED_ON") {
        digitalWrite(LED_BUILTIN, HIGH);
        Serial.println("LED turned ON");
      } else if (receivedMessage == "LED_OFF") {
        digitalWrite(LED_BUILTIN, LOW);
        Serial.println("LED turned OFF");
      } else {
        Serial.print("Unknown command: ");
        Serial.println(receivedMessage);
      }
      receivedMessage = ""; // Clear the message for the next command
    } else {
      receivedMessage += incomingChar;
    }
  }
}