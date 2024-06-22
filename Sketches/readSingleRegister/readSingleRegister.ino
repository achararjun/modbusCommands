#include <HardwareSerial.h>

const int mdDeRe = 15;  
const int rxPin = 16;
const int txPin = 17;    

HardwareSerial mySerial(2);

void setup() {
  pinMode(mdDeRe, OUTPUT);    
  digitalWrite(mdDeRe, LOW); // Set to receive mode initially
 
  mySerial.begin(9600, SERIAL_8N1, rxPin, txPin);
  Serial.begin(9600);    
  Serial.println("Setup complete");
}

void loop() {
  uint8_t modbusMessage[] = {0x01, 0x03, 0x01, 0x91, 0x00, 0x01, 0xD4, 0x1B}; // Modbus message

  // Set RS485 to transmit mode
  Serial.println("Setting RS485 to transmit mode...");
  digitalWrite(mdDeRe, HIGH); 
  delay(10); // Ensure the pin state is stable
  
  // Send Modbus message
  Serial.println("Sending Modbus message...");
  mySerial.write(modbusMessage, sizeof(modbusMessage));
  mySerial.flush(); // Ensure all data is sent
  delay(10); // Short delay to ensure message is sent completely
  
  // Set RS485 to receive mode
  Serial.println("Setting RS485 to receive mode...");
  digitalWrite(mdDeRe, LOW); 
  delay(10); // Short delay to allow for mode switch
  
  // Read response from slave
  Serial.println("Checking for response...");
  if (mySerial.available()) {
    Serial.println("Data received:");
    while (mySerial.available() > 0) {
      byte incomingByte = mySerial.read();
      Serial.print(incomingByte, HEX);
      Serial.print(" ");
    }
    Serial.println();
  } else {
    Serial.println("No data received.");
  }

  delay(5000); // Delay before next loop iteration
}
