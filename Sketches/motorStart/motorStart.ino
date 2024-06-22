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
}

void loop() {
  uint8_t readPeakCurrent[] = {0x01, 0x03, 0x01, 0x91, 0x00, 0x01, 0xD3, 0x1B}; // Modbus message
  uint8_t motorEnable[] = {0x01, 0x06, 0x18, 0x01, 0x11, 0x22, 0xD3, 0x6E};

  size_of_readPeakCurrent = sizeof(readPeakCurrent);
  

  // Set RS485 to transmit mode
  Serial.println("Setting RS485 to transmit mode...");
  digitalWrite(mdDeRe, HIGH); 
  delay(10); // Ensure the pin state is stable
  
  // Send Modbus message
  Serial.println("Sending Modbus message...");
  mySerial.write(readPeakCurrent, sizeof(readPeakCurrent));
  mySerial.flush(); // Ensure all data is sent
  delay(10); // Short delay to ensure message is sent completely
  
  // Set RS485 to receive mode
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
