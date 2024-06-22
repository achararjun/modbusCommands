#include <HardwareSerial.h>
#include <string.h>

const int mdDeRe = 15;  // Define GPIO for DE/RE pin
const int rxPin = 16;   // Define GPIO for RX pin
const int txPin = 17;   // Define GPIO for TX pin    

HardwareSerial mySerial(2); // Use UART2

// CRC calculation function
uint16_t calculateCRC16(const uint8_t* data, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

// Function to append CRC to the message
void appendCRC(uint8_t* message, size_t length) {
    uint16_t crc = calculateCRC16(message, length);
    message[length] = crc & 0xFF;       // Low byte of CRC
    message[length + 1] = (crc >> 8) & 0xFF; // High byte of CRC
}

void setup() {
  pinMode(mdDeRe, OUTPUT);    
  digitalWrite(mdDeRe, LOW); // Set to receive mode initially
 
  mySerial.begin(115200, SERIAL_8N1, rxPin, txPin); // Initialize UART2
  Serial.begin(115200);    // Initialize Serial Monitor for debugging

  // Initialize motor and set to continuous run
  initializeMotor();
}

void loop() {
  // Main loop can be used for other tasks
}

// Function to initialize and start motor
void initializeMotor() {
  uint8_t enableMotor[] = {0x01, 0x06, 0x18, 0x01, 0x11, 0x22};     // Enable the motor
  uint8_t setVelocityMode[] = {0x01, 0x06, 0x00, 0x03, 0x00, 0x02}; // Set control mode to velocity mode
  uint8_t setSpeed200[] = {0x01, 0x06, 0x01, 0xE1, 0x00, 0xC8};     // Set speed to 200 RPM
  uint8_t startMotor[] = {0x01, 0x06, 0x18, 0x01, 0x40, 0x01};      // Start the motor

  // Define the lengths of the original messages
  size_t enableMotorLength = sizeof(enableMotor);
  size_t setVelocityModeLength = sizeof(setVelocityMode);
  size_t setSpeed200Length = sizeof(setSpeed200);
  size_t startMotorLength = sizeof(startMotor);

  // Create buffers with space for CRC
  uint8_t enableMotorWithCRC[enableMotorLength + 2];
  uint8_t setVelocityModeWithCRC[setVelocityModeLength + 2];
  uint8_t setSpeed200WithCRC[setSpeed200Length + 2];
  uint8_t startMotorWithCRC[startMotorLength + 2];

  // Copy original messages to buffers
  memcpy(enableMotorWithCRC, enableMotor, enableMotorLength);
  memcpy(setVelocityModeWithCRC, setVelocityMode, setVelocityModeLength);
  memcpy(setSpeed200WithCRC, setSpeed200, setSpeed200Length);
  memcpy(startMotorWithCRC, startMotor, startMotorLength);

  // Calculate and append CRC
  appendCRC(enableMotorWithCRC, enableMotorLength);
  appendCRC(setVelocityModeWithCRC, setVelocityModeLength);
  appendCRC(setSpeed200WithCRC, setSpeed200Length);
  appendCRC(startMotorWithCRC, startMotorLength);

  // Send messages to initialize and start motor
  sendMessage(enableMotorWithCRC, enableMotorLength + 2);
  delay(500); // Short delay
  sendMessage(setVelocityModeWithCRC, setVelocityModeLength + 2);
  delay(500); // Short delay
  sendMessage(setSpeed200WithCRC, setSpeed200Length + 2);
  delay(500); // Short delay
  sendMessage(startMotorWithCRC, startMotorLength + 2);
}

// Function to send a message over RS485
void sendMessage(uint8_t* message, size_t length) {
  // Set RS485 to transmit mode
  digitalWrite(mdDeRe, HIGH); 
  delay(10); // Ensure the pin state is stable

  // Send Modbus message
  Serial.print("Sending Modbus message: ");
  for (int i = 0; i < length; i++) {
      Serial.print(message[i], HEX);
      Serial.print(" ");
  }
  Serial.println();
  mySerial.write(message, length);
  mySerial.flush(); // Ensure all data is sent
  delay(100); // Short delay to ensure message is sent completely

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
}
