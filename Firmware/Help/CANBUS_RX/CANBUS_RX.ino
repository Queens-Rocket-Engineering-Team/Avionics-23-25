// haha oh SHIT (receiver edition)

#include "STM32_CAN.h"
#include <SoftwareSerial.h>

#define CAN_RX_PIN PB8
#define CAN_TX_PIN PB9
#define USB_RX_PIN PC13 // NOT CONNECTED
#define USB_TX_PIN PC14 // NOT CONNECTED
#define DB_LED_PIN PA15

#define USB_BAUD 9600 // FUCK IT 9600 BOYSSS


// FUCK hardware serial periperhals we doin SOFTWARE
SoftwareSerial usb(USB_RX_PIN, USB_TX_PIN); 
// CANBUS trash wahooo
STM32_CAN canb( CAN1, ALT );    //CAN1 ALT is PB8+PB9
static CAN_message_t CAN_msg ;




void setup() {
  // setup canubs
  canb.begin(); //automatic retransmission can be done using arg "true"
  canb.setBaudRate(62500); //62.5kbps
  // setup usb serial
  usb.begin(USB_BAUD);

  pinMode(DB_LED_PIN, OUTPUT);
}//setup()

void loop() {
  // put your main code here, to run repeatedly:
  while (canb.read(CAN_msg)) {
    usb.print(F("CAN ID = "));
    usb.println(CAN_msg.id, BIN);
    usb.print("CAN PAYLOAD = ");
    usb.print(CAN_msg.buf[0], HEX);
    usb.print(CAN_msg.buf[1], HEX);
    usb.print(CAN_msg.buf[2], HEX);
    usb.print(CAN_msg.buf[3], HEX);
    digitalWrite(DB_LED_PIN, !digitalRead(DB_LED_PIN));
  }//while
}//loop()
