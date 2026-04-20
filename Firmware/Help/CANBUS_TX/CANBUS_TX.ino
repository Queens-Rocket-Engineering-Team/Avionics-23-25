// haha oh SHIT (transmitter edition)

#include "STM32_CAN.h"
#include <SoftwareSerial.h>

#define CAN_RX_PIN PB8
#define CAN_TX_PIN PB9
#define USB_RX_PIN PB10 // NOT CONNECTED
#define USB_TX_PIN PB11 // NOT CONNECTED
#define DB_LED_PIN PA15


#define USB_BAUD 9600 // FUCK IT 9600 BOYSSS


// FUCK hardware serial periperhals we doin SOFTWARE
SoftwareSerial usb(USB_RX_PIN, USB_TX_PIN); 
// CANBUS trash wahooo
STM32_CAN canb( CAN1, ALT );    //CAN1 ALT is PB8+PB9
static CAN_message_t CAN_msg;



void canSend(){

  //Start shifting bits (b0 = LSB)
  uint8_t b0 = 0x12;
  uint8_t b1 = 0x34;
  uint8_t b2 = 0x56;
  uint8_t b3 = 0x78;
  
  //Build the CANBUS message
  CAN_msg.id = 0x1;
  CAN_msg.len = 4;
  
  CAN_msg.buf[0] = b0;
  CAN_msg.buf[1] = b1 ;
  CAN_msg.buf[2] = b2 ;
  CAN_msg.buf[3] = b3 ;
  
  canb.write(CAN_msg);     //send
}//canSend()



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
  usb.println("Sending CAN packet");
  canSend();
  delay(1000);
  digitalWrite(DB_LED_PIN, !digitalRead(DB_LED_PIN));
}//loop()
