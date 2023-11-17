#include "STM32_CAN.h"
STM32_CAN Can1 (CAN1, ALT_2, RX_SIZE_256, TX_SIZE_256);
static CAN_message_t CAN_msg;
void setup() {
 
Can1.setBaudRate(500000);
Can1.setFilter( 0, 0x153, 0x1FFFFFFF );
Can1.setFilter( 1, 0x613, 0x1FFFFFFF );
}

void loop() {

}