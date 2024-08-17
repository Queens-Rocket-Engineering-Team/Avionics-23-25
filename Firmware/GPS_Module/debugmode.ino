// Empties all bytes from incoming serial buffer.
// Used by Debug mode
void emptySerialBuffer() {
  while (softSerial.available()) {softSerial.read();}
}//emptySerialBuffer()



// Called when Debug mode is activated;
// all normal board functions will cease.
// Only purpose is to respond to serial
// commands.
void debugMode() {

  while (true) {

    // Empty buffer
    emptySerialBuffer();

    // Wait for command
    while (!softSerial.available()) {}
    uint8_t cmd = softSerial.read();

    if (cmd == 'I') {
      // "Identify" command; return board name
      softSerial.println(F("GPS"));
    }//if
    if (cmd == 'F') {
      // "FlashInfo" command; return flash usage stats
      softSerial.print(table.getMaxSize());
      softSerial.print(F(","));
      softSerial.println(table.getCurSize());   
    }//if
    if (cmd == 'D') {
      // "DumpFlash" command; dump all flash contents via serial
      table.beginDataDump(&softSerial); //func overload; dump everything
    }//if
    if (cmd == 'E') {
      // "EraseFlash" command; completely erase contents of flash.
      // Should be restarted afterwards
      softSerial.println(F("Erasing Flash"));
      SerialFlash.eraseAll();
      while (SerialFlash.ready() == false) {}
      softSerial.println(F("Complete"));
    }//if
    if (cmd == 'Q') {
      // QUERY SENSORS
      softSerial.print(F("Null"));
    }//if

  }//while

}//debugMode()
