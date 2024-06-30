// flashTable.cpp
#include "flashTable.h"
//https://github.com/PaulStoffregen/SerialFlash
#include <SerialFlash.h>
#include <SPI.h>

// Constructor
FlashTable::FlashTable(uint8_t numCols, uint16_t originRefreshInt, uint32_t maxSize, uint8_t tableNum, uint16_t buffSize) {

  _numCols = numCols;
  _originRefreshInt = originRefreshInt;
  _rowsSinceRaw = 0;

  //_flash = serialFlash;
  _maxSize = maxSize;
  _tableNum = tableNum;
  _buffSize = buffSize;

  _initialized = false;
  _lastValsPntr = new uint32_t[numCols];
  _bufPos = 0;
  _bufferPntr = new uint8_t[buffSize];

}//FlashTable()

// Destructor
FlashTable::~FlashTable() {
    delete[] _lastValsPntr;
    delete[] _bufferPntr;
}//~FlashTable()

// Creates or opens the nessesary file on
// the flash chip.
// Must be run before running any other function.
void FlashTable::init(SerialFlashChip *serialFlash) {
  // Generate filename
  char fName[8];
  itoa(_tableNum, fName, 8);

  // Create file if it doesn't exist
  bool findEmptySpot = true;
  if (!serialFlash->exists(fName)) {
    serialFlash->create(fName, _maxSize);
    findEmptySpot = false;
  }//if

  // Open file
  _file = serialFlash->open(fName);

  // Find the next empty spot in the file if file
  // exists (to append data instead of overwriting)
  if (findEmptySpot) {
    seekToEmpty();
  }//if
}//init()

// Applies offset to make signed
// int into unsigned int
uint32_t FlashTable::unsignify(int32_t value) {
  // apply offset of ((2^30)-1)//2
  return (uint32_t)(value + 536870911U);
}//unsignify()

// Returns the number of compressed
// bytes in the file (NOT equal to pos)
uint32_t FlashTable::getCurSize() {
  return _file.position()+1;
}//getCurSize()

// Returns the maximum number of
// bytes in the file
uint32_t FlashTable::getMaxSize() {
  return _file.size();
}//getMaxSize()



/*
 * Reads through the current data file and 
 * attempts to find where the data stops
 * and blank space begins. Does so by scanning
 * through bytes until a threshold of consecutive
 * empty bytes are found (e.g. 16).
 * 
 * Should only be run once, at startup.
 * 
 * Not ideal, and definitely wastes space, but
 * should be good enough to prevent corruption.
 */
void FlashTable::seekToEmpty() {

  // TODO: Make the course algorithm much more course, but then it slows down to check surroundings for a false complete. If bad, continue course. If good, move to fine. This will result in a much faster course algorithm overall.
  // Alternatively, make it seek 1000B forward each time, and then once it sees a 0xFF, then it scans NUM_EMPTY_TRIG forward for 0xFF bytes. If that fails, keep going 1000 forward. If it passes, it seeks BACKWARDS until it finds a non-0xFF byte, then it moves NUM_EMPTY_TRIG forward.

  // NOTE: WHEN COURSE_RES IS LARGE (>255) THE COURSE SEEK ALGORITHM BREAKS (seems to overflow past the size of the file?)

  const uint8_t NUM_EMPTY_TRIG = 16; // Number of bytes required to trigger an "empty" flag.
  const uint8_t EMPTY_VAL = 0xFF; // Value of an empty byte in flash
  const uint8_t COURSE_RES = 1000; // How many bytes the course seek algorithm should jump forward (higher = faster, lower = more reliable)
  uint8_t numEmpties = 0; // Current number of consecutive empty bytes (0xFF)

  bool courseDone = false;
  
  // Ensure at start
  _file.seek(0);

  // Run course seek algorithm
  uint8_t serBuffer[1];
  while (_file.position() < _file.size() && !courseDone) {
    _file.read(serBuffer, 1);
    if (serBuffer[0] == EMPTY_VAL) {
      // Found potential empty, scan forward to meet thresh
      //serial.println(F("Found potential empty"));
      numEmpties++;
      while (numEmpties < NUM_EMPTY_TRIG && _file.position() < _file.size()) {
        _file.read(serBuffer, 1);
        if (serBuffer[0] == EMPTY_VAL) {
          numEmpties++;
        } else {
          // Not empty area; continue course search
          numEmpties = 0;
          //serial.println(F("Not empty; continue"));
          break;
        }//if
      }//while (numEmpty < thresh)
      
      // Scan forward done; meet thresh of empty bytes?
      if (numEmpties >= NUM_EMPTY_TRIG) {
        // Thresh met, seek backwards until find non-EMPTY byte
        //serial.println(F("Empty section found; seeking backwards"));
        _file.seek(_file.position()-NUM_EMPTY_TRIG); // We know NUM_EMPTY_TRIG previous bytes are empty
        while (_file.position() > 0) {
          //digitalWrite(PA15, !digitalRead(PA15));
          //delay(25);
          _file.read(serBuffer, 1);
          if (serBuffer[0] != EMPTY_VAL) {
            // Data found; move NUM_EMPTY_TRIG-1 forward and that's new start pos
            //serial.println(F("Data found; ending seek"));
            _file.seek(_file.position()+NUM_EMPTY_TRIG);
            courseDone = true;
            break;
          }//if
          // Data not found yet; move one back from last read byte
          _file.seek(_file.position()-2);
          
        }//while (pos>0)
        
      }//if( numEmpty >= thresh)
      
    } else {
      // Not EMPTY; jump forward and keep searching
      numEmpties = 0;
      _file.seek(_file.position()+COURSE_RES-1);
    }//if (buf = EMPTY_VAL)

  }//while

  /*
  numEmpties = 0;
  _file.seek(_file.position() - NUM_EMPTY_TRIG+1);

  // Run fine seek algorithm
  while (numEmpties < NUM_EMPTY_TRIG && _file.position() < _file.size()) {
    _file.read(serBuffer, 1);
    if (serBuffer[0] == EMPTY_VAL) {
      numEmpties++;
    } else {
      numEmpties = 0;
    }//if
  }//while

  */

  // Move one position back and start writing
  // (to avoid triggering "findEmpty" here next
  // time due to NUM_EMPTY_TRIG empty bytes)
  _file.seek(_file.position() - 1);

  // Cap to size of file
  if ( _file.position() >= _file.size() ) {
    _file.seek(_file.size()-1);
  }//if
  
  //serial.print(F("Found new position at p="));
  //serial.println(_file.position());
}//seekToEmpty()



// Dumps all file data over the given
// Stream object using RDT techniques.
// WARNING: WILL PERMANENTLY BLOCK IF
// NO RESPONSES ARE GIVEN OVER STREAM.
// Start/End positions are inclusive.
void FlashTable::beginDataDump(Stream *stream, uint32_t strtPos, uint32_t endPos) {

  /* TODO: maybe break the block sending algorithm into it's own
   * "rdtSendBlock()" function that accepts an array of of size BLOCK_SIZE,
   * number of actual bytes in it, and BLOCK_SIZE itself.
   * Handles zero-padding, sending, and handles re-sending if
   * nessesary, to a limit. Returns true if continue, false if cancel.
  */

  // Setup
  const uint16_t BLOCK_SIZE = 512;
  uint8_t serBuffer[BLOCK_SIZE];
  uint32_t NUM_BYTES = (endPos-strtPos); //DATA bytes
  // (add extra block if data isn't perfectly divisible by block size)
  uint32_t NUM_BLOCKS = (NUM_BYTES/BLOCK_SIZE) + (NUM_BYTES%BLOCK_SIZE!=0);
  uint32_t curBlock = 0;

  // Seek to starting point
  uint32_t origPoint = _file.position();
  _file.seek(strtPos);
  
  // Clear incoming stream buffer
  while (stream->available()) {stream->read();}

  // Handshake; Send block size & incoming number of DATA bytes (!= payload size)
  // Send BLOCK_SIZE, NUM_BLOCKS, NUM_BYTES
  stream->write((byte)0x00);             //0 Blank
  stream->write(BLOCK_SIZE & 0xFF);      //1 LSB
  stream->write(BLOCK_SIZE>>8 & 0xFF);   //2 MSB
  stream->write(NUM_BLOCKS & 0xFF);      //3 LSB
  stream->write(NUM_BLOCKS>>8 & 0xFF);   //4 MSB
  stream->write(NUM_BYTES & 0xFF);       //5 LSB
  stream->write(NUM_BYTES>>8 & 0xFF);    //6
  stream->write(NUM_BYTES>>16 & 0xFF);   //7
  stream->write(NUM_BYTES>>24 & 0xFF);   //8 MSB
  stream->write((byte)0x00);             //9 Blank

  // Wait for ANY acknowledgement to send first block
  while (not stream->available()) {}
  while (stream->available()) {stream->read();}

  // Begin sending blocks
  while (curBlock < NUM_BLOCKS) {
    // ACTION: Compose & Send Block
    uint8_t byteBuff[BLOCK_SIZE];
    uint8_t byteIndex = 0;
    
    // Read block of bytes from file
    uint32_t amnt = endPos - _file.position();
    if (amnt > BLOCK_SIZE) {amnt = BLOCK_SIZE;}    
    _file.read(byteBuff, amnt);
    
    // Send real data
    for (uint32_t i=0; i<amnt; i++) {
      // TODO: SPLIT BYTE INTO TWO BYTES FOR ERROR CORRECTION
      stream->write(byteBuff[i]);
      byteIndex = (byteIndex+1)%16; //max 4-bits
    }//for(byte in block)
    
    // Send zero-padding if nessesary
    if (amnt < BLOCK_SIZE) {
      for (uint32_t i=amnt; i<BLOCK_SIZE; i++) {
        // TODO: SPLIT BYTE INTO TWO BYTES FOR ERROR CORRECTION
        stream->write((byte)0x00);
        byteIndex = (byteIndex+1)%16; //max 4-bits
      }//for(byte in block)
    }//if(need zero-pad)

    // ACTION: Wait for ACK of block
    while (not stream->available()) {}
    uint8_t ack = stream->read();
    while (stream->available()) {stream->read();}
    if (ack == 'N') {
      // Success; advance to next block
      curBlock++;
    } else if (ack == 'L') {
      // Failure; re-send last block
      _file.seek(_file.position()-amnt);
    } else {
      // Unknown; Cancel transaction
      return;
    }//if(ack)

  }//while(blocks are left)

  // TRANSMISSION CONCLUDED; return to original position in file
  _file.seek(origPoint);

  //TODO: Normal operation does NOT resume after running this function; something is broken somewhere.

}//beginDataDump()



void FlashTable::beginDataDump(Stream *stream) {
  beginDataDump(stream, 0, _file.position()); //whole file
}//beginDataDump()

void FlashTable::beginDataDump() {
  beginDataDump(&Serial, 0, _file.position()); //whole file
}//beginDataDump()



// Writes the given byte into the
// storage array. Flushes if needed.
bool FlashTable::writeByte(uint8_t in) {
  // Ensure file is not full
  if (_file.position() >= _file.size()) {
    return false;
  }//if
  
  // Write byte to buffer
  _bufferPntr[_bufPos] = in;
  _bufPos++;
  // Flush to file if buffer full
  if (_bufPos == _buffSize) {
    // Buffer is full, flush
    _file.write(_bufferPntr, _buffSize);
    _bufPos = 0;
    //Serial.println(F("DumpBuff"));
  }//if
  return true;
}//writeByte

// Writes the given uint32_t into the
// storage array (adjusts MSB = 0)
bool FlashTable::writeUint32(uint32_t in) {

  // Set Bit8 = 0
  uint8_t byte1 = 0b01111111 & ( in >> 24 );
  uint8_t byte2 = in >> 16;
  uint8_t byte3 = in >> 8;
  uint8_t byte4 = in;
  // Write
  writeByte(byte1);
  writeByte(byte2);
  writeByte(byte3);
  return writeByte(byte4);
  
}//writeUint32()

// Compresses and saves the given row
// of data to the storage array
bool FlashTable::writeRow(uint32_t rowData[]) {

  bool ok = true; // If write was successful

  // Check if should write raw values
  bool refreshOrigin = (_originRefreshInt > 0) && (_rowsSinceRaw >= _originRefreshInt);
  if (!_initialized || refreshOrigin) {
    // Pass through all columns
    for (int col=0; col<_numCols; col++) {
      _lastValsPntr[col] = rowData[col];
      ok = writeUint32(rowData[col]);
    }//for
    // Write complete
    _rowsSinceRaw = 0;
    _initialized = true;
    return ok;
  }//if

  // Iterate over all columns
  for (int col=0; col<_numCols; col++) {
    uint32_t lastVal = _lastValsPntr[col];
    uint32_t curVal = rowData[col];

    // Determine sign of offset
    bool signAdd = (curVal >= lastVal);
    // Determine magnitude of offset
    uint32_t offset = 0;
    if (signAdd) { offset = curVal-lastVal; }
    else { offset = lastVal-curVal; }

    // Determine how many bytes compression will yield
    uint8_t lvl = 4;
    if (offset <= LVL_2_MAX) { lvl=2; }
    else if (offset <= LVL_3_MAX) { lvl=3; }

    // Compress value to bytes
    switch (lvl) {
      case 2: {
        // Compress to 2 bytes
        // Bit8 = 1 (offset)
        uint8_t byte1 = 0b11100000 | (offset >> 8);  // Captures D13 to D09
        uint8_t byte2 = offset; // Captures D08 to D01
        // Bit7 = add (0 = subtract)
        if (!signAdd) {byte1 = byte1 & 0b10111111;}
        // Bit6 = size (0 = 2Byte)
        byte1 = byte1 & 0b11011111;
        // Write bytes
        writeByte(byte1);
        ok = writeByte(byte2);
        break;
      }//case2
      
      case 3: {
        // Compress to 3 bytes
        // Bit8 = 1 (offset)
        uint8_t byte1 = 0b11100000 | (offset >> 16); // Captures D21 to D17
        uint8_t byte2 = (offset >> 8); // Captures D16 to D09
        uint8_t byte3 = offset; // Captures D08 to D01
        // Bit7 = add (0 = subtract)
        if (!signAdd) {byte1 = byte1 & 0b10111111;}
        // Bit6 = size (1 = 3Byte, already set)
        // Write bytes
        writeByte(byte1);
        writeByte(byte2);
        ok = writeByte(byte3);
        break;
      }//case3
      
      default: {
        // Store uncompressed value
        ok = writeUint32(rowData[col]);
        break;
      }//case4
    }//switch
    
    // Store new value for this column
    _lastValsPntr[col] = rowData[col];
  }//for
  return ok;
}//writeRow()
