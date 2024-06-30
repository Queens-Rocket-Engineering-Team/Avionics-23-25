// flashTable.h
#ifndef flashTable_h
#define flashTable_h
#include <Arduino.h>
//https://github.com/PaulStoffregen/SerialFlash
#include <SerialFlash.h>
#include <SPI.h>

class FlashTable {
  public:
    FlashTable(uint8_t numCols, uint16_t originRefreshInt, uint32_t maxSize, uint8_t tableNum, uint16_t buffSize);
    ~FlashTable(); // Destructor
    void init(SerialFlashChip *serialFlash, Stream *stream);
    bool writeRow(uint32_t rowData[]);
    uint32_t unsignify(int32_t value);
    uint32_t getCurSize();
    uint32_t getMaxSize();
    void beginDataDump(Stream *stream, uint32_t strtPos, uint32_t endPos);
    void beginDataDump(Stream *stream);
    void beginDataDump();
    
  private:
    const uint16_t LVL_2_MAX = 8191;    //2^13-1
    const uint32_t LVL_3_MAX = 2097151; //2^21-1

    bool writeByte(uint8_t in);
    bool writeUint32(uint32_t in);
    void seekToEmpty(Stream *stream);

    //SerialFlashChip *_flash;
    SerialFlashFile _file;
    //SerialFlashFile _posFile;
    uint32_t _maxSize;
    uint8_t _tableNum;
    uint16_t _buffSize;
    
    uint8_t _numCols;
    uint16_t _originRefreshInt;
    uint16_t _rowsSinceRaw;
    bool _initialized;
    uint32_t *_lastValsPntr;
    
    uint8_t *_bufferPntr;
    uint16_t _bufPos;
    
};

#endif
