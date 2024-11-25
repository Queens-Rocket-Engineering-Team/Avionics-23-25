/*
 * File containing web server configuration.
 * 
 * To be appended to main file upon compilation.
 */



// Replaces placeholders in the Debug page's
// HTML before serving it.
// Called for each placeholder
String processorDebug(const String& var) {
  if (var == "FirmwareVersion") {
    return FIRMWARE_VERSION;
  } else if (var == "CompileDate") {
    return (String(__DATE__) + ", " + String(__TIME__));
  } else if (var == "WiFiPower") {
    return String("IUnk");
  } else if (var == "WiFiChannel") {
    return String(WIFI_CHANNEL);
  }
  return String();
}//processorDebug()



String readSDFile(String path) {
  File file = SD_MMC.open(path);
  if (!file) {
    Serial.println("Failed to open file for reading");
    return "";
  }

  String fileContent;
  while (file.available()) {
    fileContent += (char)file.read();
  }
  file.close();
  return fileContent;
}//readSDFile



/*
 * Sets up all the required endpoints for
 * the async webserver
 */
void configWebServer() {

  // Route for root / web page
  webServer.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/index.html", String(), false, processorDebug);
  });

  // Route for debug web page
  webServer.on("/debug", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/debug.html", String(), false, processorDebug);
  });

  // Route for debug web page
  webServer.on("/dashboard", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/dashboard.html", String(), false, processorDebug);
  });

  // Debug webpage DATA endpoint
  webServer.on("/debugData", HTTP_GET, [](AsyncWebServerRequest *request){
    // Return string containing LOTS of debug data
    // Only include dynamic variables
    
    // Uptime,MainLoopSpeed,MaxMainLoopSpeed,FreeHeap,AmbientTemp (4)
    String resp = String(millis());
    resp += "," + String(core1LoopFilter.GetFiltered());
    resp += "," + String(maxCore1LoopTime);
    resp += "," + String(ESP.getFreeHeap()/1024.0, 1);
    resp += "," + String(getAmbTemperature());

    // PSUVolt,BattVolt,BattSoC,SysCurrent,BattStatus,USBMode (5)
    resp += "," + String(getPSUVoltage());
    uint16_t battVolt = getBatteryVoltage();
    resp += "," + String(battVolt);
    resp += "," + String(voltToPercent(battVolt));
    resp += "," + String(getSystemCurrent());
    uint8_t chrgStat = getChargingStatus();
    if      (chrgStat == 0) { resp += ",Charging"; }
    else if (chrgStat == 1) { resp += ",Fully Charged"; }
    else                    { resp += ",Discharging"; }
    resp += "," + ((usbDataOnly) ? String("Data Only") : String("Power + Data"));

    // FixAge,NumSat,Lat,Lon,Alt,Timestamp (6)
    resp += "," + String(gps.location.age());
    resp += "," + String(gps.satellites.value());
    resp += "," + String(gps.location.lat(), 6);
    resp += "," + String(gps.location.lng(), 6);
    resp += "," + String(gps.altitude.feet());
    resp += "," + String(1);

    // SDPresent,SDCapacity,SDAvailable,LogID,SPIFFSSize,SPIFFSFree (6)
    if (SD_MMC.cardType() == CARD_NONE) {
      resp += ",No";
      resp += ",?";
      resp += ",?";
      resp += ",?";
    } else {
      resp += ",Yes";
      uint32_t sdSize = SD_MMC.totalBytes() / (1024 * 1024);
      uint32_t sdUsed = SD_MMC.usedBytes() / (1024 * 1024);
      uint32_t sdFree = sdSize - sdUsed;
      resp += "," + String(sdSize);
      resp += "," + String(sdFree);
      resp += "," + String(logFileNumber);
    }//if (SD card present
    resp += "," + String(SPIFFS.totalBytes()/1024.0, 1);
    uint32_t freeSpace = SPIFFS.totalBytes() - SPIFFS.usedBytes();
    resp += "," + String(freeSpace/1024.0, 1);

    // LoRaFreq,LoRaBand,LoRaSF,LastPingTime,LastPacket,LastRSSI,LastSNR,LastFreqErr,LastPcktValid,AFCOn (10)
    resp += "," + String(freqOpts[freqSelected],2);
    resp += "," + String(bandwidthOpts[bandwidthSelected],2);
    resp += "," + String(spreadOpts[spreadSelected]);
    resp += "," + String(millis()-rfmLastRFReceived);
    resp += "," + String(byteArrayToHexString(rfmLastPacket, RFM_PACKET_SIZE));
    resp += "," + String(rfmLastRSSI);
    resp += "," + String(rfmLastSNR);
    resp += "," + String(rfmLastFreqErr);
    resp += "," + ((rfmLastPacketValid) ? String("Yes") : String("NO"));
    resp += "," + ((false) ? String("Yes") : String("NO"));

    // RcktSats,RcktLat,RcktLon,RcktAltm,RcktStatus (5)
    resp += "," + String(rocketGPSSats);
    resp += "," + String(rocketGPSLat/1000000.0, 6);
    resp += "," + String(rocketGPSLon/1000000.0, 6);
    resp += "," + String(rocketAltitude);
    resp += "," + String(rocketStatus, BIN);

    resp += "," + String(curFreqOffset);

    resp += "," + String(core0FreeStack);
    resp += "," + String(core0LoopTime);
    
    request->send(200, "text/plain", resp);
  });

  // Route to load style.css file
  webServer.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/style.css", "text/css");
  });

  // Route to load style.css file
  webServer.on("/terms", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/terms.html", "text/html");
  });

  // Route to load qret.svg file
  webServer.on("/qret.svg", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/qret.svg", "image/svg+xml");
  });



  // Route for frequency offset increase
  webServer.on("/FreqUp", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/style.css", "text/css");
    curFreqOffset += 1000;
  });

  // Route for frequency offset increase
  webServer.on("/FreqDown", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/style.css", "text/css");
    curFreqOffset -= 1000;
  });

  // Route for reload radio
  webServer.on("/RadioReload", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/style.css", "text/css");
    rfmInit();
  });

  // Route for downloading logs
  webServer.on("/log", HTTP_GET, [](AsyncWebServerRequest *request){
    // Ensure valid request
    AsyncWebParameter* p = request->getParam(0);
    if (p->name() == "id") {
      // Generate requested log file name
      String filename = "/logs/Kuhglocke_Log" + String(p->value()) +".txt";

      // Open log file
      File file = SD_MMC.open(filename);
      if (!file) {
        request->send(404, "text/plain", "Cannot load file.");
        return;
      }//if

      // Ensure the file stays open throughout the response lifecycle
      AsyncWebServerResponse *response = request->beginChunkedResponse("text/plain", 
        [file = std::move(file)](uint8_t *buffer, size_t maxLen, size_t index) mutable -> size_t {
          if (!file) {
            return 0; // File is not open, return 0 to indicate end of data
          }
          size_t length = file.read(buffer, maxLen);
          if (length == 0) {
            file.close(); // Close the file when done
          }
          return length;
        });
      request->send(response);

    }//if
    request->send(404, "text/plain", "No id paramter given.");
  });
  
}//configWebServer
