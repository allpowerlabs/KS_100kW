

// CANbus via RS232 Serial2 Connection to ODB Dongle

void InitOBD(){
  #ifdef CANDIAGNOSTICS
  Serial.println("#Initializing RS232");
  #endif
  Serial2.end();
  Serial2.begin(38400);
  Serial2.println("AT E0"); //turn off echoing
}
 
void SendOBDPGNRequestSeries() {
    if (millis()-lastReceivedOBDMsgTime>6000) { //reestablish serial communication if lost...
      InitOBD();
    }
    if (lastPGNPollSuccess == true || PGNPollAttempts>=3) { //successfully got last PGN
      //find next PGN to sample...
      for (int i = 0; i<PGNPollCount; i++) { //code as of 12/5 doesn't seem to re-poll PGNs that have shorter sampling period...(e.g. 20 ms, vs 1000 ms)
        int i_wrap = (i+PGNPollIndex+1) % (PGNPollCount - 1); //shouldn't depriotize polling items later in the array, so wrap around checks from last index position
//        #ifdef CANDIAGNOSTICS
//        Serial.print("#PGN Poll I:");
//        Serial.print(i_wrap);
//        Serial.print(" Time:");
//        Serial.println(millis() - PGNLastSampleTime[i_wrap]);
//        #endif
        if (millis() - PGNLastSampleTime[i_wrap] > PGNSamplePeriod[i_wrap]) {
          PGNPollIndex = i_wrap;
          SendOBDPGNRequest(PGNSeries[PGNPollIndex]);
          PGNPollAttempts = 0;
          lastPGNPollSuccess = false;
          #ifdef CANDIAGNOSTICS
            Serial.print("#New PGN Poll I:");
            Serial.print(PGNPollIndex); 
            Serial.print(" PGN Series I:");
            Serial.print(PGNSeries[PGNPollIndex]);
            Serial.print(" Req:");
            Serial.print(OBDPGNHex[PGNSeries[PGNPollIndex]]);
            Serial.print(" Att:");
            Serial.println(PGNPollAttempts);
          #endif
          break; //found a PGN in need of updating, now exit
        }
      }
    } else {
      if (millis()-lastPGNPollTime>30) {
        SendOBDPGNRequest(lastPGNrequest);
        PGNPollAttempts++;
        #ifdef CANDIAGNOSTICS
          Serial.print("#PGN Resend I/Att:");
          Serial.print(lastPGNrequest);
          Serial.print("/");
          Serial.println(PGNPollAttempts);
        #endif
      }
    }
}

void ResetOBDPGNRequestSeries() {
  PGNPollIndex = 0;
}

void SendOBDPGNRequest(int PGN_DEFINE){
  Serial2.println(OBDPGNHex[PGN_DEFINE]);
  lastPGNrequest = PGN_DEFINE;
  lastPGNPollTime = millis();
}

void ReadOBD(){
  int index = 0;
  //code modified to be similar to http://arduino.cc/forum/index.php/topic,48925.0.html
  //TODO: Do timeout from last request send up if no line char or character received
  while (Serial2.available() > 0){
    char aChar = Serial2.read();
    if (aChar == '\n' || aChar == '\r' || OBDDataIndex >= OBDDATAINDEXSIZE-1) { //parse on carriage return or data buffer full
      if (lastPGNPollSuccess == false) { //a message has not yet been processed
        // End of record detected. Time to parse
        if (OBDDataIndex >= 3) { //don't acknowledge blank lines
          processMessage();
          OBDDataIndex = 0;
        }
      } else {
        if (OBDDataIndex >= 3) { //don't acknowledge blank lines or "OK"
          AcknowledgeOBDMessage(); //stop sending messages! (Speed and Fuel Rate come out streaming...)
         }
      }
    } else {
      if (!(aChar == '>' || aChar == '?') && lastPGNPollSuccess==false) { //Skip the prompt character
        if (OBDDataIndex <= OBDDATAINDEXSIZE-1) { //prevent the message from overflowing
          OBDData[OBDDataIndex] = aChar;
          OBDDataIndex++;
          OBDData[OBDDataIndex] = '\0'; // Keep the string NULL terminated      
        }
      }
    }
  }
}

void processMessage() {
  //Process OBD message
  int scan = 0;
  unsigned int OBDmsgbytes[8] = {0,0,0,0,0,0,0,0};
  scan = sscanf(OBDData, "%x %x %x %x %x %x %x %x", &OBDmsgbytes[0], &OBDmsgbytes[1], &OBDmsgbytes[2], &OBDmsgbytes[3], &OBDmsgbytes[4], &OBDmsgbytes[5], &OBDmsgbytes[6], &OBDmsgbytes[7]);
  if (scan == 8) { //scan returns number of successful spaces filled. In theory error checking should only accept 8 received bytes.
    //move parsed bytes into last parsed array
    for (int i=0; i <= 7; i++){
      lastParsedPGNMessageBytes[i] = OBDmsgbytes[i];
    }
    lastReceivedPGNMessage = PGNSeries[PGNPollIndex]; //Only use successfully parsed messages
    parseOBDmessage(); //should add bounds checking with a success return here - request again if bounds are bad (data is corrupt)
    lastPGNPollSuccess = true;
    lastReceivedOBDMsgTime = millis();
    PGNLastSampleTime[PGNPollIndex] = lastReceivedOBDMsgTime;
  } else {
    //the message wasn't parsable, should try again
    //SendOBDPGNRequest(lastPGNrequest);
    //PGNPollAttempts++;
  }
  AcknowledgeOBDMessage();
  #ifdef CANDIAGNOSTICS
  Serial.print("#PGN Req:");
  Serial.print(OBDPGNHex[PGNSeries[PGNPollIndex]]);
  //Serial.print("/");
  //Serial.print(OBDPGNHex[lastPGNrequest]);
  //Serial.print("/");
  //Serial.print(OBDPGNHex[lastReceivedPGNMessage]);
  Serial.print(" Time:");
  Serial.print(millis()/1000.0);
  Serial.print(" Byte N:");
  Serial.print(scan);
  Serial.print(" Msg:");
  Serial.print(OBDData);
  Serial.print(" Size:");
  Serial.print(OBDDataIndex);
  if (scan == 8) {
    Serial.print(" Conv:");
    Serial.print(lastParsedPGNMessageBytes[0],DEC);
    Serial.print(",");
    Serial.print(lastParsedPGNMessageBytes[1],DEC);
    Serial.print(",");
    Serial.print(lastParsedPGNMessageBytes[2],DEC);
    Serial.print(",");
    Serial.print(lastParsedPGNMessageBytes[3],DEC);
    Serial.print(",");
    Serial.print(lastParsedPGNMessageBytes[4],DEC);
    Serial.print(",");
    Serial.print(lastParsedPGNMessageBytes[5],DEC);
    Serial.print(",");
    Serial.print(lastParsedPGNMessageBytes[6],DEC);
    Serial.print(",");
    Serial.print(lastParsedPGNMessageBytes[7],DEC);
  }
  Serial.println();
  Serial.println();
  #endif
}

void AcknowledgeOBDMessage() {
  Serial2.println("OK"); //acknowledge receipt of message to OBD?
}

void parseOBDmessage() {
  //TODO: should do value bounds checking
  unsigned long value = 0;
  double converted;
  #ifdef CANDIAGNOSTICS
    Serial.print("#Parse PGN I: ");
    Serial.print(lastReceivedPGNMessage);
  #endif
  switch (lastReceivedPGNMessage) {
//		case PGN_FUEL:
//			//PGN 65203 / 00FEB3 - Fuel Information (Liquid)
//			//Trip Fuel Rate trip - bytes 5 & 6 [.05 L/hour]
//			value = lastParsedPGNMessageBytes[4] << 8 & lastParsedPGNMessageBytes[5];
//			//fuel_consumption = value * 0.05; //should convert to gallons
//                        fuel_consumption = value * 0.01321; // in gallons/hr
//                        Serial.print("#Fuel Rate:");
//                        Serial.println(fuel_consumption);
//			break;
    case PGN_SPEED:
      // PGN 61444 / 00F004 - Electronic Engine Controller #1
      // Engine Speed - bytes 4 & 5 [.125 rpm/bit]
      // Percent Engine Torque - byte 3 [1 percent/bit?]
      value = (lastParsedPGNMessageBytes[4]) << 8;
      value = (value | lastParsedPGNMessageBytes[3]);
      //value = lastParsedPGNMessageBytes[3] << 8 & lastParsedPGNMessageBytes[4];
      engine_speed = value * .125;
      #ifdef CANDIAGNOSTICS
      Serial.print(" Speed:");
      Serial.print(engine_speed);
      #endif
//      if (converted>0 && converted<250) { //bounds checking...
//        engine_speed = converted;
//        return true;
//      } else {
//        return false;
//      }
      engine_torque = lastParsedPGNMessageBytes[2]-125;
      #ifdef CANDIAGNOSTICS
      Serial.print(" Torque:");
      Serial.println(engine_torque);
      #endif
      break;
    case PGN_TEMP:
      // Engine Coolant Temperature - byte 1 [1C/bit, -40 C offset]
      coolant_temperature = lastParsedPGNMessageBytes[0] - 40;
      #ifdef CANDIAGNOSTICS
      Serial.print(" Temp:");
      Serial.println(coolant_temperature);
      #endif
      break;
    case PGN_HOURS:
      //Total Engine Hours - bytes 1-4 [0.05 hr/bit]
      value = (lastParsedPGNMessageBytes[3]) << 8;
      value = (value | lastParsedPGNMessageBytes[2]) << 8;
      value = (value | lastParsedPGNMessageBytes[1]) << 8;
      value = (value | lastParsedPGNMessageBytes[0]);
      //value= lastParsedPGNMessageBytes[0] << 24 & lastParsedPGNMessageBytes[1] << 16 & lastParsedPGNMessageBytes[2] << 8 & lastParsedPGNMessageBytes[3];
      total_engine_hours = value * 0.05;
      #ifdef CANDIAGNOSTICS
      Serial.print(" Total Hours:");
      Serial.println(total_engine_hours); 
      #endif
      break;
    case PGN_AMBIENT:
      break;
    case PGN_EXHAUST:
      //Boost Pressure - byte 2 - [2 kPa/bit]
      value =  lastParsedPGNMessageBytes[1];
      engine_boost = value * 0.29; //in PSI
      #ifdef CANDIAGNOSTICS
      Serial.print(" Boost:");
      Serial.println(engine_boost);
      #endif   
      break;
    case PGN_FUEL_ACCUM:
      //Trip Fuel Consumption - bytes 1-4 [.5L/ bit]
      value = (lastParsedPGNMessageBytes[3]) << 8;
      value = (value | lastParsedPGNMessageBytes[2]) << 8;
      value = (value | lastParsedPGNMessageBytes[1]) << 8;
      value = (value | lastParsedPGNMessageBytes[0]);
      //value = lastParsedPGNMessageBytes[0] << 24 & lastParsedPGNMessageBytes[1] << 16 & lastParsedPGNMessageBytes[2] << 8 & lastParsedPGNMessageBytes[3];
      total_fuel_consumption = value * 0.1321; // in gallons/hr
      #ifdef CANDIAGNOSTICS
      Serial.print(" Total Consumption :");
      Serial.println(total_fuel_consumption); 
      #endif
      break;
    case PGN_FUEL_ECON:
      //Fuel Rate - bytes 1,2 [0.05 L/h]
      value = (lastParsedPGNMessageBytes[1]) << 8;
      value = (value | lastParsedPGNMessageBytes[0]);
      //value = lastParsedPGNMessageBytes[0] << 8 & lastParsedPGNMessageBytes[1];
      fuel_consumption = value * 0.01321; // in gallons/hr
      #ifdef CANDIAGNOSTICS
      Serial.print(" Fuel Rate:");
      Serial.println(fuel_consumption);
      #endif
      break;
    case PGN_TACH:
      break;
    case PGN_EEC1:
      //Engine Speed - bytes 4,5 - [0.125 rpm/bit]
      value = (lastParsedPGNMessageBytes[4]) << 8;
      value = (value | lastParsedPGNMessageBytes[3]);
      //value = lastParsedPGNMessageBytes[3] << 8 & lastParsedPGNMessageBytes[4];
      engine_speed = value * .125;
      #ifdef CANDIAGNOSTICS
      Serial.print(" Speed:");
      Serial.println(engine_speed); 
      #endif
      break;
  }
}
  // int ReadOBD(){
//   byte readbyte;
//   int returnvalue = 0;
//   int i = 0;
//   while (Serial2.available() > 0){
//     readbyte = Serial2.read();
//     if (i == 128) break;
//     if (readbyte == '\n') break; 
//     if (readbyte == '\0') break; 
//     if (readbyte == -1) continue;
//     if (readbyte == '>') continue; //removes > character
//     if (i==0){
//       Serial.print("#ODBMsg: ");
//     }
//     readStream[i] = readbyte;
//     readStream[i+1] = '\0';
//     Serial.print(readbyte);
//     i += 1;
//   }
//   Serial.println();
//   if (sizeof(readstream[]) > 0){
//     
//   }
//   // Scub value from  readStream
//   int returnbytes[] = {0,0,0,0,0,0,0,0};
//   sscanf(readstream, "%x %x %x %x %x %x %x %x", &returnbytes[0], &returnbytes[1], &returnbytes[3], &returnbytes[4], &returnbytes[5], &returnbytes[6], &returnbytes[7],);  
//   //combine the important bytes
//   returnvalue = (returnbytes[6] >> 1)  && (returnbytes[7] >> 1)  //not sure how to combine these yet...probably need a seperate bit filter for each.
//   
//   return returnvalue;
// }



// void SendPGN(long PGN){
//   char hex[6] = "";
//   char hex2[8];
//   sprintf(hex, "%06X", PGN);
//   hex2[0] = toupper(hex[4]);
//   hex2[1] = toupper(hex[5]);
//   hex2[2] = ' ';
//   hex2[3] = toupper(hex[2]);
//   hex2[4] = toupper(hex[3]);
//   hex2[5] = ' ';
//   hex2[6] = toupper(hex[0]);
//   hex2[7] = toupper(hex[1]);
//   hex2[8]='\0';
//   Serial.print("Sending PGN: "); 
//   Serial.print(PGN); 
//   Serial.print(", "); 
//   Serial.println(hex2);
//   Serial2.println(hex2);
// }

// double getFuel(){
//   double fuel_consumption = -1;
//   Serial2.flush();  //clear out old responces??
//   SendPGN(65203);
//   while (fuel_consumption = -1) {
//     fuel_consumption = ReadRS232();
//   //obtain responce
//   //convert to G/hr
//   }
//   return fuel_consumption;
// }


//CANbus via internal MCP2515 chip

//void InitCAN(){
//  if(Canbus.init(CANSPEED_250)){  //in kbps: CANSPEED_125 CANSPEED_250 CANSPEED_500
//  //if(Canbus.init(CANbus_speed)) {
//    Serial.println("CAN Init ok");
//    can_init = true;
//  } else {
//      Serial.println("Can't init CAN");
//      can_init = false;
//  }
//}

//void logCANbus(){
//  if (!can_init) InitCAN();
//  
//  char buffer[512];
//  if(Canbus.ecu_req(ENGINE_RPM,buffer) == 1){         /* Request for engine RPM */
//    Serial.print("#Engine RPM: "); Serial.println(buffer);                         
//  } 
////  if(Canbus.ecu_req(VEHICLE_SPEED,buffer) == 1){
////    Serial.print("#Engine Speed: "); Serial.println(buffer);
////  }
//  if(Canbus.ecu_req(ENGINE_COOLANT_TEMP,buffer) == 1){
//    Serial.print("#Engine Cooland Temp: "); Serial.println(buffer);   
//  }
//  if(Canbus.ecu_req(THROTTLE,buffer) == 1){
//    Serial.print("#Throttle: "); Serial.println(buffer);
//  }  
//  if(Canbus.ecu_req(O2_VOLTAGE,buffer) == 1){
//    Serial.print("#O2 Voltage: "); Serial.println(buffer);
//  }   
//  if (Canbus.ecu_req(0xFEB3,buffer) == 1){   //Request for PGN 65203 - .05 L/hour trip fuel rate
//    Serial.print("#PGN 65203: "); Serial.println(buffer);
//  }
//  if (Canbus.ecu_req(0xF004,buffer) == 1){   
//    Serial.print("#PGN 61444: "); Serial.println(buffer);
//  }
//  if (Canbus.ecu_req(0xFEE9,buffer) == 1){   
//    Serial.print("#PGN 65257: "); Serial.println(buffer);
//  }

/*
 Engine Speed is in PGN 61444(0xF004).  It is two bytes long with a resolution of .125 rpm/bit, bytes 4 and 5. Byte 3 of that PGN is the percent engine torque of the engine.
 PGN 65203(0xFEB3) has a parameter called trip fuel rate in .05 L/hour. It is the fifth and sixth bytes.  
 PGN 65257(0xFEE9) has two separate parameters total fuel consumption and trip fuel consumption both are .5L/ bit.  Trip fuel consumption is bytes 1-4 and Total Fuel Consumption is bytes 5-8.  
 */
//}

//void sendCANbus(&message, boolean J1939){
//  if (J1939){
//    mcp2515_send_message_J1939(&message);
//  } else {
//    mcp2515_send_message(&message);
//  }
//}


