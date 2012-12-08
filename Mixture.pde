// mixture
void InitMixture() {
  LoadMixture();
  LoadPressurePID();
  if (PID_Control == 0){
      Serial.println("#Mix Pot.");
      TransitionMixture(MIXTURE_POT_CONTROL); 
  } else {
      Serial.println("#Mixe Rate");
      TransitionMixture(MIXTURE_OPEN); 
  }
}

void DoMixture() { 
    mixture_input = fuel_consumption;
    pressure_input = Press[P_COMB];
    switch(mixture_state) {
      case MIXTURE_POT_CONTROL:
        Servo_Mixture.write( air_butterfly_position);
        if (engine_state == ENGINE_OVERSPEED) {
          TransitionMixture(MIXTURE_OVERSPEED);
        }
        break;
      case MIXTURE_P_COMB: // not implemented - two stage PID to deal with slow speed refresh
        pressure_PID.Compute();
        mixture. Compute();
        SetPremixServoAngle(mixture_output);
        break;
      case MIXTURE_RATE_BASED:
        mixture.Compute();
        SetPremixServoAngle(mixture_output);
        if (engine_state == ENGINE_OVERSPEED) {
          TransitionMixture(MIXTURE_OVERSPEED);
        }
        if (millis()-lastReceivedOBDMsgTime>5000) { //CAN Error - should be a global value...  
          TransitionMixture(MIXTURE_OPEN);
        }
        if (engine_state == ENGINE_OFF) {
          TransitionMixture(MIXTURE_OPEN);
        }
        break;
      case MIXTURE_OVERSPEED:
        Servo_Mixture.write(throttle_valve_open);
        if (engine_state == ENGINE_OFF) {
          InitMixture();
        }
        break;
      case MIXTURE_OPEN:
        Servo_Mixture.write(throttle_valve_open);
        if (engine_state == ENGINE_ON) {
          TransitionMixture(MIXTURE_RATE_BASED);
        }
        break;
     } 
}

void TransitionMixture(int new_state) {
  Serial.print("#Mixture ");
  Serial.print(mixture_state_name);
  
  //Enter
  mixture_state=new_state;
  mixture_state_entered = millis();
  switch (new_state) {
    case MIXTURE_POT_CONTROL:
      mixture_state_name = "Pot. Control";
      break;
    case MIXTURE_P_COMB:
      mixture_state_name = "P Comb";
      mixture_setpoint = mixture_setpoint_mode[0];
      //mixture_output = ??;
      mixture.SetMode(AUTO);
      mixture.SetSampleTime(20);
      mixture.SetInputLimits(0.0,10.0);  //0-10 G/hr
      mixture.SetOutputLimits(-7000,7000);
      
      pressure_PID.SetMode(AUTO);
      pressure_PID.SetInputLimits(-7000, 7000);
      pressure_PID.SetOutputLimits(0,1);
      pressure_PID.SetTunings(pressure_P[0], pressure_I[0], pressure_D[0]);
      break;
    case MIXTURE_RATE_BASED:
      mixture_state_name = "Fuel Rate";
      //mixture_setpoint = mixture_setpoint_mode[0];
      mixture_output = premix_valve_open;
      mixture.SetMode(AUTO);
      mixture.SetSampleTime(20);
      mixture.SetInputLimits(0.0,12.0);  //0-10 G/hr
      mixture.SetOutputLimits(0,1);
      break;
    }
  Serial.print(" to ");  
  Serial.println(mixture_state_name);
}
    
void SetPremixServoAngle(double percent) {
 Servo_Mixture.write(premix_valve_closed + percent*(premix_valve_open-premix_valve_closed));
}

void WriteMixture() {
  WriteMixture(mixture_setpoint);
}

void WriteMixture(double setpoint) {
  int val,p,i;
  p = constrain(mixture.GetP_Param()*100,0,255);
  i = constrain(mixture.GetI_Param()*10,0,255);
  mixture_setpoint_mode[0] = setpoint; //?
  val = constrain(setpoint,0,255); //
  EEPROM.write(12,128); //check point
  EEPROM.write(13, val);
  EEPROM.write(14, p);
  EEPROM.write(15, i);
  //Message:Serial.println("#Writing mixture setttings to EEPROM");
}

void LoadMixture() {
  byte check;
  double val,p,i;
  check = EEPROM.read(12); 
  val = 1.0+(EEPROM.read(13)-128)*0.01;
  p = EEPROM.read(14)*0.01;
  i = EEPROM.read(15)*0.1;
  if (check == 128 && val >= 0.5 && val <= 1.5) { //check to see if mixture has been set
    //Message:Serial.println("#Loading mixture from EEPROM");
    mixture_setpoint = val;
    mixture.SetTunings(p,i,0);
  } else {
    //Message:Serial.println("#Saving default mixture setpoint to EEPROM");
    val = mixture_setpoint_mode[0];
    WriteMixture(val);
  }
  mixture_setpoint = val;
  mixture_setpoint_mode[0] = val;
}


//PressurePID 
void WritePressurePID() {
  WritePressurePID(pressure_setpoint);
}

void WritePressurePID(double setpoint) {
  int p,i;
  p = constrain(pressure_PID.GetP_Param()*50,0,255);
  i = constrain(pressure_PID.GetI_Param()*5,0,255);
  //pressure_setpoint_mode[0] = setpoint;
  //val = constrain(setpoint,0,255);
  EEPROM.write(112,128); //check point
  //EEPROM.write(113, val);
  EEPROM.write(114, p);
  EEPROM.write(115, i);
  //Message:Serial.println("#Writing pressure_pid settings to EEPROM");
}

void LoadPressurePID() {
  byte check;
  double p,i;
  check = EEPROM.read(112); 
  //val = EEPROM.read(113);
  p = EEPROM.read(114)*.02;
  i = EEPROM.read(115)*.2;
  if (check == 128) { //check to see if mixture has been set
    //Message:Serial.println("#Loading pressure_PID values from EEPROM");
    //pressure_setpoint = val;
    pressure_PID.SetTunings(p,i,0);
  } else {
    //Message:Serial.println("#Saving default pressure_PID values to EEPROM");
    //val = pressure_setpoint_mode[0];
    WritePressurePID(pressure_setpoint);
  }
  //pressure_setpoint = val;
  //pressure_setpoint_mode[0] = val;
}
