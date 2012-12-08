void DoEngine() {
  switch (engine_state) {
    case ENGINE_OFF:
      if (engine_speed>500 && fuel_consumption != 0) {
        TransitionEngine(ENGINE_ON);
      }
      break;
    case ENGINE_ON:
      if (engine_speed==0 || fuel_consumption == 0) {  //speed isn't returning to zero, so also using fueling rate which does.
        TransitionEngine(ENGINE_OFF);
      }
      if (engine_speed>1900) {
        TransitionEngine(ENGINE_OVERSPEED);
      }
      break;
    case ENGINE_OVERSPEED:
      if (engine_speed==0 && millis()-engine_state_entered>1500) {
        TransitionEngine(ENGINE_OFF);
      }
  }
}

void TransitionEngine(int new_state) {
  //can look at engine_state for "old" state before transitioning at the end of this method
  engine_state_entered = millis();
  switch (new_state) {
    case ENGINE_OFF:
      Serial.println("#Off");
      TransitionMessage("Engine: Off");
      break;
    case ENGINE_ON:
      Serial.println("#On");
      TransitionMessage("Engine: Running");
      break;
    case ENGINE_OVERSPEED:
      Serial.println("#Overspeed");
      TransitionMessage("Engine: Overspeed");
      break;
  }
  engine_state=new_state;
}

//void SetThrottleAngle(double percent) {
// Servo_Throttle.write(throttle_valve_closed + percent*(throttle_valve_open-throttle_valve_closed));
// //servo2_pos = percent;
//}


