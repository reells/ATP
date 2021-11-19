//Trigger receive and timer functions

void trigRcv() {
  currentMillis = millis();
  bool count_up = digitalRead(proxTrig);
  if (count_up != prox_state) {
    prox_state = count_up;
    if (!count_up) {
      count++;
    }
  }
}

//PID may be used with either crseErr or crseOff variables.
//crseErr is angular error, crseOff is displacement error.
//crseOff initially used as crseErr will allow long run inaccuracy.

void PID() {
  double K = 1;
  double tI = 5;
  double tD = 5;
  reset = reset + (K / tI) * hdgErr;
  output = K * hdgErr + reset + ((preErr - hdgErr) * K / tD);
  preErr = hdgErr;
}
