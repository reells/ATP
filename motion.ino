void mtrOp() {

  double mtrFloor = 150;
  double mtrMax = 200;
  mtrOut = mtrFloor + mtrAccel;

  if (mtrRunFwd == true) {
    digitalWrite(mtrEn2, LOW);
    digitalWrite(mtrEn1, HIGH);
    if (mtrOut <= mtrMax) {
      analogWrite(mtrSpd, 150);
      mtrAccel++;
      delay(2);
    }
  }

  if (mtrRunFwd == false) {
    analogWrite(mtrSpd, 0);
    mtrAccel = 0;
  }
}

void stepOp() {

  if (calWait == true) {
    bool cal = digitalRead(grnBtn);
    if (cal != grn_state) {
      grn_state = cal;
      if (!cal) {
        calWait = false;

      }
    }
    if (calMsg == true) {
      Serial.println("Calibration Needed: Wait for yaw to stabilize, then press green button.");
      delay(1000);
      calMsg = false;

    }
    if (calWait == true);
    Serial.print("Yaw: ");
    Serial.println(yaw);
  }

  if (calWait == false && stpEn == false) {
    calOff = yaw;
    Serial.print("IMU calibrated - yaw offset: ");
    Serial.println(ToDeg(calOff));
    Serial.println("Enter coordinates:");
    stpEn = true;

  }

  if (IMUcalc == true) {
    yawPreOut = ToDeg(yaw - calOff);
    if (yawPreOut < -180) {
      yawOut = yawPreOut + 360;
    }
    if (yawPreOut > -180 && yawPreOut < 180) {
      yawOut = yawPreOut;
    }
    if (yawPreOut > 180){
      yawOut = yawPreOut - 360;
    }

    if (stpEn == true) {
      stpAng = stpPul*pulToAng;
      if (hdgErr >= 1.5 && stpAng <= 30) {
        for (k = 0 ; k < 5 ; k++) {
          digitalWrite(stpDir, LOW);
          digitalWrite(stpPulOut, !digitalRead(stpPulOut));
          stpPul++;
          delay(1);
        }
        k = 0;
        delay(2);

      }
      if (hdgErr <= - 1.5 && stpAng >= -30) {
        for (l = 0 ; l < 5 ; l++) {
          digitalWrite(stpDir, HIGH);
          digitalWrite(stpPulOut, !digitalRead(stpPulOut));
          stpPul--;
          delay(1);
        }
        l = 0;
        delay(2);
      }
    }
  }
}
