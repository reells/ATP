void nav() {

  if (movCalcTgt == true) {
    hdgSet = atan2(yDelta, xDelta) * rad2deg;
    distSet = sqrt(sq(xDelta) + sq(yDelta));
    Serial.print("Heading Setpoint: ");
    Serial.println(hdgSet, 1);
    hdgInit = IMUhdg;
    distTrav = count * trigDist;
    distLeft = distSet - distTrav;
    movCalcTgt = false;
    IMUcalc = true;
    coordRcv = false;
    coordEntry = false;
    startUp = false;
    countPrev = -1;
  }
  if (IMUcalc == true && count != countPrev) {
    hdgErr = hdgSet - yawOut;
    Serial.print("  Hdg: ");
    Serial.print(yawOut);
    Serial.print("  HdgErr: ");
    Serial.print(hdgErr);
    Serial.print("  Dist: ");
    Serial.print(distTrav);
    Serial.print("  ToGo: ");
    Serial.print(distLeft);
    Serial.print("  U/S RFL: ");
    Serial.print(RIGHTdistance);
    Serial.print("  ");
    Serial.print(FRONTdistance);
    Serial.print("  ");
    Serial.println(LEFTdistance);

    countPrev = count;
    stpEn = true;

    distTrav = count * trigDist;
    distLeft = distSet - distTrav;

    if (distLeft >= turnRad) {
      mtrRunFwd = true;
    }
    if (distLeft < turnRad) {
      mtrRunFwd = false;
      cycleCoords = true;
    }
  }
}
