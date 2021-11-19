void coordBuilder() {

  if (coordRcv == true) {
    if (xCoord == 0 && yCoord == 0 && coordEntry == true) {
      Serial.println("Complete");
      j = i;
      coordEntry = false;
    }
    
    if (coordEntry == true) {
      i++;
      coordRcv = false;
      Serial.print("X");
      Serial.print(i - 1);
      Serial.print(": ");
      Serial.print(xCoord);
      Serial.print("  Y");
      Serial.print(i - 1);
      Serial.print(": ");
      Serial.println(yCoord);
      coordArrayX[i - 1] = xCoord;
      coordArrayY[i - 1] = yCoord;
    }
  }
}

void coordExe() {
    if (cycleCoords == true && i < j) {
    i++;
    Serial.print("Point count: ");
    Serial.println(i);
    xDelta = coordArrayX[i - 1] - xCoordSto;
    yDelta = coordArrayY[i - 1] - yCoordSto;
    Serial.print("X dest: ");
    Serial.println(coordArrayX[i - 1]);
    Serial.print("Y dest: ");
    Serial.println(coordArrayY[i - 1]);
    Serial.print("Prev X coord: ");
    Serial.print(xCoordSto);
    Serial.print(",   X Delta: ");
    Serial.println(xDelta);
    Serial.print("Prev Y coord: ");
    Serial.print(yCoordSto);
    Serial.print(",   Y Delta: ");
    Serial.println(yDelta);
    xCoordSto = coordArrayX[i - 1];
    yCoordSto = coordArrayY[i - 1];
    movCalcTgt = true;
    cycleCoords = false;
    startUp = false;
    count = 0;
  }
  if (i >= j && cycleCoords == true) {
    coordEntry = true;
    j = 0;
    i = 0;
    cycleCoords = false;
  }
  
  if (coordEntry == false && coordBuilt == false) {
    while (i > 0) {
      Serial.print("X");
      Serial.print(i - 1);
      Serial.print(" element: ");
      Serial.print(coordArrayX[i - 1]);
      Serial.print("  Y");
      Serial.print(i - 1);
      Serial.print(" element: ");
      Serial.println(coordArrayY[i - 1]);
      i--;
    }
    coordBuilt = true;
    Serial.println("Built");
    Serial.print("Points: ");
    Serial.println(j);
    Serial.print('\n');
    cycleCoords = true;
  }
}
