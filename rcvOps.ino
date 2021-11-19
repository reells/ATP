//Serial receive function

void recvWithStartEndMarkers() {
  if (startUp == true) {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;

    while (Serial.available() > 0 && newData == false) {
      rc = Serial.read();

      if (recvInProgress == true) {
        if (rc != endMarker) {
          receivedChars[ndx] = rc;
          ndx++;
          if (ndx >= numChars) {
            ndx = numChars - 1;
          }
        }

        else {
          receivedChars[ndx] = '\0'; // terminate the string
          recvInProgress = false;
          ndx = 0;
          newData = true;
        }
      }
      else if (rc == startMarker) {
        recvInProgress = true;
      }
    }
    if (newData == true) {
      strcpy(tempChars, receivedChars);
      parseData();
      showParsedData();
      newData = false;
    }
  }
}

void parseData() {     // split the data into its parts

  char * strtokIndx; // this is used by strtok() as an index

  strtokIndx = strtok(tempChars, ",");     // get the first part - the string
  xCoordRcv = atof(strtokIndx);

  strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
  yCoordRcv = atof(strtokIndx);     // convert this part to a double
}

void showParsedData() {
  if (newData == true) {
    xCoord = xCoordRcv;
    yCoord = yCoordRcv;
    newData = false;
    coordRcv = true;
  }
}
