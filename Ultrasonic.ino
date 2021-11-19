int getFRONTdistance()
{

  digitalWrite(RIGHTtrigPin, LOW);
  digitalWrite(RIGHTtrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(RIGHTtrigPin, LOW);
  RIGHTduration = pulseIn(RIGHTechoPin, HIGH);
  RIGHTdistance = FRONTduration * 0.034 / 2;
//  Serial.print('\n');
//  Serial.print("U/S: RIGHT: ");
//  Serial.print(RIGHTdistance);

  digitalWrite(FRONTtrigPin, LOW);
  digitalWrite(FRONTtrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(FRONTtrigPin, LOW);
  FRONTduration = pulseIn(FRONTechoPin, HIGH);
  FRONTdistance = FRONTduration * 0.034 / 2;
//  Serial.print("  FRONT: ");
//  Serial.print(FRONTdistance);

  digitalWrite(LEFTtrigPin, LOW);
  digitalWrite(LEFTtrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(LEFTtrigPin, LOW);
  LEFTduration = pulseIn(LEFTechoPin, HIGH);
  LEFTdistance = LEFTduration * 0.034 / 2;
//  Serial.print("  LEFT: ");
//  Serial.print(LEFTdistance);
  return FRONTdistance;

}
