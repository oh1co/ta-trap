#include "Trap.h"
#include "SIM900.h"
#include <stdio.h>
#include <string.h>

Trap::Trap()
{
  SendStatusMsg = false;
  batteryStatusSent = false;
  armed = false;
  alarmSent = false;
  memset(SetPhoneNumber, '\0', sizeof(SetPhoneNumber) / sizeof(SetPhoneNumber[0]));
  startTime = 0;
  nextSyncTime = 0;
  strcpy(armedTime, "No");
  Traplaunched = false;
  gsmOn = false;
}

Trap::~Trap()
{

}

bool Trap::sendStatus()
{
  if (SendStatusMsg && armed)
    return true;
  return false;
}

void Trap::toggleGsm()
{
  //Toggle GSM ON/OFF
  pinMode(GSM_ON, OUTPUT);
  digitalWrite(GSM_ON, HIGH);
  delay(1200);
  digitalWrite(GSM_ON, LOW);
  delay(10000);
  gsmOn = ~gsmOn;
}

