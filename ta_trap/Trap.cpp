#include "Trap.h"
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

