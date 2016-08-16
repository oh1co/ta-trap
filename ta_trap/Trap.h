#ifndef _TRAP_H
#define _TRAP_H
#include <time.h>

class Trap
{
  public:
    Trap();
    ~Trap();
    void toggleGsm();
    bool sendStatus();
    bool SendStatusMsg;
    bool batteryStatusSent;
    bool armed;
    bool alarmSent;
    char SetPhoneNumber[15];
    time_t startTime;
    unsigned long nextSyncTime;
    char armedTime[20];
    bool Traplaunched;
    bool gsmOn;
    
  private:
};

#endif
