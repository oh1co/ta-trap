/*
   power consumption requirements!
   10days with 1.2Ah battery
   => 240h / 1200mAh = 0.2mA
*/
#include "SIM900.h"
#include <avr/power.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <Time.h>
#include "Trap.h"
#include "sms.h"

#define DEBUG true // flag to turn on/off debugging
#define Serial if(DEBUG)Serial

#define LowVoltageLimit 1.75
/* ****************
   I/O pin definitions
   **************** */
const byte led1Pin = 10;
const byte buzzer1Pin = 12;

/* ****************
   Interrupts pins
   **************** */
const byte pirInterrupt = 2;
const byte gsmInterrupt = 3;

/* ****************
   Analog pins
   **************** */
const byte voltageInputPin = 1;

/* ****************
   Function list
   **************** */
void setup();
void loop();
void handleSms(char * phone_num, char* sms_text);
void getTime(char* dateAndTime);
void alarmSet(int highDelay, int lowDelay, byte times);
void deleteAllSms();
void sendSms();
void delayWdt(unsigned nsec);
void getGsmTime();

/* ****************
   Global variables
   **************** */
Trap trap;
SMSGSM sms;

const byte SmsMaxSize = 160;
const unsigned long statusSmsInterval = SECS_PER_DAY;
volatile boolean PirAlarmValue = false;
volatile boolean GsmAlarmValue = false;
unsigned int WdTime = SECS_PER_DAY / 4;
unsigned int nbr_remaining = 0;

volatile byte wdExpired = 0;
volatile double SecondsCounter = 0;

const char registerClkSer[] = {"AT+CLTS=1"};
const char updateCCLK[] = {"AT+CCLK?"};
const char setMessage[] = {"ASETA"};
const char statusMessage[] = {"TILA"};
const char stopGsm[] = {"STOP GSM"};

/*
   Watchdog interrupt routine.
   Setup function will enable 1s WD interrupt without reset
*/
ISR (WDT_vect)
{
  wdExpired++;
  SecondsCounter++;
}
/*
   Pir alarms ISR (Interrupt Service Routine)
*/
void pirAlarm()
{
  detachInterrupt(digitalPinToInterrupt(pirInterrupt));
  PirAlarmValue = true;
  //  tone(led1Pin, 532 , 2500);
}

/*
   GSM Ring indicator (RI) ISR (Interrupt Service Routine)
*/
void gsmAlarm()
{
  detachInterrupt(digitalPinToInterrupt(gsmInterrupt));
  GsmAlarmValue = true;
}

/*
    Setup function
    -Initialize Serial port
    -Initialize gms module
    -Set Pin Inputs & Outputs
    -Initialize watchdog
    -Used Pins 0,1 RX / TX (arduino)
               2 PIR alamrs
               3 GSM Ring Indicator
               4 GSM POWER UP
               7,8 RX,TX (GSM shield)
               9 NONE
               10 Led1
               11 NONE
               12 buzzer
               13 NONE
*/
void setup()
{
  Serial.begin(9600);
  Serial.println(F("GSM Shield testing."));
  pinMode(led1Pin, OUTPUT);
  pinMode(buzzer1Pin, OUTPUT);
  digitalWrite(buzzer1Pin, LOW);

  pinMode(pirInterrupt, INPUT_PULLUP);

  //Start configuration of shield with baudrate.
  if (gsm.begin(2400))
  {
    delay(2000);
    gsm.SimpleWriteln(registerClkSer);
    delay(2000);
    getGsmTime();
    attachInterrupt(digitalPinToInterrupt(gsmInterrupt), gsmAlarm, LOW);
    alarmSet(50, 50, 5);
    trap.gsmOn = true;
    Serial.println(F("\nREADY"));
  }
  else
    Serial.println(F("\nNO GSM"));

  cli();
  wdt_disable();
  /* Clear the reset flag. */
  MCUSR &= ~(1 << WDRF);

  /* In order to change WDE or the prescaler, we need to
     set WDCE (This will allow updates for 4 clock cycles).
  */
  WDTCSR |= (1 << WDCE) | (1 << WDE);

  /* set new watchdog timeout prescaler value */
  WDTCSR = 1 << WDP1 | 1 << WDP2; /* 1.0 seconds */
  //  WDTCSR = 1 << WDP3; /* 4.0 seconds */
  //  WDTCSR = 1 << WDP0 | 1 << WDP3; /* 8.0 seconds */

  /* Enable the WD interrupt (note no reset). */
  WDTCSR |= _BV(WDIE);
  sei();
  trap.nextSyncTime = statusSmsInterval;
};

/*
   Loop Function
   -Check pir ISR alarm and raise alarm when needed
   -Check gsm ISR alarm and send alarm message when needed
   -Check battery voltage after every 12h
   -Send status SMS message after every 24h
*/
void loop()
{
  delayWdt(WdTime);
  if (wdExpired >= WdTime )
  {
    noInterrupts();
    wdExpired = 0;
    interrupts();
    Serial.println(F("WD expired"));
  }

  // Check PIR alarms
  if (PirAlarmValue == true)
  {
    Serial.println("PIR");
    noInterrupts();
    PirAlarmValue = false;
    interrupts();
    trap.Traplaunched = true;;
    alarmSet(250, 150, 20);
    alarmSet(150, 50, 10);
    alarmSet(75, 25, 5);
    tone(buzzer1Pin, 333, 10000);

    attachInterrupt(digitalPinToInterrupt(pirInterrupt), pirAlarm, LOW);
    Serial.println("re-attach PIR");
    if (trap.alarmSent == false)
    {
      sendSms();
      trap.alarmSent = true;
    }
    noTone(buzzer1Pin);
  }

  //Check time 24h from GSM network
  //6% safety margin because of clock drift..
  if (SecondsCounter >= trap.nextSyncTime * 0.94 && trap.armed)
  {
    getGsmTime();
    time_t currentTime = now();
    if ((currentTime - trap.startTime) >= trap.nextSyncTime)
    {
      trap.SendStatusMsg = true;
      int overtime = (currentTime - trap.startTime) - (statusSmsInterval);

      trap.nextSyncTime = (statusSmsInterval) - overtime;
      trap.startTime = currentTime;
      noInterrupts();
      SecondsCounter = 0;
      interrupts();
    }
    else
    {
      noInterrupts();
      SecondsCounter = currentTime - trap.startTime;
      interrupts();
    }
    Serial.println("Next sync Time");
    Serial.println(trap.nextSyncTime);
  }

  //Check GSM Module status after every 30 sec
  //  if (checkGsm)
  if (GsmAlarmValue && trap.gsmOn )
  {
    noInterrupts();
    GsmAlarmValue = false;
    interrupts();
    Serial.println(F("Check new SMS"));
    char position = sms.IsSMSPresent(SMS_UNREAD);
    if (position > 0)
    {
      Serial.println(F("New SMS received!"));
      // there is new SMS => read it
      char sms_text[SmsMaxSize];
      char phone_num[15];// array for the phone number string
      memset(sms_text, 0, SmsMaxSize);
      memset(phone_num, 0, 15);
      sms.GetSMS(position, phone_num, sms_text, SmsMaxSize);
      Serial.println(sms_text);
      handleSms(phone_num, sms_text);
    }
    attachInterrupt(digitalPinToInterrupt(pirInterrupt), pirAlarm, LOW);
    attachInterrupt(digitalPinToInterrupt(gsmInterrupt), gsmAlarm, LOW);
  }

  bool shutDownGsm = false;
  float voltage = (float) analogRead(A1) * 5 / 1024;
  if (voltage < LowVoltageLimit && trap.armed && (!trap.batteryStatusSent))
  {
    if (!trap.gsmOn)
    {
      trap.toggleGsm();
      if (gsm.begin(2400))
        shutDownGsm = true;
    }
    Serial.println("Send Battery Status!");
    char trapBatteryStatus[30] = "VAROITUS! Akku loppumassa!!";
    sms.SendSMS(trap.SetPhoneNumber, trapBatteryStatus);
    trap.batteryStatusSent = true;
  }

  if (trap.sendStatus())
  {
    if (!trap.gsmOn)
    {
      trap.toggleGsm();
      if (gsm.begin(2400))
        shutDownGsm = true;
    }

    Serial.println(F("Status Time.."));
    char sms_text[SmsMaxSize];
    memset(sms_text, 0, SmsMaxSize);
    memcpy(sms_text, statusMessage, 7);
    handleSms(trap.SetPhoneNumber, sms_text);
    trap.SendStatusMsg = false;
  }
  if (shutDownGsm)
  {
    trap.toggleGsm();
  }
};

/*
   This function will handle all received sms mesages
   Currently supports two (2) different messages.
   "setMessage" will arm trap and "statusMessage" gives sms response to arm status and battery level.
*/
void handleSms(char* phone_num, char* sms_text)
{
  Serial.println("handleSms");
  String receivedSms(sms_text);
  receivedSms.toUpperCase();
  receivedSms.trim();
  char tmpBuffer[19];
  if (receivedSms.startsWith(setMessage))
  {
    //Serial.println("SET message received. -> Arm trap");
    char smsReply[SmsMaxSize] = "Aktivoitu";
    deleteAllSms();
    trap.armed = true;
    trap.Traplaunched = false;
    trap.alarmSent = false;
    trap.batteryStatusSent = false;
    delay(10000);
    getGsmTime();
    snprintf(trap.armedTime, 19, "%02d/%02d/%02d,%02d:%02d:%02d",
             year() % 100, month(), day(),
             hour(), minute(), second());
    trap.startTime = now();
    Serial.println(smsReply);
    sms.SendSMS(phone_num, smsReply);
    memcpy(trap.SetPhoneNumber, phone_num, 15);
    attachInterrupt(digitalPinToInterrupt(pirInterrupt), pirAlarm, LOW);
    Serial.println(F("Set Done"));
  }

  else if (receivedSms == statusMessage)
  {
    //Serial.println("STATUS message received. -> Resp with status & battery level");
    char smsReply[SmsMaxSize];
    String reply;
    if (trap.armed)
    {
      getGsmTime();
      reply += "Toiminta: OK";

      reply += "\nAika:";
      snprintf(tmpBuffer, 19, "%02d/%02d/%02d,%02d:%02d:%02d",
               year() % 100, month(), day(),
               hour(), minute(), second());
      reply += tmpBuffer;
    }

    reply += "\nKytketty:";
    reply += trap.armedTime;

    reply += "\nAktivoija:";
    reply += trap.SetPhoneNumber;

    reply += "\nHalytys:";
    reply += trap.Traplaunched;

    reply += "\nAkku: ";
    float voltage = (float) analogRead(A1) * 5 / 1024;
    reply += voltage;

    reply.toCharArray(smsReply, SmsMaxSize);
    sms.SendSMS(phone_num, smsReply);
    if (strcmp(trap.SetPhoneNumber, phone_num) != 0 && trap.armed)
    {
      reply += "\nKysely:";
      reply += phone_num;
      reply.length();
      reply.toCharArray(smsReply, SmsMaxSize);
      sms.SendSMS(trap.SetPhoneNumber, smsReply);
    }
  }
  else if (receivedSms == stopGsm)
  {
    trap.toggleGsm();
  }
};

/*
    Query time from GSM server.
    NOTE: 2s delay between req & response reading
*/
void getTime(char* dateAndTime)
{
  gsm.SimpleWriteln(updateCCLK);
  delay(2000);
  int i = gsm.read(dateAndTime, 63);
  dateAndTime[i + 1] = '\0';

  String tmp(dateAndTime);
  int startIndex = tmp.indexOf("\"") + 1;
  int endIndex = tmp.lastIndexOf("\"");
  tmp = tmp.substring(startIndex, endIndex);
  Serial.println(tmp);
  tmp.toCharArray(dateAndTime, 21);
}

/*
   This function blink led lights.
*/
void alarmSet(int highDelay, int lowDelay, byte times)
{
  for (int i = 0; i < times; i++)
  {
    digitalWrite(led1Pin, HIGH);
    tone(buzzer1Pin, highDelay);
    delay(highDelay);
    noTone(buzzer1Pin);
    digitalWrite(led1Pin, LOW);
    delay(lowDelay);
  }
}

/*
   This function will delete all SMS from SIM card
*/
void deleteAllSms()
{
  gsm.SimpleWriteln("AT+CMGDA=\"DEL ALL\"");
  delay(1000);
}

/*
    This function send alarm message to user
*/
void sendSms()
{
  char trapArmed[17] = "Tarkista pyydys!";
  sms.SendSMS(trap.SetPhoneNumber, trapArmed);
  trap.alarmSent = true;
}

/*
   Put arduino in PWR_DOWN_SLEEP mode.
   Wake up after nSec has been elapsed, PIR alarm or
   when need to send daily status SMS
*/
void delayWdt(unsigned nsec)
{
  nbr_remaining = nsec;
  power_adc_disable();
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  while (nbr_remaining > 0 && PirAlarmValue == false)
  {
    sleep_mode();
    sleep_disable();
    if (SecondsCounter >= trap.nextSyncTime)
      break;
    if (GsmAlarmValue == true)
      break;
    nbr_remaining--;
  }
  power_all_enable();
}

/*
   Query time from GSM server using getTime()
   and store time to arduino RTC
*/
void getGsmTime()
{
  char dateNow[64];
  memset(dateNow, 0, 64);
  getTime(dateNow);
  String currentTime(dateNow);
  byte nowYear = currentTime.substring(0, 2).toInt();
  byte nowMonth = currentTime.substring(3, 5).toInt();
  byte nowDate = currentTime.substring(6, 8).toInt();
  byte nowHH = currentTime.substring(9, 11).toInt();
  byte nowMM = currentTime.substring(12, 14).toInt();
  byte nowSec = currentTime.substring(15, 17).toInt();
  setTime((int)nowHH, (int)nowMM, (int)nowSec, (int)nowDate, (int)nowMonth, (int)nowYear);
  adjustTime(3);
}

