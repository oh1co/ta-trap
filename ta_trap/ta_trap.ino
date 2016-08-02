/*
   power consumption requirements!
   10days with 1.2Ah battery
   => 240h / 1200mAh = 0.2mA
*/
#include "SIM900.h"
#include <SoftwareSerial.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <Time.h>

SoftwareSerial mySerial(7, 8);

#include "sms.h"
SMSGSM sms;

#define DEBUG true // flag to turn on/off debugging
#define Serial if(DEBUG)Serial

#define LowVoltageLimit 2.40
/* ****************
   Interrupts pins
   **************** */
const byte pirInterrupt = 2;

/* ****************
   Analog pins
   **************** */
const byte voltageInputPin = 1;
/* ****************
   Function list
   **************** */
void handleSms();
void getTime();
void alarmSet(int highDelay, int lowDelay, byte times);
void deleteAllSms();

/* ****************
   Global variables
   **************** */
const byte SmsMaxSize = 128;
const unsigned long statusSmsInterval = SECS_PER_DAY;
boolean armed = false;
volatile boolean Traplaunched = false;
volatile boolean PirAlarmValue = false;
boolean SendStatusMsg = false;
boolean alarmSent = false;
char phone_num[15] = {'\0'}; // array for the phone number string
char SetPhoneNumber[15] = {'\0'}; // number where trap launched information will be sent!
char sms_text[SmsMaxSize]; // array for the SMS text string
char dateAndTime[64];
String armedTime = "No";
unsigned long nextSyncTime = statusSmsInterval;
time_t startTime;
time_t currentTime;
byte buzzerPin = 10;
byte WdTime = 30;

volatile byte wdExpired = 0;
volatile double SecondsCounter = 0;
volatile byte nbr_remaining = 0;

const char registerClkSer[] = {"AT+CLTS=1"};
const char updateCCLK[] = {"AT+CCLK?"};
const char setMessage[] = {"ASETA"};
const char statusMessage[] = {"TILA"};

/*
   Watchdog interrupt routine.
   Setup function will enable 1s WD interrupt without reset
*/
ISR (WDT_vect)
{
  nbr_remaining--;
  wdExpired++;
  SecondsCounter++;
}

/*
    Setup function
    -Initialize Serial port
    -Initialize gms module
    -Set Pin Inputs & Outputs
    -Used Pins 0,1 RX / TX
               2,3 PIR alamrs
               4-6 NONE
               7,8 RX,TX
               9 GSM POWER ON / OFF
               10 buzzer
               11 GsmLed
               12 TrapLed
               13 BatteryLed
*/
void setup()
{
  Serial.begin(9600);
  Serial.println(F("GSM Shield testing."));
  pinMode(buzzerPin, OUTPUT);

  pinMode(pirInterrupt, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pirInterrupt), pirAlarm, LOW);

  //Start configuration of shield with baudrate.
  if (gsm.begin(2400))
  {
    delay(2000);
    gsm.SimpleWriteln(registerClkSer);
    delay(2000);
    getGsmTime();
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
};

/*
   Loop Function
   -Check pir ISR alarm and raise alarm when needed
   -Check new sms from Gsm module after every 30sec (WdTime)
   -Send status SMS message after every 24h
*/
void loop()
{
  boolean checkGsm = false;
  delayWdt(WdTime);
  if (wdExpired >= WdTime )
  {
    wdExpired = 0;
    //Serial.println(F("WD expired"));
    checkGsm = true;
  }

  // Check PIR alarms
  if (PirAlarmValue == true)
  {
    Serial.println("PIR");
    PirAlarmValue = false;
    alarmSet(250, 150, 20);
    alarmSet(150, 50, 10);
    alarmSet(75, 25, 5);

    attachInterrupt(digitalPinToInterrupt(pirInterrupt), pirAlarm, LOW);
    Serial.println("re-attach PIR");
    if (alarmSent == false)
    {
      sendSms();
      alarmSent = true;
    }
  }

  //Check time 24h from GSM network
  //5% safety margin because of clock drift..
  if (SecondsCounter >= nextSyncTime * 0.95 && armed)
  {
    getGsmTime();
    currentTime = now();
    if ((currentTime - startTime) >= nextSyncTime)
    {
      SendStatusMsg = true;
      int overtime = (currentTime - startTime) - (statusSmsInterval);

      nextSyncTime = (statusSmsInterval) - overtime;
      startTime = currentTime;
      noInterrupts();
      SecondsCounter = 0;
      interrupts();
    }
    else
    {
      noInterrupts();
      SecondsCounter = currentTime - startTime;
      interrupts();
    }
    Serial.println("Next sync Time");
    Serial.println(nextSyncTime);
  }

  //Check GSM Module status after every 30 sec
  if (checkGsm)
  {
    //Serial.println(F("Check new SMS"));
    char position = sms.IsSMSPresent(SMS_UNREAD);
    if (position)
    {
      // there is new SMS => read it
      sms.GetSMS(position, phone_num, sms_text, SmsMaxSize);
      handleSms();
    }
  }

  if (SendStatusMsg && armed)
  {
    Serial.println(F("Status Time.."));

    if (SetPhoneNumber[0] != '\0')
    {
      Serial.print(F("Send SMS!"));
      memcpy(sms_text, statusMessage, 7);
      handleSms();
    }
    SendStatusMsg = false;
  }
};

/*
   This function will handle all received sms mesages
   Currently supports two (2) different messages.
   "Arm" will arm trap and "STATUS" gives sms response to where is
   arm status and battery level.
*/
void handleSms()
{
  Serial.println("handleSms");
  String receivedSms(sms_text);
  receivedSms.toUpperCase();
  receivedSms.trim();
  if (receivedSms.startsWith(setMessage))
  {
    //Serial.println("SET message received. -> Arm trap");
    char smsReply[SmsMaxSize] = "Aktivoitu";
    armed = true;
    sms_text[0] = '\0';
    deleteAllSms();
    Traplaunched = false;
    delay(10000);
    getGsmTime();
    armedTime = dateAndTime;
    armedTime = armedTime.substring(0, armedTime.length() - 3);
    startTime = now();
    Serial.println(smsReply);
    sms.SendSMS(phone_num, smsReply);
    alarmSent = false;
    memcpy(SetPhoneNumber, phone_num, 15);
    attachInterrupt(digitalPinToInterrupt(pirInterrupt), pirAlarm, LOW);
    Serial.println(F("Set Done"));
  }

  else if (receivedSms == statusMessage)
  {
    //Serial.println("STATUS message received. -> Resp with status & battery level");
    char smsReply[SmsMaxSize];
    String reply;
    if (armed)
    {
      reply += "Toiminta: OK";

      reply += "\nAika:";
      char tmpBuffer[19];
      snprintf(tmpBuffer, 19, "%02d/%02d/%02d,%02d:%02d:%02d",
               year() % 100, month(), day(),
               hour(), minute(), second());
      reply += tmpBuffer;
    }

    reply += "\nKytketty:";
    reply += armedTime;

    reply += "\nAktivoija:";
    reply += SetPhoneNumber;

    reply += "\nHalytys:";
    reply += Traplaunched;

    reply += "\nAkku: ";
    float voltage = (float) analogRead(A1) * 5 / 1024;
    reply += voltage;

    reply.toCharArray(smsReply, SmsMaxSize);
    sms.SendSMS(phone_num, smsReply);
    if (strcmp(SetPhoneNumber, phone_num) != 0 && armed)
    {
      reply += "\nKysely:";
      reply += phone_num;
      reply.length();
      reply.toCharArray(smsReply, SmsMaxSize);
      sms.SendSMS(SetPhoneNumber, smsReply);
    }
    sms_text[0] = '\0';
  }
};

/*
    Query time from GSM server.
    NOTE: 2s delay between req & response reading
*/
void getTime()
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
};

/*
   This function blink led lights.
   NOTE: Tone command would be better here
*/
void alarmSet(int highDelay, int lowDelay, byte times)
{
  Serial.println(F("Alarm!"));
  for (int i = 0; i < times; i++)
  {
    digitalWrite(buzzerPin, HIGH);
    delay(highDelay);
    digitalWrite(buzzerPin, LOW);
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
  sms.SendSMS(SetPhoneNumber, trapArmed);
  alarmSent = true;
}

/*
   Pir alarms ISR (Interrupt Service Routine)
*/
void pirAlarm()
{
  detachInterrupt(digitalPinToInterrupt(pirInterrupt));
  Traplaunched = true;
  PirAlarmValue = true;
}

/*
   Put arduino in PWR_DOWN_SLEEP mode.
   Wake up after nSec has been elapsed, PIR alarm or
   when need to send daily status SMS
*/
void delayWdt(byte nsec)
{
  nbr_remaining = nsec;
  power_adc_disable();
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  while (nbr_remaining > 0 && PirAlarmValue == false)
  {
    sleep_mode();
    sleep_disable();
    if (SecondsCounter >= nextSyncTime)
      break;
  }
  power_all_enable();
}

/*
   Query time from GSM server using getTime()
   and store time to arduino RTC
*/
void getGsmTime()
{
  getTime();
  String currentTime(dateAndTime);
  byte nowYear = currentTime.substring(0, 2).toInt();
  byte nowMonth = currentTime.substring(3, 5).toInt();
  byte nowDate = currentTime.substring(6, 8).toInt();
  byte nowHH = currentTime.substring(9, 11).toInt();
  byte nowMM = currentTime.substring(12, 14).toInt();
  byte nowSec = currentTime.substring(15, 17).toInt();
  setTime((int)nowHH, (int)nowMM, (int)nowSec, (int)nowDate, (int)nowMonth, (int)nowYear);
  adjustTime(3);
}

