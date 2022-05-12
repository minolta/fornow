//=================================================================================//
//   ___    _   ___    _   __                ______   ___    _    _____   _____    //
//  |   \  | | |   \  | | |  |              |  ____| |   \  | |  / ____| |  __ \   //
//  | |\ \ | | | |\ \ | | |  |              |  \___  | |\ \ | | | /  __  | |__ /   //
//  | | \ \| | | | \ \| | |  |___   ______  |  /___  | | \ \| | | \__| | | | \ \   //
//  |_|  \___| |_|  \___| |______| |______| |______| |_|  \___|  \____/  |_|  \_\  //
//                                                                                 //
//=================================================================================//
//    Name:ESP12F_Blynk    Hardware:        Location: 
//=================================================================================//
#include <ArduinoOTA.h>
#include <Ticker.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <BlynkSimpleEsp8266.h>
#include <ESP8266HTTPClient.h>
//===================================================================================
//    Definitions
//===================================================================================
#define host          "ESP12F_Blynk"
#define ota_pass      "test"
#define ssid          "TP Link"
#define pass          "noolekjdee"
#define ntp_serv      "time.windows.com"
#define ntp_offset    25200
#define ntp_interval  60000
#define blynk_serv    "192.168.1.9"
#define blynk_port    8080
#define blynk_auth    "KB0ZRHEn27l9lHaz35VHtM_5XV5zjZ0d"
#define OGS_auth      "YjPI1TjGgEAgzWxbxsIbUHsgwGke7ci7"

//----------Physical IOs-------------------------------------------------------------
// Map On Board LED to physical IO
#define b_led    2    //  2 for ESP-12, 2 for D1 Mini 
// Map channels to physical IO [depend on hardware]
#define p_io1    4    //  4 for ESP-12, 12 for D1 Mini
#define p_io2    5    //  5 for ESP-12, 14 for D1 Mini
#define p_io3   12    // 12 for ESP-12,  5 for D1 Mini
#define p_io4   14    // 14 for ESP-12,  4 for D1 Mini
#define o_buz   i0    // Output for Buzzer
//----------Virtual Pin for Mode-----------------------------------------------------
#define v_sw_mode   0
//----------Virtual Pin for Touch Button--------------------
#define v_b1           1
#define v_b2           2
#define v_b3           3
#define v_b4           4
//------------- Virtual Pin for getvalue ogs
#define v_ogs_bv       5
#define v_ogs_bi       6
//----------Virtual Pin for Channel Status LED--------------
#define v_l1          11
#define v_l2          12
#define v_l3          13
#define v_l4          14
//----------Virtual Pin for Meter---------------------------
#define v_stat        20
#define v_volt        21
#define v_rssi        22
#define v_temp        23
#define v_humi        24
//----------Virtual Pin for Timer's Status LED--------------
#define v_tmr1        31
#define v_tmr2        32
#define v_tmr3        33
#define v_tmr4        34
//----------Virtual Pin for Timer Enable--------------------
#define v_en1         35
#define v_en2         36
#define v_en3         37
#define v_en4         38
#define v_en_all      39
#define v_dis_all     40
//----------Virtual Pin for Timer Input---------------------
#define v_ti1         41
#define v_ti2         42
#define v_ti3         43
#define v_ti4         44 
#define v_ti5         45
#define v_ti6         46
#define v_ti7         47
#define v_ti8         48
//----------Virtual Pin for Timer Map-----------------------
#define v_ct1         51
#define v_ct2         52
#define v_ct3         53
#define v_ct4         54
//----------Virtual Pin for Remote to Channel Map------------------------------------
#define v_rc1         101
#define v_rc2         102
#define v_rc3         103
#define v_rc4         104
//----------Virtual Pin for Channel to Remote's Inputs-------------------------------
#define v_rem1        111
#define v_rem2        112
#define v_rem3        113
#define v_rem4        114
//----------Virtual Pin for Alarm----------------------------------------------------
#define v_buz_on      120    // Buzzer on
#define v_buz_short   121    // Buzzer short
#define v_buz_long    122    // Buzzer long
#define v_alm_on      123    // Buzzer on
#define v_alm_short   124    // Buzzer short
#define v_alm_long    125    // Buzzer long
//----------Virtual Pin for Mass Control---------------------------------------------
#define v_all_on      126
#define v_all_off     127
//===================================================================================
//    Dclaration
//===================================================================================
Ticker tk_10ms;
WiFiClient client;
WiFiUDP ntpUDP;
HTTPClient http;
NTPClient timeClient(ntpUDP,ntp_serv,ntp_offset,ntp_interval);
//===================================================================================
//    Variables
//===================================================================================
unsigned int prog,progb;             //OTA Progress
unsigned int buff;                   //Buffer
unsigned int cnt=0;                  //Counter
unsigned int led_cnt;                //LED Blink Counter
unsigned int buz_cnt;                //Buzzer Counter
unsigned int sec_cnt=0;              // one sencond counter
bool fg_sec;
unsigned int time_now,day_now;
unsigned int t1_start,t1_stop,t1_day;
unsigned int day_and;
unsigned char sw_mode;
unsigned char i0,i1,i2,i3,i4;
unsigned char o0,o0b;
unsigned char o1,o1b;
unsigned char o2,o2b;
unsigned char o3,o3b;
unsigned char o4,o4b;
unsigned char tmr1,tmr2,tmr3,tmr4;   //Timer logics
unsigned char tmr5,tmr6,tmr7,tmr8;
bool fg_tmr1,fg_tmr2,fg_tmr3,fg_tmr4;
bool fg_tmr5,fg_tmr6,fg_tmr7,fg_tmr8;
unsigned int start1,start2,start3,start4;
unsigned int start5,start6,start7,start8;
unsigned int stop1,stop2,stop3,stop4;
unsigned int stop5,stop6,stop7,stop8;
unsigned int day1,day2,day3,day4;
unsigned int day5,day6,day7,day8;
unsigned char ct1,ct2,ct3,ct4;          //Channel to Timer
// unsigned char rc1,rc2,rc3,rc4;       //Remote to Channel
// unsigned char rem1,rem2,rem3,rem4;   //Remote logics
// unsigned char rch1,rch2,rch3,rch4;   //Remote Channel
unsigned char all_on,all_off;
float volt;
float rssi;
float humi, humib;
float temp, tempb;
bool fg_dht = 0;
float ogs_bv,ogs_bi;

//===================================================================================
//    function get request
float blynk_get(String auth,int vpin) {

  String url = "http://";
  url += blynk_serv;
  url += ":";
  url += blynk_port;
  url += "/";
  url += auth;
  url += "/get/v";
  url += String(vpin);
  http.begin(client,url);
  int httpCode = http.GET();
  if (httpCode == 200) {
    String content = http.getString();
    return content.substring(2).toFloat();;
  }
  return 0.0;
}
//
// mdbv=blynk_get(OGS_auth,v_mdbv);
//===================================================================================

//===================================================================================
//    Subroutines
//===================================================================================
//----------On Board LED------------------------------------
void b_led_on() {
  digitalWrite(b_led,0);
  led_cnt=2;
}
void b_led_blink() {
  digitalWrite(b_led,0);
  delay(10);
  digitalWrite(b_led,1);
}
void b_led_toggle() {
  digitalWrite(b_led,!digitalRead(b_led));
}
void b_led_off() {            
  digitalWrite(b_led,1);
}
void b_led_auto_off() {            
  if (led_cnt) {
    led_cnt--;
    if (led_cnt==0) {
      digitalWrite(b_led,1);
    }
  }
}

//----------Buzzer--------------------------------------------
void buzz_on() {   
  o_buz=1;
  buz_cnt=0;
}
void buzz_short() {   
  o_buz=1;
  buz_cnt=5;
}
void buzz_long() {   
  o_buz=1;
  buz_cnt=100;
}
void buzz_auto_off() {            
  if (buz_cnt) {
    buz_cnt--;
    if (buz_cnt==0) {
      o_buz=0;
  Blynk.virtualWrite(v_buz_on,0);
  Blynk.virtualWrite(v_buz_short,0);
  Blynk.virtualWrite(v_buz_long,0);
    }
  }
}
void buzz_off() {
  buz_cnt=0;
  o_buz=0;
  Blynk.virtualWrite(v_buz_on,0);
  Blynk.virtualWrite(v_buz_short,0);
  Blynk.virtualWrite(v_buz_long,0);
}

//----------Get time----------------------------------------
void get_time() {
  timeClient.update();
  time_now=3600*timeClient.getHours()+60*timeClient.getMinutes()+timeClient.getSeconds();
  if (timeClient.getDay()==0) day_now=0b01000000;
  if (timeClient.getDay()==1) day_now=0b00100000;
  if (timeClient.getDay()==2) day_now=0b00010000;
  if (timeClient.getDay()==3) day_now=0b00001000;
  if (timeClient.getDay()==4) day_now=0b00000100;
  if (timeClient.getDay()==5) day_now=0b00000010;
  if (timeClient.getDay()==6) day_now=0b00000001;
}

//----------Ten millsecond----------------------------------
void ten_ms() {
  if (sec_cnt==0) {
    sec_cnt=100;
    fg_sec=1;
  }
  sec_cnt--;
  b_led_auto_off();
  buzz_auto_off();
}

//----------Timer Compare-----------------------------------
bool time_compare(int t_start,int t_stop,int t_day) {    //no sunrise, no sunset
  bool t_result;
  if (day_now&t_day) {
    if (t_start<t_stop) {
      if (time_now>=t_start&&time_now<t_stop) {
        t_result=1;
      }
      else t_result=0;
    }
    else if (t_stop<t_start) {
      if (time_now>=t_stop&&time_now<t_start) {
        t_result=0;
      }
      else t_result=1;
    }
  }
  else t_result=0;
  return t_result;
}

//----------Process Timers----------------------------------
void timer_calc() {
  fg_tmr1=0;
  if (start1<86400&&stop1<86400) {
    fg_tmr1=1;
    tmr1=time_compare(start1,stop1,day1);
  }
  fg_tmr2=0;
  if (start2<86400&&stop2<86400) {
    fg_tmr2=1;
    tmr2=time_compare(start2,stop2,day2);
  }
  fg_tmr3=0;
  if (start3<86400&&stop3<86400) {
    fg_tmr3=1;
    tmr3=time_compare(start3,stop3,day3);
  }
  fg_tmr4=0;
  if (start4<86400&&stop4<86400) {
    fg_tmr4=1;
    tmr4=time_compare(start4,stop4,day4);
  }
  fg_tmr5=0;
  if (start5<86400&&stop5<86400) {
    fg_tmr5=1;
    tmr5=time_compare(start5,stop5,day5);
  }
  fg_tmr6=0;
  if (start6<86400&&stop6<86400) {
    fg_tmr6=1;
    tmr6=time_compare(start6,stop6,day6);
  }
  fg_tmr7=0;
  if (start7<86400&&stop7<86400) {
    fg_tmr7=1;
    tmr7=time_compare(start7,stop7,day7);
  }
  fg_tmr8=0;
  if (start8<86400&&stop8<86400) {
    fg_tmr8=1;
    tmr8=time_compare(start8,stop8,day8);
  }
}

void manual() {
  o1=i1;
  o2=i2;
  o3=i3;
  o4=i4;
}

void automatic() {
  switch (ct1) {
    default: {
      o1=i1;
      break;
    }
    case 2: {
      if (fg_tmr1) o1=tmr1;
      else o1=i1;
      break;
    }
    case 3: {
      if (fg_tmr2) o1=tmr2;
      else o1=i1;
      break;
    }
    case 4: {
      if (fg_tmr3) o1=tmr3;
      else o1=i1;
      break;
    }
    case 5: {
      if (fg_tmr4) o1=tmr4;
      else o1=i1;
      break;
    }
    case 6: {
      if (fg_tmr5) o1=tmr5;
        else o1=i1;
      break;
    }
    case 7: {
      if (fg_tmr6) o1=tmr6;
      else o1=i1;
      break;
    }
    case 8: {
      if (fg_tmr7) o1=tmr7;
      else o1=i1;
      break;
    }
    case 9: {
      if (fg_tmr8) o1=tmr8;
      else o1=i1;
      break;
    }
  }
  switch (ct2) {
    default: {
      o2=i2;
      break;
    }
    case 2: {
      if (fg_tmr1) o2=tmr1;
      else o2=i2;
      break;
    }
    case 3: {
      if (fg_tmr2) o2=tmr2;
      else o2=i2;
      break;
    }
    case 4: {
      if (fg_tmr3) o2=tmr3;
      else o2=i2;
      break;
    }
    case 5: {
      if (fg_tmr4) o2=tmr4;
      else o2=i2;
      break;
    }
    case 6: {
      if (fg_tmr5) o2=tmr5;
        else o2=i2;
      break;
    }
    case 7: {
      if (fg_tmr6) o2=tmr6;
      else o2=i2;
      break;
    }
    case 8: {
      if (fg_tmr7) o2=tmr7;
      else o2=i2;
      break;
    }
    case 9: {
      if (fg_tmr8) o2=tmr8;
      else o2=i2;
      break;
    }
  }
  switch (ct3) {
    default: {
      o3=i3;
      break;
    }
    case 2: {
      if (fg_tmr1) o3=tmr1;
      else o3=i3;
      break;
    }
    case 3: {
      if (fg_tmr2) o3=tmr2;
      else o3=i3;
      break;
    }
    case 4: {
      if (fg_tmr3) o3=tmr3;
      else o3=i3;
      break;
    }
    case 5: {
      if (fg_tmr4) o3=tmr4;
      else o3=i3;
      break;
    }
    case 6: {
      if (fg_tmr5) o3=tmr5;
        else o3=i3;
      break;
    }
    case 7: {
      if (fg_tmr6) o3=tmr6;
      else o3=i3;
      break;
    }
    case 8: {
      if (fg_tmr7) o3=tmr7;
      else o3=i3;
      break;
    }
    case 9: {
      if (fg_tmr8) o3=tmr8;
      else o3=i3;
      break;
    }
  }
  switch (ct4) {
    default: {
      o4=i4;
      break;
    }
    case 2: {
      if (fg_tmr1) o4=tmr1;
      else o4=i4;
      break;
    }
    case 3: {
      if (fg_tmr2) o4=tmr2;
      else o4=i4;
      break;
    }
    case 4: {
      if (fg_tmr3) o4=tmr3;
      else o4=i4;
      break;
    }
    case 5: {
      if (fg_tmr4) o4=tmr4;
      else o4=i4;
      break;
    }
    case 6: {
      if (fg_tmr5) o4=tmr5;
        else o4=i4;
      break;
    }
    case 7: {
      if (fg_tmr6) o4=tmr6;
      else o4=i4;
      break;
    }
    case 8: {
      if (fg_tmr7) o4=tmr7;
      else o4=i4;
      break;
    }
    case 9: {
      if (fg_tmr8) o4=tmr8;
      else o4=i4;
      break;
    }
  }
}

void butt_sync() {
  if (i1!=o1) {
    i1=o1;
    Blynk.virtualWrite(v_b1,o1);
  }
  if (i2!=o2) {
    i2=o2;
    Blynk.virtualWrite(v_b2,o2);
  }
  if (i3!=o3) {
    i3=o3;
    Blynk.virtualWrite(v_b3,o3);
  }
  if (i4!=o4) {
    i4=o4;
    Blynk.virtualWrite(v_b4,o4);
  }
}


void manauto() {
  if (sw_mode) automatic();
  else manual();
  butt_sync();  
}

void io_update() {
  digitalWrite(p_io1,o1);
  if (o1!=o1b) {
    Blynk.virtualWrite(v_l1,o1*255);
    Blynk.virtualWrite(v_b1,o1);
  }
  digitalWrite(p_io2,o2);
  if (o2!=o2b) {
    Blynk.virtualWrite(v_l2,o2*255);
    Blynk.virtualWrite(v_b2,o2);
  }
  digitalWrite(p_io3,o3);
  if (o3!=o3b) {
    Blynk.virtualWrite(v_l3,o3*255);
    Blynk.virtualWrite(v_b3,o3);
  }
  digitalWrite(p_io4,o4);
  if (o4!=o4b) {
    Blynk.virtualWrite(v_l4,o4*255);
    Blynk.virtualWrite(v_b4,o4);
  }
}

void io_buff() {
  o1b=o1;
  o2b=o2;
  o3b=o3;
  o4b=o4;
}

//----------DHT Read--------------------------------------
void dht_read() {
  if (fg_dht) {
    fg_dht=0;
    b_led_on();
    rssi = WiFi.RSSI();
    Blynk.virtualWrite(v_rssi,rssi);
  }
}

//===================================================================================
//    Blynk Service Routine
//===================================================================================
BLYNK_CONNECTED() {
  Blynk.syncAll();
}

BLYNK_WRITE(v_sw_mode) {
  b_led_on();
  sw_mode = param.asInt();
}

BLYNK_WRITE(v_b1) {
  b_led_on();
  i1 = param.asInt();
}

BLYNK_WRITE(v_b2) {
  b_led_on();
  i2 = param.asInt();
}

BLYNK_WRITE(v_b3) {
  b_led_on();
  i3 = param.asInt();
}

BLYNK_WRITE(v_b4) {
  b_led_on();
  i4 = param.asInt();
}

BLYNK_WRITE(v_ct1) {
  b_led_on();
  ct1 = param.asInt();
}

BLYNK_WRITE(v_ct2) {
  b_led_on();
  ct2 = param.asInt();
}

BLYNK_WRITE(v_ct3) {
  b_led_on();
  ct3 = param.asInt();
}

BLYNK_WRITE(v_ct4) {
  b_led_on();
  ct4 = param.asInt();
}

// ---- get data

//--------------

BLYNK_WRITE(v_ti1) {
  TimeInputParam t(param);
  // Process start time
  if (t.hasStartTime())
  {
    start1=3600*t.getStartHour()+60*t.getStartMinute()+t.getStartSecond();
  }
  else
  {
    start1=86400;
  }
  // Process stop time
  if (t.hasStopTime())
  {
    stop1=3600*t.getStopHour()+60*t.getStopMinute()+t.getStopSecond();
  }
  else
  {
    stop1=86400;
  }
  // Process weekdays (1. Mon, 2. Tue, 3. Wed, ...)
  day1=0;
  if (t.isWeekdaySelected(7)) day1++;
  day1=day1<<1;
  if (t.isWeekdaySelected(1)) day1++;
  day1=day1<<1;
  if (t.isWeekdaySelected(2)) day1++;
  day1=day1<<1;
  if (t.isWeekdaySelected(3)) day1++;
  day1=day1<<1;
  if (t.isWeekdaySelected(4)) day1++;
  day1=day1<<1;
  if (t.isWeekdaySelected(5)) day1++;
  day1=day1<<1;
  if (t.isWeekdaySelected(6)) day1++;
}

BLYNK_WRITE(v_ti2) {
  TimeInputParam t(param);
  // Process start time
  if (t.hasStartTime())
  {
    start2=3600*t.getStartHour()+60*t.getStartMinute()+t.getStartSecond();
  }
  else
  {
    start2=86400;
  }
  // Process stop time
  if (t.hasStopTime())
  {
    stop2=3600*t.getStopHour()+60*t.getStopMinute()+t.getStopSecond();
  }
  else
  {
    stop2=86400;
  }
  // Process weekdays (1. Mon, 2. Tue, 3. Wed, ...)
  day2=0;
  if (t.isWeekdaySelected(7)) day2++;
  day2=day2<<1;
  if (t.isWeekdaySelected(1)) day2++;
  day2=day2<<1;
  if (t.isWeekdaySelected(2)) day2++;
  day2=day2<<1;
  if (t.isWeekdaySelected(3)) day2++;
  day2=day2<<1;
  if (t.isWeekdaySelected(4)) day2++;
  day2=day2<<1;
  if (t.isWeekdaySelected(5)) day2++;
  day2=day2<<1;
  if (t.isWeekdaySelected(6)) day2++;
}

BLYNK_WRITE(v_ti3) {
  TimeInputParam t(param);
  // Process start time
  if (t.hasStartTime())
  {
    start3=3600*t.getStartHour()+60*t.getStartMinute()+t.getStartSecond();
  }
  else
  {
    start3=86400;
  }
  // Process stop time
  if (t.hasStopTime())
  {
    stop3=3600*t.getStopHour()+60*t.getStopMinute()+t.getStopSecond();
  }
  else
  {
    stop3=86400;
  }
  // Process weekdays (1. Mon, 2. Tue, 3. Wed, ...)
  day3=0;
  if (t.isWeekdaySelected(7)) day3++;
  day3=day3<<1;
  if (t.isWeekdaySelected(1)) day3++;
  day3=day3<<1;
  if (t.isWeekdaySelected(2)) day3++;
  day3=day3<<1;
  if (t.isWeekdaySelected(3)) day3++;
  day3=day3<<1;
  if (t.isWeekdaySelected(4)) day3++;
  day3=day3<<1;
  if (t.isWeekdaySelected(5)) day3++;
  day3=day3<<1;
  if (t.isWeekdaySelected(6)) day3++;
}

BLYNK_WRITE(v_ti4) {
  TimeInputParam t(param);
  // Process start time
  if (t.hasStartTime())
  {
    start4=3600*t.getStartHour()+60*t.getStartMinute()+t.getStartSecond();
  }
  else
  {
    start4=86400;
  }
  // Process stop time
  if (t.hasStopTime())
  {
    stop4=3600*t.getStopHour()+60*t.getStopMinute()+t.getStopSecond();
  }
  else
  {
    stop4=86400;
  }
  // Process weekdays (1. Mon, 2. Tue, 3. Wed, ...)
  day4=0;
  if (t.isWeekdaySelected(7)) day4++;
  day4=day4<<1;
  if (t.isWeekdaySelected(1)) day4++;
  day4=day4<<1;
  if (t.isWeekdaySelected(2)) day4++;
  day4=day4<<1;
  if (t.isWeekdaySelected(3)) day4++;
  day4=day4<<1;
  if (t.isWeekdaySelected(4)) day4++;
  day4=day4<<1;
  if (t.isWeekdaySelected(5)) day4++;
  day4=day4<<1;
  if (t.isWeekdaySelected(6)) day4++;
}

BLYNK_WRITE(v_ti5) {
  TimeInputParam t(param);
  // Process start time
  if (t.hasStartTime())
  {
    start5=3600*t.getStartHour()+60*t.getStartMinute()+t.getStartSecond();
  }
  else
  {
    start5=86400;
  }
  // Process stop time
  if (t.hasStopTime())
  {
    stop5=3600*t.getStopHour()+60*t.getStopMinute()+t.getStopSecond();
  }
  else
  {
    stop5=86400;
  }
  // Process weekdays (1. Mon, 2. Tue, 3. Wed, ...)
  day5=0;
  if (t.isWeekdaySelected(7)) day5++;
  day5=day5<<1;
  if (t.isWeekdaySelected(1)) day5++;
  day5=day5<<1;
  if (t.isWeekdaySelected(2)) day5++;
  day5=day5<<1;
  if (t.isWeekdaySelected(3)) day5++;
  day5=day5<<1;
  if (t.isWeekdaySelected(4)) day5++;
  day5=day5<<1;
  if (t.isWeekdaySelected(5)) day5++;
  day5=day5<<1;
  if (t.isWeekdaySelected(6)) day5++;
}

BLYNK_WRITE(v_ti6) {
  TimeInputParam t(param);
  // Process start time
  if (t.hasStartTime())
  {
    start6=3600*t.getStartHour()+60*t.getStartMinute()+t.getStartSecond();
  }
  else
  {
    start6=86400;
  }
  // Process stop time
  if (t.hasStopTime())
  {
    stop6=3600*t.getStopHour()+60*t.getStopMinute()+t.getStopSecond();
  }
  else
  {
    stop6=86400;
  }
  // Process weekdays (1. Mon, 2. Tue, 3. Wed, ...)
  day6=0;
  if (t.isWeekdaySelected(7)) day6++;
  day6=day6<<1;
  if (t.isWeekdaySelected(1)) day6++;
  day6=day6<<1;
  if (t.isWeekdaySelected(2)) day6++;
  day6=day6<<1;
  if (t.isWeekdaySelected(3)) day6++;
  day6=day6<<1;
  if (t.isWeekdaySelected(4)) day6++;
  day6=day6<<1;
  if (t.isWeekdaySelected(5)) day6++;
  day6=day6<<1;
  if (t.isWeekdaySelected(6)) day6++;
}

BLYNK_WRITE(v_ti7) {
  TimeInputParam t(param);
  // Process start time
  if (t.hasStartTime())
  {
    start7=3600*t.getStartHour()+60*t.getStartMinute()+t.getStartSecond();
  }
  else
  {
    start7=86400;
  }
  // Process stop time
  if (t.hasStopTime())
  {
    stop7=3600*t.getStopHour()+60*t.getStopMinute()+t.getStopSecond();
  }
  else
  {
    stop7=86400;
  }
  // Process weekdays (1. Mon, 2. Tue, 3. Wed, ...)
  day7=0;
  if (t.isWeekdaySelected(7)) day7++;
  day7=day7<<1;
  if (t.isWeekdaySelected(1)) day7++;
  day7=day7<<1;
  if (t.isWeekdaySelected(2)) day7++;
  day7=day7<<1;
  if (t.isWeekdaySelected(3)) day7++;
  day7=day7<<1;
  if (t.isWeekdaySelected(4)) day7++;
  day7=day7<<1;
  if (t.isWeekdaySelected(5)) day7++;
  day7=day7<<1;
  if (t.isWeekdaySelected(6)) day7++;
}

BLYNK_WRITE(v_ti8) {
  TimeInputParam t(param);
  // Process start time
  if (t.hasStartTime())
  {
    start8=3600*t.getStartHour()+60*t.getStartMinute()+t.getStartSecond();
  }
  else
  {
    start8=86400;
  }
  // Process stop time
  if (t.hasStopTime())
  {
    stop8=3600*t.getStopHour()+60*t.getStopMinute()+t.getStopSecond();
  }
  else
  {
    stop8=86400;
  }
  // Process weekdays (1. Mon, 2. Tue, 3. Wed, ...)
  day8=0;
  if (t.isWeekdaySelected(7)) day8++;
  day8=day8<<1;
  if (t.isWeekdaySelected(1)) day8++;
  day8=day8<<1;
  if (t.isWeekdaySelected(2)) day8++;
  day8=day8<<1;
  if (t.isWeekdaySelected(3)) day8++;
  day8=day8<<1;
  if (t.isWeekdaySelected(4)) day8++;
  day8=day8<<1;
  if (t.isWeekdaySelected(5)) day8++;
  day8=day8<<1;
  if (t.isWeekdaySelected(6)) day8++;
}

BLYNK_WRITE(v_buz_on) {
  b_led_on();
  if (param.asInt()==1) {
    buzz_on();
  }
  else {
    buzz_off();
  }
}

BLYNK_WRITE(v_buz_short) {
  b_led_on();
  if (param.asInt()==1) {
    buzz_short();
  }
}

BLYNK_WRITE(v_buz_long) {
  b_led_on();
  if (param.asInt()==1) {
    buzz_long();
  }
  else {
    buzz_off();
  }
}

BLYNK_WRITE(v_all_on) {
  b_led_on();
  all_on=param.asInt();
  if (all_on) {
    Blynk.virtualWrite(v_all_on,0);
    i1=1;
    i2=1;
    i3=1;
    i4=1;
    buz_cnt=1;
  }
}

BLYNK_WRITE(v_all_off) {
  b_led_on();
  all_off=param.asInt();
  if (all_off) {
    Blynk.virtualWrite(v_all_off,0);
    i1=0;
    i2=0;
    i3=0;
    i4=0;
  }
}
//===================================================================================
//    Setup Routine
//===================================================================================
void setup() {
  //---------- initial GPIO pins ----------
  pinMode(p_io1,OUTPUT);
  pinMode(p_io2,OUTPUT);
  pinMode(p_io3,OUTPUT);
  pinMode(p_io4,OUTPUT);
  pinMode(b_led,OUTPUT);          //On Board LED
  digitalWrite(b_led,1);

  //---------- Start WiFi ----------
  Serial.begin(115200);
  Serial.println("");
  Serial.println("][ NNL_ENGINEERING ][");
  Serial.println("Booting...");
  WiFi.mode(WIFI_STA);
  WiFi.hostname(host);
  WiFi.begin(ssid,pass);
  Serial.print("WiFi connecting");
  while(WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    b_led_blink();
    delay(500);
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("Local IP address: ");
  Serial.println(WiFi.localIP());

  //---------- Start Blynk ----------
  Blynk.config(blynk_auth,blynk_serv,blynk_port);

  //---------- Start 10ms Timer ----------
  tk_10ms.attach_ms(10,ten_ms);

  //---------- OTA Setup ----------
  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);
  ArduinoOTA.setHostname(host);
  ArduinoOTA.setPassword(ota_pass);
  ArduinoOTA.onStart([]() {
    Serial.println("OTA Start");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("OTA End");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    prog=(progress/(total/10))*10;
    if (prog!=progb) {
      Serial.printf("OTA Progress: %u%%\r\n", prog);
    }
    progb=prog;
    // Serial.printf("OTA Progress: %u%%\n", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
}
//===================================================================================
//    Main Routine
//===================================================================================
void loop() {
  ArduinoOTA.handle();
  Blynk.run();
  if (fg_sec) {
    fg_sec=0;
    b_led_on();
    get_time();
    timer_calc();
    rssi=WiFi.RSSI();
    Blynk.virtualWrite(v_rssi,rssi);
    Blynk.virtualWrite(v_volt,volt);
    Blynk.virtualWrite(v_temp,temp);
    Blynk.virtualWrite(v_humi,humi);

    ogs_bv = blynk_get(OGS_auth,22);
    Blynk.virtualWrite(v_ogs_bv,ogs_bv);
    ogs_bi = blynk_get(OGS_auth,23);
    Blynk.virtualWrite(v_ogs_bi,ogs_bi);


  }
  manauto();
  io_update();
  io_buff();
}
//===================================================================================
//                                      THE END                                      
//===================================================================================
