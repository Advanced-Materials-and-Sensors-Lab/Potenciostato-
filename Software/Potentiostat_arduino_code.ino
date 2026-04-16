//Codigo Arduino Potenciostato, Codigo para Arduino UNO 
/* =========================================================
   JUAMI Potentiostat – Firmware v1.2
   UNO: D9=PWM, A0=VMON, A1=IMON
   Serie: 500000 8N1
   Cmd:
     IDN?
     CAL  <scale> <offset>
     CA   <V> <ms>
     CAS  <V> <sec>
     HOLD <V> <ms>
     LSV  <Vstart> <Vend> <rate_mV_s>
     CV   <Vlow> <Vhigh> <rate_mV_s> <cycles>
     STOP
   CSV:
     t_ms,set_V,VMON_V,IMON_V,ADC0,ADC1,PWMCODE
   ========================================================= */

#include <Arduino.h>
#include <string.h>
#include <stdlib.h>

// Pines
const uint8_t PIN_PWM  = 9;
const uint8_t PIN_VMON = A0;
const uint8_t PIN_IMON = A1;

// Calibración PWM
volatile float DAC_SCALE  = 128.0f;
volatile float DAC_OFFSET = 128.0f;

// ADC
const float ADC_REF_V = 5.0f;
const int   ADC_MAX   = 1023;
const uint8_t N_AVG   = 2; // más rápido (algo más de ruido)

// Estado
volatile bool g_abort = false;

// Utilidades
inline uint8_t v_to_pwm(float v){
  long code = lroundf(DAC_SCALE*v + DAC_OFFSET);
  if (code < 0)   code = 0;
  if (code > 255) code = 255;
  return (uint8_t)code;
}
inline float pwm_to_v(uint8_t code){ return ((float)code - DAC_OFFSET)/DAC_SCALE; }

long read_adc_avg(uint8_t pin){
  long acc=0; for(uint8_t i=0;i<N_AVG;i++) acc += analogRead(pin);
  return acc / N_AVG;
}
inline float adc_to_volt(long a){ return ((float)a/(float)ADC_MAX)*ADC_REF_V; }
inline void set_pwm_code(uint8_t code){ analogWrite(PIN_PWM, code); }

unsigned long step_delay_us_for_rate(float rate_mV_s){
  float rVs = rate_mV_s/1000.0f; if (rVs<=0) rVs=0.001f;
  float dV  = 1.0f/DAC_SCALE;
  float dt  = (dV/rVs)*1e6f;
  if (dt < 100.0f) dt = 100.0f;  // mínimo 0.1 ms
  return (unsigned long)dt;
}

void emit_csv(unsigned long t_ms, uint8_t code, long a0, long a1){
  Serial.print(t_ms); Serial.print(',');
  Serial.print(pwm_to_v(code),6); Serial.print(',');
  Serial.print(adc_to_volt(a0),6); Serial.print(',');
  Serial.print(adc_to_volt(a1),6); Serial.print(',');
  Serial.print(a0); Serial.print(',');
  Serial.print(a1); Serial.print(',');
  Serial.println(code);
}

// Modos
void do_hold_ms(float Vset, unsigned long dur_ms){
  g_abort=false;
  uint8_t code = v_to_pwm(Vset);
  unsigned long t0=millis();
  Serial.println("START");
  while(!g_abort && (millis()-t0 < dur_ms)){
    set_pwm_code(code);
    long a0=read_adc_avg(PIN_VMON), a1=read_adc_avg(PIN_IMON);
    emit_csv(millis()-t0, code, a0, a1);
  }
  Serial.println("END");
}

void do_lsv(float Vstart, float Vend, float rate_mV_s){
  g_abort=false;
  int8_t dir = (Vend>=Vstart)? 1 : -1;
  uint8_t code=v_to_pwm(Vstart), cend=v_to_pwm(Vend);
  unsigned long dt=step_delay_us_for_rate(rate_mV_s);
  unsigned long t0=millis();
  Serial.println("START");
  while(!g_abort && ((dir>0 && code<=cend) || (dir<0 && code>=cend))){
    set_pwm_code(code);
    long a0=read_adc_avg(PIN_VMON), a1=read_adc_avg(PIN_IMON);
    emit_csv(millis()-t0, code, a0, a1);
    int nx=(int)code + dir; if(nx<0 || nx>255) break;
    code=(uint8_t)nx;
    delayMicroseconds(dt);
  }
  Serial.println("END");
}

void do_cv(float Vlow, float Vhigh, float rate_mV_s, uint16_t cycles){
  if (Vhigh < Vlow){ float t=Vlow; Vlow=Vhigh; Vhigh=t; }
  g_abort=false;
  unsigned long dt=step_delay_us_for_rate(rate_mV_s);
  uint8_t cL=v_to_pwm(Vlow), cH=v_to_pwm(Vhigh);
  unsigned long t0=millis();
  Serial.println("START");
  for(uint16_t k=0;k<cycles && !g_abort;k++){
    for(int c=cL;c<=cH && !g_abort;c++){
      set_pwm_code((uint8_t)c);
      long a0=read_adc_avg(PIN_VMON), a1=read_adc_avg(PIN_IMON);
      emit_csv(millis()-t0,(uint8_t)c,a0,a1);
      delayMicroseconds(dt);
    }
    for(int c=cH;c>=cL && !g_abort;c--){
      set_pwm_code((uint8_t)c);
      long a0=read_adc_avg(PIN_VMON), a1=read_adc_avg(PIN_IMON);
      emit_csv(millis()-t0,(uint8_t)c,a0,a1);
      delayMicroseconds(dt);
    }
  }
  Serial.println("END");
}

// Parser robusto
static char buf[160];
static uint8_t bi=0;

float parse_float(char* s){ return strtod(s, nullptr); }
unsigned long parse_ulong(char* s){ return strtoul(s, nullptr, 10); }

void handle_line(char* line){
  while(*line==' '||*line=='\t') ++line;
  if(!*line) return;

  Serial.print("ECHO "); Serial.println(line);

  char *cmd = strtok(line, " \t");
  if(!cmd){ Serial.println("ERR"); return; }

  if(!strncmp(cmd,"IDN?",4)){ Serial.println("JUAMI-UNO"); return; }
  if(!strncmp(cmd,"STOP",4)){ g_abort=true; Serial.println("OK"); return; }

  if(!strcmp(cmd,"CAL")){
    char *p1=strtok(nullptr," \t"), *p2=strtok(nullptr," \t");
    if(p1 && p2){
      float sc=parse_float(p1), of=parse_float(p2);
      if(sc>0){ DAC_SCALE=sc; DAC_OFFSET=of; Serial.println("OK"); return; }
    } Serial.println("ERR"); return;
  }

  if(!strcmp(cmd,"CA") || !strcmp(cmd,"HOLD")){
    char *p1=strtok(nullptr," \t"), *p2=strtok(nullptr," \t");
    if(p1 && p2){
      float v = parse_float(p1);
      unsigned long ms = parse_ulong(p2);
      if(ms>0){ do_hold_ms(v, ms); return; }
    } Serial.println("ERR"); return;
  }

  if(!strcmp(cmd,"CAS")){
    char *p1=strtok(nullptr," \t"), *p2=strtok(nullptr," \t");
    if(p1 && p2){
      float v = parse_float(p1);
      float sec = parse_float(p2);
      if(sec>0){ unsigned long ms = (unsigned long)lroundf(sec*1000.0f);
        do_hold_ms(v, ms); return; }
    } Serial.println("ERR"); return;
  }

  if(!strcmp(cmd,"LSV")){
    char *p1=strtok(nullptr," \t"), *p2=strtok(nullptr," \t"), *p3=strtok(nullptr," \t");
    if(p1 && p2 && p3){
      float v0=parse_float(p1), v1=parse_float(p2), rate=parse_float(p3);
      if(rate>0){ do_lsv(v0, v1, rate); return; }
    } Serial.println("ERR"); return;
  }

  if(!strcmp(cmd,"CV")){
    char *p1=strtok(nullptr," \t"), *p2=strtok(nullptr," \t"), *p3=strtok(nullptr," \t"), *p4=strtok(nullptr," \t");
    if(p1 && p2 && p3 && p4){
      float vl=parse_float(p1), vh=parse_float(p2), rate=parse_float(p3);
      long cyc = strtol(p4, nullptr, 10);
      if(rate>0 && cyc>0){ do_cv(vl, vh, rate, (uint16_t)cyc); return; }
    } Serial.println("ERR"); return;
  }

  Serial.println("ERR");
}

// ---------- setup/loop ----------
void setup(){
  Serial.begin(500000);                  // más rápido
  pinMode(PIN_PWM, OUTPUT);
  pinMode(PIN_VMON, INPUT);
  pinMode(PIN_IMON, INPUT);

  // PWM ~31 kHz en D9 (Timer1 prescaler=1)
  TCCR1B = (TCCR1B & 0b11111000) | 0x01;

  // ADC rápido (prescaler=16)
  ADCSRA = (ADCSRA & 0b11111000) | 0x04;

  analogWrite(PIN_PWM, (uint8_t)DAC_OFFSET);
  Serial.println("READY");
}

void loop(){
  while(Serial.available()){
    char c = Serial.read();
    if(c=='\r') continue;
    if(c=='\n'){ buf[bi]=0; bi=0; handle_line(buf); }
    else if(bi < sizeof(buf)-1){ buf[bi++] = c; }
  }
}
