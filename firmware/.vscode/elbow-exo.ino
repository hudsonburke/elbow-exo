/*********** BLE-triggered dual-motor PID with real-time θ plots (θ1 θ2) ***********/
#include <Arduino.h>

/* ===== Stop mode & polarity ===== */
#define VNH_STOP_BRAKE 1
#define M1_DIR_INV 0
#define M1_ENC_INV 0
#define M2_DIR_INV 0
#define M2_ENC_INV 0

#if defined(ARDUINO_ARCH_ESP32)
  #include <math.h>
  #include <stdlib.h>
  #include <ctype.h>
  #define ATOMIC_READ(stmt) do { noInterrupts(); stmt; interrupts(); } while (0)
#else
  #include <util/atomic.h>
  #include <math.h>
  #include <stdlib.h>
  #include <ctype.h>
  #define ATOMIC_READ(stmt) ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { stmt; }
#endif

/* ===== Pins (adjust if your wiring differs) ===== */
#define ENCA  2
#define ENCB  3
#define PWM   5
#define IN1   7
#define IN2   6

#define ENCA2 8
#define ENCB2 9
#define PWM2  10
#define IN3   11
#define IN4   12

/* ===== Encoder CPR guess (auto-calibrates after first full cycle) ===== */
static const float CPR_GUESS = 300.0f;

/* ===== Motion / PID ===== */
const float ANGLE_MIN = -1000.0f, ANGLE_MAX = 1000.0f;
float thetaHigh = -1000.0f, thetaLow = +180.0f;

float kp = 1.2f, ki = 0.10f, kd = 0.025f;
const unsigned long TOP_HOLD_MS    = 2000;
const unsigned long BOTTOM_HOLD_MS = 0;
const int   PWM_MIN = 50;
const float UMAX    = 160.0f;
const float D_ALPHA = 0.08f;
const float I_MAX   = 2000.0f;
const float VEL_CAP = 1e6f;

/* ===== Slew & plotting ===== */
int last_pwm1 = 0, last_pwm2 = 0;
const int PWM_SLEW = 12;
const uint16_t PLOT_HZ = 40;
static unsigned long plot_prev_ms = 0;

/* ===== Timing/state ===== */
unsigned long t_prev = 0, hold_t = 0;
volatile long pos1_i = 0, pos2_i = 0;
float i1_acc = 0.0f, i2_acc = 0.0f;
float d1_filt = 0.0f, d2_filt = 0.0f;
long  pos1_prev = 0, pos2_prev = 0;

enum MoveState { IDLE, TO_HIGH, HOLD_TOP, TO_LOW, HOLD_BOTTOM, DONE };
MoveState m1_state = IDLE, m2_state = IDLE;
enum Active { NONE, M1_ACTIVE, M2_ACTIVE };
Active active = NONE;

/* ===== Angle mapping ===== */
struct AngleMap { float cpr; int sign; long c0; float theta0; };
static inline float clampA(float a){ if(a<ANGLE_MIN)a=ANGLE_MIN; if(a>ANGLE_MAX)a=ANGLE_MAX; return a; }
static inline void angle_init(AngleMap &m, float cpr_exact, int sign=+1, long c0=0, float theta0=0.0f){
  m.cpr = (cpr_exact>1.0f)? cpr_exact : 360.0f; m.sign = (sign>=0)? +1 : -1; m.c0 = c0; m.theta0 = theta0;
}
static inline void angle_set_zero_here(AngleMap &m, long current_count, float theta0=0.0f){ m.c0 = current_count; m.theta0 = theta0; }
static inline float counts_to_deg(const AngleMap &m, long c){ return m.theta0 + m.sign * ((float)(c - m.c0) * 360.0f / m.cpr); }
static inline long  deg_to_counts(const AngleMap &m, float theta_cmd){
  float rel = ((theta_cmd - m.theta0) / (float)m.sign) * (m.cpr / 360.0f);
  return m.c0 + (long)(rel + (rel >= 0 ? 0.5f : -0.5f));
}
static inline void calibrate_from_two_points(AngleMap &M, long cl, long ch, float thetalow, float thetahigh){
  long dcounts = ch - cl; float dtheta = thetahigh - thetalow;
  if (dcounts == 0 || fabsf(dtheta) < 1e-3f) return;
  float new_cpr = fabsf(360.0f * (float)dcounts / dtheta); if (new_cpr < 1.0f) return;
  M.cpr=new_cpr; M.c0=cl; M.theta0=thetalow;
}
AngleMap M1, M2;
#define TOL_DEG 2.0f
static inline long tol_counts_for(const AngleMap &m){ return (long)(TOL_DEG * m.cpr / 360.0f + 0.5f); }
bool  m1_have_high=false, m1_have_low=false; long m1_high_meas=0,  m1_low_meas=0;
bool  m2_have_high=false, m2_have_low=false; long m2_high_meas=0,  m2_low_meas=0;

/* ===== Motor helpers ===== */
static inline void motorCoast(int INa, int INb, int PWMp){ analogWrite(PWMp, 0); digitalWrite(INa, LOW); digitalWrite(INb, LOW); }
static inline void motorBrake(int INa, int INb, int PWMp){ analogWrite(PWMp, 255); digitalWrite(INa, HIGH); digitalWrite(INb, HIGH); }
static inline void motorStop(int INa, int INb, int PWMp){
#if VNH_STOP_BRAKE
  motorBrake(INa, INb, PWMp);
#else
  motorCoast(INa, INb, PWMp);
#endif
}
static inline int dirWithInv(int dir, bool inv){ return inv ? -dir : dir; }
static inline void motorDrive(int INa, int INb, int PWMp, int dir, int pwm){
  if (pwm < 0) pwm = 0; if (pwm > 255) pwm = 255;
  if (dir == 0){ motorStop(INa, INb, PWMp); return; }
  if (pwm > 0 && pwm < PWM_MIN) pwm = PWM_MIN;
  if (dir > 0){ digitalWrite(INa, HIGH); digitalWrite(INb, LOW); }
  else        { digitalWrite(INa, LOW);  digitalWrite(INb, HIGH); }
  analogWrite(PWMp, pwm);
}
static inline void setMotor1(int dir, int pwm){ motorDrive(IN1, IN2, PWM,  dirWithInv(dir, M1_DIR_INV), pwm); }
static inline void setMotor2(int dir, int pwm){ motorDrive(IN3, IN4, PWM2, dirWithInv(dir, M2_DIR_INV), pwm); }
static inline int applySlew(int target, int &last){
  int delta = target - last;
  if (delta >  PWM_SLEW) target = last + PWM_SLEW;
  if (delta < -PWM_SLEW) target = last - PWM_SLEW;
  last = target; return target;
}

/* ===== Encoders (1×: ENCA rising; read ENCB) ===== */
#if defined(ARDUINO_ARCH_ESP32)
void IRAM_ATTR readEncoder1(){
#else
void readEncoder1(){
#endif
  int step = (digitalRead(ENCB) > 0) ? +1 : -1;
  if (M1_ENC_INV) step = -step;
  pos1_i += step;
}
#if defined(ARDUINO_ARCH_ESP32)
void IRAM_ATTR readEncoder2(){
  int step = (digitalRead(ENCB2) > 0) ? +1 : -1;
  if (M2_ENC_INV) step = -step;
  pos2_i += step;
}
static inline void enablePCINT_for_8_9(){ attachInterrupt(digitalPinToInterrupt(ENCA2), readEncoder2, RISING); }
#else
volatile uint8_t prevBmask = 0;
void enablePCINT_for_8_9(){
  PCICR |= (1 << PCIE0);
  PCMSK0 |= (1 << PCINT0) | (1 << PCINT1); // PB0(D8), PB1(D9)
  prevBmask = PINB;
}
ISR(PCINT0_vect){
  uint8_t now = PINB, changed = now ^ prevBmask;
  if (changed & (1 << 0)){ // A changed
    bool A_was = prevBmask & (1 << 0);
    if (!A_was && (now & (1 << 0))){ // rising A
      int step = (now & (1 << 1)) ? +1 : -1;
      if (M2_ENC_INV) step = -step;
      pos2_i += step;
    }
  }
  prevBmask = now;
}
#endif

inline void bumpless_start_M1(long p){ d1_filt = 0.0f; pos1_prev = p; if (i1_acc > I_MAX) i1_acc = I_MAX; if (i1_acc < -I_MAX) i1_acc = -I_MAX; }
inline void bumpless_start_M2(long p){ d2_filt = 0.0f; pos2_prev = p; if (i2_acc > I_MAX) i2_acc = I_MAX; if (i2_acc < -I_MAX) i2_acc = -I_MAX; }

/* ===== Command parser (serial & BLE both call this) ===== */
void process_line(char *line){
  while (*line && isspace((unsigned char)*line)) line++;
  if (!*line) return;
  char cmd = tolower((unsigned char)*line++);
  while (*line && isspace((unsigned char)*line)) line++;

  if (cmd=='z'){ long c1,c2; ATOMIC_READ({ c1=pos1_i; c2=pos2_i; }); angle_set_zero_here(M1,c1,0.0f); angle_set_zero_here(M2,c2,0.0f); return; }

  char *endp=nullptr;
  double a1=strtod(line,&endp); bool have_high=(endp!=line);
  double a2=0.0; bool have_low=false;
  if (have_high){ line=endp; while(*line && isspace((unsigned char)*line)) line++;
                  if (*line){ a2=strtod(line,&endp); have_low=(endp!=line); } }

  if (cmd=='a'){ if (have_high){ thetaHigh = clampA((float)a1); thetaLow = have_low ? clampA((float)a2) : thetaLow; } return; }

  if (cmd=='w' && active==NONE){
    long p; ATOMIC_READ({ p=pos1_i; });
    m1_have_high = m1_have_low = false;
    m1_state=TO_HIGH; m2_state=IDLE; active=M1_ACTIVE; bumpless_start_M1(p);
  } else if (cmd=='e' && active==NONE){
    long p; ATOMIC_READ({ p=pos2_i; });
    m2_have_high = m2_have_low = false;
    m2_state=TO_HIGH; m1_state=IDLE; active=M2_ACTIVE; bumpless_start_M2(p);
  } else {
    active=NONE; m1_state=IDLE; m2_state=IDLE;
    last_pwm1=0; last_pwm2=0; setMotor1(0,0); setMotor2(0,0);
  }
}

/* ===== Serial console passthrough ===== */
void pollSerialLine(){
  static char ibuf[40]; static uint8_t n=0;
  while (Serial.available()){
    char c = Serial.read();
    if ((c=='w'||c=='W') && n==0){ char t[]="w"; process_line(t); continue; }
    if ((c=='e'||c=='E') && n==0){ char t[]="e"; process_line(t); continue; }
    if ((c=='a'||c=='A') && n==0){ char t[]="a"; process_line(t); continue; }
    if ((c=='z'||c=='Z') && n==0){ char t[]="z"; process_line(t); continue; }
    if (c=='\r'||c=='\n'){ ibuf[n]='\0'; if(n>0) process_line(ibuf); n=0; continue; }
    if (n < sizeof(ibuf)-1) ibuf[n++]=c; else n=0;
  }
}

/* ===== BLE (ArduinoBLE, commands + data notify) ===== */
#include <ArduinoBLE.h>

// Must match your Python code
#define SVC_UUID     "19b10000-e8f2-537e-4f6c-d104768a1214"
#define CHR_CMD_UUID "19b10001-e8f2-537e-4f6c-d104768a1214"

// NEW data characteristic for streaming samples
#define CHR_DATA_UUID "19b10002-e8f2-537e-4f6c-d104768a1214"

BLEService svc(SVC_UUID);
BLEByteCharacteristic cmdChr(CHR_CMD_UUID, BLERead | BLEWrite);

// Compact binary sample (<=20 bytes)
typedef struct __attribute__((packed)) {
  float time_s;
  float theta1_deg;
  float theta2_deg;
  uint8_t flags; // 0=idle, 1=M1 active, 2=M2 active
} Sample;

BLECharacteristic dataChr(
  CHR_DATA_UUID,
  BLERead | BLENotify,
  sizeof(Sample),
  /*fixedLen=*/true
);

void initBLE(){
  if (!BLE.begin()){
    Serial.println("FATAL: BLE init failed");
    while(1);
  }
  BLE.setLocalName("ESP32");
  BLE.setAdvertisedService(svc);

  svc.addCharacteristic(cmdChr);
  svc.addCharacteristic(dataChr);   // add data characteristic
  BLE.addService(svc);

  cmdChr.writeValue((byte)'E');     // arbitrary initial value
  BLE.advertise();
  Serial.println("BLE advertising... (W->M1, E->M2, data notify on 19b1...1214:...02)");
}

void pollBLE(){
  BLE.poll(); // non-blocking

  if (cmdChr.written()){
    byte v;
    if (cmdChr.readValue(v)){
      char c = (char)v;
      if (c=='W' || c=='w'){ char t[]="w"; process_line(t); }
      else if (c=='E' || c=='e'){ char t[]="e"; process_line(t); }
      // (extend with 'X','Z','A' later if needed)
    }
  }
}

/* ===== Setup / loop ===== */
void setup(){
  Serial.begin(115200);
  // DON'T BLOCK waiting for Serial; short timeout is fine
  unsigned long _t0 = millis();
  while (!Serial && (millis() - _t0) < 1500) { delay(10); }

  pinMode(ENCA, INPUT_PULLUP); pinMode(ENCB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder1, RISING);
  pinMode(ENCA2, INPUT_PULLUP); pinMode(ENCB2, INPUT_PULLUP);
  enablePCINT_for_8_9();

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(PWM, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT); pinMode(PWM2, OUTPUT);
  motorStop(IN1, IN2, PWM); motorStop(IN3, IN4, PWM2);

#if defined(ARDUINO_ARCH_ESP32)
  analogWriteFrequency(2000);
  analogWriteResolution(8);
#endif

  t_prev = micros();
  angle_init(M1, CPR_GUESS, M1_ENC_INV? -1:+1, 0, 0.0f);
  angle_init(M2, CPR_GUESS, M2_ENC_INV? -1:+1, 0, 0.0f);
  long c1,c2; ATOMIC_READ({ c1=pos1_i; c2=pos2_i; });
  angle_set_zero_here(M1, c1, 0.0f); angle_set_zero_here(M2, c2, 0.0f);

  initBLE();

  Serial.println(F("theta1 theta2")); // Arduino Serial Plotter header (two traces)
}

void loop(){
  // Non-blocking BLE + Serial polling at top
  pollBLE();
  pollSerialLine();

  unsigned long tu=micros();
  float dt=(tu - t_prev)*1e-6f; t_prev=tu;
  if (dt<1e-6f) dt=1e-6f; if (dt>0.050f) dt=0.050f;

  long pos1,pos2; ATOMIC_READ({ pos1=pos1_i; pos2=pos2_i; });

  const long T_LOW_1  = deg_to_counts(M1, thetaLow);
  const long T_HIGH_1 = deg_to_counts(M1, thetaHigh);
  const long T_LOW_2  = deg_to_counts(M2, thetaLow);
  const long T_HIGH_2 = deg_to_counts(M2, thetaHigh);

  // ---- Motor 1 ----
  if (active==M1_ACTIVE){
    long goal = (m1_state==TO_HIGH||m1_state==HOLD_TOP) ? T_HIGH_1 : T_LOW_1;

    if ((m1_state==TO_HIGH||m1_state==TO_LOW) && labs(goal-pos1)<=tol_counts_for(M1)){
      setMotor1(0,0); last_pwm1 = 0;
      if (m1_state==TO_HIGH){ m1_high_meas=pos1; m1_have_high=true; hold_t=millis(); m1_state=HOLD_TOP; }
      else { m1_low_meas=pos1; m1_have_low=true; if (BOTTOM_HOLD_MS>0){ hold_t=millis(); m1_state=HOLD_BOTTOM; } else m1_state=DONE; }
    }
    if (m1_state==HOLD_TOP    && millis()-hold_t>=TOP_HOLD_MS){ m1_state=TO_LOW;  d1_filt=0.0f; pos1_prev=pos1; }
    if (m1_state==HOLD_BOTTOM && millis()-hold_t>=BOTTOM_HOLD_MS) m1_state=DONE;

    if (m1_state==DONE && m1_have_high && m1_have_low){
      calibrate_from_two_points(M1, m1_low_meas, m1_high_meas, thetaLow, thetaHigh);
      m1_have_high = m1_have_low = false;
    }

    if (m1_state==TO_HIGH||m1_state==TO_LOW){
      float e=(float)(goal - pos1);
      if (labs((long)e) < tol_counts_for(M1)) e = 0.0f;

      float P=kp*e;
      float v=(pos1 - pos1_prev)/dt; pos1_prev=pos1;
      if (v> VEL_CAP) v= VEL_CAP; if (v<-VEL_CAP) v=-VEL_CAP;
      float d_raw=-kd*v; d1_filt += D_ALPHA*(d_raw - d1_filt);

      float i_next=i1_acc + e*dt;
      float u=P + ki*i_next + d1_filt;

      if (u> UMAX){ if (P + d1_filt > 0) i_next=i1_acc; u= UMAX; }
      if (u<-UMAX){ if (P + d1_filt < 0) i_next=i1_acc; u=-UMAX; }
      if (i_next> I_MAX) i_next= I_MAX; if (i_next<-I_MAX) i_next=-I_MAX;

      i1_acc=i_next;

      int pwm_cmd = (int)fabsf(u);
      if (pwm_cmd > 255) pwm_cmd = 255;
      pwm_cmd = applySlew(pwm_cmd, last_pwm1);
      setMotor1((u>0)?+1:((u<0)?-1:0), pwm_cmd);
    } else { setMotor1(0,0); last_pwm1=0; if (m1_state==DONE){ active=NONE; m1_state=IDLE; } }
  } else { setMotor1(0,0); last_pwm1=0; }

  // ---- Motor 2 ----
  if (active==M2_ACTIVE){
    long goal = (m2_state==TO_HIGH||m2_state==HOLD_TOP) ? T_HIGH_2 : T_LOW_2;

    if ((m2_state==TO_HIGH||m2_state==TO_LOW) && labs(goal-pos2)<=tol_counts_for(M2)){
      setMotor2(0,0); last_pwm2 = 0;
      if (m2_state==TO_HIGH){ m2_high_meas=pos2; m2_have_high=true; hold_t=millis(); m2_state=HOLD_TOP; }
      else { m2_low_meas=pos2; m2_have_low=true; if (BOTTOM_HOLD_MS>0){ hold_t=millis(); m2_state=HOLD_BOTTOM; } else m2_state=DONE; }
    }
    if (m2_state==HOLD_TOP    && millis()-hold_t>=TOP_HOLD_MS){ m2_state=TO_LOW;  d2_filt=0.0f; pos2_prev=pos2; }
    if (m2_state==HOLD_BOTTOM && millis()-hold_t>=BOTTOM_HOLD_MS) m2_state=DONE;

    if (m2_state==DONE && m2_have_high && m2_have_low){
      calibrate_from_two_points(M2, m2_low_meas, m2_high_meas, thetaLow, thetaHigh);
      m2_have_high = m2_have_low = false;
    }

    if (m2_state==TO_HIGH||m2_state==TO_LOW){
      float e=(float)(goal - pos2);
      if (labs((long)e) < tol_counts_for(M2)) e = 0.0f;

      float P=kp*e;
      float v=(pos2 - pos2_prev)/dt; pos2_prev=pos2;
      if (v> VEL_CAP) v= VEL_CAP; if (v<-VEL_CAP) v=-VEL_CAP;
      float d_raw=-kd*v; d2_filt += D_ALPHA*(d_raw - d2_filt);

      float i_next=i2_acc + e*dt;
      float u=P + ki*i_next + d2_filt;

      if (u> UMAX){ if (P + d2_filt > 0) i_next=i2_acc; u= UMAX; }
      if (u<-UMAX){ if (P + d2_filt < 0) i_next=i2_acc; u=-UMAX; }
      if (i_next> I_MAX) i_next= I_MAX; if (i_next<-I_MAX) i_next=-I_MAX;

      i2_acc=i_next;

      int pwm_cmd = (int)fabsf(u);
      if (pwm_cmd > 255) pwm_cmd = 255;
      pwm_cmd = applySlew(pwm_cmd, last_pwm2);
      setMotor2((u>0)?+1:((u<0)?-1:0), pwm_cmd);
    } else { setMotor2(0,0); last_pwm2=0; if (m2_state==DONE){ active=NONE; m2_state=IDLE; } }
  } else { setMotor2(0,0); last_pwm2=0; }

  /* ---- Serial Plotter + BLE notify (θ1 θ2) ---- */
  unsigned long now_ms=millis();
  if (now_ms - plot_prev_ms >= (1000UL / PLOT_HZ)) {
    plot_prev_ms = now_ms;
    long p1=pos1, p2=pos2;
    float theta1 = counts_to_deg(M1, p1);
    float theta2 = counts_to_deg(M2, p2);

    // Keep Serial Plotter (optional)
    Serial.print(theta1); Serial.print(' '); Serial.println(theta2);

    // BLE notify (13 bytes)
    Sample s;
    s.time_s     = now_ms * 1e-3f;
    s.theta1_deg = theta1;
    s.theta2_deg = theta2;
    s.flags      = (active==M1_ACTIVE)?1:((active==M2_ACTIVE)?2:0);
    dataChr.writeValue((const uint8_t*)&s, sizeof(s));
  }
}
