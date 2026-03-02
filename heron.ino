#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_VL53L0X.h"

// =====================================================
// MOTOR PINLERI (senin verdigin)
// =====================================================
#define R_IN1 5
#define R_IN2 3
#define L_IN1 9
#define L_IN2 6

// =====================================================
// GOAL SENSOR (TCRT5000) - hedef beyaz zemin
// =====================================================
#define GOAL_SENSOR A7
int GOAL_WHITE_TH = 700;     // 0-1023: kalibre et
bool WHITE_HIGH = false;     // true: beyaz > th, false: beyaz < th (sende ters demistin -> false)
int GOAL_CONFIRM = 8;        // kac okumada ust uste beyazsa "finished"

// =====================================================
// 3x VL53L0X (ToF) - XSHUT pinleri
// =====================================================
#define XSHUT_LEFT   2   // sol capraz
#define XSHUT_FRONT  4   // on
#define XSHUT_RIGHT  7   // sag capraz

#define ADDR_LEFT   0x30
#define ADDR_FRONT  0x31
#define ADDR_RIGHT  0x32

Adafruit_VL53L0X tofL, tofF, tofR;
VL53L0X_RangingMeasurementData_t mL, mF, mR;

int TH_FRONT = 130; // mm
int TH_DIAG  = 160; // mm

// =====================================================
// ENCODER PINLERI (PortB PCINT0) - senin son durumuna gore
// =====================================================
// Sag encoder: D12 (A), D11 (B)
// Sol encoder: D8  (A), D10 (B)
const uint8_t R_A = 12; // PB4
const uint8_t R_B = 11; // PB3
const uint8_t L_A = 8;  // PB0
const uint8_t L_B = 10; // PB2

volatile long rightCount = 0;
volatile long leftCount  = 0;
volatile uint8_t rPrev = 0;
volatile uint8_t lPrev = 0;

const int8_t qdec[16] = {
  0, +1, -1,  0,
 -1,  0,  0, +1,
 +1,  0,  0, -1,
  0, -1, +1,  0
};

ISR(PCINT0_vect) {
  uint8_t b = PINB;
  uint8_t rA = (b >> PB4) & 1; // D12
  uint8_t rB = (b >> PB3) & 1; // D11
  uint8_t lA = (b >> PB0) & 1; // D8
  uint8_t lB = (b >> PB2) & 1; // D10

  uint8_t rCurr = (rA << 1) | rB;
  uint8_t lCurr = (lA << 1) | lB;

  rightCount += qdec[(rPrev << 2) | rCurr];
  leftCount  += qdec[(lPrev << 2) | lCurr];

  rPrev = rCurr;
  lPrev = lCurr;
}

void setupPCINT_PortB() {
  cli();
  PCICR |= (1 << PCIE0);
  PCMSK0 |= (1 << PCINT0); // D8  (PB0)
  PCMSK0 |= (1 << PCINT2); // D10 (PB2)
  PCMSK0 |= (1 << PCINT3); // D11 (PB3)
  PCMSK0 |= (1 << PCINT4); // D12 (PB4)
  sei();
}

// =====================================================
// GRID / YON
// =====================================================
static const uint8_t N = 8;
static const int8_t DX[4] = {0, 1, 0, -1};
static const int8_t DY[4] = {-1, 0, 1, 0};
// 0=N,1=E,2=S,3=W

// Start: en alt 4 hucre
static const uint8_t BOTTOM4_X[4] = {2,3,4,5};
static const uint8_t BOTTOM_Y = 7;

// =====================================================
// KNOWN MAP + BFS
// =====================================================
enum EdgeState : uint8_t { UNKNOWN=0, OPEN=1, WALL=2 };
static uint8_t known[N][N][4];

static const int16_t INF = 32767;
static int16_t distMap[N][N];

// Robot state
static uint8_t rx=2, ry=7, heading=0;
static bool finished=false;

// =====================================================
// HAREKET / KONTROL AYARLARI
// =====================================================
int SPEED_PERCENT = 35;     // base hız %

long TICKS_PER_CELL = 200; // 1 hucre icin encoder tick (kalibre et!)
float Kp_enc = 1.2f;       // yamuk duzeltme
float Ki_enc = 0.08f;
int corrMax_enc = 35;

// Donus enkoderle yok; sureyle (istersen sonra enkoderle donus da yapariz)
uint16_t TURN_MS_90 = 260;

// =====================================================
// UTILS
// =====================================================
static inline bool inb(int x,int y){ return x>=0 && x<(int)N && y>=0 && y<(int)N; }

static uint8_t pctToPwm(int pct){
  if(pct<0) pct=0;
  if(pct>100) pct=100;
  return (uint8_t)((pct * 255L)/100L);
}

enum DirMove : uint8_t { MSTOP=0, MFWD=1, MREV=2 };

static void setMotor(uint8_t in1, uint8_t in2, DirMove dm, uint8_t pwm){
  switch(dm){
    case MSTOP: analogWrite(in1,0);   analogWrite(in2,0);   break;
    case MFWD:  analogWrite(in1,pwm); analogWrite(in2,0);   break;
    case MREV:  analogWrite(in1,0);   analogWrite(in2,pwm); break;
  }
}

static void motorsStop(){
  setMotor(R_IN1,R_IN2,MSTOP,0);
  setMotor(L_IN1,L_IN2,MSTOP,0);
}

static void motorsTurnRight(uint8_t pwm){
  setMotor(R_IN1,R_IN2,MREV,pwm);
  setMotor(L_IN1,L_IN2,MFWD,pwm);
}

static void motorsTurnLeft(uint8_t pwm){
  setMotor(R_IN1,R_IN2,MFWD,pwm);
  setMotor(L_IN1,L_IN2,MREV,pwm);
}

// =====================================================
// KNOWN MAP set/get (iki yonlu)
// =====================================================
static void setEdge(uint8_t x,uint8_t y,uint8_t d,uint8_t st){
  known[y][x][d]=st;
  int nx = (int)x + DX[d];
  int ny = (int)y + DY[d];
  if(inb(nx,ny)){
    known[ny][nx][(d+2)&3]=st;
  }
}

static uint8_t getEdge(uint8_t x,uint8_t y,uint8_t d){
  return known[y][x][d];
}

static void initKnown(){
  for(uint8_t y=0;y<N;y++){
    for(uint8_t x=0;x<N;x++){
      for(uint8_t d=0;d<4;d++) known[y][x][d]=UNKNOWN;
    }
  }
  for(uint8_t x=0;x<N;x++){
    setEdge(x,0,0,WALL);
    setEdge(x,N-1,2,WALL);
  }
  for(uint8_t y=0;y<N;y++){
    setEdge(0,y,3,WALL);
    setEdge(N-1,y,1,WALL);
  }
}

// =====================================================
// GOAL SENSOR
// =====================================================
static bool isWhiteNow(){
  int v = analogRead(GOAL_SENSOR); // 0-1023
  return WHITE_HIGH ? (v > GOAL_WHITE_TH) : (v < GOAL_WHITE_TH);
}

static void checkGoalAndMaybeFinish(){
  static int streak=0;
  if(isWhiteNow()) streak++;
  else streak=0;

  if(streak >= GOAL_CONFIRM){
    finished=true;
    motorsStop();
    Serial.println("✅ GOAL: beyaz zemin bulundu. DUR.");
  }
}

// =====================================================
// VL53L0X init + read
// =====================================================
static void allXshutLow(){
  digitalWrite(XSHUT_LEFT,LOW);
  digitalWrite(XSHUT_FRONT,LOW);
  digitalWrite(XSHUT_RIGHT,LOW);
  delay(10);
}

static bool initOne(Adafruit_VL53L0X &dev, int xshutPin, uint8_t newAddr){
  digitalWrite(xshutPin,HIGH);
  delay(10);
  if(!dev.begin(0x29,false,&Wire)) return false;
  dev.setAddress(newAddr);
  delay(5);
  return true;
}

static int readMM(Adafruit_VL53L0X &dev, VL53L0X_RangingMeasurementData_t &m){
  dev.rangingTest(&m,false);
  if(m.RangeStatus != 0) return -1;
  return (int)m.RangeMilliMeter;
}

static void updateKnownFromSensors(uint8_t x,uint8_t y,uint8_t head){
  int dL = readMM(tofL,mL);
  int dF = readMM(tofF,mF);
  int dR = readMM(tofR,mR);

  bool wallLeft  = (dL > 0 && dL < TH_DIAG);
  bool wallFront = (dF > 0 && dF < TH_FRONT);
  bool wallRight = (dR > 0 && dR < TH_DIAG);

  setEdge(x,y, head,       wallFront ? WALL : OPEN);
  setEdge(x,y,(head+3)&3,  wallLeft  ? WALL : OPEN);
  setEdge(x,y,(head+1)&3,  wallRight ? WALL : OPEN);

  Serial.print("TOF L/F/R: ");
  Serial.print(dL); Serial.print("/");
  Serial.print(dF); Serial.print("/");
  Serial.print(dR);
  Serial.print(" | walls ");
  Serial.print(wallLeft); Serial.print(",");
  Serial.print(wallFront); Serial.print(",");
  Serial.println(wallRight);
}

// =====================================================
// BFS recompute (UNKNOWN geçilebilir, WALL engel)
// =====================================================
static void bfsTo(uint8_t gx,uint8_t gy){
  for(uint8_t y=0;y<N;y++) for(uint8_t x=0;x<N;x++) distMap[y][x]=INF;

  uint8_t qx[64], qy[64];
  uint8_t qh=0, qt=0;

  distMap[gy][gx]=0;
  qx[qt]=gx; qy[qt]=gy; qt++;

  while(qh!=qt){
    uint8_t x=qx[qh], y=qy[qh]; qh++;
    int16_t cd = distMap[y][x];

    for(uint8_t d=0; d<4; d++){
      if(getEdge(x,y,d)==WALL) continue;
      int nx=(int)x+DX[d], ny=(int)y+DY[d];
      if(!inb(nx,ny)) continue;
      if(distMap[ny][nx] > cd + 1){
        distMap[ny][nx]=cd+1;
        qx[qt]=(uint8_t)nx; qy[qt]=(uint8_t)ny; qt++;
      }
    }
  }
}

// Frontier: en az bir UNKNOWN kenarı olan hücre
static bool isFrontierCell(uint8_t x,uint8_t y){
  for(uint8_t d=0; d<4; d++){
    if(getEdge(x,y,d)==UNKNOWN) return true;
  }
  return false;
}

// En yakın frontier'i seç (küçük grid -> brute force yeterli)
static bool pickNearestFrontier(uint8_t &tx,uint8_t &ty){
  int16_t best = INF;
  bool found=false;
  for(uint8_t y=0;y<N;y++){
    for(uint8_t x=0;x<N;x++){
      if(!isFrontierCell(x,y)) continue;
      bfsTo(x,y);
      int16_t d = distMap[ry][rx];
      if(d < best){
        best=d; tx=x; ty=y; found=true;
      }
    }
  }
  return found;
}

static uint8_t rankDir(uint8_t head,uint8_t d){
  if(d==head) return 0;
  if(d==((head+1)&3) || d==((head+3)&3)) return 1;
  return 2;
}

static bool chooseNextDir(uint8_t x,uint8_t y,uint8_t head, uint8_t &outDir){
  int16_t cur = distMap[y][x];
  if(cur>=INF) return false;

  int16_t best=INF;
  uint8_t cands[4]; uint8_t cn=0;

  for(uint8_t d=0; d<4; d++){
    if(getEdge(x,y,d)==WALL) continue;
    int nx=(int)x+DX[d], ny=(int)y+DY[d];
    if(!inb(nx,ny)) continue;
    int16_t v = distMap[ny][nx];
    if(v < best){
      best=v; cn=0; cands[cn++]=d;
    }else if(v==best){
      cands[cn++]=d;
    }
  }
  if(cn==0) return false;

  uint8_t bestD=cands[0], bestR=rankDir(head,bestD);
  for(uint8_t i=1;i<cn;i++){
    uint8_t d=cands[i];
    uint8_t r=rankDir(head,d);
    if(r<bestR){ bestR=r; bestD=d; }
  }
  outDir=bestD;
  return true;
}

// =====================================================
// DONUS (sure ile)
// =====================================================
static void turnTo(uint8_t desired){
  uint8_t pwm = pctToPwm(SPEED_PERCENT);
  uint8_t diff = (desired + 4 - heading) & 3;

  if(diff==0) return;

  if(diff==1){
    motorsTurnRight(pwm);
    delay(TURN_MS_90);
  }else if(diff==3){
    motorsTurnLeft(pwm);
    delay(TURN_MS_90);
  }else{ // 180
    motorsTurnRight(pwm);
    delay((uint16_t)(2*TURN_MS_90));
  }
  motorsStop();
  delay(60);
  heading=desired;
}

// =====================================================
// 1 HUCRE ILERI (ENKODER + DUZ GITME PI)
// =====================================================
static bool forwardOneCell_Encoder(){
  if(getEdge(rx,ry,heading)==WALL) return false;

  long startR, startL;
  noInterrupts();
  startR = rightCount;
  startL = leftCount;
  interrupts();

  float I = 0.0f;
  float corrFilt = 0.0f;
  uint8_t basePWM = pctToPwm(SPEED_PERCENT);

  while(true){
    checkGoalAndMaybeFinish();
    if(finished){ motorsStop(); return true; }

    long r, l;
    noInterrupts();
    r = rightCount;
    l = leftCount;
    interrupts();

    long dR = r - startR;
    long dL = l - startL;
    long avg = (dR + dL) / 2;
    if(avg >= TICKS_PER_CELL) break;

    float error = (float)(dL - dR); // sol hizliysa +

    // deadband (cok minik farklar titresim yapmasin)
    if (abs((int)error) <= 1) error = 0;

    I += error;
    if(I > 80) I = 80;
    if(I < -80) I = -80;

    float corr = Kp_enc * error + Ki_enc * I;
    if(corr > corrMax_enc) corr = corrMax_enc;
    if(corr < -corrMax_enc) corr = -corrMax_enc;

    corrFilt = 0.85f * corrFilt + 0.15f * corr;
    int correction = (int)corrFilt;

    int pwmR = (int)basePWM + correction;
    int pwmL = (int)basePWM - correction;

    if(pwmR < 0) pwmR = 0; if(pwmR > 255) pwmR = 255;
    if(pwmL < 0) pwmL = 0; if(pwmL > 255) pwmL = 255;

    setMotor(R_IN1,R_IN2,MFWD,(uint8_t)pwmR);
    setMotor(L_IN1,L_IN2,MFWD,(uint8_t)pwmL);

    delay(15);
  }

  motorsStop();
  delay(60);

  int nx=(int)rx+DX[heading], ny=(int)ry+DY[heading];
  if(!inb(nx,ny)) return false;
  rx=(uint8_t)nx; ry=(uint8_t)ny;
  return true;
}

// =====================================================
// SERIAL AYAR (opsiyonel, kolay tuning)
//   speed35
//   cell200
//   kp12   (1.2 için 12 yaz)
//   ki08   (0.08 için 08 yaz)
//   th700
//   inv1
//   front130
//   diag160
//   turn260
// =====================================================
static void handleSerial(){
  if(!Serial.available()) return;
  String s = Serial.readStringUntil('\n');
  s.trim(); s.toLowerCase();

  if(s.startsWith("speed")){
    SPEED_PERCENT = s.substring(5).toInt();
    Serial.println("OK speed");
  }else if(s.startsWith("cell")){
    TICKS_PER_CELL = s.substring(4).toInt();
    Serial.println("OK cell");
  }else if(s.startsWith("kp")){
    int v = s.substring(2).toInt();
    Kp_enc = v / 10.0f; // kp12 => 1.2
    Serial.println("OK kp");
  }else if(s.startsWith("ki")){
    int v = s.substring(2).toInt();
    Ki_enc = v / 100.0f; // ki08 => 0.08
    Serial.println("OK ki");
  }else if(s.startsWith("th")){
    GOAL_WHITE_TH = s.substring(2).toInt();
    Serial.println("OK th");
  }else if(s.startsWith("inv")){
    WHITE_HIGH = (s.substring(3).toInt()!=0);
    Serial.println("OK inv");
  }else if(s.startsWith("front")){
    TH_FRONT = s.substring(5).toInt();
    Serial.println("OK front");
  }else if(s.startsWith("diag")){
    TH_DIAG = s.substring(4).toInt();
    Serial.println("OK diag");
  }else if(s.startsWith("turn")){
    TURN_MS_90 = s.substring(4).toInt();
    Serial.println("OK turn");
  }else{
    Serial.println("Cmd? speed35 cell200 kp12 ki08 th700 inv1 front130 diag160 turn260");
  }
}

// =====================================================
// START SEC
// =====================================================
static void pickStart(){
  randomSeed(analogRead(A0) ^ micros());
  uint8_t idx = (uint8_t)random(0,4);
  rx = BOTTOM4_X[idx];
  ry = BOTTOM_Y;
  heading = 0;
}

// =====================================================
// SETUP
// =====================================================
void setup(){
  Serial.begin(115200);
  Wire.begin();

  pinMode(R_IN1, OUTPUT);
  pinMode(R_IN2, OUTPUT);
  pinMode(L_IN1, OUTPUT);
  pinMode(L_IN2, OUTPUT);
  motorsStop();

  pinMode(GOAL_SENSOR, INPUT);

  // encoder input pullup
  pinMode(R_A, INPUT_PULLUP);
  pinMode(R_B, INPUT_PULLUP);
  pinMode(L_A, INPUT_PULLUP);
  pinMode(L_B, INPUT_PULLUP);

  // init prev states
  uint8_t b = PINB;
  rPrev = (((b >> PB4) & 1) << 1) | ((b >> PB3) & 1);
  lPrev = (((b >> PB0) & 1) << 1) | ((b >> PB2) & 1);

  setupPCINT_PortB();

  // ToF XSHUT
  pinMode(XSHUT_LEFT, OUTPUT);
  pinMode(XSHUT_FRONT, OUTPUT);
  pinMode(XSHUT_RIGHT, OUTPUT);

  allXshutLow();

  if(!initOne(tofL, XSHUT_LEFT, ADDR_LEFT)){
    Serial.println("TOF LEFT init FAIL"); while(1){}
  }
  if(!initOne(tofF, XSHUT_FRONT, ADDR_FRONT)){
    Serial.println("TOF FRONT init FAIL"); while(1){}
  }
  if(!initOne(tofR, XSHUT_RIGHT, ADDR_RIGHT)){
    Serial.println("TOF RIGHT init FAIL"); while(1){}
  }

  initKnown();
  pickStart();

  Serial.println("=== Maze Robot | Online Recomputed Flood Fill (BFS) ===");
  Serial.print("Start: "); Serial.print(rx); Serial.print(","); Serial.println(ry);
  Serial.println("Cmd: speed35 cell200 kp12 ki08 th700 inv1 front130 diag160 turn260");
}

// =====================================================
// LOOP
// =====================================================
void loop(){
  handleSerial();

  if(finished){
    motorsStop();
    delay(50);
    return;
  }

  // 0) goal kontrol
  checkGoalAndMaybeFinish();
  if(finished) return;

  // 1) bulundugun hucrede duvarlari ogren
  updateKnownFromSensors(rx, ry, heading);

  // 2) hedef bilinmiyor -> en yakin frontier sec
  uint8_t tx=0, ty=0;
  if(!pickNearestFrontier(tx,ty)){
    Serial.println("Frontier yok -> harita bitti. Dur.");
    motorsStop();
    delay(200);
    return;
  }

  // 3) o frontier'e BFS
  bfsTo(tx,ty);

  // 4) bir sonraki yon
  uint8_t nd;
  if(!chooseNextDir(rx,ry,heading,nd)){
    Serial.println("Secilecek yon yok. Dur.");
    motorsStop();
    delay(200);
    return;
  }

  // 5) don
  turnTo(nd);

  // 6) donunce on duvari tekrar ogren (cok kritik)
  updateKnownFromSensors(rx, ry, heading);

  // 7) duvar varsa ilerleme
  if(getEdge(rx,ry,heading)==WALL){
    Serial.println("On duvar varmis. Ilerlemedim.");
    delay(80);
    return;
  }

  // 8) 1 hucre ileri (enkoder + yamuk duzeltme)
  bool moved = forwardOneCell_Encoder();
  if(!moved){
    setEdge(rx,ry,heading,WALL);
    Serial.println("Ilerleyemedim -> WALL yazdim.");
    delay(80);
    return;
  }

  // 9) yeni hucrede ogren + goal kontrol
  updateKnownFromSensors(rx, ry, heading);
  checkGoalAndMaybeFinish();

  Serial.print("Pos: "); Serial.print(rx); Serial.print(","); Serial.print(ry);
  Serial.print(" head="); Serial.print(heading);
  Serial.print(" ticksR/L="); Serial.print(rightCount); Serial.print("/"); Serial.println(leftCount);

  delay(40);
}
