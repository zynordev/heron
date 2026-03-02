#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_VL53L0X.h"

// =====================
// MOTOR PINLERI (senin)
// =====================
#define R_IN1 5
#define R_IN2 3
#define L_IN1 9
#define L_IN2 6

// =====================
// GOAL (beyaz zemin) - TCRT5000
// =====================
#define GOAL_SENSOR A7
int GOAL_WHITE_TH = 100;     // 0-1023: kalibre et
bool WHITE_HIGH = false;     // true: beyaz büyükse, false: beyaz küçükse (sen "ters" demiştin -> false)
int GOAL_CONFIRM = 8;        // kaç ölçüm üst üste beyazsa "bitir"

// =====================
// VL53L0X XSHUT PINLERI
// =====================
#define XSHUT_LEFT   2   // sol çapraz
#define XSHUT_FRONT  4   // ön
#define XSHUT_RIGHT  7   // sağ çapraz

// Yeni I2C adresleri
#define ADDR_LEFT   0x30
#define ADDR_FRONT  0x31
#define ADDR_RIGHT  0x32

Adafruit_VL53L0X tofL, tofF, tofR;
VL53L0X_RangingMeasurementData_t mL, mF, mR;

// ToF eşikleri (mm)
int TH_FRONT = 130;
int TH_DIAG  = 160;

// =====================
// GRID / YON
// =====================
static const uint8_t N = 8; // 8x8
// 0=N,1=E,2=S,3=W
static const int8_t DX[4] = {0, 1, 0, -1};
static const int8_t DY[4] = {-1, 0, 1, 0};

// Start: en alt 4 hücre
static const uint8_t BOTTOM4_X[4] = {2,3,4,5};
static const uint8_t BOTTOM_Y = 7;

// =====================
// Known Map (edge states)
// =====================
enum EdgeState : uint8_t { UNKNOWN=0, OPEN=1, WALL=2 };
static uint8_t known[N][N][4];

// BFS dist
static const int16_t INF = 32767;
static int16_t distMap[N][N];

// Robot state (grid)
static uint8_t rx = 2, ry = 7;  // start seçilecek
static uint8_t heading = 0;     // 0=N
static bool finished = false;

// =====================
// HAREKET AYARLARI
// =====================
int SPEED_PERCENT = 35;
uint16_t TURN_MS_90  = 260;   // 90 derece (robotuna göre ayarla)
uint16_t FWD_MS_CELL = 520;   // 1 hücre ileri (robotuna göre ayarla)

// =====================
// UTILS
// =====================
static inline bool inb(int x,int y){ return x>=0 && x<N && y>=0 && y<N; }

static uint8_t pctToPwm(int pct){
  if(pct<0) pct=0; if(pct>100) pct=100;
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

static void motorsForward(uint8_t pwm){
  setMotor(R_IN1,R_IN2,MFWD,pwm);
  setMotor(L_IN1,L_IN2,MFWD,pwm);
}

static void motorsTurnRight(uint8_t pwm){
  // sağ dön: sağ geri, sol ileri
  setMotor(R_IN1,R_IN2,MREV,pwm);
  setMotor(L_IN1,L_IN2,MFWD,pwm);
}

static void motorsTurnLeft(uint8_t pwm){
  // sol dön: sağ ileri, sol geri
  setMotor(R_IN1,R_IN2,MFWD,pwm);
  setMotor(L_IN1,L_IN2,MREV,pwm);
}

// =====================
// Known map set/get (iki yönlü)
// =====================
static void setEdge(uint8_t x,uint8_t y,uint8_t d,uint8_t st){
  known[y][x][d]=st;
  int nx = (int)x + DX[d];
  int ny = (int)y + DY[d];
  if(inb(nx,ny)){
    known[ny][nx][(d+2)&3] = st;
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
  // outer walls
  for(uint8_t x=0;x<N;x++){
    setEdge(x,0,0,WALL);
    setEdge(x,N-1,2,WALL);
  }
  for(uint8_t y=0;y<N;y++){
    setEdge(0,y,3,WALL);
    setEdge(N-1,y,1,WALL);
  }
}

// =====================
// GOAL SENSOR
// =====================
static bool isWhiteNow(){
  int v = analogRead(GOAL_SENSOR); // 0-1023
  bool white = WHITE_HIGH ? (v > GOAL_WHITE_TH) : (v < GOAL_WHITE_TH);
  return white;
}

static void checkGoalAndMaybeFinish(){
  static int whiteStreak = 0;
  if(isWhiteNow()) whiteStreak++;
  else whiteStreak = 0;

  if(whiteStreak >= GOAL_CONFIRM){
    finished = true;
    motorsStop();
    Serial.println("✅ HEDEF (beyaz zemin) bulundu. DUR.");
  }
}

// =====================
// VL53L0X init + read
// =====================
static void allXshutLow(){
  digitalWrite(XSHUT_LEFT,LOW);
  digitalWrite(XSHUT_FRONT,LOW);
  digitalWrite(XSHUT_RIGHT,LOW);
  delay(10);
}

static bool initOne(Adafruit_VL53L0X &dev,int xshutPin,uint8_t newAddr){
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

// Sensör -> duvar güncelle
static void updateKnownFromSensors(uint8_t x,uint8_t y,uint8_t heading){
  int dL = readMM(tofL,mL);
  int dF = readMM(tofF,mF);
  int dR = readMM(tofR,mR);

  bool wallLeft  = (dL > 0 && dL < TH_DIAG);
  bool wallFront = (dF > 0 && dF < TH_FRONT);
  bool wallRight = (dR > 0 && dR < TH_DIAG);

  setEdge(x,y, heading,       wallFront ? WALL : OPEN);
  setEdge(x,y,(heading+3)&3,  wallLeft  ? WALL : OPEN);
  setEdge(x,y,(heading+1)&3,  wallRight ? WALL : OPEN);

  // debug kısa
  Serial.print("TOF L/F/R: ");
  Serial.print(dL); Serial.print("/");
  Serial.print(dF); Serial.print("/");
  Serial.print(dR);
  Serial.print(" | walls ");
  Serial.print(wallLeft); Serial.print(",");
  Serial.print(wallFront); Serial.print(",");
  Serial.println(wallRight);
}

// =====================
// BFS recompute to target
// UNKNOWN geçilebilir, WALL engel
// =====================
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
        distMap[ny][nx] = cd + 1;
        qx[qt]=(uint8_t)nx; qy[qt]=(uint8_t)ny; qt++;
      }
    }
  }
}

// =====================
// Frontier (keşif hedefi):
// "en az bir UNKNOWN kenarı olan" hücre
// =====================
static bool isFrontierCell(uint8_t x,uint8_t y){
  for(uint8_t d=0; d<4; d++){
    if(getEdge(x,y,d)==UNKNOWN) return true;
  }
  return false;
}

static bool pickNearestFrontier(uint8_t &tx,uint8_t &ty){
  // Önce current'tan BFS (mesafe) çıkaralım
  // Bunun için "current hedef" gibi kullanacağız: distFromStart
  // Basitçe: her hücreyi gezip rx,ry'den BFS yapmak yerine
  // frontier'i deneye deneye BFS yapmak pahalı. 8x8 küçük olduğu için:
  // Her frontier adayına bfsTo(aday) yapıp distMap[ry][rx]'e bakacağız.
  int16_t best = INF;
  bool found = false;

  for(uint8_t y=0;y<N;y++){
    for(uint8_t x=0;x<N;x++){
      if(!isFrontierCell(x,y)) continue;
      bfsTo(x,y);
      int16_t d = distMap[ry][rx];
      if(d < best){
        best = d;
        tx = x; ty = y;
        found = true;
      }
    }
  }
  return found;
}

// Tie-break: düz > sağ/sol > geri
static uint8_t rankDir(uint8_t head, uint8_t d){
  if(d==head) return 0;
  if(d==((head+1)&3) || d==((head+3)&3)) return 1;
  return 2;
}

// distMap target'e göre hesaplandıktan sonra bir sonraki yön seç
static bool chooseNextDir(uint8_t x,uint8_t y,uint8_t head, uint8_t &outDir){
  int16_t cur = distMap[y][x];
  int16_t best = INF;
  uint8_t cands[4];
  uint8_t cn=0;

  for(uint8_t d=0; d<4; d++){
    if(getEdge(x,y,d)==WALL) continue;
    int nx=(int)x+DX[d], ny=(int)y+DY[d];
    if(!inb(nx,ny)) continue;
    int16_t v = distMap[ny][nx];
    if(v < best){
      best = v;
      cn = 0;
      cands[cn++] = d;
    }else if(v == best){
      cands[cn++] = d;
    }
  }
  if(cn==0 || cur>=INF) return false;

  // tie-break
  uint8_t bestD = cands[0];
  uint8_t bestR = rankDir(head, bestD);
  for(uint8_t i=1;i<cn;i++){
    uint8_t d = cands[i];
    uint8_t r = rankDir(head, d);
    if(r < bestR){
      bestR = r;
      bestD = d;
    }
  }
  outDir = bestD;
  return true;
}

// =====================
// HAREKET: dön + 1 hücre ileri
// =====================
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
  heading = desired;
}

static bool forwardOneCell(){
  // Önce bilinen haritada duvar var mı bak (UNKNOWN/OPEN ise dene)
  if(getEdge(rx,ry,heading)==WALL) return false;

  uint8_t pwm = pctToPwm(SPEED_PERCENT);
  motorsForward(pwm);
  delay(FWD_MS_CELL);
  motorsStop();
  delay(60);

  int nx=(int)rx+DX[heading], ny=(int)ry+DY[heading];
  if(!inb(nx,ny)) return false;
  rx=(uint8_t)nx; ry=(uint8_t)ny;
  return true;
}

// =====================
// SERIAL mini ayar (opsiyonel)
//  speed35, th700, inv1, front130, diag160, turn260, cell520
// =====================
static void handleSerial(){
  if(!Serial.available()) return;
  String s = Serial.readStringUntil('\n');
  s.trim(); s.toLowerCase();
  if(s.startsWith("speed")) { SPEED_PERCENT = s.substring(5).toInt(); Serial.println("OK speed"); }
  else if(s.startsWith("th")) { GOAL_WHITE_TH = s.substring(2).toInt(); Serial.println("OK th"); }
  else if(s.startsWith("inv")) { WHITE_HIGH = (s.substring(3).toInt()!=0); Serial.println("OK inv"); }
  else if(s.startsWith("front")) { TH_FRONT = s.substring(5).toInt(); Serial.println("OK front"); }
  else if(s.startsWith("diag")) { TH_DIAG = s.substring(4).toInt(); Serial.println("OK diag"); }
  else if(s.startsWith("turn")) { TURN_MS_90 = s.substring(4).toInt(); Serial.println("OK turn"); }
  else if(s.startsWith("cell")) { FWD_MS_CELL = s.substring(4).toInt(); Serial.println("OK cell"); }
}

// =====================
// SETUP
// =====================
static void pickStart(){
  randomSeed(analogRead(A0) ^ micros());
  uint8_t idx = (uint8_t)(random(0,4));
  rx = BOTTOM4_X[idx];
  ry = BOTTOM_Y;
  heading = 0;
}

void setup(){
  Serial.begin(115200);
  Wire.begin();

  pinMode(R_IN1, OUTPUT);
  pinMode(R_IN2, OUTPUT);
  pinMode(L_IN1, OUTPUT);
  pinMode(L_IN2, OUTPUT);
  motorsStop();

  pinMode(GOAL_SENSOR, INPUT);

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

  Serial.println("=== Online Recomputed Flood Fill (BFS) + TOF + White Goal(A7) ===");
  Serial.print("Start: "); Serial.print(rx); Serial.print(","); Serial.println(ry);
  Serial.println("Komutlar: speed35 th700 inv1 front130 diag160 turn260 cell520");
}

// =====================
// LOOP: keşif + BFS + hareket
// =====================
void loop(){
  handleSerial();

  if(finished){
    motorsStop();
    delay(50);
    return;
  }

  // 1) hedef (beyaz zemin) kontrolü
  checkGoalAndMaybeFinish();
  if(finished) return;

  // 2) sensörle bulunduğun hücrede duvarları öğren
  updateKnownFromSensors(rx, ry, heading);

  // 3) hedef bilinmiyor => en yakın frontier seç
  uint8_t tx=0, ty=0;
  bool ok = pickNearestFrontier(tx,ty);
  if(!ok){
    Serial.println("Frontier kalmadi (her yer biliniyor). Dur.");
    motorsStop();
    delay(200);
    return;
  }

  // 4) seçilen frontier'e BFS
  bfsTo(tx,ty);

  // 5) bir sonraki yönü seç
  uint8_t nd;
  if(!chooseNextDir(rx,ry,heading,nd)){
    Serial.println("Yol yok gibi. Dur.");
    motorsStop();
    delay(200);
    return;
  }

  // 6) dön, tekrar sensör, sonra 1 hücre ilerle
  turnTo(nd);

  // dönünce ön duvarı güncelle (kritik)
  updateKnownFromSensors(rx, ry, heading);

  // eğer duvar çıktıysa map'e yazıldı zaten; ilerleme
  if(getEdge(rx,ry,heading)==WALL){
    Serial.println("On duvar varmis. Ilerlemedim.");
    delay(80);
    return;
  }

  bool moved = forwardOneCell();
  if(!moved){
    // güvenlik: duvar/limit
    setEdge(rx,ry,heading,WALL);
    Serial.println("Ilerleyemedim -> WALL yazdim.");
    delay(80);
    return;
  }

  // yeni hücrede öğren + hedef kontrol
  updateKnownFromSensors(rx, ry, heading);
  checkGoalAndMaybeFinish();

  Serial.print("Pos: "); Serial.print(rx); Serial.print(","); Serial.print(ry);
  Serial.print(" head="); Serial.println(heading);

  delay(40);
}
