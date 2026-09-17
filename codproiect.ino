#define BLYNK_TEMPLATE_ID "-"
#define BLYNK_TEMPLATE_NAME "Sistem inteligent"
#define BLYNK_AUTH_TOKEN "-"
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <EEPROM.h>

char ssid[] = "-";
char pass[] = "p-";

#define ZMPT_PIN 34
#define ZMPT_INV 32
#define PIN_CURENT 35
#define PIN_26 26    // K2 — rupe sarcina (HIGH = rupt)
#define PIN_27 27    // K1 — alege sursa (HIGH = invertor, LOW = retea)

#define MOD_RETEA    0
#define MOD_INVERTOR 1
#define MOD_MANUAL   2
int modOperare = MOD_RETEA;
bool manualInvertor = false;

#define PRAG_RETEA_JOS  100.0
#define PRAG_RETEA_SUS  110.0
#define SOC_MIN         20.0
#define SOC_MAX         80.0
#define CAPACITATE_WH   60.0
#define CAPACITATE_AH   5.05

#define EEPROM_SIZE     16
#define ADDR_COMUTARI   0
#define ADDR_T_RETEA    4
#define ADDR_T_INV      8

Adafruit_INA219 ina219;
BlynkTimer timer;

bool peInvertor = false;
float curentFiltrat = 0;
float socCoulomb = -1.0;

unsigned long nrComutari = 0;
unsigned long timpRetea = 0;
unsigned long timpInvertor = 0;
unsigned long ultimulMoment = 0;

bool alertaReteaCazuta = false;
bool alertaInvertorCazut = false;
bool alertaBaterieCritica = false;

void salveaza() {
  EEPROM.put(ADDR_COMUTARI, nrComutari);
  EEPROM.put(ADDR_T_RETEA, timpRetea);
  EEPROM.put(ADDR_T_INV, timpInvertor);
  EEPROM.commit();
}
void citesteEEPROM() {
  EEPROM.get(ADDR_COMUTARI, nrComutari);
  EEPROM.get(ADDR_T_RETEA, timpRetea);
  EEPROM.get(ADDR_T_INV, timpInvertor);
  if (nrComutari > 1000000) nrComutari = 0;
  if (timpRetea > 31536000) timpRetea = 0;
  if (timpInvertor > 31536000) timpInvertor = 0;
}

// ─── Curent AC (ZMCT103C) — calibrat: 0.72, offset 0.0046 ────────────────────
float readCurent() {
  const int samples = 500;
  float suma_offset = 0;
  for (int i = 0; i < samples; i++) {
    suma_offset += (analogRead(PIN_CURENT) / 4095.0) * 3.3;
    delayMicroseconds(100);
  }
  float offset_dinamic = suma_offset / samples;
  float suma_patrate = 0;
  for (int i = 0; i < samples; i++) {
    float tensiune = (analogRead(PIN_CURENT) / 4095.0) * 3.3;
    float centrat  = tensiune - offset_dinamic;
    suma_patrate  += centrat * centrat;
    delayMicroseconds(100);
  }
  float vrms   = sqrt(suma_patrate / samples);
  float curent = vrms * 5.0 * 0.165;
  if (curent < 0.05) curent = 0.0;
  return curent;
}

float readVoltage() {
  const int samples = 2000;
  float offset = 0;
  for (int i = 0; i < samples; i++) offset += analogRead(ZMPT_PIN);
  offset /= samples;
  float sumSquares = 0;
  for (int i = 0; i < samples; i++) {
    float value = analogRead(ZMPT_PIN) - offset;
    sumSquares += value * value;
  }
  float voltage = sqrt(sumSquares / samples) * 0.426;
  if (voltage < 20.0) voltage = 0.0;
  return voltage;
}

float readVoltageInv() {
  const int samples = 2000;
  float offset = 0;
  for (int i = 0; i < samples; i++) offset += analogRead(ZMPT_INV);
  offset /= samples;
  float sumSquares = 0;
  for (int i = 0; i < samples; i++) {
    float value = analogRead(ZMPT_INV) - offset;
    sumSquares += value * value;
  }
  float voltage = sqrt(sumSquares / samples) * 0.265;
  if (voltage < 20.0) voltage = 0.0;
  return voltage;
}

// ─── Curent DC (INA219) — brut, fara calibrare ───────────────────────────────
float readCurentDC() {
  const int NR = 100;
  float suma = 0;
  for (int i = 0; i < NR; i++) {
    float c = ina219.getCurrent_mA();
    if (c < 0) c = -c;
    suma += c;
    delayMicroseconds(500);
  }
  float media = suma / NR;
  curentFiltrat = curentFiltrat * 0.8 + media * 0.2;
  return curentFiltrat;
}

float calculeazaSOC(float t) {
  if (t >= 12.70) return 100.0;
  if (t >= 12.50) return 90.0;
  if (t >= 12.30) return 80.0;
  if (t >= 12.10) return 70.0;
  if (t >= 11.90) return 60.0;
  if (t >= 11.75) return 50.0;
  if (t >= 11.60) return 40.0;
  if (t >= 11.45) return 30.0;
  if (t >= 11.30) return 20.0;
  if (t >= 11.10) return 10.0;
  return 0.0;
}

void switchToGenerator() {
  Serial.println("Comutare pe generator");
  digitalWrite(PIN_26, HIGH);
  delay(300);
  digitalWrite(PIN_27, HIGH);
  delay(300);
  digitalWrite(PIN_26, LOW);
  peInvertor = true;
  nrComutari++;
  salveaza();
}
void switchToGrid() {
  Serial.println("Revenire pe retea");
  digitalWrite(PIN_26, HIGH);
  delay(300);
  digitalWrite(PIN_27, LOW);
  delay(300);
  digitalWrite(PIN_26, LOW);
  peInvertor = false;
  nrComutari++;
  salveaza();
}

BLYNK_CONNECTED() {
  Blynk.syncVirtual(V6, V7, V8, V9);
}

BLYNK_WRITE(V6) {
  if (param.asInt()) {
    modOperare = MOD_RETEA;
    Blynk.virtualWrite(V7, 0);
    Blynk.virtualWrite(V8, 0);
    Serial.println("Mod: RETEA");
    if (peInvertor) switchToGrid();
  }
}
BLYNK_WRITE(V7) {
  if (param.asInt()) {
    modOperare = MOD_INVERTOR;
    Blynk.virtualWrite(V6, 0);
    Blynk.virtualWrite(V8, 0);
    Serial.println("Mod: INVERTOR");
    if (!peInvertor && socCoulomb > SOC_MIN) switchToGenerator();
  }
}
BLYNK_WRITE(V8) {
  if (param.asInt()) {
    modOperare = MOD_MANUAL;
    Blynk.virtualWrite(V6, 0);
    Blynk.virtualWrite(V7, 0);
    Serial.println("Mod: MANUAL");
    if (manualInvertor && !peInvertor) switchToGenerator();
    else if (!manualInvertor && peInvertor) switchToGrid();
  }
}
BLYNK_WRITE(V9) {
  manualInvertor = param.asInt();
  if (modOperare == MOD_MANUAL) {
    if (manualInvertor && !peInvertor) switchToGenerator();
    else if (!manualInvertor && peInvertor) switchToGrid();
  } else {
    Serial.println("Switch ignorat — nu esti in mod Manual");
  }
}

void sendVoltage() {
  float voltage     = readVoltage();
  float voltageInv  = readVoltageInv();
  float curent      = readCurent();
  float tensiuneBat = ina219.getBusVoltage_V() + (ina219.getShuntVoltage_mV() / 1000.0);
  float curentBat   = readCurentDC();

  if (socCoulomb < 0) socCoulomb = calculeazaSOC(tensiuneBat);
  float dt = 2.0 / 3600.0;
  socCoulomb -= (curentBat / 1000.0 * dt / CAPACITATE_AH) * 100.0;
  if (socCoulomb < 0) socCoulomb = 0;
  if (socCoulomb > 100) socCoulomb = 100;

  float putere = tensiuneBat * (curentBat / 1000.0);
  float energieRamasa = CAPACITATE_WH * (socCoulomb / 100.0);
  float autonomieMin = (putere > 0.1) ? (energieRamasa / putere * 60.0) : 0;

  int ore = (int)autonomieMin / 60;
  int minute = (int)autonomieMin % 60;
  String autonomieText = String(ore) + "h " + String(minute) + "min";

  unsigned long acum = millis();
  unsigned long delta = (acum - ultimulMoment) / 1000;
  if (delta > 0) {
    ultimulMoment = acum;
    if (peInvertor) timpInvertor += delta; else timpRetea += delta;
    salveaza();
  }

  unsigned long timpTotal = timpRetea + timpInvertor;
  int procentRetea = 0, procentInvertor = 0;
  if (timpTotal > 0) {
    procentRetea = (timpRetea * 100) / timpTotal;
    procentInvertor = (timpInvertor * 100) / timpTotal;
  }
  String utilizareText = "Retea " + String(procentRetea) + "% / Invertor " + String(procentInvertor) + "%";

  // ─── NOTIFICARI (globale) ────────────────────────────────────────────────
  if (voltage < PRAG_RETEA_JOS && !alertaReteaCazuta) {
    Blynk.logEvent("retea_cazuta");
    alertaReteaCazuta = true;
  }
  if (voltage > PRAG_RETEA_SUS && alertaReteaCazuta) {
    Blynk.logEvent("retea_revenita");
    alertaReteaCazuta = false;
  }
  if (peInvertor && voltageInv < 50.0 && !alertaInvertorCazut) {
    Blynk.logEvent("invertor_cazut");
    alertaInvertorCazut = true;
  }
  if (voltageInv > 100.0) alertaInvertorCazut = false;
  if (socCoulomb <= SOC_MIN && !alertaBaterieCritica) {
    Blynk.logEvent("baterie_critica");
    alertaBaterieCritica = true;
  }
  if (socCoulomb > SOC_MIN + 5) alertaBaterieCritica = false;

  // ─── LOGICA MODURI ───────────────────────────────────────────────────────
  if (modOperare == MOD_RETEA) {
    if (!peInvertor && voltage < PRAG_RETEA_JOS && socCoulomb > SOC_MIN) switchToGenerator();
    if (peInvertor && voltage > PRAG_RETEA_SUS) switchToGrid();
  }
  else if (modOperare == MOD_INVERTOR) {
    if (!peInvertor && socCoulomb > SOC_MIN) switchToGenerator();
    if (peInvertor && socCoulomb <= SOC_MIN) switchToGrid();
  }
  else if (modOperare == MOD_MANUAL) {
    if (peInvertor && socCoulomb <= SOC_MIN) switchToGrid();
  }

  Serial.print("U retea: ");    Serial.println(voltage);
  Serial.print("U invertor: "); Serial.println(voltageInv);
  Serial.print("I sarcina: ");  Serial.println(curent, 3);
  Serial.print("U baterie: ");  Serial.println(tensiuneBat, 2);
  Serial.print("I baterie: ");  Serial.println(curentBat, 1);
  Serial.print("SOC: ");        Serial.println(socCoulomb, 0);
  Serial.print("Autonomie: ");  Serial.println(autonomieText);
  Serial.print("Utilizare: ");  Serial.println(utilizareText);
  Serial.print("Comutari: ");   Serial.println(nrComutari);
  Serial.println("---");

  Blynk.virtualWrite(V0,  voltage);
  Blynk.virtualWrite(V1,  curent);
  Blynk.virtualWrite(V2,  voltageInv);
  Blynk.virtualWrite(V4,  socCoulomb);
  Blynk.virtualWrite(V5,  autonomieText);
  Blynk.virtualWrite(V10, tensiuneBat);
  Blynk.virtualWrite(V11, curentBat / 1000.0);
  Blynk.virtualWrite(V12, nrComutari);
  Blynk.virtualWrite(V14, utilizareText);
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);

  EEPROM.begin(EEPROM_SIZE);
  citesteEEPROM();

  Wire.begin(21, 22);
  delay(500);
  if (!ina219.begin()) Serial.println("INA219 negasit!");
  else Serial.println("INA219 OK");

  pinMode(PIN_26, OUTPUT);
  pinMode(PIN_27, OUTPUT);
  pinMode(PIN_CURENT, INPUT);
  pinMode(ZMPT_INV, INPUT);
  digitalWrite(PIN_26, LOW);
  digitalWrite(PIN_27, LOW);

  ultimulMoment = millis();

  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  timer.setInterval(2000L, sendVoltage);
}

void loop() {
  Blynk.run();
  timer.run();
}