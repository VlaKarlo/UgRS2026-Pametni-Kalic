#include <Arduino.h>
#include <Wire.h>
#include <BH1750.h>
#include <HX711.h>
#include <ESP32Servo.h>
#include <Adafruit_NeoPixel.h>
#include <Preferences.h>
#include <WiFi.h>
#include <time.h>
#include <sys/time.h>

//Pinovi
const uint8_t PIN_VLAGA[3] = { 36, 39, 34 };
const uint8_t PIN_I2C_SDA = 21;
const uint8_t PIN_I2C_SCL = 22;
const uint8_t PIN_HX_VODA_DT   = 16;
const uint8_t PIN_HX_VODA_SCK  = 4;
const uint8_t PIN_HX_GNOJ_DT   = 17;
const uint8_t PIN_HX_GNOJ_SCK  = 5;
const uint8_t PIN_SV1 = 13;   // ventil spremnika vode
const uint8_t PIN_SV2 = 14;   // ventil spremnika gnojiva
const uint8_t PIN_SK[3] = { 25, 26, 27 };   // ventili kalića 1..3
const uint8_t PIN_RELEJ_PUMPA = 23;
const uint8_t RELEJ_AKTIVNA_RAZINA = LOW;
const uint8_t PIN_LED = 18;
const uint16_t LED_PO_PRSTENU = 8;
const uint16_t BROJ_PRSTENOVA = 3;
const uint16_t BROJ_LED = LED_PO_PRSTENU * BROJ_PRSTENOVA;

//Vrijednosti
const float V_UK   = 50.0;    // [ml] volumen jednog obroka
const float OMJER_K = 200.0;  // omjer miješanja 1 : K (1 ml gnojiva na K ml vode)
const float V_ISP  = 10.0;    // [ml] završni dio vode za ispiranje zajedničke cijevi
const float MIN_DOZA_GNOJIVA = 2.0;   // [ml]
const float RHO_VODA    = 1.00;   // [g/ml] gustoća vode
const float RHO_GNOJIVO = 1.05;   // [g/ml] gustoća tekućeg gnojiva (provjeriti na deklaraciji)
const float MASA_MIN_MJERLJIVA = 5.0;    // [g] ispod ove mase koristi se vremensko doziranje
const uint32_t T_MAX_PUMPA_MS  = 60000;  // [ms] najdulje neprekidno vrijeme rada pumpe
const float W_MIN = 35.0;                    // [%] donja granica vlažnosti supstrata
const uint32_t T_CEKANJA_MS = 15UL * 60000;  // [ms] pauza između dva obroka u isti kalić
const uint32_t T_CIKLUS_MS  = 10000;         // [ms] razdoblje glavne petlje
const float M_MIN_VODA    = 200.0;   // [g] najmanja dopuštena masa u spremniku vode
const float M_MIN_GNOJIVO = 50.0;    // [g] najmanja dopuštena masa u spremniku gnojiva
const float E_REF  = 5000.0;   // [lx] zadana osvijetljenost u razini biljaka
const float E_HIST = 500.0;    // [lx] širina histereze (sprječava učestalo paljenje)
const float P_MAX  = 60.0;
const int SAT_PALJENJA = 6;    // rasvjeta je dopuštena od 06:00
const int SAT_GASENJA  = 22;   // do 22:00 (fotoperiod 16 h)
const char *VREMENSKA_ZONA = "CET-1CEST,M3.5.0/2,M10.5.0/3";   // Srednjoeuropsko vrijeme
const char *WIFI_SSID    = "NAZIV_MREZE";
const char *WIFI_LOZINKA = "LOZINKA_MREZE";
const char *NTP_1 = "hr.pool.ntp.org";
const char *NTP_2 = "pool.ntp.org";
const uint32_t T_CEKANJE_WIFI_MS = 15000;      // najdulje čekanje na spajanje
const uint32_t T_CEKANJE_NTP_MS  = 10000;      // najdulje čekanje na dohvat vremena
const uint32_t T_SINKRONIZACIJA_MS = 6UL * 3600000;   // ponovni dohvat vremena svakih 6 h
const uint32_t T_PONOVNO_SPAJANJE_MS = 60000;  // razmak pokušaja ponovnog spajanja
const uint32_t T_SPREMI_VRIJEME_MS = 600000;   // 10 min
const int KUT_ZATVORENO = 20;
const int KUT_OTVORENO  = 90;
const uint16_t T_ZAKRET_MS = 500;   // [ms] vrijeme potrebno servo motoru za zakret
#define DRZI_MOMENT 0   // 0 = otpuštanje nakon zakreta (preporučeno)
                        // 1 = trajno držanje momenta (samo ako konstrukcija to zahtijeva!)
const uint32_t T_OSVJEZI_MS = 300000;      // [ms] razdoblje osvježavanja (5 min)
const uint32_t T_MAX_DRZANJA_MS = 10000;   // [ms]
const uint16_t T_PAUZA_POMAK_MS = 300;     // [ms]
enum Ventil  { SV1 = 0, SV2 = 1, SK1 = 2, SK2 = 3, SK3 = 4 };
enum Spremnik { VODA = 0, GNOJIVO = 1 };

//Objekti i globalne varijable
BH1750 senzorSvjetla(0x23);        // ADDR na niskoj razini -> adresa 0x23
HX711 vagaVode;
HX711 vagaGnojiva;
Servo servo[5];                    // SV1, SV2, SK1, SK2, SK3
const uint8_t PIN_VENTIL[5] = { PIN_SV1, PIN_SV2, PIN_SK[0], PIN_SK[1], PIN_SK[2] };
Adafruit_NeoPixel rasvjeta(BROJ_LED, PIN_LED, NEO_GRB + NEO_KHZ800);
Preferences memorija;              // trajna pohrana kalibracijskih konstanti (NVS)
float W[3]       = { 0, 0, 0 };    // [%] relativna vlažnost supstrata po kalićima
float E          = 0;              // [lx] prirodna osvijetljenost (bez doprinosa rasvjete)
float mVoda      = 0;              // [g] masa spremnika vode
float mGnojivo   = 0;              // [g] masa spremnika gnojiva
float P          = 0;              // [%] trenutna snaga rasvjete
bool  rasvjetaUkljucena = false;   // stanje rasvjete (za histerezu)
bool  vrijemePostavljeno = false;  // je li vrijeme poznato (dohvaćeno ili postavljeno)
bool  vrijemePriblizno = false;    // vrijeme je obnovljeno iz memorije nakon nestanka napajanja
uint32_t tSpremljenoVrijeme = 0;   // trenutak zadnjeg spremanja vremena
uint32_t tSinkronizacija = 0;      // trenutak zadnjeg uspješnog dohvata vremena
uint32_t tPokusajSpajanja = 0;     // trenutak zadnjeg pokušaja spajanja na mrežu
bool  zalihaOK   = true;
uint32_t tZadnje[3]   = { 0, 0, 0 };   // trenutak posljednjeg doziranja u kalić k
bool     prvoDoziranje[3] = { true, true, true };
float    gnojivoDug[3] = { 0, 0, 0 };  // [ml] nakupljena, još nedozirana količina gnojiva
uint32_t tCiklus = 0;
bool doziranjeUTijeku = false;
bool     ventilOtvoren[5]    = { false, false, false, false, false };
bool     ventilPricvrscen[5] = { false, false, false, false, false };  // prima li PWM signal
uint32_t tPricvrscen[5]      = { 0, 0, 0, 0, 0 };   // trenutak uključenja PWM signala
uint32_t tOsvjezeno[5]       = { 0, 0, 0, 0, 0 };   // trenutak zadnjeg osvježavanja položaja
uint32_t tZadnjiPomak[5]     = { 0, 0, 0, 0, 0 };   // trenutak zadnjeg zakreta
long  N0_VODA = 0,    N0_GNOJIVO = 0;       // tara vage (očitanje neopterećene vage)
float K_VODA  = 420.0, K_GNOJIVO = 420.0;   // faktor skaliranja [očitanje/g]
int   N_SUHO[3]  = { 3000, 3000, 3000 };    // očitanje ADC-a u suhom supstratu
int   N_MOKRO[3] = { 1350, 1350, 1350 };    // očitanje ADC-a u zasićenom supstratu
float PROTOK_ML_S = 1.2;                    // [ml/s] protok pumpe (naredba "f")
float E_LED_MAX = 3000.0;                   // [lx]

//Prototipi funkcija
void  citajSenzore();
void  regulirajRasvjetu();
void  dozirajObrok(uint8_t k);
float dozirajIzSpremnika(uint8_t s, float Vcilj);
void  postaviVentil(uint8_t id, bool otvoren);
void  pricvrstiVentil(uint8_t id);
void  otpustiVentil(uint8_t id);
void  nadziriVentile();
void  pumpa(bool ukljuci);
float masaSpremnika(uint8_t s, uint8_t uzoraka = 8);
float citajVlagu(uint8_t k);
void  postaviRasvjetu(float snagaPosto);
void  signalizirajGresku(const char *poruka);
void  ispisiStanje();
void  obradiSerijskuNaredbu();
void  ucitajKalibraciju();
bool  unutarFotoperioda();
void  spremiVrijeme(bool prisilno = false);
bool  spojiNaMrezu();
bool  dohvatiVrijeme();
void  odrziMrezuIVrijeme();
String vrijemeTekst();
void  spremiKalibraciju();


void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println(F("\n=== Pametni kalic – pokretanje ==="));
  digitalWrite(PIN_RELEJ_PUMPA, !RELEJ_AKTIVNA_RAZINA);
  pinMode(PIN_RELEJ_PUMPA, OUTPUT);
  digitalWrite(PIN_RELEJ_PUMPA, !RELEJ_AKTIVNA_RAZINA);
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  for (uint8_t i = 0; i < 5; i++) {
    postaviVentil(i, false);              // svi ventili zatvoreni, motori se potom otpuštaju
  }
  rasvjeta.begin();
  rasvjeta.clear();
  rasvjeta.show();                        // rasvjeta isključena
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  if (senzorSvjetla.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x23, &Wire)) {
    Serial.println(F("BH1750: OK"));
  } else {
    Serial.println(F("BH1750: GRESKA – senzor se ne javlja na adresi 0x23"));
  }
  setenv("TZ", VREMENSKA_ZONA, 1);
  tzset();
  memorija.begin("kalic", true);
  uint32_t spremljeno = (uint32_t)memorija.getLong("vrij", 0);
  memorija.end();
  if (spremljeno > 0) {
    struct timeval tv = { (time_t)spremljeno, 0 };
    settimeofday(&tv, nullptr);
    vrijemePostavljeno = true;
    vrijemePriblizno = true;
  }
  if (spojiNaMrezu()) {
    dohvatiVrijeme();
  }
  if (!vrijemePostavljeno) {
    Serial.println(F("UPOZORENJE: vrijeme nije poznato – rasvjeta ostaje iskljucena."));
    Serial.println(F("Rucno postavljanje: c 2026-09-28 08:30"));
  }
  vagaVode.begin(PIN_HX_VODA_DT, PIN_HX_VODA_SCK);
  vagaGnojiva.begin(PIN_HX_GNOJ_DT, PIN_HX_GNOJ_SCK);
  ucitajKalibraciju();
  Serial.println(F("Naredbe: t | k1 <g> | k2 <g> | s<k> | m<k> | d<k> | f | r <%> | e <lx> | c <vrijeme> | p | w"));
  Serial.println(F("=== Pokretanje zavrseno ===\n"));
  tCiklus = millis();
}

void loop() {
  obradiSerijskuNaredbu();
  nadziriVentile();                 // zaštita servo motora (izvodi se u svakom prolasku)
  if (millis() - tCiklus < T_CIKLUS_MS) {
    return;                       // čekanje do sljedećeg ciklusa (neblokirajuće)
  }
  tCiklus = millis();
  citajSenzore();
  if (mVoda < M_MIN_VODA || mGnojivo < M_MIN_GNOJIVO) {
    zalihaOK = false;
    signalizirajGresku("Provjerite zalihe u spremnicima");
  } else {
    zalihaOK = true;
  }
  regulirajRasvjetu();
  for (uint8_t k = 0; k < 3; k++) {
    bool isteklaPauza = prvoDoziranje[k] || (millis() - tZadnje[k] >= T_CEKANJA_MS);
    if (W[k] < W_MIN && zalihaOK && isteklaPauza) {
      dozirajObrok(k);
      tZadnje[k] = millis();
      prvoDoziranje[k] = false;
    }
  }
  odrziMrezuIVrijeme();
  spremiVrijeme();
  ispisiStanje();
}

//Čitanje senzora
float citajVlagu(uint8_t k) {
  uint32_t zbroj = 0;
  for (uint8_t i = 0; i < 16; i++) {
    zbroj += analogRead(PIN_VLAGA[k]);
    delay(5);
  }

  float N = zbroj / 16.0;
  float raspon = (float)(N_SUHO[k] - N_MOKRO[k]);
  if (fabs(raspon) < 1.0) return 0;            // neispravna kalibracija
  float w = (N_SUHO[k] - N) / raspon * 100.0;
  return constrain(w, 0.0, 100.0);
}

float masaSpremnika(uint8_t s, uint8_t uzoraka) {
  HX711 &vaga = (s == VODA) ? vagaVode : vagaGnojiva;
  if (!vaga.wait_ready_timeout(500)) {
    Serial.println(F("HX711: vaga se ne javlja"));
    return (s == VODA) ? mVoda : mGnojivo;     // zadrži zadnju poznatu vrijednost
  }
  long N = vaga.read_average(uzoraka);
  long N0 = (s == VODA) ? N0_VODA : N0_GNOJIVO;
  float K = (s == VODA) ? K_VODA : K_GNOJIVO;
  if (fabs(K) < 0.001) return 0;
  return (N - N0) / K;
}

void citajSenzore() {
  for (uint8_t k = 0; k < 3; k++) {
    W[k] = citajVlagu(k);
  }
  float lux = senzorSvjetla.readLightLevel();
  if (lux >= 0) E = lux;                       // negativna vrijednost = greška očitanja
  mVoda    = masaSpremnika(VODA);
  mGnojivo = masaSpremnika(GNOJIVO);
}

//Rasvjeta
void postaviRasvjetu(float snagaPosto) {
  snagaPosto = constrain(snagaPosto, 0.0, P_MAX);
  uint8_t v = (uint8_t)(snagaPosto * 255.0 / 100.0);
  for (uint16_t i = 0; i < BROJ_LED; i++) {
    rasvjeta.setPixelColor(i, rasvjeta.Color(v, v, v));
  }
  rasvjeta.show();
}

bool unutarFotoperioda() {
  if (!vrijemePostavljeno) return false;
  time_t sada = time(nullptr);
  struct tm t;
  localtime_r(&sada, &t);
  int minutaUDanu = t.tm_hour * 60 + t.tm_min;
  return (minutaUDanu >= SAT_PALJENJA * 60) && (minutaUDanu < SAT_GASENJA * 60);
}

bool spojiNaMrezu() {
  tPokusajSpajanja = millis();
  if (WiFi.status() == WL_CONNECTED) return true;
  Serial.print(F("Spajanje na mrezu "));
  Serial.print(WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_LOZINKA);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < T_CEKANJE_WIFI_MS) {
    delay(250);
    Serial.print(F("."));
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print(F(" OK, IP: "));
    Serial.println(WiFi.localIP());
    return true;
  }
  Serial.println(F(" neuspjesno."));
  return false;
}

bool dohvatiVrijeme() {
  if (WiFi.status() != WL_CONNECTED) return false;
  configTzTime(VREMENSKA_ZONA, NTP_1, NTP_2);
  uint32_t t0 = millis();
  while (millis() - t0 < T_CEKANJE_NTP_MS) {
    time_t sada = time(nullptr);
    if (sada > 1700000000) {                   // vrijeme je očito ispravno
      vrijemePostavljeno = true;
      vrijemePriblizno = false;
      tSinkronizacija = millis();
      spremiVrijeme(true);
      Serial.print(F("Vrijeme dohvaceno s posluzitelja: "));
      Serial.println(vrijemeTekst());
      return true;
    }
    delay(200);
  }
  Serial.println(F("Dohvat vremena nije uspio."));
  return false;
}

void odrziMrezuIVrijeme() {
  bool spojen = (WiFi.status() == WL_CONNECTED);

  if (!spojen && (millis() - tPokusajSpajanja > T_PONOVNO_SPAJANJE_MS)) {
    spojen = spojiNaMrezu();
  }
  if (spojen && (!vrijemePostavljeno || vrijemePriblizno ||
                 millis() - tSinkronizacija > T_SINKRONIZACIJA_MS)) {
    dohvatiVrijeme();
  }
}

void spremiVrijeme(bool prisilno) {
  if (!vrijemePostavljeno) return;
  if (!prisilno && (millis() - tSpremljenoVrijeme < T_SPREMI_VRIJEME_MS)) return;
  tSpremljenoVrijeme = millis();
  memorija.begin("kalic", false);
  memorija.putLong("vrij", (long)time(nullptr));
  memorija.end();
}

String vrijemeTekst() {
  if (!vrijemePostavljeno) return String("--:--:--");
  time_t sada = time(nullptr);
  struct tm t;
  localtime_r(&sada, &t);
  char buf[24];
  strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &t);
  return String(buf);
}

void regulirajRasvjetu() {
  if (!unutarFotoperioda()) {
    P = 0;
    rasvjetaUkljucena = false;
    postaviRasvjetu(P);
    return;
  }

  if (E > E_REF) {
    rasvjetaUkljucena = false;
  } else if (E < E_REF - E_HIST) {
    rasvjetaUkljucena = true;
  }

  if (!rasvjetaUkljucena) {
    P = 0;
  } else {
    P = (E_REF - E) / E_LED_MAX * 100.0;
    P = constrain(P, 0.0, P_MAX);
  }
  postaviRasvjetu(P);
}

//Tekući sustav
void pricvrstiVentil(uint8_t id) {
  if (ventilPricvrscen[id]) return;
  servo[id].setPeriodHertz(50);                // PWM 50 Hz (perioda 20 ms)
  servo[id].attach(PIN_VENTIL[id], 500, 2400);
  ventilPricvrscen[id] = true;
  tPricvrscen[id] = millis();
}

void otpustiVentil(uint8_t id) {
  if (!ventilPricvrscen[id]) return;
  servo[id].detach();
  pinMode(PIN_VENTIL[id], OUTPUT);
  digitalWrite(PIN_VENTIL[id], LOW);           // signalni vod ostaje u niskoj razini
  ventilPricvrscen[id] = false;
}

void postaviVentil(uint8_t id, bool otvoren) {
  uint32_t proteklo = millis() - tZadnjiPomak[id];
  if (proteklo < T_PAUZA_POMAK_MS) delay(T_PAUZA_POMAK_MS - proteklo);

  pricvrstiVentil(id);
  servo[id].write(otvoren ? KUT_OTVORENO : KUT_ZATVORENO);
  delay(T_ZAKRET_MS);                          

  ventilOtvoren[id] = otvoren;
  tZadnjiPomak[id] = millis();
  tOsvjezeno[id] = millis();
#if !DRZI_MOMENT
  otpustiVentil(id);
#endif
}

void nadziriVentile() {
  uint32_t t = millis();
  for (uint8_t i = 0; i < 5; i++) {
    if (ventilPricvrscen[i] && (t - tPricvrscen[i] > T_MAX_DRZANJA_MS)) {
      if (ventilOtvoren[i]) pumpa(false);
      otpustiVentil(i);
      Serial.print(F("ZASTITA: servo ventila "));
      Serial.print(i + 1);
      Serial.println(F(" predugo pod naponom – otpusten."));
    }
  }
#if !DRZI_MOMENT
  if (doziranjeUTijeku) return;               
  for (uint8_t i = 0; i < 5; i++) {
    if (!ventilOtvoren[i] && (t - tOsvjezeno[i] > T_OSVJEZI_MS)) {
      pricvrstiVentil(i);
      servo[i].write(KUT_ZATVORENO);
      delay(T_ZAKRET_MS);
      otpustiVentil(i);
      tOsvjezeno[i] = millis();
    }
  }
#endif
}

void pumpa(bool ukljuci) {
  digitalWrite(PIN_RELEJ_PUMPA, ukljuci ? RELEJ_AKTIVNA_RAZINA : !RELEJ_AKTIVNA_RAZINA);
}

//Sustav doziranja
float dozirajIzSpremnika(uint8_t s, float Vcilj) {
  if (Vcilj <= 0.01) return 0;

  float rho = (s == VODA) ? RHO_VODA : RHO_GNOJIVO;
  uint8_t ventil = (s == VODA) ? SV1 : SV2;
  float m0 = masaSpremnika(s);
  float V = 0;
  bool greska = false;

  doziranjeUTijeku = true;
  postaviVentil(ventil, true);
  uint32_t t0 = millis();
  pumpa(true);

  if (Vcilj * rho >= MASA_MIN_MJERLJIVA) {
    while (true) {
      float m = masaSpremnika(s, 2);           // manje uzoraka = brži odziv tijekom doziranja
      V = (m0 - m) / rho;
      if (V >= Vcilj) break;
      if (millis() - t0 > T_MAX_PUMPA_MS) {
        greska = true;                         // pumpa se zaustavlja prije dojave greške
        break;
      }
      delay(10);                               // predah za nadzorni brojač (watchdog)
      nadziriVentile();                        // zaštita servo motora i tijekom doziranja
    }
  } else {
    uint32_t trajanje = (uint32_t)(Vcilj / PROTOK_ML_S * 1000.0);
    if (trajanje > T_MAX_PUMPA_MS) trajanje = T_MAX_PUMPA_MS;
    while (millis() - t0 < trajanje) {
      delay(10);
    }
    V = Vcilj;
  }
  pumpa(false);
  delay(300);                                  // izjednačavanje tlaka u cijevi
  postaviVentil(ventil, false);
  doziranjeUTijeku = false;
  if (greska) {
    signalizirajGresku("Isteklo vrijeme doziranja – prazan spremnik ili kvar");
  }
  Serial.print(F("  dozirano iz spremnika "));
  Serial.print(s == VODA ? F("VODA") : F("GNOJIVO"));
  Serial.print(F(": "));
  Serial.print(V, 2);
  Serial.println(F(" ml"));
  return V;
}

void dozirajObrok(uint8_t k) {
  Serial.print(F("Doziranje obroka u kalic "));
  Serial.println(k + 1);

  float Vg = V_UK / (OMJER_K + 1.0);
  float Vv = V_UK - Vg;

  gnojivoDug[k] += Vg;
  float VgSada = 0;
  if (gnojivoDug[k] >= MIN_DOZA_GNOJIVA) {
    VgSada = gnojivoDug[k];
    gnojivoDug[k] = 0;
  }

  postaviVentil(SK1 + k, true);
  dozirajIzSpremnika(VODA, Vv - V_ISP);        // 1. dio vode
  if (VgSada > 0) {
    dozirajIzSpremnika(GNOJIVO, VgSada);       // gnojivo
  }
  dozirajIzSpremnika(VODA, V_ISP);             // ispiranje zajedničke cijevi
  postaviVentil(SK1 + k, false);

  Serial.println(F("Obrok doziran."));
}

//Sustav za serijsku komunikaciju
void signalizirajGresku(const char *poruka) {
  Serial.print(F("GRESKA: "));
  Serial.println(poruka);
  for (uint8_t i = 0; i < 3; i++) {
    for (uint16_t j = 0; j < BROJ_LED; j++) rasvjeta.setPixelColor(j, rasvjeta.Color(60, 0, 0));
    rasvjeta.show();
    delay(200);
    rasvjeta.clear();
    rasvjeta.show();
    delay(200);
  }
  postaviRasvjetu(P);                          // povratak na prethodnu snagu rasvjete
}

void ispisiStanje() {
  Serial.print(vrijemeTekst());
  if (vrijemePriblizno) Serial.print(F("(~)"));
  Serial.print(F("  "));
  Serial.print(F("W1="));  Serial.print(W[0], 1);
  Serial.print(F("%  W2=")); Serial.print(W[1], 1);
  Serial.print(F("%  W3=")); Serial.print(W[2], 1);
  Serial.print(F("%  Evanj=")); Serial.print(E, 0);
  Serial.print(F("lx  P=")); Serial.print(P, 0);
  Serial.print(F("%  mVoda=")); Serial.print(mVoda, 0);
  Serial.print(F("g  mGnojivo=")); Serial.print(mGnojivo, 0);
  Serial.print(F("g  zalihe="));
  Serial.println(zalihaOK ? F("OK") : F("NEDOSTATNE"));
}

// Kalibracija kroz serijsko sučelje
void obradiSerijskuNaredbu() {
  if (!Serial.available()) return;
  String nared = Serial.readStringUntil('\n');
  nared.trim();
  if (nared.length() == 0) return;
  char c = nared.charAt(0);

  switch (c) {
    case 't': {
      if (vagaVode.wait_ready_timeout(1000))    N0_VODA = vagaVode.read_average(20);
      if (vagaGnojiva.wait_ready_timeout(1000)) N0_GNOJIVO = vagaGnojiva.read_average(20);
      Serial.print(F("Tara: N0_VODA=")); Serial.print(N0_VODA);
      Serial.print(F("  N0_GNOJIVO=")); Serial.println(N0_GNOJIVO);
      break;
    }
    case 'k': {
      float masa = nared.substring(2).toFloat();
      if (masa <= 0) { Serial.println(F("Primjer: k1 500")); break; }
      bool voda = (nared.charAt(1) == '1');
      HX711 &vaga = voda ? vagaVode : vagaGnojiva;
      if (!vaga.wait_ready_timeout(1000)) { Serial.println(F("Vaga se ne javlja")); break; }
      long N = vaga.read_average(20);
      long N0 = voda ? N0_VODA : N0_GNOJIVO;
      float K = (N - N0) / masa;
      if (voda) K_VODA = K; else K_GNOJIVO = K;
      Serial.print(F("Faktor K = ")); Serial.println(K, 3);
      break;
    }
    case 's':
    case 'm': {
      uint8_t k = nared.charAt(1) - '1';
      if (k > 2) { Serial.println(F("Primjer: s1 ili m1")); break; }
      uint32_t zbroj = 0;
      for (uint8_t i = 0; i < 16; i++) { zbroj += analogRead(PIN_VLAGA[k]); delay(5); }
      int N = zbroj / 16;
      if (c == 's') N_SUHO[k] = N; else N_MOKRO[k] = N;
      Serial.print(c == 's' ? F("N_SUHO[") : F("N_MOKRO["));
      Serial.print(k + 1); Serial.print(F("] = ")); Serial.println(N);
      break;
    }
    case 'd': {
      uint8_t k = nared.charAt(1) - '1';
      if (k > 2) { Serial.println(F("Primjer: d1")); break; }
      dozirajObrok(k);
      tZadnje[k] = millis();
      prvoDoziranje[k] = false;
      break;
    }
    case 'f': {
      Serial.println(F("Kalibracija protoka: pumpa radi 30 s – izvadite cijev kalica 1 u mjernu posudu."));
      float m0 = masaSpremnika(VODA);
      postaviVentil(SK1, true);
      postaviVentil(SV1, true);
      pumpa(true);
      delay(30000);
      pumpa(false);
      delay(300);
      postaviVentil(SV1, false);
      postaviVentil(SK1, false);
      float m1 = masaSpremnika(VODA);
      PROTOK_ML_S = (m0 - m1) / RHO_VODA / 30.0;
      Serial.print(F("Protok = ")); Serial.print(PROTOK_ML_S, 3);
      Serial.print(F(" ml/s  (")); Serial.print(PROTOK_ML_S * 60.0, 1);
      Serial.println(F(" ml/min)"));
      break;
    }
    case 'r': {
      float snaga = nared.substring(1).toFloat();
      P = constrain(snaga, 0.0, P_MAX);
      postaviRasvjetu(P);
      Serial.print(F("Snaga rasvjete = ")); Serial.println(P, 0);
      break;
    }
    case 'e': {
      float lx = nared.substring(1).toFloat();
      if (lx <= 0) { Serial.println(F("Primjer: e 3000")); break; }
      E_LED_MAX = lx;
      Serial.print(F("E_LED_MAX = ")); Serial.print(E_LED_MAX, 0); Serial.println(F(" lx"));
      break;
    }
    case 'c': {
      struct tm t = {};
      int god, mj, dan, sat, min;
      if (sscanf(nared.substring(1).c_str(), "%d-%d-%d %d:%d", &god, &mj, &dan, &sat, &min) != 5) {
        Serial.println(F("Primjer: c 2026-09-28 08:30"));
        break;
      }
      t.tm_year = god - 1900; t.tm_mon = mj - 1; t.tm_mday = dan;
      t.tm_hour = sat; t.tm_min = min; t.tm_sec = 0;
      t.tm_isdst = -1;                           // ljetno/zimsko vrijeme određuje se iz zone
      time_t epoha = mktime(&t);
      struct timeval tv = { epoha, 0 };
      settimeofday(&tv, nullptr);
      vrijemePostavljeno = true;
      vrijemePriblizno = false;
      spremiVrijeme(true);
      Serial.print(F("Sat postavljen: ")); Serial.println(vrijemeTekst());
      break;
    }
    case 'p':
      ispisiStanje();
      break;
    case 'w':
      spremiKalibraciju();
      Serial.println(F("Kalibracija spremljena."));
      break;
    default:
      Serial.println(F("Nepoznata naredba."));
  }
}

//Dugotrajna memorija
void ucitajKalibraciju() {
  memorija.begin("kalic", true);                // samo čitanje
  N0_VODA     = memorija.getLong("n0v", N0_VODA);
  N0_GNOJIVO  = memorija.getLong("n0g", N0_GNOJIVO);
  K_VODA      = memorija.getFloat("kv", K_VODA);
  K_GNOJIVO   = memorija.getFloat("kg", K_GNOJIVO);
  PROTOK_ML_S = memorija.getFloat("q", PROTOK_ML_S);
  E_LED_MAX   = memorija.getFloat("elm", E_LED_MAX);
  for (uint8_t k = 0; k < 3; k++) {
    char kljuc[6];
    snprintf(kljuc, sizeof(kljuc), "s%u", k);
    N_SUHO[k] = memorija.getInt(kljuc, N_SUHO[k]);
    snprintf(kljuc, sizeof(kljuc), "m%u", k);
    N_MOKRO[k] = memorija.getInt(kljuc, N_MOKRO[k]);
  }
  memorija.end();
}

void spremiKalibraciju() {
  memorija.begin("kalic", false);               
  memorija.putLong("n0v", N0_VODA);
  memorija.putLong("n0g", N0_GNOJIVO);
  memorija.putFloat("kv", K_VODA);
  memorija.putFloat("kg", K_GNOJIVO);
  memorija.putFloat("q", PROTOK_ML_S);
  memorija.putFloat("elm", E_LED_MAX);
  for (uint8_t k = 0; k < 3; k++) {
    char kljuc[6];
    snprintf(kljuc, sizeof(kljuc), "s%u", k);
    memorija.putInt(kljuc, N_SUHO[k]);
    snprintf(kljuc, sizeof(kljuc), "m%u", k);
    memorija.putInt(kljuc, N_MOKRO[k]);
  }
  memorija.end();
}
