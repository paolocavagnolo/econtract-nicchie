// =====================================================================
// PaginaWebNicchie - implementazione (vedi PaginaWebNicchie.h)
// =====================================================================
#include "PaginaWebNicchie.h"
#include <WiFi.h>
#include <WebServer.h>
#include "pagina_html.h"

PaginaWebNicchie paginaWeb;

// ---- POSIZIONE DEL SELETTORE VISTA DALLA PAGINA ----
#define SEL_AUTO   0
#define SEL_MAN    1
#define SEL_REMOTE 2   // avvio esterno attivo (REMOTE o comando dal gioco)

// ---- DATI CONDIVISI tra loop() e compiti del server ----
// Tutto quello che e' qui dentro si legge/scrive SOLO dentro la sezione
// critica (portENTER_CRITICAL), cosi' i due core non si pestano i piedi.
struct Finecorsa { const char* nome; uint8_t pin; bool analogico; };
struct Avvio     { const char* nome; uint8_t pinAvvio; uint8_t pinMan; bool analogico; bool web; };
struct Motore    { const char* nome; uint32_t lento, medio, veloce;
                   int8_t dir; uint32_t vel; uint32_t tJog; long pos; bool inMoto; };
struct Spia      { const char* nome; bool valore; };

static portMUX_TYPE lucchetto = portMUX_INITIALIZER_UNLOCKED;

static Finecorsa finecorsa[PWN_MAX_FINECORSA];
static Avvio     avvii[PWN_MAX_AVVII];
static Motore    motori[PWN_MAX_MOTORI];
static Spia      spie[PWN_MAX_SPIE];
static uint8_t nFinecorsa = 0, nAvvii = 0, nMotori = 0, nSpie = 0;

static bool     manuale = false;          // manuale da pagina attivo
static bool     permessi = false;         // selettore su AUTO: la pagina puo' comandare
static uint8_t  selettore = SEL_AUTO;
static bool     blocco = false;
static uint32_t tUltimoComando = 0;       // ultimo comando ricevuto (per uscire dal manuale)
static uint32_t tFineAvvio = 0;           // quando si e' spento l'ultimo AVVIO da pagina
static uint32_t tFineManuale = 0;         // quando si e' spento il manuale da pagina
static char     testoStato[64] = "Accensione";

static const char* nomeRete = "";
static const char* titolo = "";
static String descrizione;                // JSON della macchina (fisso)
static WebServer server(80);

// ---------------------------------------------------------------------
// Funzioni di appoggio
// ---------------------------------------------------------------------
static bool leggiIngresso(uint8_t pin, bool analogico) {
  if (pin == PWN_NESSUN_PIN) return false;
  if (analogico) return analogRead(pin) > PWN_SOGLIA_ANALOGICA;
  return digitalRead(pin) == HIGH;
}

// Da chiamare DENTRO la sezione critica
static void fermaJog() {
  for (uint8_t i = 0; i < nMotori; i++) motori[i].dir = 0;
}
static void spegniAvvii(uint32_t ora) {
  for (uint8_t i = 0; i < nAvvii; i++) {
    if (avvii[i].web) { avvii[i].web = false; tFineAvvio = ora; }
  }
}
static void spegniManuale(uint32_t ora) {
  fermaJog();
  if (manuale) { manuale = false; tFineManuale = ora; }
}
static bool avvioWebAttivo() {
  for (uint8_t i = 0; i < nAvvii; i++) if (avvii[i].web) return true;
  return false;
}

static void aggiungiTesto(String &s, const char* t) {
  s += '"';
  for (const char* c = t; *c; c++) {
    if (*c == '"' || *c == '\\') s += '\\';
    if ((uint8_t)*c >= 0x20) s += *c;
  }
  s += '"';
}

// ---------------------------------------------------------------------
// Configurazione
// ---------------------------------------------------------------------
void PaginaWebNicchie::aggiungiFinecorsa(const char* nome, uint8_t pin, bool analogico) {
  if (nFinecorsa < PWN_MAX_FINECORSA) finecorsa[nFinecorsa++] = { nome, pin, analogico };
}
void PaginaWebNicchie::aggiungiAvvio(const char* nome, uint8_t pinAvvio, uint8_t pinMan, bool analogico) {
  if (nAvvii < PWN_MAX_AVVII) avvii[nAvvii++] = { nome, pinAvvio, pinMan, analogico, false };
}
void PaginaWebNicchie::aggiungiMotore(const char* nome, uint32_t lento, uint32_t medio, uint32_t veloce) {
  if (nMotori < PWN_MAX_MOTORI) motori[nMotori++] = { nome, lento, medio, veloce, 0, 0, 0, 0, false };
}
void PaginaWebNicchie::aggiungiSpia(const char* nome) {
  if (nSpie < PWN_MAX_SPIE) spie[nSpie++] = { nome, false };
}

// ---------------------------------------------------------------------
// Letture per il loop()
// ---------------------------------------------------------------------
bool PaginaWebNicchie::avvioWeb(uint8_t canale) {
  if (canale >= nAvvii) return false;
  portENTER_CRITICAL(&lucchetto);
  bool v = avvii[canale].web && permessi;
  portEXIT_CRITICAL(&lucchetto);
  return v;
}

bool PaginaWebNicchie::manualeWeb() {
  portENTER_CRITICAL(&lucchetto);
  bool v = manuale && permessi;
  portEXIT_CRITICAL(&lucchetto);
  return v;
}

int8_t PaginaWebNicchie::jog(uint8_t m, uint32_t &vel) {
  vel = 0;
  if (m >= nMotori) return 0;
  uint32_t ora = millis();
  portENTER_CRITICAL(&lucchetto);
  int8_t d = motori[m].dir;
  // uomo morto: se il telefono non ripete il comando, il motore si ferma
  if (!manuale || !permessi || (ora - motori[m].tJog) > PWN_JOG_UOMO_MORTO_MS) d = 0;
  vel = motori[m].vel;
  portEXIT_CRITICAL(&lucchetto);
  return d;
}

// ---------------------------------------------------------------------
// Scritture dallo sketch
// ---------------------------------------------------------------------
void PaginaWebNicchie::impostaStato(const char* testo) {
  portENTER_CRITICAL(&lucchetto);
  strncpy(testoStato, testo, sizeof(testoStato) - 1);
  testoStato[sizeof(testoStato) - 1] = 0;
  portEXIT_CRITICAL(&lucchetto);
}
void PaginaWebNicchie::impostaBlocco(bool b) {
  portENTER_CRITICAL(&lucchetto);
  blocco = b;
  if (b) spegniAvvii(millis());   // in BLOCCO nessun AVVIO da pagina
  portEXIT_CRITICAL(&lucchetto);
}
void PaginaWebNicchie::impostaSpia(uint8_t i, bool valore) {
  if (i >= nSpie) return;
  portENTER_CRITICAL(&lucchetto);
  spie[i].valore = valore;
  portEXIT_CRITICAL(&lucchetto);
}
void PaginaWebNicchie::impostaMotore(uint8_t m, long posizione, bool inMoto) {
  if (m >= nMotori) return;
  portENTER_CRITICAL(&lucchetto);
  motori[m].pos = posizione;
  motori[m].inMoto = inMoto;
  portEXIT_CRITICAL(&lucchetto);
}
void PaginaWebNicchie::annullaAvvio(uint8_t canale) {
  if (canale >= nAvvii) return;
  portENTER_CRITICAL(&lucchetto);
  if (avvii[canale].web) { avvii[canale].web = false; tFineAvvio = millis(); }
  portEXIT_CRITICAL(&lucchetto);
}

// ---------------------------------------------------------------------
// Controlli di sicurezza (compito sul core 0, ogni 10 ms)
// ---------------------------------------------------------------------
void PaginaWebNicchie::_controlla() {
  static uint8_t selLetto = SEL_AUTO, selStabile = SEL_AUTO;
  static uint32_t tCambio = 0;

  bool man = false, esterno = false;
  for (uint8_t i = 0; i < nAvvii; i++) {
    man     |= leggiIngresso(avvii[i].pinMan, false);
    esterno |= leggiIngresso(avvii[i].pinAvvio, avvii[i].analogico);
  }
  uint8_t sel = man ? SEL_MAN : (esterno ? SEL_REMOTE : SEL_AUTO);
  uint32_t ora = millis();
  if (sel != selLetto) { selLetto = sel; tCambio = ora; }
  if ((ora - tCambio) >= PWN_STABILITA_SEL_MS) selStabile = selLetto;

  portENTER_CRITICAL(&lucchetto);
  selettore = selStabile;
  permessi = (selStabile == SEL_AUTO);
  if (!permessi) {                // selettore fuori da AUTO: comanda il quadro
    spegniManuale(ora);
    spegniAvvii(ora);
  }
  if (manuale && (ora - tUltimoComando) > PWN_MANUALE_TIMEOUT_MS) {
    spegniManuale(ora);           // 2 minuti senza comandi
  }
  for (uint8_t i = 0; i < nMotori; i++) {
    if (motori[i].dir != 0 && (ora - motori[i].tJog) > PWN_JOG_UOMO_MORTO_MS) motori[i].dir = 0;
  }
  portEXIT_CRITICAL(&lucchetto);
}

// Un telefono si e' scollegato dal WiFi
void PaginaWebNicchie::_scollegato() {
  bool nessuno = (WiFi.softAPgetStationNum() == 0);
  uint32_t ora = millis();
  portENTER_CRITICAL(&lucchetto);
  fermaJog();
  if (nessuno) { spegniManuale(ora); spegniAvvii(ora); }
  portEXIT_CRITICAL(&lucchetto);
  Serial.println(nessuno ? "PAGINA WEB: nessun telefono collegato, comandi annullati"
                         : "PAGINA WEB: un telefono si e' scollegato, jog fermato");
}

// ---------------------------------------------------------------------
// Risposte HTTP
// ---------------------------------------------------------------------
static const char* testoSelettore(uint8_t s) {
  if (s == SEL_MAN) return "MAN";
  if (s == SEL_REMOTE) return "REMOTE";
  return "AUTO";
}

static void inviaStato(const char* errore) {
  // letture "dal vivo" degli ingressi (fuori dalla sezione critica)
  bool fc[PWN_MAX_FINECORSA];
  bool est[PWN_MAX_AVVII];
  for (uint8_t i = 0; i < nFinecorsa; i++) fc[i] = leggiIngresso(finecorsa[i].pin, finecorsa[i].analogico);
  for (uint8_t i = 0; i < nAvvii; i++) est[i] = leggiIngresso(avvii[i].pinAvvio, avvii[i].analogico);
  uint32_t ora = millis();

  // copia dei dati condivisi
  Avvio a[PWN_MAX_AVVII]; Motore m[PWN_MAX_MOTORI]; Spia sp[PWN_MAX_SPIE];
  char st[sizeof(testoStato)];
  bool manC, permC, bloccoC; uint8_t selC; uint32_t tUltC;
  portENTER_CRITICAL(&lucchetto);
  memcpy(a, avvii, sizeof(a)); memcpy(m, motori, sizeof(m)); memcpy(sp, spie, sizeof(sp));
  memcpy(st, testoStato, sizeof(st));
  manC = manuale; permC = permessi; bloccoC = blocco; selC = selettore; tUltC = tUltimoComando;
  portEXIT_CRITICAL(&lucchetto);

  String s;
  s.reserve(600);
  s += "{\"sel\":\""; s += testoSelettore(selC);
  s += "\",\"cmd\":"; s += permC ? "true" : "false";
  s += ",\"blocco\":"; s += bloccoC ? "true" : "false";
  s += ",\"stato\":"; aggiungiTesto(s, st);
  s += ",\"man\":"; s += manC ? "true" : "false";
  long resta = 0;
  if (manC) { long r = (long)PWN_MANUALE_TIMEOUT_MS - (long)(ora - tUltC); resta = r > 0 ? r / 1000 : 0; }
  s += ",\"manResta\":"; s += resta;
  s += ",\"avvio\":[";
  for (uint8_t i = 0; i < nAvvii; i++) { if (i) s += ','; s += a[i].web ? '1' : '0'; }
  s += "],\"est\":[";
  for (uint8_t i = 0; i < nAvvii; i++) { if (i) s += ','; s += est[i] ? '1' : '0'; }
  s += "],\"fc\":[";
  for (uint8_t i = 0; i < nFinecorsa; i++) { if (i) s += ','; s += fc[i] ? '1' : '0'; }
  s += "],\"spie\":[";
  for (uint8_t i = 0; i < nSpie; i++) { if (i) s += ','; s += sp[i].valore ? '1' : '0'; }
  s += "],\"mot\":[";
  for (uint8_t i = 0; i < nMotori; i++) {
    if (i) s += ',';
    bool vivo = manC && permC && (ora - m[i].tJog) <= PWN_JOG_UOMO_MORTO_MS;
    s += "{\"d\":"; s += vivo ? m[i].dir : 0;
    s += ",\"v\":"; s += m[i].vel;
    s += ",\"p\":"; s += m[i].pos;
    s += ",\"r\":"; s += m[i].inMoto ? '1' : '0';
    s += '}';
  }
  s += "],\"clienti\":"; s += WiFi.softAPgetStationNum();
  if (errore) { s += ",\"errore\":"; aggiungiTesto(s, errore); }
  s += '}';
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", s);
}

// Motivo per cui la pagina non puo' comandare (NULL = puo')
static const char* motivoRifiuto() {
  if (permessi) return NULL;
  if (selettore == SEL_MAN) return "Selettore su MANUALE: comandi dalla pagina disattivati";
  return "Avvio esterno attivo (REMOTE): comandi dalla pagina disattivati";
}

static void gestisciAvvio() {
  uint8_t c = server.arg("c").toInt();
  bool on = server.arg("on") == "1";
  uint32_t ora = millis();
  const char* errore = NULL;
  portENTER_CRITICAL(&lucchetto);
  if (c >= nAvvii) errore = "Canale di avvio sconosciuto";
  else if (!on) { if (avvii[c].web) { avvii[c].web = false; tFineAvvio = ora; } }
  else if ((errore = motivoRifiuto()) != NULL) {}
  else if (blocco) errore = "Macchina in BLOCCO: AVVIO non permesso";
  else if (manuale) errore = "Esci prima dal MANUALE";
  else if ((ora - tFineManuale) < PWN_PAUSA_CAMBIO_MS) errore = "Attendi un attimo e riprova";
  else { avvii[c].web = true; tUltimoComando = ora; }
  portEXIT_CRITICAL(&lucchetto);
  inviaStato(errore);
}

static void gestisciManuale() {
  bool on = server.arg("on") == "1";
  uint32_t ora = millis();
  const char* errore = NULL;
  portENTER_CRITICAL(&lucchetto);
  if (!on) spegniManuale(ora);
  else if ((errore = motivoRifiuto()) != NULL) {}
  else if (nMotori == 0) errore = "Questa scheda non ha motori da muovere";
  else if (avvioWebAttivo()) errore = "Prima ferma il ciclo con FERMA";
  else if ((ora - tFineAvvio) < PWN_PAUSA_CAMBIO_MS) errore = "Attendi un attimo e riprova";
  else { manuale = true; tUltimoComando = ora; }
  portEXIT_CRITICAL(&lucchetto);
  inviaStato(errore);
}

static void gestisciJog() {
  uint8_t m = server.arg("m").toInt();
  long d = server.arg("d").toInt();
  long v = server.arg("v").toInt();
  uint32_t ora = millis();
  const char* errore = NULL;
  portENTER_CRITICAL(&lucchetto);
  if (m >= nMotori) errore = "Motore sconosciuto";
  else if ((errore = motivoRifiuto()) != NULL) {}
  else if (!manuale) errore = "Attiva prima il MANUALE";
  else {
    if (v < 1) v = 1;
    if ((uint32_t)v > motori[m].veloce) v = motori[m].veloce;   // mai oltre il massimo dello sketch
    motori[m].dir = d > 0 ? 1 : (d < 0 ? -1 : 0);
    motori[m].vel = (uint32_t)v;
    motori[m].tJog = ora;
    tUltimoComando = ora;
  }
  portEXIT_CRITICAL(&lucchetto);
  inviaStato(errore);
}

static void gestisciStop() {
  uint32_t ora = millis();
  portENTER_CRITICAL(&lucchetto);
  fermaJog();
  spegniAvvii(ora);
  if (manuale) tUltimoComando = ora;
  portEXIT_CRITICAL(&lucchetto);
  inviaStato(NULL);
}

void PaginaWebNicchie::_servi() {
  server.handleClient();
}

// ---------------------------------------------------------------------
// Compiti FreeRTOS sul core 0 (il loop() gira sul core 1)
// ---------------------------------------------------------------------
static void compitoServer(void*) {
  for (;;) { paginaWeb._servi(); vTaskDelay(pdMS_TO_TICKS(2)); }
}
static void compitoControlli(void*) {
  for (;;) { paginaWeb._controlla(); vTaskDelay(pdMS_TO_TICKS(10)); }
}

static void costruisciDescrizione() {
  bool haSelettore = false;
  for (uint8_t i = 0; i < nAvvii; i++) if (avvii[i].pinMan != PWN_NESSUN_PIN) haSelettore = true;
  String &s = descrizione;
  s = "{\"rete\":"; aggiungiTesto(s, nomeRete);
  s += ",\"titolo\":"; aggiungiTesto(s, titolo);
  s += ",\"selettore\":"; s += haSelettore ? "true" : "false";
  s += ",\"fc\":[";
  for (uint8_t i = 0; i < nFinecorsa; i++) { if (i) s += ','; aggiungiTesto(s, finecorsa[i].nome); }
  s += "],\"avvii\":[";
  for (uint8_t i = 0; i < nAvvii; i++) { if (i) s += ','; aggiungiTesto(s, avvii[i].nome); }
  s += "],\"spie\":[";
  for (uint8_t i = 0; i < nSpie; i++) { if (i) s += ','; aggiungiTesto(s, spie[i].nome); }
  s += "],\"motori\":[";
  for (uint8_t i = 0; i < nMotori; i++) {
    if (i) s += ',';
    s += "{\"nome\":"; aggiungiTesto(s, motori[i].nome);
    s += ",\"lento\":"; s += motori[i].lento;
    s += ",\"medio\":"; s += motori[i].medio;
    s += ",\"veloce\":"; s += motori[i].veloce;
    s += '}';
  }
  s += "],\"jogMs\":150,\"uomoMortoMs\":"; s += PWN_JOG_UOMO_MORTO_MS;
  s += ",\"manualeMs\":"; s += PWN_MANUALE_TIMEOUT_MS;
  s += '}';
}

void PaginaWebNicchie::begin(const char* rete, const char* tit) {
  nomeRete = rete;
  titolo = tit;
  costruisciDescrizione();

  WiFi.persistent(false);   // niente scritture in memoria permanente
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1), IPAddress(255, 255, 255, 0));
  bool ok = WiFi.softAP(nomeRete, PWN_PASSWORD_WIFI, 6, 1, 4);
  WiFi.setSleep(false);     // risposte piu' rapide
  WiFi.onEvent([](WiFiEvent_t, WiFiEventInfo_t) { paginaWeb._scollegato(); },
               ARDUINO_EVENT_WIFI_AP_STADISCONNECTED);

  server.on("/", HTTP_GET, []() {
    server.sendHeader("Cache-Control", "no-store");
    server.send_P(200, "text/html", PAGINA_HTML, sizeof(PAGINA_HTML) - 1);
  });
  server.on("/macchina", HTTP_GET, []() { server.send(200, "application/json", descrizione); });
  server.on("/stato", HTTP_GET, []() { inviaStato(NULL); });
  server.on("/avvio", HTTP_POST, gestisciAvvio);
  server.on("/manuale", HTTP_POST, gestisciManuale);
  server.on("/jog", HTTP_POST, gestisciJog);
  server.on("/stop", HTTP_POST, gestisciStop);
  server.onNotFound([]() { server.sendHeader("Location", "/"); server.send(302, "text/plain", ""); });
  server.begin();

  xTaskCreatePinnedToCore(compitoServer, "pwnServer", 8192, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(compitoControlli, "pwnControlli", 3072, NULL, 2, NULL, 0);

  Serial.print("PAGINA WEB: rete ");
  Serial.print(nomeRete);
  Serial.println(ok ? " attiva, indirizzo 192.168.4.1" : " NON partita (password di almeno 8 caratteri?)");
}
