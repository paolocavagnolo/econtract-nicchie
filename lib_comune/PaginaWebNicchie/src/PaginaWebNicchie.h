// =====================================================================
// PaginaWebNicchie - libreria comune a tutte le schede delle nicchie
//
// Ogni scheda crea una sua rete WiFi (access point) con il nome della
// macchina e mostra una pagina dei comandi all'indirizzo 192.168.4.1.
// La pagina e' tutta dentro la scheda: funziona senza internet.
//
// REGOLE DI SICUREZZA (decise da Paolo):
//  - La pagina comanda SOLO se il selettore fisico e' su AUTO e non c'e'
//    un avvio esterno attivo. Su MAN o REMOTE la pagina mostra solo lo stato.
//  - Il server web NON muove mai i motori: scrive solo delle richieste.
//    E' il loop() dello sketch che le legge e le applica.
//  - Jog "a uomo morto": il telefono ripete il comando ogni 150 ms;
//    se per JOG_UOMO_MORTO_MS non arriva niente, il motore si ferma.
//  - Il manuale da pagina si spegne da solo dopo MANUALE_TIMEOUT_MS senza
//    comandi, se il selettore lascia AUTO o se il telefono si scollega.
//  - Il manuale da pagina NON cancella il BLOCCO (si cancella solo
//    accendendo la scheda con il selettore fisico su MANUALE).
//
// Il server gira in un compito separato sul core 0, cosi' risponde anche
// quando il loop() e' fermo in un azzeramento o in un'attesa lunga.
//
// NOTA libreria EQSP32: nessuno sketch chiama eqsp32.begin(), quindi la
// libreria EQSP32 non accende il suo WiFi e non c'e' conflitto. Se un
// giorno si usasse eqsp32.begin(), il suo WiFi andrebbe disattivato.
// =====================================================================
#pragma once
#include <Arduino.h>
#include "soc/gpio_reg.h"

// ---- PASSWORD DELLA RETE WIFI (uguale per tutte le schede) ----
// SEGNAPOSTO: Paolo, cambiala prima di caricare! Almeno 8 caratteri.
#ifndef PWN_PASSWORD_WIFI
#define PWN_PASSWORD_WIFI "nicchie-manut"
#endif

// ---- TEMPI DI SICUREZZA ----
#define PWN_JOG_UOMO_MORTO_MS   400UL     // ms: senza comandi il jog si ferma
#define PWN_MANUALE_TIMEOUT_MS  120000UL  // ms: 2 minuti senza comandi -> esce dal manuale
#define PWN_PAUSA_CAMBIO_MS     1500UL    // ms: pausa minima tra fine AVVIO e MANUALE (e viceversa)
#define PWN_STABILITA_SEL_MS    50UL      // ms: il selettore deve restare fuori da AUTO per annullare i comandi
#define PWN_SOGLIA_ANALOGICA    4000      // ingressi letti con analogRead: attivo se > soglia

// ---- LIMITI ----
#define PWN_MAX_FINECORSA 6
#define PWN_MAX_AVVII     2
#define PWN_MAX_MOTORI    4
#define PWN_MAX_SPIE      6
#define PWN_NESSUN_PIN    255

class PaginaWebNicchie {
public:
  // ---- configurazione (chiamare nel setup, prima di begin) ----
  // Finecorsa: attivo = ingresso alto (o > soglia se analogico)
  void aggiungiFinecorsa(const char* nome, uint8_t pin, bool analogico = false);
  // Canale di avvio: pin dell'avvio esterno (filo AUTO/REMOTE) e pin del selettore MAN.
  // Usare PWN_NESSUN_PIN se la scheda non ha selettore (C01, C02, C03).
  void aggiungiAvvio(const char* nome, uint8_t pinAvvio, uint8_t pinMan, bool analogico = false);
  // Motore passo-passo per il jog: velocita' in passi al secondo
  void aggiungiMotore(const char* nome, uint32_t lento, uint32_t medio, uint32_t veloce);
  // Spia di sola lettura (es. "Rele' ESP32"): il valore lo scrive lo sketch
  void aggiungiSpia(const char* nome);
  // Avvia rete WiFi, server e controlli di sicurezza
  void begin(const char* nomeRete, const char* titolo);

  // ---- letture per il loop() dello sketch ----
  bool avvioWeb(uint8_t canale = 0);   // true = la pagina simula l'avvio esterno
  bool manualeWeb();                   // true = manuale da pagina attivo
  // Jog del motore m: ritorna -1 indietro, 0 fermo, +1 avanti; vel in passi/s
  int8_t jog(uint8_t m, uint32_t &vel);

  // ---- scritture dallo sketch (stato mostrato sulla pagina) ----
  void impostaStato(const char* testo);
  void impostaBlocco(bool blocco);
  void impostaSpia(uint8_t i, bool valore);
  void impostaMotore(uint8_t m, long posizione, bool inMoto);
  void annullaAvvio(uint8_t canale);   // es. tempo massimo scaduto

  // ---- uso interno (compiti del server) ----
  void _servi();
  void _controlla();
  void _scollegato();
};

extern PaginaWebNicchie paginaWeb;

// Legge lo stato di un'USCITA (es. rele', audio) senza toccarla: serve per le spie
inline bool pwnUscita(uint8_t pin) {
  if (pin < 32) return (REG_READ(GPIO_OUT_REG) >> pin) & 1;
  return (REG_READ(GPIO_OUT1_REG) >> (pin - 32)) & 1;
}
