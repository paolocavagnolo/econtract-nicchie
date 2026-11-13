// =====================================================================
// JogWebFAS - applica il jog della pagina web a un motore FastAccelStepper
//
// Da chiamare a OGNI giro del loop(), per ogni motore:
//   jogWeb(0, M1, MAN_ACC, MAN_SPEED, STATE == 2);
// - permesso = true solo quando la macchina e' in MANUALE
// - se arriva un comando nuovo (direzione o velocita') lo applica
// - se il comando sparisce (rilascio, uomo morto, uscita dal manuale)
//   ferma il motore con la sua rampa e rimette la velocita' del manuale
//   da quadro, cosi' i pulsanti fisici funzionano come prima.
// In manuale i finecorsa NON vengono controllati (regola di Paolo).
// =====================================================================
#pragma once
#include <FastAccelStepper.h>
#include "PaginaWebNicchie.h"

struct JogWebMemoria { int8_t dir; uint32_t vel; };

inline JogWebMemoria* jogWebMemoria(uint8_t m) {
  static JogWebMemoria memoria[PWN_MAX_MOTORI] = {};
  return &memoria[m < PWN_MAX_MOTORI ? m : 0];
}

inline void jogWeb(uint8_t m, FastAccelStepper* motore, uint32_t accelerazione,
                   uint32_t velocitaQuadro, bool permesso) {
  if (motore == NULL) return;
  JogWebMemoria* mem = jogWebMemoria(m);
  uint32_t vel = 0;
  int8_t dir = permesso ? paginaWeb.jog(m, vel) : 0;

  if (dir == 0) {
    if (mem->dir != 0) {               // il jog da pagina e' finito: fermo il motore
      motore->stopMove();
      motore->setSpeedInHz(velocitaQuadro);
      mem->dir = 0;
      mem->vel = 0;
    }
  } else if (dir != mem->dir || !motore->isRunning()) {
    // nuova direzione (o motore fermato da altro): riparte
    motore->setSpeedInHz(vel);
    motore->setAcceleration(accelerazione);
    if (dir > 0) motore->runForward();
    else motore->runBackward();
    mem->dir = dir;
    mem->vel = vel;
  } else if (vel != mem->vel) {
    // stessa direzione, velocita' diversa (preset o cursore)
    motore->setSpeedInHz(vel);
    motore->setAcceleration(accelerazione);
    motore->applySpeedAcceleration();
    mem->vel = vel;
  }

  paginaWeb.impostaMotore(m, motore->getCurrentPosition(), motore->isRunning());
}
