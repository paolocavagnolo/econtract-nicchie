# Pagina web delle schede

Ogni scheda crea una sua rete WiFi con il nome della macchina (es. `01_PRESSA_GRANDE`).

**Come collegarsi**
1. Sul telefono scegli la rete della macchina. Password: `nicchie-manut` (provvisoria, da cambiare).
2. Se il telefono avvisa "nessuna connessione a internet", scegli **Resta connesso**.
3. Apri il browser all'indirizzo **http://192.168.4.1**

**Cambiare la password**: in `src/PaginaWebNicchie.h`, riga `#define PWN_PASSWORD_WIFI` (almeno 8 caratteri), poi ricarica tutte le schede.

**Modificare la pagina**: si modifica solo `pagina/pagina.html`, poi `python3 genera_pagina.py`.

**Regole**: la pagina comanda solo con il selettore del quadro su AUTO e nessun avvio esterno attivo.
In manuale da pagina i finecorsa sono ignorati; il motore si muove solo finche' tieni premuto.
Il manuale da pagina non toglie il BLOCCO.
