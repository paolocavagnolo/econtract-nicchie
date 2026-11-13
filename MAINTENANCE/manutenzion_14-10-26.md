# Manutenzione Champs-Elysees del 14/10/2026

Ora inizio: 20.45
Ora fine:   04.00

Paolo Cavagnolo con Alexandre
Erano presenti anche 2 tecnici che hanno smontato i pannelli del mobile centrale e che hanno pulito le nicchie.
Nelle prime ore erano presenti anche Salvatore e Arnaud

## Lista interventi

###  Pressa grande

La pressa grande era in blocco. Il finecorsa ottico non rispondeva più e sforzando il motore ha allentato la tenuta del giunto tra l'albero motore e la vite senza fine della guida lineare.

 - Ho cambiato il finecorsa ottico con quello meccanico mettendo i fili nel seguente modo:
 	BLU (gnd) 	  - NC
 	MARRONE (24v) - NO
 	NERO (sig) 	  - COM
Il PLC l'ha riconosciuto subito correttamente.

 - Ho smontato il motore, aggiunto un giro di nasto sulla vite senza fine, ricollegato il giunto e stretto.

Ho notato che quando il motore accelera tutta la struttura trema leggermente. Non ricordo fosse così all'inizio.

Sicuramente una guida lineare IGUS darebbe più stabilità. Secondo me conviene sceglierne una e fare un offerta per la sostituzione.

Nel codice ho notato che mancava il timeout sul homing, quindi quando sono arrivato ho trovato il motore che girava all'infinito. Ho inserito il timeout, ma bisognerà ricontrollare tutti i codici e irrobustirli rispetto a queste casistiche strane. E' un lavoro che posso fare a Torino e poi verificarlo alla prossima manutenzione.

### Ago e filo

Al mio arrivo la struttura aveva attorcigliato il filo attorno al tubo per 2 giri completi. Facendo partire il gioco ho notato che ad ogni esecuzione l'ago veniva spostato un po' di gradi oltre al finecorsa, dopo un paio di giochi aveva già fatto quasi 90 gradi oltre. 

Il finecorsa funzionava correttamente, noi però non l'usiamo per fermare il gioco, lo usiamo solamente quando il gioco finisce per tornare a 0.

Il problema è nel motore che gestisce la rotazione verticale che non tiene più e si fa trascinare dall'attrito causato dal motore che fa la verticale, che tocca il cilindro probabilmente nel punto di saldatura della vite senza fine.

Ho bloccato quindi il motore orizzontale dandogli corrente ma togliendo i movimenti dal gioco. Il risultato è che l'ago adesso si muove correttamente in verticale, e l'attrito rimasto fa anche muovere leggeremente in orrizontale, meglio che lasciarlo spento.

Dobbiamo ridurre l'attritto e sostituire il motore

### 5 martelli

Il problema della vibrazione dei martelli A e D, quelli ribaltati in alto, non è semplice da affrontare. La corrente dei driver è già al minimo, forse potremmo prendere dei driver più piccoli, ma quelli della OMC stepper non sono compatibili con l'esp32, dovremmo creargli un'interfaccia custom. Oppure usare i driver delle stampanti 3D, forse vendono già una scheda con 5 driver già collegati ad un arduino. La cerco.

Per il resto funziona bene, ho fatto molti test. Ho aumentato il tempo di avvio per permettere alla vibrazione iniziale di non propagarsi nel movimento, e ho provato ad aumentare e poi diminuire le velocità e le curve di accelerazione ma mi è sembrato che le microvibrazione non cambiassero.

Secondo me sono dovute all'enorme sbilanciamento tra il carico del martello e le dimensioni del motore. Ho provato ad aumentare la corrente per dargli più forza ma le vibrazioni sono aumentate, forse potrebbe valere la pena provare a diminuarla, ma non è detto che risolva.

### Galvanico

Ho cambiato l'altezza di attivazione dell'aria in modo che si attivi subito anche quando alzano il livello del liquido

## Mobile centrale

### Cinture

Ho rimosso il motoriduttore e quindi diminuito la velocità del motore. Il giunto da 8 a 8 era perfetto, ho solamente aggiunto un pezzo di nastro sulla vite senza fine. Ho notato che al fondo della vite senza fine c'era un dado fissato sulla barra che sforzava sulla base della guida lineare aumentando il rumore.

Inoltre ho sostituito il finecorsa ottico con uno meccanico con lo stesso schema di prima.

Il rumore è diminuito e anche le vibrazioni che si riverberavano sulle cinturine sono quasi sparite del tutto.

### Patacabra

Qua la situazione richiede il cambio di tutto, anche dei quadri elettrici, così portiamo dentro i relay e non li teniamo più accatastati fuori dal quadro. Magari usando quel PLC con 25 relay.

Per prima cosa ho notato che molti martelli erano bloccati con l'asse e che i relay non erano tutti spenti. Ho ravviato il PLC, ha spento i relay, ma i motorini dentro ai martelli non si sono mossi e sono rimasti tutti ancora bloccati.

Ho notato allora che la presa dell'alimentatore della 12V era leggermente staccata! Sistemandola ho finalmente sentito il rumore dei motorini che si muovevano liberandosi dall'asse.

Ho lanciato un test per verificare che tutti i relay e tutti i motorini funzionassero, risultato:

Su 39 relay ne funzionano 35
Su 39 motorini ne funzionano 13! 4 sul primo asse, 2 sul secondo e 7 sul terzo (il più alto)

Può essere successo di tutto, fili tranciati, fili in corto, motorini bruciati.. 

Ho forzato quindi il codice ad usare solamente i motorini che funzionano e avviato il gioco. 
I movimenti erano incompleti e un po' a scatti, ho controllato i motori e loro sono perfetti, anche il giunto verso il rimando a 90 gradi. Il problema è tra il rimando a 90 e l'asse.. li l'usura si sta mangiando l'accoppiamento e sta perdendo passi. Il risultato è che l'asse 1 ha un movimento visibile piccolissimo
l'asse 2 e l'asse 3 un po' meglio.

I martelli adesso si bloccano e sbloccano bene, però quando l'asse si muove se ne trascina su molti, non solo quelli bloccati.

### Led

non sono riuscito ad aggiungere la scheda arduino perché non avevano scale abbastanza lunghe per aprire lo scomparto.



