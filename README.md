# PostBot🤖 - Il tuo Robot Postino Intelligente!

---

## Cosa è PostBot?🤖

**PostBot** è un progetto che simula un robot postino autonomo, progettato per raccogliere e consegnare palline colorate a scatole corrispondenti in base al colore. Il tutto progettato con **ROS** in un ambiente simulato come **Gazebo** per vedere al meglio come il robot si comporta!

L'ambiente simulato inserisce palline che compaiono in posizioni casuali ad ogni ciclo, comunicate al robot tramite un topic personalizzato. PostBot si sposta verso la pallina, "la raccoglie" e la consegna alla scatola del colore corrispondente. Quando tutte le scatole sono piene, il sistema si resetta riposizionando tutte le scatole in maniera casuale. Pronto ad effettuare un nuovo giro di consegne!

---

## Requisiti👾

- **ROS**: Per la gestione di nodi, topic, servizi e parametri.
- **Gazebo**: Per la simulazione dell'ambiente con palline e scatole posizionate randomicamente.
- **RVIZ**: Per monitorare in tempo reale navigazione e stato del robot.
- **Navigation Stack di ROS**: Per il movimento autonomo.
- **TurtleBot3**: Il robot autonomo peer la consegna delle palline.

---

## Componenti Principali🕹️

### Topics Principali
- `/current ball`: Posizione e colore della pallina attuale.
- `/box status`: Stato delle scatole (posizioni, colore e se piene o meno).
- `/box goal`: Obiettivo del robot (coordinate della scatola da raggiungere per la consegna).

### Messaggi Personalizzati
- `BallInfo.msg`: Colore e posizione della pallina.
- `BoxStatus.msg`: Stato, colore e posizioni delle scatole.
- `BoxGoal.msg`: Obiettivo della consegna.

### Servizi
- `/spawn ball`: Genera una nuova pallina.
- `/reset boxes`: Svuota e riposiziona le scatole.

### Parametri
- `Velocità del Robot`.
- `Posizioni iniziali delle Scatole`.

---

## Flusso Operativo🖲️

1. La pallina viene generata tramite `/spawn ball` e il robot ne riceve le informazioni tramite `/current ball`.  
2. Determina l'obiettivo e si sposta verso di esso usando `/box goal`.  
3. Consegna la pallina alla scatola corretta e aggiorna lo stato della scatola su `/box status`.  
4. Quando tutte le scatole sono piene, attiva il servizio `/reset boxes` e riparte.

---

## Come Installare
(ancora da completare!)
