# Demo-Sketch für die xDuinoRails_MotorDriver-Bibliothek

Dieser Sketch demonstriert die grundlegenden Funktionen der `xDuinoRails_MotorDriver`-Bibliothek zur Ansteuerung eines Gleichstrommotors mit Back-EMF-basierter Drehzahlregelung.

## Zweck

Der Sketch führt eine automatische, sich wiederholende Testsequenz durch, um die verschiedenen Zustände des Motors zu zeigen. Er dient als einfaches Beispiel dafür, wie die öffentliche API der Bibliothek verwendet wird, um den Motor zu steuern.

## Ablauf der Demonstration

Die Demo verwendet eine Zustandsmaschine, um den Motor durch eine vordefinierte Sequenz von Aktionen zu führen:

1.  **`ACCELERATE`**: Der Motor beschleunigt sanft über 5 Sekunden auf seine maximale Geschwindigkeit.
2.  **`COAST_HIGH`**: Der Motor läuft 3 Sekunden lang mit maximaler Geschwindigkeit.
3.  **`DECELERATE`**: Der Motor bremst sanft über 3 Sekunden auf eine niedrige Geschwindigkeit ab (ca. 10 %, Rangiermodus).
4.  **`COAST_LOW`**: Der Motor läuft 3 Sekunden lang im Rangiermodus.
5.  **`STOP`**: Der Motor bremst über 1 Sekunde bis zum Stillstand.
6.  **`CHANGE_DIRECTION`**: Nach einer kurzen Pause von 2 Sekunden wird die Drehrichtung des Motors umgekehrt.
7.  Die Sequenz beginnt von vorne.

## Status-LED (NeoPixel)

Eine am Board angeschlossene NeoPixel-RGB-LED zeigt den aktuellen Status des Motors an:

*   **Grün**: Der Motor bewegt sich (beschleunigt, bremst oder fährt).
*   **Rot**: Der Motor steht still.

## Konfiguration

Die Pin-Belegung für den Motortreiber und die NeoPixel-LED ist am Anfang des Sketches (`Demo.ino`) definiert. Stellen Sie sicher, dass diese mit Ihrem Hardware-Aufbau übereinstimmt.

---
*Hinweis: Dies ist ein einfaches Beispiel. Die Bibliothek bietet erweiterte Funktionen, die hier nicht alle gezeigt werden.*
