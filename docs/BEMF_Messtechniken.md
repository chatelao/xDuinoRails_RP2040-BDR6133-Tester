# Alternative BEMF-Messtechniken zur Verbesserung der Genauigkeit

Die Messung der Back-EMF (BEMF) ist entscheidend für eine präzise, sensorlose Motorsteuerung. Die Genauigkeit dieser Messung kann durch verschiedene Techniken verbessert werden.

## 1. Differenzielle Messung

Die aktuell implementierte Methode, die BEMF als Differenz zwischen den beiden Motoranschlüssen zu messen (`bemfAPin` und `bemfBPin`), ist bereits eine robuste Grundlage. Sie minimiert Gleichtaktstörungen, die durch die PWM-Ansteuerung induziert werden.

**Vorteile:**
- Unterdrückung von Gleichtaktrauschen.
- Einfache Implementierung mit zwei ADC-Kanälen.

**Nachteile:**
- Erfordert zwei synchronisierte ADC-Messungen für optimale Ergebnisse.

## 2. Hardware-Tiefpassfilter

Ein einfacher RC-Tiefpassfilter (Widerstand-Kondensator-Glied) kann an den ADC-Eingängen platziert werden, um hochfrequentes Rauschen aus dem PWM-Signal zu filtern, bevor es den ADC erreicht.

**Schaltung:**
Ein Widerstand in Serie zum ADC-Pin und ein Kondensator vom ADC-Pin gegen Masse.

**Vorteile:**
- Einfache und kostengünstige Hardwarelösung.
- Reduziert die Last auf die Software, da das Signal bereits "sauberer" ist.

**Nachteile:**
- Führt eine Phasenverschiebung ein, die bei der Synchronisation mit dem PWM-Zyklus berücksichtigt werden muss.
- Die Grenzfrequenz des Filters muss sorgfältig auf die PWM-Frequenz abgestimmt werden.

## 3. Oversampling und Mittelwertbildung

Diese rein softwarebasierte Technik besteht darin, mehrere ADC-Messungen in schneller Folge durchzuführen und den Durchschnittswert zu berechnen.

**Implementierung:**
Innerhalb des Zeitfensters, in dem die BEMF gemessen wird, wird der ADC mehrmals ausgelesen und die Ergebnisse werden gemittelt.

**Vorteile:**
- Verbessert das Signal-Rausch-Verhältnis (SNR).
- Keine zusätzliche Hardware erforderlich.

**Nachteile:**
- Erhöht die Prozessorlast.
- Benötigt ausreichend Zeit für die Messungen innerhalb eines PWM-Zyklus.

## 4. Kalman-Filter zur Glättung der Messwerte

Ein Kalman-Filter ist ein fortschrittlicher digitaler Filter, der auf einem mathematischen Modell des Systems basiert, um eine optimale Schätzung des Zustands (in diesem Fall der BEMF) zu liefern. Er ist besonders effektiv bei der Reduzierung von Rauschen in dynamischen Systemen.

### Funktionsweise und Logik

Der Kalman-Filter arbeitet in einem Zwei-Phasen-Prozess:

1.  **Vorhersage (Prediction):** Basierend auf dem letzten geschätzten Zustand und einem Systemmodell sagt der Filter den nächsten Zustand voraus. Im Kontext der Motorsteuerung könnte das Modell annehmen, dass die Motorgeschwindigkeit (und damit die BEMF) sich nicht abrupt ändert.
2.  **Korrektur (Update):** Der Filter vergleicht die Vorhersage mit dem neuen Messwert. Basierend auf der Unsicherheit der Vorhersage und der Messung berechnet er einen korrigierten, optimalen Zustand. Dieser korrigierte Wert ist eine gewichtete Mischung aus der Vorhersage und der Messung.

Einfach ausgedrückt: Der Filter "weiß", dass der Motor eine gewisse Trägheit hat und die BEMF sich nicht sprunghaft ändern kann. Wenn ein Messwert stark von der Erwartung abweicht, geht der Filter davon aus, dass es sich wahrscheinlich um Rauschen handelt, und gewichtet die Vorhersage stärker. Wenn Messung und Vorhersage nahe beieinander liegen, "vertraut" der Filter dem Messwert mehr.

### Vor- und Nachteile

**Vorteile:**
- **Optimale Schätzung:** Liefert mathematisch die bestmögliche Schätzung, wenn das Rauschen normalverteilt ist (Gauß'sches Rauschen).
- **Adaptiv:** Kann sich an ändernde Systemdynamiken anpassen.
- **Glättung ohne Phasenverschiebung:** Im Vergleich zu einfachen Tiefpassfiltern führt der Kalman-Filter eine deutlich geringere Verzögerung (Phasenverschiebung) in das Signal ein, was für Regelkreise sehr wichtig ist.
- **Zustandsschätzung:** Kann nicht nur die BEMF glätten, sondern auch andere Zustände wie die Beschleunigung schätzen, falls das Modell entsprechend erweitert wird.

**Nachteile:**
- **Komplexität:** Die Implementierung ist mathematisch anspruchsvoll und erfordert ein gutes Verständnis der Theorie.
- **Rechenintensiv:** Benötigt mehr Rechenleistung (insbesondere Fließkommaarithmetik) als einfache Filter. Dies kann auf kleineren Mikrocontrollern eine Herausforderung sein.
- **Modellabhängigkeit:** Die Leistung des Filters hängt stark von der Genauigkeit des Systemmodells ab. Ein ungenaues Modell kann zu schlechten Schätzungen führen.
- **Tuning:** Der Filter hat mehrere Parameter (Prozessrauschkovarianz Q, Messrauschkovarianz R), die sorgfältig "getunt" werden müssen, um eine optimale Leistung zu erzielen. Dies ist oft ein iterativer, empirischer Prozess.

## Zusammenfassung

| Technik | Vorteile | Nachteile | Komplexität (Implementierung) |
| :--- | :--- | :--- | :--- |
| **Differenzielle Messung** | Gleichtaktstörungs-Unterdrückung | Benötigt 2 synchrone ADCs | Niedrig |
| **Hardware-Tiefpassfilter**| Einfach, entlastet Software | Phasenverschiebung, Abstim-mung nötig | Niedrig (Hardware) |
| **Oversampling** | Verbessert SNR, keine Hardware | Höhere CPU-Last | Niedrig (Software) |
| **Kalman-Filter** | Optimale Glättung, geringe Latenz| Komplex, rechenintensiv, Tuning | Hoch (Software) |

Für die aktuelle Anwendung wäre eine Kombination aus der **differenziellen Messung** und **Oversampling** ein guter erster Schritt. Wenn das Rauschen weiterhin ein Problem darstellt, könnte ein **Kalman-Filter** als fortgeschrittene Softwarelösung implementiert werden, um die Regelungsgüte, insbesondere bei niedrigen Drehzahlen, signifikant zu verbessern.
