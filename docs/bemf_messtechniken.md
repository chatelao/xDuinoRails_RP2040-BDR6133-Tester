# BEMF-Messtechnik und Filterung

Die genaue Messung der Back-EMF (BEMF) ist entscheidend für die präzise, sensorlose Regelung des Motors. Dieses Dokument beschreibt die im Projekt implementierten Techniken, um aus dem verrauschten Rohsignal eine stabile Geschwindigkeitsinformation zu gewinnen.

Der Prozess gliedert sich in zwei Hauptschritte: die grundlegende Messmethode und eine zweistufige Software-Filterung.

## 1. Grundlage: Differenzielle Messung

Die BEMF wird als Differenzspannung zwischen den beiden Motoranschlüssen (`bemfAPin` und `bemfBPin`) gemessen. Dieser Ansatz ist die Grundlage für die gesamte weitere Verarbeitung.

```cpp
// Aus src/main.cpp
int bemfA = analogRead(bemfAPin);
int bemfB = analogRead(bemfBPin);
int measured_bemf = abs(bemfA - bemfB);
```

**Vorteile:**
- **Unterdrückung von Gleichtaktstörungen:** Rauschen, das auf beiden Leitungen gleichzeitig auftritt (z.B. durch die PWM-Ansteuerung induziert), wird durch die Differenzbildung effektiv eliminiert.
- **Robuste Grundlage:** Liefert ein besseres Rohsignal als eine Messung gegen Masse.

## 2. Software-Filterung zur Signalglättung

Das differenzielle BEMF-Signal ist, besonders bei einem Motor mit 3-poligem Kollektor, immer noch stark verrauscht. Um ein stabiles Signal für die Pulszählung und die PI-Regelung zu erhalten, wird eine zweistufige Filterkaskade eingesetzt.

### Stufe 1: EMA-Filter (Exponential Moving Average)

Das rohe BEMF-Signal wird zuerst durch einen einfachen, aber effektiven EMA-Tiefpassfilter geleitet. Dieser führt eine erste Glättung durch und reduziert hochfrequente Rauschspitzen.

```cpp
// Aus src/main.cpp
smoothed_bemf = (EMA_ALPHA * measured_bemf) + ((1.0 - EMA_ALPHA) * smoothed_bemf);
```

- **Zweck:** Dämpfung von starkem, schnellem Rauschen.
- **Parameter:** Der `EMA_ALPHA`-Wert (`0.21`) bestimmt die Stärke der Glättung. Ein kleinerer Wert glättet stärker, führt aber auch zu mehr Signalverzögerung.

### Stufe 2: Kalman-Filter

Der bereits vorgeglättete Wert aus dem EMA-Filter wird anschließend an einen Kalman-Filter übergeben. Dieser fortschrittliche Filter ist die Kernkomponente für die Signalanalyse, da er den Zustand des Signals (die "wahre" BEMF) unter Berücksichtigung der physikalischen Eigenschaften des Motors schätzt.

#### Funktionsweise und Logik

Der Kalman-Filter ist ideal für diese Aufgabe, da er von einem Systemmodell ausgeht. Vereinfacht gesagt: Der Filter "weiß", dass der Motor eine gewisse Trägheit besitzt und seine Geschwindigkeit (und somit die BEMF) sich nicht beliebig schnell ändern kann.

Er arbeitet in zwei Phasen:
1.  **Vorhersage (Prediction):** Basierend auf dem letzten Zustand sagt der Filter den nächsten Zustand voraus. Er erwartet eine gewisse Kontinuität.
2.  **Korrektur (Update):** Er vergleicht die Vorhersage mit dem neuen (bereits EMA-gefilterten) Messwert. Weicht der Messwert stark von der Vorhersage ab, wird er als wahrscheinliches Rauschen eingestuft und nur gering gewichtet. Liegen Vorhersage und Messung nahe beieinander, wird der Messwert stärker gewichtet.

#### Vorteile im Projekt
- **Optimale Glättung:** Liefert eine mathematisch optimale Schätzung des BEMF-Signals, was zu einer sehr zuverlässigen Erkennung der Kommutierungspulse führt.
- **Geringe Latenz:** Verursacht weniger Phasenverschiebung (Signalverzögerung) als ein starker EMA- oder RC-Filter allein, was für die Stabilität des Regelkreises entscheidend ist.
- **Anpassungsfähigkeit:** Die Leistung wird über Parameter an die Motorcharakteristik angepasst. Im Code sind dies `BEMF_MEA_E` (Messunsicherheit) und `BEMF_Q` (Prozessvarianz), die für einen Motor mit hohem Messrauschen und geringer Prozessdynamik (Trägheit) voreingestellt sind.

## Zusammenfassung der Implementierung

Die aktuelle Lösung kombiniert bewährte Techniken zu einer robusten Verarbeitungskette:

| Stufe | Technik | Zweck |
| :--- | :--- | :--- |
| **1** | **Differenzielle Messung** | Grundlage: Erfassung des Rohsignals mit Unterdrückung von Gleichtaktstörungen. |
| **2** | **EMA-Filter** | Erste Glättung: Dämpfung von hochfrequenten Rauschspitzen im Rohsignal. |
| **3** | **Kalman-Filter** | Finale Schätzung: Optimale, adaptive Filterung des Signals basierend auf einem Systemmodell. |

Diese Kaskade sorgt dafür, dass aus dem stark verrauschten Signal eines einfachen Bürstenmotors eine zuverlässige Geschwindigkeitsinformation für den PI-Regler extrahiert werden kann.
