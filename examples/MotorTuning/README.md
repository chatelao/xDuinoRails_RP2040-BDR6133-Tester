# Beispiel: MotorTuning

Dieses Beispiel führt einen automatisierten Prozess durch, um die optimalen Regelparameter für einen spezifischen Motor zu ermitteln. Es testet verschiedene Einstellungen für den Kalman-Filter und den PI-Regler und gibt am Ende die besten gefundenen Werte auf der seriellen Konsole aus.

## Zweck

Jeder Motor verhält sich anders. Mechanische Eigenschaften wie Trägheit, Reibung und das Ansprechverhalten beeinflussen die Regelung. Dieser Sketch hilft dabei, die Software-Parameter der `xDuinoRails_MotorDriver`-Bibliothek an die Hardware anzupassen, um eine möglichst stabile und präzise Geschwindigkeitsregelung zu erreichen.

Der Prozess ermittelt die Parameter getrennt für die **Vorwärts-** und **Rückwärtsfahrt**, da Motoren in beide Richtungen oft leicht unterschiedliche Laufeigenschaften haben.

## Voraussetzungen

1.  **Freilaufender Motor:** Der Motor muss für den gesamten Testlauf frei und **ohne Last** (z.B. ohne angekoppelte Waggons) laufen können. Jegliche Behinderung würde die Messergebnisse verfälschen.
2.  **Stabile Stromversorgung:** Stellen Sie eine stabile und ausreichende Stromversorgung für den Motor und die Steuerung sicher.
3.  **Serieller Monitor:** Sie benötigen den Seriellen Monitor (z.B. aus der Arduino IDE), um den Prozess zu verfolgen und die Endergebnisse abzulesen. Stellen Sie die Baudrate auf **115200** ein.

## Ablauf des Tuning-Prozesses

Der Sketch führt nach dem Start eine mehrstufige Testsequenz durch, die mehrere Minuten dauern kann.

1.  **Initialisierung:** Nach dem Hochladen startet der Sketch und wartet 5 Sekunden, damit Sie Zeit haben, den Seriellen Monitor zu öffnen.
2.  **Tuning Vorwärtslauf:**
    *   **Kalman-Filter:** Zuerst werden verschiedene Werte für das Messrauschen (`e_mea`) und das Prozessrauschen (`q`) des Kalman-Filters getestet. Ziel ist es, ein möglichst stabiles und gleichzeitig reaktionsschnelles Geschwindigkeitssignal zu erhalten.
    *   **PI-Regler:** Anschließend werden, mit den besten Kalman-Einstellungen, die optimalen Werte für den P-Anteil (`Kp`) und den I-Anteil (`Ki`) des PI-Reglers gesucht. Ziel ist eine Regelung, die schnell die Soll-Geschwindigkeit erreicht, ohne überzuschwingen oder zu oszillieren.
3.  **Richtungswechsel:** Der Motor wird angehalten und die Laufrichtung auf "rückwärts" geändert.
4.  **Tuning Rückwärtslauf:**
    *   Der gesamte Prozess (Kalman-Filter und PI-Regler) wird für die Rückwärtsrichtung wiederholt.
5.  **Ausgabe der Ergebnisse:** Sobald alle Tests abgeschlossen sind, gibt der Sketch eine übersichtliche Zusammenfassung der empfohlenen Parameter für beide Laufrichtungen aus.

### Wie funktioniert der Test?

Für jede einzelne Parameterkombination (z.B. `Kp=1.0`, `Ki=10.0`) führt der Motor ein standardisiertes **Bewegungsprofil** aus:
- Sanft auf eine niedrige Geschwindigkeit beschleunigen.
- Diese Geschwindigkeit für einige Sekunden halten.
- Auf eine hohe Geschwindigkeit beschleunigen.
- Diese Geschwindigkeit für einige Sekunden halten.
- Sanft zum Stillstand abbremsen.

Während dieses Profils misst der Sketch kontinuierlich die Abweichung zwischen der **Soll-Geschwindigkeit** und der **Ist-Geschwindigkeit**. Die Summe dieser Abweichungen ergibt einen **"Stabilitäts-Score"**. Die Parameter-Kombination mit dem **niedrigsten Score** gewinnt.

## Verwendung der Ergebnisse

Nach Abschluss des Prozesses sehen Sie im Seriellen Monitor eine Ausgabe wie diese:

```
===================================
Tuning Abgeschlossen!
Empfohlene Einstellungen:
===================================

--- VORWAERTSLAUF ---
Kalman-Filter: e_mea = 40.00, q = 0.05
PI-Regler: Kp = 2.00, Ki = 20.00

--- RUECKWAERTSLAUF ---
Kalman-Filter: e_mea = 40.00, q = 0.05
PI-Regler: Kp = 2.00, Ki = 20.00

Um diese Einstellungen zu verwenden, uebertragen Sie sie in Ihren Haupt-Sketch.
```

Diese Werte können Sie in Ihrem eigenen Sketch verwenden, um die Motor-Instanz zu konfigurieren:

```cpp
// In Ihrem Haupt-Sketch
XDuinoRails_MotorDriver motor(INA_PIN, INB_PIN, BEMFA_PIN, BEMFB_PIN);

void setup() {
  motor.begin();

  // Wenden Sie hier die gefundenen Tuning-Werte an.
  // Es ist ratsam, je nach Fahrtrichtung unterschiedliche Werte zu setzen.
  // Hier als Beispiel die Vorwärts-Werte:
  motor.setKalmanGains(40.00, 0.05);
  motor.setGains(2.00, 20.00);
}
```

Es wird empfohlen, die `set...`-Methoden immer dann aufzurufen, wenn sich die Fahrtrichtung ändert, um die jeweils optimalen Parameter zu laden.
