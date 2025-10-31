# 🤖 Jules: Build- & Test-Agent

Dies ist die Dokumentation für "Jules", unseren automatisierten CI/CD-Prozess zur Qualitätssicherung der [Name Ihrer Arduino-Bibliothek].

Der Agent ist dafür verantwortlich:
* Den Code-Stil zu prüfen (Linting).
* Die Bibliothek für verschiedene Zielplattformen zu kompilieren (Build).
* Unit-Tests auf einer nativen Plattform auszuführen (Test).
* (Optional) Releases zu erstellen und zu veröffentlichen.

## Stack

* **Build-System:** [PlatformIO](https://platformio.org/)
* **CI-Plattform:** [GitHub Actions](https://github.com/features/actions) (oder GitLab CI, Jenkins etc.)
* **Test-Framework:** [PlatformIO Unit Testing](https://docs.platformio.org/en/latest/advanced/unit-testing/index.html) (meist mit [Unity](https://github.com/ThrowTheSwitch/Unity))
* **Linting:** `pio check` (mit `clang-tidy` und `cppcheck`)

---

## 🛠️ Lokaler Build & Test

Um die CI-Schritte lokal zu replizieren, bevor Sie Code pushen:

1.  **Abhängigkeiten installieren:**
    ```bash
    pip install platformio
    # Ggf. C++ Compiler für native Tests
    # (z.B. build-essential unter Linux, Xcode Command Line Tools unter macOS)
    ```

2.  **Linting (Code-Stil prüfen):**
    ```bash
    pio check --fail-on-defect
    ```

3.  **Alle Umgebungen bauen:**
    * Dies kompiliert den Code für alle in `platformio.ini` definierten Boards (z.B. `uno`, `esp32`).
    ```bash
    pio run
    ```

4.  **Unit-Tests ausführen (Der wichtigste Schritt):**
    * Dies kompiliert und führt die Tests in der `native` Umgebung aus.
    ```bash
    pio test -e native
    ```

---

## 🔬 Test-Strategie (Robustheit)

Unser Ziel ist es, die Logik der Bibliothek **unabhängig von der physischen Hardware** robust zu testen.

### 1. Native Unit-Tests (`env:native`)

* Wir verwenden eine spezielle PlatformIO-Umgebung `[env:native]`.
* Diese kompiliert den Code als normales C++ Programm für Ihren PC (Host-System), nicht für einen Mikrocontroller.
* **Vorteil:** Tests laufen extrem schnell (Sekunden statt Minuten) und können in jeder CI-Pipeline (wie GitHub Actions) ohne angeschlossene Hardware ausgeführt werden.

### 2. Mocking (Hardware-Abstraktion)

* Um Arduino-spezifische Funktionen (z.B. `digitalWrite`, `millis()`, `Wire.h`) in der `native` Umgebung testen zu können, verwenden wir **Mocks**.
* **Wie:** Wir nutzen C++ Präprozessor-Makros (`#ifdef NATIVE_TEST`). Wenn dieses Flag gesetzt ist, wird statt der echten Arduino-Header eine "Fake"-Version (Mock) eingebunden, die das Verhalten simuliert oder aufzeichnet.
* **Beispiel (`test/mocks/Arduino.h`):**
    ```cpp
    #ifndef ARDUINO_MOCK_H
    #define ARDUINO_MOCK_H

    // Mock für millis()
    extern uint32_t mock_millis_value;
    uint32_t millis(void); // Im .cpp-File: return mock_millis_value;

    // Mock für digitalWrite()
    extern int mock_last_pin_written;
    extern int mock_last_value_written;
    void digitalWrite(int pin, int value);

    #endif
    ```

### 3. Test-Coverage

* Wir streben eine hohe Testabdeckung an. Alle Kernlogik-Pfade sollten durch Unit-Tests abgedeckt sein.

---

## 🚀 CI-Pipeline (.github/workflows/main.yml)

Die Pipeline wird bei jedem **Push** und **Pull Request** ausgelöst und führt folgende Schritte aus:

1.  **`Lint`**: Führt `pio check` aus. Schlägt fehl bei Stil-Verletzungen.
2.  **`Unit-Test (native)`**:
    * Installiert Python & PlatformIO.
    * Führt `pio test -e native` aus.
    * Dies ist der wichtigste Gatekeeper. Wenn die Tests fehlschlagen, wird der PR blockiert.
3.  **`Build (Matrix)`**:
    * Führt `pio run` parallel auf einer **Matrix** von verschiedenen Plattformen aus (z.B. `atmelavr`, `espressif32`, `stmicroelectronics`).
    * **Zweck:** Stellt sicher, dass der Code für alle unterstützten Mikrocontroller fehlerfrei kompiliert.

```yaml
# Beispielausschnitt für .github/workflows/main.yml
jobs:
  test_native:
    name: "Unit-Test (Native)"
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - uses: actions/setup-python@v4
        with:
          python-version: '3.9'
      - run: pip install platformio
      - run: pio test -e native

  build_targets:
    name: "Build (Targets)"
    runs-on: ubuntu-latest
    strategy:
      matrix:
        # Definieren Sie hier Ihre Ziel-Envs aus der platformio.ini
        platform_env: [uno, esp32_dev, nucleo_f401re]
    steps:
      - uses: actions/checkout@v3
      - uses: actions/setup-python@v4
      - run: pip install platformio
      - run: pio run -e ${{ matrix.platform_env }}
