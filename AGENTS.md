# 🤖 Jules: Build & Test Agent

This is the documentation for "Jules", our automated CI/CD process for quality assurance of the [Name of your Arduino library].

The agent is responsible for:
* Checking the code style (Linting).
* Compiling the library for various target platforms (Build).
* Running unit tests on a native platform (Test).
* (Optional) Creating and publishing releases.

## Stack

* **Build System:** [PlatformIO](https://platformio.org/)
* **CI Platform:** [GitHub Actions](https://github.com/features/actions) (or GitLab CI, Jenkins etc.)
* **Test Framework:** [PlatformIO Unit Testing](https://docs.platformio.org/en/latest/advanced/unit-testing/index.html) (usually with [Unity](https://github.com/ThrowTheSwitch/Unity))
* **Linting:** `pio check` (with `clang-tidy` and `cppcheck`)

---

## 🛠️ Local Build & Test

To replicate the CI steps locally before pushing code:

1.  **Install Dependencies:**
    ```bash
    pip install platformio
    # If necessary, C++ compiler for native tests
    # (e.g., build-essential under Linux, Xcode Command Line Tools under macOS)
    ```

2.  **Linting (Check Code Style):**
    ```bash
    pio check --fail-on-defect
    ```

3.  **Build All Environments:**
    * This compiles the code for all boards defined in `platformio.ini` (e.g., `uno`, `esp32`).
    ```bash
    pio run
    ```

4.  **Run Unit Tests (The most important step):**
    * This compiles and runs the tests in the `native` environment.
    ```bash
    pio test -e native
    ```

---

## 🔬 Test Strategy (Robustness)

Our goal is to robustly test the library's logic **independently of the physical hardware**.

### 1. Native Unit Tests (`env:native`)

* We use a special PlatformIO environment `[env:native]`.
* This compiles the code as a normal C++ program for your PC (host system), not for a microcontroller.
* **Advantage:** Tests run extremely fast (seconds instead of minutes) and can be executed in any CI pipeline (like GitHub Actions) without connected hardware.

### 2. Mocking (Hardware Abstraction)

* To test Arduino-specific functions (e.g., `digitalWrite`, `millis()`, `Wire.h`) in the `native` environment, we use **mocks**.
* **How:** We use C++ preprocessor macros (`#ifdef NATIVE_TEST`). When this flag is set, a "fake" version (mock) is included instead of the real Arduino header, which simulates or records the behavior.
* **Example (`test/mocks/Arduino.h`):**
    ```cpp
    #ifndef ARDUINO_MOCK_H
    #define ARDUINO_MOCK_H

    // Mock for millis()
    extern uint32_t mock_millis_value;
    uint32_t millis(void); // In the .cpp file: return mock_millis_value;

    // Mock for digitalWrite()
    extern int mock_last_pin_written;
    extern int mock_last_value_written;
    void digitalWrite(int pin, int value);

    #endif
    ```

### 3. Test Coverage

* We aim for high test coverage. All core logic paths should be covered by unit tests.

---

## 🚀 CI Pipeline (.github/workflows/main.yml)

The pipeline is triggered on every **push** and **pull request** and executes the following steps:

1.  **`Lint`**: Runs `pio check`. Fails on style violations.
2.  **`Unit-Test (native)`**:
    * Installs Python & PlatformIO.
    * Runs `pio test -e native`.
    * This is the most important gatekeeper. If the tests fail, the PR is blocked.
3.  **`Build (Matrix)`**:
    * Runs `pio run` in parallel on a **matrix** of different platforms (e.g., `atmelavr`, `espressif32`, `stmicroelectronics`).
    * **Purpose:** Ensures that the code compiles without errors for all supported microcontrollers.

```yaml
# Example snippet for .github/workflows/main.yml
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
        # Define your target envs from platformio.ini here
        platform_env: [uno, esp32_dev, nucleo_f401re]
    steps:
      - uses: actions/checkout@v3
      - uses: actions/setup-python@v4
      - run: pip install platformio
      - run: pio run -e ${{ matrix.platform_env }}
```
