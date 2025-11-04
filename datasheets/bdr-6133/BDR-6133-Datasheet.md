# BDR6133
**1CH DC Motor Driver**

## DESCRIPTION

The BDR6133 is 1 Full-On Drive H-Bridge channel with two different packages. The driver features wide range operating from 2V to 24Vand low power consumption by fast switching speed.

## FEATURES

- It is low consumption by BCD process adoption
- Small packages: ESOP8
- Wide power-supply voltage range:
  - Control (VCC): 2.7V~5.5V
  - Motor (VM): 2.0V~24V
- High DC output current: Max.=2.8A
- Ultra low RDSON(TOP+BOT): 0.51Ω TYP@25°C, 1A for ESOP8;
- Low current consumption when power-down: <0.05μA @25°C
- PWM control, Max. input frequency: 200KHz.
- Operating temperature range: -40~+85°C
- Charge-pump less
- Shoot-through current protection
- Built-in protection circuits
  - Under voltage lock out
  - Thermal shut down

## APPLICATIONS

- DC brushed motor
- Auto icemaker or dumper drive for refrigerator
- Intelligent electronic lock

## BLOCK DIAGRAM

```
VCC                 VM
 │                   │
 │                   │
┌┴────────┬──────────┴┐
│ TSD     │           │
│ UVLO    │           │
│ VREF    │           │
├─────────┴──┐  ┌─────┴──────┐
│            │  │            │
│  Control   ├──┤ Pre-Driver ├── OUTA
│   Logic    │  │            │
│            ├──┤            ├── OUTB
├─────────┬──┘  └────────────┘
│ INA     │
│ INB     │
└┬────────┘
 │
GND PGND/Exposed Pad
```
**Note:** GND and PGND/Exposed Pad are connected together internally.

## APPLICATION CIRCUITS
ESOP8

## ORDER INFORMATION
| Valid Part Number | Package Type | Top Code |
| :--- | :--- | :--- |
| BDR6133 | 8pins,ESOP | BDR6133 |

## PIN CONFIGURATION
ESOP8

## PIN DESCRIPTION

| Pin Name    | Pin No. | I/O   | Description                               |
| :---------- | :------ | :---- | :---------------------------------------- |
| NC          | 8       | -     | NC pin                                    |
| GND         | 1       | GND   | Ground                                    |
| VCC         | 2       | Power | Power supply for logic circuit            |
| VM          | 3       | Power | Power supply for driver                   |
| OUTA        | 4       | O     | H-Bridge output terminal A of the driver  |
| OUTB        | 5       | O     | H-Bridge output terminal B of the driver  |
| INA         | 6       | I     | Control input                             |
| INB         | 7       | I     | Control input                             |
| Thermal PAD | 3       | GND   | Power MOS GND                             |

## FUNCTION TABLE

### INPUT-OUTPUT LOGIC TABLE

| INA | INB | OUTA | OUTB | Actuator status |
| :-- | :-- | :--- | :--- | :-------------- |
| L   | L   | Z    | Z    | Stand-by(Stop)  |
| L   | H   | L    | H    | Reverse         |
| H   | L   | H    | L    | Forward         |
| H   | H   | L    | L    | Brake           |

### FUNCTION SEQUENCE
```
VM
VCC
INA
INB
OUTA
OUTB
Standby Reverse Forward Brake Standby
```
**Note:** VM & VCC power on have no timing sequence
VM & VCC power off have no timing sequence

## PROTECTION FUNCTION
### THERMAL SHUTDOWN (TSD) CIRCUIT
The BDR6133 includes a thermal shutdown circuit, which turns the output transistors off when the junction temperature (Tj) exceeds 175°C (typ.).
The output transistors are automatically turned on when Tj cools past the shutdown threshold, which is lowered by a hysteresis of 30°C.
TSD = 175°C
ΔTSD = 30°C
* In thermal shutdown mode, the circuits powered by VCC are work normal, and the circuits powered by VM are shut down.

### UNDER VOLTAGE LOCKOUT (UVLO) CIRCUIT
The BDR6133 includes an under voltage lockout circuit, which puts the output transistors in the high-impedance state when VCC decreases to 2.13V (typ.) or lower.
The output transistors are automatically turned on when VCC increases past the lockout threshold, which is raised to 2.21 V by a hysteresis of 0.08 V.
*In UVLO shutdown mode, a part of circuits powered by VCC are work normal, and the circuits powered by VM are shut down.

### SHOOT-THROUGH CURRENT PROTECTION
During Dead Time (Shoot through current circuit is operated.), Power MOS both of HI side and Low side are turned off. But in this time, internal parasitic diode is turned on according to current direction.

## ABSOLUTE MAXIMUM RATINGS

| Parameter                     | Symbol        | Min   | Max   | Unit | Note  |
| :---------------------------- | :------------ | :---- | :---- | :--- | :---- |
| Supply voltage VCC            | VCC           | -0.5  | 6     | V    |       |
| Control input voltage         | INA/INB       | -0.5  | 6     | V    |       |
| Supply voltage VM             | VM            | -0.5  | 26    | V    |       |
| H-Bridge output current DC    | Iload_dc_MD   |       | 2.8   | A    | Note1 |
| H-Bridge output current AC    | Iload_peak_MD |       | 4.8   | A    | Note2 |
| Continuous power dissipation  | Pd Ta=25℃     |       | 3     | W    | Note4 |
|                               | Pd Ta=85℃     |       | 1.6   | W    |       |
| Operation temperature         | Ta            | -40   | 85    | ℃    |       |
| Junction temperature          | Tj            | -40   | 150   | ℃    |       |
| Storage temperature           | Tstg          | -40   | 150   | ℃    |       |
| Minimum ESD rating(HBM)       | Vesd          | 2000  | -     | V    |       |
| Minimum ESD rating(MM)        | Vesd          | 200   | -     | V    |       |

**Notes:**
1. Terminal OUTA,OUTB pulse with =<200ms:Duty 5%
2. Terminal OUTA,OUTB pulse with =<200ms:Duty 1%
3. Maximum power dissipation is a function of TJ(max), Rja, and TA. The maximum allowable power dissipation at any allowable ambient temperature is PD = (TJ(max) − TA)/Rja. Operating at the absolute maximum TJ of 150°C can affect reliability.
4. The package thermal impedance for ESOP8 is calculated in accordance with JEDEC, 2S2P test PCB, Rja=41℃/W

## RECOMMENDED OPERATION CONDITIONS

| Parameter                               | Symbol | Min   | Typ.      | Max   | Unit |
| :-------------------------------------- | :----- | :---- | :-------- | :---- | :--- |
| Supply voltage VCC                      | VCC    | 2.7   | 3.3       | 5.5   | V    |
| Control input voltage                   | INA/INB| 1.62  | 1.8/3.3   | VCC   | V    |
| Supply voltage VM                       | VM     | 2     | -         | 24    | V    |
| Logic input frequency                   | Fin    | 0     |           | 200   | KHz  |
| Logic input duty for frequency=200KHz   | Duty   | 6%    | -         | 94%   | %    |

## ELECTRICAL CHARACTERISTICS
(Unless otherwise specified, Ta=25℃, VCC=3.3V, VM=7.4V)

| Parameter                        | Symbol      | Conditions                  | Min.     | Typ.    | Max.      | Unit |
| :------------------------------- | :---------- | :-------------------------- | :------- | :------ | :-------- | :--- |
| VCC UVLO                         | VDET1       |                             | 1.90     | 2.13    | 2.50      | V    |
| Thermal shut down temperature    | TDET        |                             |          | 175     | -         | ℃    |
| Hysteresis                       | TDETHYS     |                             |          | 30      |           | ℃    |
| VM standby current1              | IVM_NOPOW   | VCC=L                       |          | 0.005   | 0.05      | μA   |
| VM standby current2              | IVM_STBY    | INA=INB=L                   |          | 0.005   | 0.05      | μA   |
| VCC work current                 | IVCC_ WORK  | INA=H, INB=L                |          | 130     | 300       | μA   |
| Operation circuit current        | IVCC_PWM    | INA=200KHz,INB=H            |          | 0.38    | 0.8       | mA   |
| Output on-resistance 1           | RON1        | VCC=3.3V,IOUT=100mA         |          | 0.25    | 0.27      | Ω    |
| Output on-resistance 2           | RON2        | VCC=3.3V,IOUT=1.0A          |          | 0.255   | 0.29      | Ω    |
| Output on-resistance 3           | RON3        | VCC=3.3V,IOUT=1.0A          |          | 0.295   | 0.35      | Ω    |
| Diode forward voltage            | VF_MD       | IF=100mA                    |          | 0.7     | 1.2       | V    |
| H level input voltage(INA, INB)  | VIH         |                             | 0.7xVCC  | -       |           | V    |
| L level input voltage(INA, INB)  | VIL         |                             |          |         | 0.3xVCC   | V    |
| H level input current(INA, INB)  | IIH1        |                             |          |         | 1         | μA   |
| L level input current(INA, INB)  | IIL1        |                             |          |         | 1         | μA   |
| **Full Swing (Forward -> Reverse, Fig.1)** | | VCC=3.3V,VM=7.4V, IOUT=500mA | | | | |
| Turn on time 1                   | TfONH       |                             |          | 0.42    | 1.0       | μs   |
| Turn off time 1                  | TfOFFH      |                             |          | 0.11    | 0.5       | μs   |
| Output rise time 1               | Tfr         |                             |          | 0.09    | 1.0       | μs   |
| Output fall time 1               | Tff         |                             |          | 0.04    | 0.5       | μs   |
| **Full Swing (Reverse -> Forward, Fig.1)** | | VCC=3.3V,VM=7.4V, IOUT=500mA | | | | |
| Turn on time 2                   | TrONH       |                             |          | 0.38    | 1.0       | μs   |
| Turn off time 2                  | TrOFFH      |                             |          | 0.11    | 0.5       | μs   |
| Output rise time 2               | Trr         |                             |          | 0.09    | 1.0       | μs   |
| Output fall time 2               | Trf         |                             |          | 0.04    | 0.5       | μs   |
| **Full Swing (STBY -> Fwd/Rev, Fig.2)** | | VCC=3.3V,VM=7.4V, IOUT=500mA | | | | |
| Turn on time 1                   | TfONH       |                             |          | 2.10    | 10        | μs   |
| Output rise time 1               | Tfr         |                             |          | 0.09    | 1.0       | μs   |
| **Full Swing (Fwd/Rev -> STBY, Fig.2)** | | VCC=3.3V,VM=7.4V, IOUT=500mA | | | | |
| Turn off time 1                  | TfOFFH      |                             |          | 0.11    | 0.5       | μs   |
| Output fall time 1               | Tff         |                             |          | 0.04    | 0.5       | μs   |

Note: OUTA and OUTB are Hi-Z (off state) at thermal shut down.

## SWITCHING CHARACTERISTICS WAVEFORM
### SWITCHING WAVEFORM
```
100%                 100%
INA                  50%                  50%
100%                 100%
INB
Electric current in the OUTA—OUTB
direction is being made (+)=forward
rotation
50%                  50%
TfONH                TrONH
TfOFFH
DRIVER
100%                 100%
90%                  90%
TrOFFH
50%                  50%
10%                  10%
-10%                 -10%
-50%                 -50%
Tff
-90%                 -90%
-100%                -100%
Trr
Tfr                  Trf
```
Fig.1 switching characteristics waveform
```
100%                 100%
INA                  50%                  50%
Electric current in the OUTA—OUTB
direction is being made (+)=forward
rotation
INB
TfONH
TrONH
TfOFFH
DRIVER
100%                 100%
90%                  90%
50%                  50%
10%                  10%
Tff
Trr
```
Fig.2 switching characteristics waveform

## PCBLAYOUT
8-PIN, ESOP

## PACKAGE INFORMATION
**8-PIN, ESOP**

| Symbol | Min. | Nom.       | Max. |
| :----- | :--- | :--------- | :--- |
| A      |      | 1.27 BSC   | 1.70 |
| A1     | 0.00 |            | 0.15 |
| A2     | 1.25 |            |      |
| b      | 0.31 |            | 0.51 |
| c      | 0.17 |            | 0.25 |
| e      |      | 4.90 BSC   |      |
| D      |      | 3.3        |      |
| D1     | 3.1  |            | 3.5  |
| E      |      | 6.00 BSC   |      |
| E1     |      | 3.90 BSC   |      |
| E2     | 2.2  | 2.4        | 2.6  |
| L      | 0.40 | 1.04 REF   | 1.27 |
| L1     |      | -          |      |
| θ      | 0°   |            | 8°   |

**Notes:**
1. Refer to JEDEC MS-012 BA
2. All dimensions are in millimeter.
3. D1 and E2 refer to supplier spec. The JEDEC MS-012BA classify D1 and E2 minimum value are 1.5mm and 1.0mm.

## IMPORTANT NOTICE
Shenzhen Bardeen Microelectronics(BDM) CO.,LTD reserves the right to make corrections, modifications, enhancements, improvements, and other changes to its products and to discontinue any product without notice at any time.

BDM cannot assume responsibility for use of any circuitry other than circuitry entirely embodied in a BDM product. No circuit patent licenses are implied.

Shenzhen Bardeen Microelectronics(BDM) CO.,LTD.
208-209,Building No.1 Yoho Space QunHui Road No.1,Xin’an Street, Bao’an District ,ShenZhen
Tel: 86-755-23505821
http://www.bdasic.com

V1.3, 2018
