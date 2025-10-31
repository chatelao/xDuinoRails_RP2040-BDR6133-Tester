BDR6133
1CH DC Motor Driver

DESCRIPTION

FEATURES

The BDR6133 is 1 Full-On Drive H-Bridge channel with
two different packages. The driver features wide range
operating from 2V to 24Vand low power consumption
by fast switching speed.

 It is low consumption by BCD process adoption
 Small packages: ESOP8
 Wide power-supply voltage range:
-Control (VCC): 2.7V~5.5V
- Motor (VM): 2.0V~24V
 High DC output current: Max.=2.8A
 Ultra low RDSON(TOP+BOT):
0.51ΩTYP@25°C, 1A for ESOP8;
 Low current consumption when power-down:
<0.05μA @25°C
 PWM control, Max. input frequency: 200KHz.
 Operating temperature range: -40~+85°C
 Charge-pump less
 Shoot-through current protection
 Built-in protection circuits
- Under voltage lock out
 - Thermal shut down

APPLICATIONS
 DC brushed motor
 Auto icemaker or dumper drive for refrigerator
 Intelligent electronic lock

BLOCK DIAGRAM
VCC

VM

TSD
UVLO
VREF

INA
INB

OUTA
Control
Logic

GND

Pre
Driver

OUTB

PGND/Exposed Pad

Note: GND and PGND/Exposed Pad are connected together internally.
Tel: 0755-23505821

Web: http://www.bdasic.com

BDR6133

APPLICATION CIRCUITS
ESOP8

ORDER INFORMATION
Valid Part Number

Package Type

BDR6133

8pins,ESOP

Top Code
BDR6133

PIN CONFIGURATION
ESOP8

V1.3

2

2018

BDR6133

PIN DESCRIPTION

V1.3

Pin Name

I/O

Description

NC

-

NC pin

8

GND

GND

Ground

1

VCC

Power

Power supply for logic circuit

2

VM

Power

Power supply for driver

3

OUTA

O

H-Bridge output terminal A of the driver

4

OUTB

O

H-Bridge output terminal B of the driver

5

INA

I

Control input

6

INB

I

Control input

7

PGND

GND

Power MOS GND

3

Pin No.

Thermal PAD

2018

BDR6133

FUNCTION TABLE
INPUT-OUTPUT LOGIC TABLE
Input Signal
INA
INB
L
L
L
H
H
L
H
H

Output Driver
OUTA
OUTB
Z
Z
L
H
H
L
L
L

Actuator status

Stand-by(Stop)
Reverse
Forward
Brake

FUNCTION SEQUENCE

VM

VCC

INA

INB

OUTA

OUTB
Standby Reverse Forward

Brake

Standby

Note: VM & VCC power on have no timing sequence
VM & VCC power off have no timing sequence

V1.3

4

2018

BDR6133

PROTECTION FUNCTION
THERMAL SHUTDOWN (TSD) CIRCUIT
The BDR6133 includes a thermal shutdown circuit, which turns the output transistors off when the junction temperature
(Tj) exceeds 175°C (typ.).
The output transistors are automatically turned on when Tj cools past the shutdown threshold, which is lowered by a
hysteresis of 30°C.
TSD = 175°C
ΔTSD = 30°C
* In thermal shutdown mode, the circuits powered by VCC are work normal, and the circuits powered by VM are shut
down.

UNDER VOLTAGE LOCKOUT (UVLO) CIRCUIT
The BDR6133 includes an under voltage lockout circuit, which puts the output transistors in the high-impedance state
when VCC decreases to 2.13V (typ.) or lower.
The output transistors are automatically turned on when VCC increases past the lockout threshold, which is raised to
2.21 V by a hysteresis of 0.08 V.
*In UVLO shutdown mode, a part of circuits powered by VCC are work normal, and the circuits powered by VM are shut
down.

SHOOT-THROUGH CURRENT PROTECTION
During Dead Time (Shoot through current circuit is operated.), Power MOS both of HI side and Low side are turned off.
But in this time, internal parasitic diode is turned on according to current direction.

V1.3

5

2018

BDR6133

ABSOLUTE MAXIMUM RATINGS
Parameter
Supply voltage VCC
Control input voltage
Supply voltage VM
H-Bridge output current DC

Symbol
VCC
INA/INB
VM
Iload_dc_MD

H-Bridge output current AC

Iload_peak_MD

Continuous power dissipation

Pd Ta=25℃
Pd Ta=85℃

Operation temperature
Junction temperature
Storage temperature
Minimum ESD rating(HBM)
Minimum ESD rating(MM)

Ta
Tj
Tstg
Vesd
Vesd

Min
-0.5
-0.5
-0.5
-40
-40
2000
200

Max
6
6
26
2.8
4.8
7.5
3
1.6
85
150
150
-

Unit
V
V
V
A
A
A
W
W
℃
℃
℃

Note

Note1
Note2
Note4

V
V

Notes:
1.

Terminal OUTA,OUTB pulse with =<200ms:Duty 5%

2.

Terminal OUTA,OUTB pulse with =<200ms:Duty 1%

3.

Maximum power dissipation is a function of TJ(max), Rja, and TA. The maximum allowable power dissipation at any allowable ambient
temperature is PD = (TJ(max) − TA)/Rja. Operating at the absolute maximum TJ of 150°C can affect reliability.

4.

The package thermal impedance for ESOP8 is calculated in accordance with JEDEC, 2S2P test PCB, Rja=41℃/W

RECOMMENDED OPERATION CONDITIONS
Parameter
Supply voltage VCC
Control input voltage
Supply voltage VM
Logic input frequency
Logic input duty for frequency=200KHz
(Ta=25℃, VCC=3.3V,VM=12V, Rload=50Ω,
Output state: Forward↔Reverse)

V1.3

Symbol
VCC
INA/INB
VM
Fin

Min
2.7
1.62
2
0

Typ.
3.3
1.8/3.3
-

Max
5.5
VCC
24
200

Unit
V
V
V
KHz

Duty

6%

-

94%

%

6

2018

BDR6133

ELECTRICAL CHARACTERISTICS
(Unless otherwise specified, Ta=25℃, VCC=3.3V, VM=7.4V)
Parameter
Symbol
Conditions
Min.
VDET1
VCC UVLO
VCDET_LV
1.90
TSD (Note)
Thermal shut down temperature
TDET
Hysteresis
TDETHYS
Power Supply Current
VM standby current1
IVM_NOPOW
VCC=L
VM standby current2
IVM_STBY
INA=INB=L
VCC work current
IVCC_ WORK
INA=H, INB=L
Operation circuit current
IVCC_PWM
INA=200KHz,INB=H
Driver
VCC=3.3V,IOUT=100mA
Output onresistance 1
RON1
(HSD or LSD)
Ta=25℃
Output onresistance 2
VCC=3.3V,IOUT=1.0A
RON2
(HSD or LSD)
Ta=25℃(Tj=65℃)
VCC=3.3V,IOUT=1.0A
Output onresistance 3
RON3
(HSD or LSD)
Ta=85℃(Tj=125℃)
Diode forward voltage
VF_MD
IF=100mA
Control Terminal
H level input voltage(INA, INB)
VIH
0.7xVCC
L level input voltage(INA, INB)
VIL
H level input current(INA, INB)
IIH1
L level input current(INA, INB)
IIL1
Full Swing
VCC=3.3V,VM=7.4V
Turn on time 1
TfONH
IOUT=500mA,
Turn off time 1
TfOFFH
Output state:
Output rise time 1
Tfr
ForwardReverse.
Output fall time 1
Tff
Refer to Fig.1
Turn on time 2
TrONH
VCC=3.3V,VM=7.4V
I
=500mA,
OUT
Turn off time 2
TrOFFH
Output state:
Output rise time 2
Trr
ReverseForward.
Output fall time 2
Trf
Refer to Fig.1
VCC=3.3V,VM=7.4V
Turn on time 1
TfONH
IOUT=500mA,
Output state:
STBYForward/Reverse.
Output rise time 1
Tfr
Refer to Fig.2
VCC=3.3V,VM=7.4V
Turn off time 1
TfOFFH
IOUT=500mA,
Output state:
Forward/ReverseSTBY
Output fall time 1
Tff
Refer to Fig.2

Typ.

Max.

Unit

2.13

2.50

V

175
30

-

℃
℃

0.005
0.005
130
0.38

0.05
0.05
300
0.8

μA
μA
μA
mA

0.25

0.27

Ω

0.255

0.29

Ω

0.295

0.35

Ω

0.7

1.2

V

-

0.3xVCC
1
1

V
V
μA
μA

0.42
0.11
0.09
0.04
0.38
0.11

1.0
0.5
1.0
0.5
1.0
0.5

μs
μs
μs
μs
μs
μs

0.09

1.0

μs

0.04

0.5

μs

2.10

10

μs

0.09

1.0

μs

0.11

0.5

μs

0.04

0.5

μs

Note: OUTA and OUTB are Hi-Z (off state) at thermal shut down.

V1.3

7

2018

BDR6133

SWITCHING CHARACTERISTICS WAVEFORM
SWITCHING WAVEFORM
100%

100%

INA

50%

50%

100%

100%

INB

Electric current in the OUTA—OUTB
direction is being made (+)=forward
rotation

50%

50%
TfONH

TrONH
TfOFFH
DRIVER

100%
90%

TrOFFH

50%

50%
10%

100%
90%

10%
-10%
-50%

Tff

-10%
-50%

-90%
-100%

Trr

-90%
-100%

Tfr

Trf

Fig.1 switching characteristics waveform
100%

100%

INA

50%

50%

Electric current in the OUTA—OUTB
direction is being made (+)=forward
rotation

INB
TfONH
TrONH
TfOFFH
DRIVER

100%
90%

50%

50%
10%

100%
90%

10%

Tff

Trr

Fig.2 switching characteristics waveform

V1.3

8

2018

BDR6133

PCBLAYOUT
8-PIN, ESOP

V1.3

9

2018

BDR6133

PACKAGE INFORMATION
8-PIN, ESOP

Symbol
A
A1
A2
b
c
e
D
D1
E
E1
E2
L
L1
θ

Dimensions(mm)
Nom.
1.27 BSC
4.90 BSC
3.3
6.00 BSC
3.90 BSC
2.4
1.04 REF
-

Min.
0.00
1.25
0.31
0.17

3.1

2.2
0.40
0°

Max.
1.70
0.15
0.51
0.25

3.5

2.6
1.27
8°

Notes:
1. Refer to JEDEC MS-012 BA
2. All dimensions are in millimeter.
3. D1 and E2 refer to supplier spec. The JEDEC MS-012BA classify D1 and E2 minimum value are 1.5mm and 1.0mm.

V1.3

10

2018

BDR6133
IMPORTANT NOTICE
Shenzhen Bardeen Microelectronics(BDM) CO.,LTD
reserves the right to make corrections, modifications,
enhancements, improvements, and other changes to its products and to discontinue any product without notice at any
time.
BDM cannot assume responsibility for use of any circuitry other than circuitry entirely embodied in a BDM product. No
circuit patent licenses are implied.
Shenzhen Bardeen Microelectronics(BDM) CO.,LTD.
208-209,Building No.1 Yoho Space QunHui Road No.1,Xin’an Street, Bao’an District ,ShenZhen
Tel: 86-755-23505821
http://www.bdasic.com

V1.3

11

2018


