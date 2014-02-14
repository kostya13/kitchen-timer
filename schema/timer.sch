EESchema Schematic File Version 2
LIBS:power
LIBS:mixture
LIBS:digital_IC
LIBS:analog_IC
LIBS:conn
LIBS:gost
LIBS:device
LIBS:elements
LIBS:timer-cache
EELAYER 27 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date "14 feb 2014"
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L SPEAKER SP1
U 1 1 52F14041
P 8350 2400
F 0 "SP1" H 8350 2150 70  0000 C CNN
F 1 "SPEAKER" H 8250 2150 70  0001 C CNN
F 2 "" H 8350 2400 60  0001 C CNN
F 3 "" H 8350 2400 60  0001 C CNN
	1    8350 2400
	1    0    0    1   
$EndComp
$Comp
L VT_NPN_BCE VT1
U 1 1 52F13E6A
P 4250 2100
F 0 "VT1" V 4450 2150 60  0000 R CNN
F 1 "VT_NPN_BCE" V 4300 1900 60  0001 R CNN
F 2 "" H 4250 2100 60  0001 C CNN
F 3 "" H 4250 2100 60  0001 C CNN
	1    4250 2100
	0    -1   -1   0   
$EndComp
$Comp
L POWER_GND #01
U 1 1 52F13E15
P 8100 3900
F 0 "#01" V 8050 3850 60  0001 C CNN
F 1 "POWER_GND" V 8150 3850 60  0001 C CNN
F 2 "" H 8100 3900 60  0001 C CNN
F 3 "" H 8100 3900 60  0001 C CNN
	1    8100 3900
	1    0    0    -1  
$EndComp
$Comp
L BAT GB1
U 1 1 52F13DFB
P 8950 2750
F 0 "GB1" H 8975 2625 60  0000 L CNN
F 1 "3V" H 8975 2425 60  0000 L CNN
F 2 "" H 8950 2750 60  0001 C CNN
F 3 "" H 8950 2750 60  0001 C CNN
	1    8950 2750
	1    0    0    -1  
$EndComp
$Comp
L SA_1-2_CL SA1
U 1 1 52F13A0C
P 8950 2300
F 0 "SA1" V 8950 2200 59  0000 C CNN
F 1 "SA_1-2_CL" H 8950 2200 60  0001 C CNN
F 2 "" H 8950 2300 60  0001 C CNN
F 3 "" H 8950 2300 60  0001 C CNN
	1    8950 2300
	0    -1   -1   0   
$EndComp
$Comp
L HG_LED_7SEG_2_1 HG1
U 1 1 52F1391F
P 5400 2800
F 0 "HG1" H 5800 3750 60  0000 C CNN
F 1 "DA03-11GWA" H 5800 2050 60  0000 C CNN
F 2 "" H 5400 2800 60  0001 C CNN
F 3 "" H 5400 2800 60  0001 C CNN
	1    5400 2800
	-1   0    0    -1  
$EndComp
$Comp
L R R1
U 1 1 52F1EB00
P 3950 3300
F 0 "R1" V 4030 3300 50  0000 C CNN
F 1 "1к" V 3950 3300 50  0000 C CNN
F 2 "" H 3950 3300 60  0000 C CNN
F 3 "" H 3950 3300 60  0000 C CNN
	1    3950 3300
	-1   0    0    1   
$EndComp
$Comp
L TR_NPN_BCE VT3
U 1 1 52F4AC00
P 7650 3000
F 0 "VT3" H 8050 3250 60  0000 R CNN
F 1 "TR_NPN_BCE" H 8050 2800 60  0001 R CNN
F 2 "" H 7650 3000 60  0000 C CNN
F 3 "" H 7650 3000 60  0000 C CNN
	1    7650 3000
	1    0    0    -1  
$EndComp
$Comp
L R R3
U 1 1 52F4AC56
P 7200 3000
F 0 "R3" V 7280 3000 50  0000 C CNN
F 1 "1к" V 7200 3000 50  0000 C CNN
F 2 "" H 7200 3000 60  0000 C CNN
F 3 "" H 7200 3000 60  0000 C CNN
	1    7200 3000
	0    -1   -1   0   
$EndComp
$Comp
L R R2
U 1 1 52F4BCDC
P 4250 3250
F 0 "R2" V 4330 3250 50  0000 C CNN
F 1 "1к" V 4250 3250 50  0000 C CNN
F 2 "" H 4250 3250 60  0000 C CNN
F 3 "" H 4250 3250 60  0000 C CNN
	1    4250 3250
	-1   0    0    1   
$EndComp
$Comp
L VT_NPN_BCE VT2
U 1 1 52F4C8E0
P 4250 2750
F 0 "VT2" V 4450 2800 60  0000 R CNN
F 1 "VT_NPN_BCE" V 4300 2550 60  0001 R CNN
F 2 "" H 4250 2750 60  0001 C CNN
F 3 "" H 4250 2750 60  0001 C CNN
	1    4250 2750
	0    -1   -1   0   
$EndComp
$Comp
L KEYBOARD Kbd1
U 1 1 52F4CEF6
P 7550 2850
F 0 "Kbd1" H 7400 4150 60  0001 C CNN
F 1 "KEYBOARD" H 7300 4300 60  0001 C CNN
F 2 "" H 7550 2850 60  0000 C CNN
F 3 "" H 7550 2850 60  0000 C CNN
	1    7550 2850
	-1   0    0    -1  
$EndComp
$Comp
L ATTINY2313 DD1
U 1 1 52F138FE
P 5800 2000
F 0 "DD1" H 6300 2150 60  0000 C CNN
F 1 "ATTINY2313" H 6300 50  60  0000 C CNN
F 2 "" H 5800 2000 60  0001 C CNN
F 3 "" H 5800 2000 60  0001 C CNN
	1    5800 2000
	1    0    0    -1  
$EndComp
Wire Wire Line
	6800 3000 6950 3000
Wire Wire Line
	8100 3300 8100 3900
Connection ~ 8100 3800
Wire Wire Line
	6800 3600 6800 3400
Wire Wire Line
	6800 3200 7300 3200
Wire Wire Line
	7300 3200 7300 4150
$Comp
L POWER_VCC #~02
U 1 1 52F4D84D
P 6900 3600
F 0 "#~02" V 6700 3350 60  0001 C CNN
F 1 "POWER_VCC" H 7050 3500 60  0001 C CNN
F 2 "" H 6900 3600 60  0000 C CNN
F 3 "" H 6900 3600 60  0000 C CNN
	1    6900 3600
	1    0    0    -1  
$EndComp
Wire Wire Line
	6900 3600 6800 3600
$Comp
L POWER_GND #03
U 1 1 52F4D89D
P 8950 3350
F 0 "#03" V 8900 3300 60  0001 C CNN
F 1 "POWER_GND" V 9000 3300 60  0001 C CNN
F 2 "" H 8950 3350 60  0001 C CNN
F 3 "" H 8950 3350 60  0001 C CNN
	1    8950 3350
	1    0    0    -1  
$EndComp
$Comp
L POWER_VCC #~04
U 1 1 52F4D8A9
P 8950 1900
F 0 "#~04" V 8750 1650 60  0001 C CNN
F 1 "POWER_VCC" H 9100 1800 60  0001 C CNN
F 2 "" H 8950 1900 60  0000 C CNN
F 3 "" H 8950 1900 60  0000 C CNN
	1    8950 1900
	0    -1   -1   0   
$EndComp
Wire Wire Line
	8950 1850 8950 2100
Connection ~ 8950 1950
Wire Wire Line
	8950 2750 8950 2500
Wire Wire Line
	8950 3200 8950 3350
Wire Wire Line
	5800 2000 5400 2000
Wire Wire Line
	5400 2200 5800 2200
Wire Wire Line
	5800 2400 5400 2400
Wire Wire Line
	5400 2600 5800 2600
Wire Wire Line
	5800 2800 5400 2800
Wire Wire Line
	5400 3000 5800 3000
Wire Wire Line
	5800 3200 5400 3200
Wire Wire Line
	4600 2650 4400 2650
Wire Wire Line
	4250 2300 3950 2300
Wire Wire Line
	4050 2650 4100 2650
Wire Wire Line
	4050 1900 4050 2650
Wire Wire Line
	4100 2000 4050 2000
Connection ~ 4050 2000
$Comp
L POWER_VCC #~05
U 1 1 52F4DE17
P 4050 1900
F 0 "#~05" V 3850 1650 60  0001 C CNN
F 1 "POWER_VCC" H 4200 1800 60  0001 C CNN
F 2 "" H 4050 1900 60  0000 C CNN
F 3 "" H 4050 1900 60  0000 C CNN
	1    4050 1900
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5800 3400 5550 3400
Wire Wire Line
	5550 3400 5550 3800
Wire Wire Line
	4250 3800 4250 3500
Wire Wire Line
	4250 2950 4250 3000
Wire Wire Line
	7300 4150 3950 4150
Wire Wire Line
	3950 4150 3950 3550
Wire Wire Line
	4600 2650 4600 2200
Wire Wire Line
	4600 2000 4400 2000
Wire Wire Line
	5550 3800 4250 3800
Wire Wire Line
	6800 2800 7000 2800
Wire Wire Line
	7000 2600 6800 2600
Wire Wire Line
	6800 2400 7000 2400
Wire Wire Line
	7000 2200 6800 2200
Wire Wire Line
	7300 2100 7300 2000
Wire Wire Line
	7300 2000 6800 2000
Wire Wire Line
	5800 3600 5650 3600
Wire Wire Line
	5650 3600 5650 4400
Wire Wire Line
	5650 4400 7750 4400
Wire Wire Line
	7750 4400 7750 2000
Wire Wire Line
	7750 2000 7400 2000
Wire Wire Line
	7400 2000 7400 2100
Wire Wire Line
	5800 3800 5800 4250
Wire Wire Line
	5800 4250 7650 4250
Wire Wire Line
	7650 4250 7650 2100
Wire Wire Line
	7650 2100 7500 2100
Wire Wire Line
	8100 1950 8950 1950
Wire Wire Line
	3950 2300 3950 3050
Wire Wire Line
	8100 2500 8100 2700
Wire Wire Line
	8100 1950 8100 2300
NoConn ~ 5400 3400
NoConn ~ 7300 2400
$Comp
L POWER_GND #?
U 1 1 52FDA0A6
P 6800 3850
F 0 "#?" V 6750 3800 60  0001 C CNN
F 1 "POWER_GND" V 6850 3800 60  0001 C CNN
F 2 "" H 6800 3850 60  0001 C CNN
F 3 "" H 6800 3850 60  0001 C CNN
	1    6800 3850
	1    0    0    -1  
$EndComp
Wire Wire Line
	6800 3800 6800 3850
Wire Wire Line
	7650 3000 7450 3000
$EndSCHEMATC
