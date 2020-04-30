EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "Z80 computer 2"
Date "2020-04-29"
Rev ""
Comp ""
Comment1 "License: Creative Commons Attribution Share-Alike (CC BY-SA)"
Comment2 "Copyright (C) 2020 John Tsiombikas <nuclear@member.fsf.org>"
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L CPU:Z80CPU U3
U 1 1 5EA29512
P 2000 4050
F 0 "U3" H 2200 2650 50  0000 C CNN
F 1 "Z80CPU" H 2000 4050 50  0000 C CNN
F 2 "" H 2000 4450 50  0001 C CNN
F 3 "www.zilog.com/manage_directlink.php?filepath=docs/z80/um0080" H 2000 4450 50  0001 C CNN
	1    2000 4050
	1    0    0    -1  
$EndComp
$Comp
L Memory_EEPROM:28C256 U6
U 1 1 5EA2C09A
P 3850 4400
F 0 "U6" H 4000 3350 50  0000 C CNN
F 1 "28C256" V 3850 4400 50  0000 C CNN
F 2 "" H 3850 4400 50  0001 C CNN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/doc0006.pdf" H 3850 4400 50  0001 C CNN
	1    3850 4400
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR08
U 1 1 5EA2ECD7
P 2000 5550
F 0 "#PWR08" H 2000 5300 50  0001 C CNN
F 1 "GND" H 2005 5377 50  0000 C CNN
F 2 "" H 2000 5550 50  0001 C CNN
F 3 "" H 2000 5550 50  0001 C CNN
	1    2000 5550
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR016
U 1 1 5EA2F4C0
P 3850 5500
F 0 "#PWR016" H 3850 5250 50  0001 C CNN
F 1 "GND" H 3855 5327 50  0000 C CNN
F 2 "" H 3850 5500 50  0001 C CNN
F 3 "" H 3850 5500 50  0001 C CNN
	1    3850 5500
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR07
U 1 1 5EA2FD95
P 2000 2550
F 0 "#PWR07" H 2000 2400 50  0001 C CNN
F 1 "VCC" H 2017 2723 50  0000 C CNN
F 2 "" H 2000 2550 50  0001 C CNN
F 3 "" H 2000 2550 50  0001 C CNN
	1    2000 2550
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR015
U 1 1 5EA30021
P 3850 3300
F 0 "#PWR015" H 3850 3150 50  0001 C CNN
F 1 "VCC" H 3867 3473 50  0000 C CNN
F 2 "" H 3850 3300 50  0001 C CNN
F 3 "" H 3850 3300 50  0001 C CNN
	1    3850 3300
	1    0    0    -1  
$EndComp
Wire Wire Line
	2700 4550 2900 4550
Wire Wire Line
	2700 4650 2900 4650
Wire Wire Line
	2700 4750 2900 4750
Wire Wire Line
	2700 4850 2900 4850
Wire Wire Line
	2700 4950 2900 4950
Wire Wire Line
	2700 5050 2900 5050
Wire Wire Line
	2700 5150 2900 5150
Wire Wire Line
	2700 5250 2900 5250
Text Label 2750 4550 0    50   ~ 0
D0
Text Label 2750 4650 0    50   ~ 0
D1
Text Label 2750 4750 0    50   ~ 0
D2
Text Label 2750 4850 0    50   ~ 0
D3
Text Label 2750 4950 0    50   ~ 0
D4
Text Label 2750 5050 0    50   ~ 0
D5
Text Label 2750 5150 0    50   ~ 0
D6
Text Label 2750 5250 0    50   ~ 0
D7
Entry Wire Line
	2900 4550 3000 4650
Entry Wire Line
	2900 4650 3000 4750
Entry Wire Line
	2900 4750 3000 4850
Entry Wire Line
	2900 4850 3000 4950
Entry Wire Line
	2900 4950 3000 5050
Entry Wire Line
	2900 5050 3000 5150
Entry Wire Line
	2900 5150 3000 5250
Entry Wire Line
	2900 5250 3000 5350
Wire Wire Line
	4250 3500 4450 3500
Wire Wire Line
	4250 3600 4450 3600
Wire Wire Line
	4250 3700 4450 3700
Wire Wire Line
	4250 3800 4450 3800
Wire Wire Line
	4250 3900 4450 3900
Wire Wire Line
	4250 4000 4450 4000
Wire Wire Line
	4250 4100 4450 4100
Wire Wire Line
	4250 4200 4450 4200
Text Label 4300 3500 0    50   ~ 0
D0
Text Label 4300 3600 0    50   ~ 0
D1
Text Label 4300 3700 0    50   ~ 0
D2
Text Label 4300 3800 0    50   ~ 0
D3
Text Label 4300 3900 0    50   ~ 0
D4
Text Label 4300 4000 0    50   ~ 0
D5
Text Label 4300 4100 0    50   ~ 0
D6
Text Label 4300 4200 0    50   ~ 0
D7
Entry Wire Line
	4450 3500 4550 3600
Entry Wire Line
	4450 3600 4550 3700
Entry Wire Line
	4450 3700 4550 3800
Entry Wire Line
	4450 3800 4550 3900
Entry Wire Line
	4450 3900 4550 4000
Entry Wire Line
	4450 4000 4550 4100
Entry Wire Line
	4450 4100 4550 4200
Entry Wire Line
	4450 4200 4550 4300
Wire Wire Line
	2700 2850 2900 2850
Wire Wire Line
	2700 2950 2900 2950
Wire Wire Line
	2700 3050 2900 3050
Wire Wire Line
	2700 3150 2900 3150
Wire Wire Line
	2700 3250 2900 3250
Wire Wire Line
	2700 3350 2900 3350
Wire Wire Line
	2700 3450 2900 3450
Wire Wire Line
	2700 3550 2900 3550
Wire Wire Line
	2700 3650 2900 3650
Wire Wire Line
	2700 3750 2900 3750
Wire Wire Line
	2700 3850 2900 3850
Wire Wire Line
	2700 3950 2900 3950
Wire Wire Line
	2700 4050 2900 4050
Wire Wire Line
	2700 4150 2900 4150
Wire Wire Line
	2700 4250 2900 4250
Wire Wire Line
	2700 4350 2900 4350
Text Label 2750 2850 0    50   ~ 0
A0
Text Label 2750 2950 0    50   ~ 0
A1
Text Label 2750 3050 0    50   ~ 0
A2
Text Label 2750 3150 0    50   ~ 0
A3
Text Label 2750 3250 0    50   ~ 0
A4
Text Label 2750 3350 0    50   ~ 0
A5
Text Label 2750 3450 0    50   ~ 0
A6
Text Label 2750 3550 0    50   ~ 0
A7
Text Label 2750 3650 0    50   ~ 0
A8
Text Label 2750 3750 0    50   ~ 0
A9
Text Label 2750 3850 0    50   ~ 0
A10
Text Label 2750 3950 0    50   ~ 0
A11
Text Label 2750 4050 0    50   ~ 0
A12
Text Label 2750 4150 0    50   ~ 0
A13
Text Label 2750 4250 0    50   ~ 0
A14
Text Label 2750 4350 0    50   ~ 0
A15
Entry Wire Line
	2900 4350 3000 4250
Entry Wire Line
	2900 4250 3000 4150
Entry Wire Line
	2900 4150 3000 4050
Entry Wire Line
	2900 4050 3000 3950
Entry Wire Line
	2900 3950 3000 3850
Entry Wire Line
	2900 3850 3000 3750
Entry Wire Line
	2900 3750 3000 3650
Entry Wire Line
	2900 3650 3000 3550
Entry Wire Line
	2900 3550 3000 3450
Entry Wire Line
	2900 3450 3000 3350
Entry Wire Line
	2900 3350 3000 3250
Entry Wire Line
	2900 3250 3000 3150
Entry Wire Line
	2900 3150 3000 3050
Entry Wire Line
	2900 3050 3000 2950
Entry Wire Line
	2900 2950 3000 2850
Entry Wire Line
	2900 2850 3000 2750
Text Label 3450 5850 0    50   ~ 0
D[0..7]
Wire Wire Line
	3450 3500 3250 3500
Wire Wire Line
	3450 3600 3250 3600
Wire Wire Line
	3450 3700 3250 3700
Wire Wire Line
	3450 3800 3250 3800
Wire Wire Line
	3450 3900 3250 3900
Wire Wire Line
	3450 4000 3250 4000
Wire Wire Line
	3450 4100 3250 4100
Wire Wire Line
	3450 4200 3250 4200
Wire Wire Line
	3450 4300 3250 4300
Wire Wire Line
	3450 4400 3250 4400
Wire Wire Line
	3450 4500 3250 4500
Wire Wire Line
	3450 4600 3250 4600
Wire Wire Line
	3450 4700 3250 4700
Wire Wire Line
	3450 4800 3250 4800
Wire Wire Line
	3450 4900 3250 4900
Text Label 3300 3500 0    50   ~ 0
A0
Text Label 3300 3600 0    50   ~ 0
A1
Text Label 3300 3700 0    50   ~ 0
A2
Text Label 3300 3800 0    50   ~ 0
A3
Text Label 3300 3900 0    50   ~ 0
A4
Text Label 3300 4000 0    50   ~ 0
A5
Text Label 3300 4100 0    50   ~ 0
A6
Text Label 3300 4200 0    50   ~ 0
A7
Text Label 3300 4300 0    50   ~ 0
A8
Text Label 3300 4400 0    50   ~ 0
A9
Text Label 3300 4500 0    50   ~ 0
A10
Text Label 3300 4600 0    50   ~ 0
A11
Text Label 3300 4700 0    50   ~ 0
A12
Text Label 3300 4800 0    50   ~ 0
A13
Text Label 3300 4900 0    50   ~ 0
A14
Entry Wire Line
	3150 3400 3250 3500
Entry Wire Line
	3150 3500 3250 3600
Entry Wire Line
	3150 3600 3250 3700
Entry Wire Line
	3150 3700 3250 3800
Entry Wire Line
	3150 3800 3250 3900
Entry Wire Line
	3150 3900 3250 4000
Entry Wire Line
	3150 4000 3250 4100
Entry Wire Line
	3150 4100 3250 4200
Entry Wire Line
	3150 4200 3250 4300
Entry Wire Line
	3150 4300 3250 4400
Entry Wire Line
	3150 4400 3250 4500
Entry Wire Line
	3150 4500 3250 4600
Entry Wire Line
	3150 4600 3250 4700
Entry Wire Line
	3150 4700 3250 4800
Entry Wire Line
	3150 4800 3250 4900
Text Label 4250 2600 0    50   ~ 0
A[0..15]
$Comp
L power:VCC #PWR012
U 1 1 5EA9AA1C
P 3350 5100
F 0 "#PWR012" H 3350 4950 50  0001 C CNN
F 1 "VCC" H 3450 5200 50  0000 C CNN
F 2 "" H 3350 5100 50  0001 C CNN
F 3 "" H 3350 5100 50  0001 C CNN
	1    3350 5100
	1    0    0    -1  
$EndComp
Wire Wire Line
	3450 5100 3350 5100
Wire Bus Line
	3000 2600 3150 2600
Connection ~ 3150 2600
Wire Wire Line
	3450 5300 3150 5300
Text Label 3150 5300 0    50   ~ 0
~ROMEN
Wire Wire Line
	3450 5200 3300 5200
Text Label 3300 5200 0    50   ~ 0
~RD
Wire Wire Line
	1300 4550 1000 4550
Wire Wire Line
	1300 4650 1000 4650
Wire Wire Line
	1300 4750 1000 4750
Wire Wire Line
	1300 4850 1000 4850
Text Label 1150 4550 0    50   ~ 0
~RD
Text Label 1150 4650 0    50   ~ 0
~WR
Text Label 1050 4750 0    50   ~ 0
~MREQ
Text Label 1050 4850 0    50   ~ 0
~IOREQ
Text Label 1150 2850 0    50   ~ 0
~RST
$Comp
L z80comp2:mc68681 U8
U 1 1 5EAC419A
P 7350 3900
F 0 "U8" H 7500 2350 50  0000 C CNN
F 1 "mc68681" V 7350 3900 50  0000 C CNN
F 2 "" H 7900 3350 50  0001 C CNN
F 3 "" H 7900 3350 50  0001 C CNN
	1    7350 3900
	1    0    0    -1  
$EndComp
$Comp
L z80comp2:sram_um61512ak U7
U 1 1 5EAC72ED
P 5550 4300
F 0 "U7" H 5700 3150 60  0000 C CNN
F 1 "sram_um61512ak" V 5550 4300 60  0000 C CNN
F 2 "" H 5450 3500 60  0000 C CNN
F 3 "" H 5450 3500 60  0000 C CNN
	1    5550 4300
	1    0    0    -1  
$EndComp
Wire Wire Line
	6100 3300 6300 3300
Wire Wire Line
	6100 3400 6300 3400
Wire Wire Line
	6100 3500 6300 3500
Wire Wire Line
	6100 3600 6300 3600
Wire Wire Line
	6100 3700 6300 3700
Wire Wire Line
	6100 3800 6300 3800
Wire Wire Line
	6100 3900 6300 3900
Wire Wire Line
	6100 4000 6300 4000
Text Label 6150 3300 0    50   ~ 0
D0
Text Label 6150 3400 0    50   ~ 0
D1
Text Label 6150 3500 0    50   ~ 0
D2
Text Label 6150 3600 0    50   ~ 0
D3
Text Label 6150 3700 0    50   ~ 0
D4
Text Label 6150 3800 0    50   ~ 0
D5
Text Label 6150 3900 0    50   ~ 0
D6
Text Label 6150 4000 0    50   ~ 0
D7
Entry Wire Line
	6300 3300 6400 3400
Entry Wire Line
	6300 3400 6400 3500
Entry Wire Line
	6300 3500 6400 3600
Entry Wire Line
	6300 3600 6400 3700
Entry Wire Line
	6300 3700 6400 3800
Entry Wire Line
	6300 3800 6400 3900
Entry Wire Line
	6300 3900 6400 4000
Entry Wire Line
	6300 4000 6400 4100
Wire Wire Line
	6500 3500 6800 3500
Wire Wire Line
	6500 3600 6800 3600
Wire Wire Line
	6500 3700 6800 3700
Wire Wire Line
	6500 3800 6800 3800
Wire Wire Line
	6500 3900 6800 3900
Wire Wire Line
	6500 4000 6800 4000
Wire Wire Line
	6500 4100 6800 4100
Wire Wire Line
	6500 4200 6800 4200
Text Label 6650 3500 0    50   ~ 0
D0
Text Label 6650 3600 0    50   ~ 0
D1
Text Label 6650 3700 0    50   ~ 0
D2
Text Label 6650 3800 0    50   ~ 0
D3
Text Label 6650 3900 0    50   ~ 0
D4
Text Label 6650 4000 0    50   ~ 0
D5
Text Label 6650 4100 0    50   ~ 0
D6
Text Label 6650 4200 0    50   ~ 0
D7
Wire Wire Line
	5000 4800 4650 4800
Text Label 4650 4800 0    50   ~ 0
BANKSEL
Wire Wire Line
	5000 3300 4800 3300
Wire Wire Line
	5000 3400 4800 3400
Wire Wire Line
	5000 3500 4800 3500
Wire Wire Line
	5000 3600 4800 3600
Wire Wire Line
	5000 3700 4800 3700
Wire Wire Line
	5000 3800 4800 3800
Wire Wire Line
	5000 3900 4800 3900
Wire Wire Line
	5000 4000 4800 4000
Wire Wire Line
	5000 4100 4800 4100
Wire Wire Line
	5000 4200 4800 4200
Wire Wire Line
	5000 4300 4800 4300
Wire Wire Line
	5000 4400 4800 4400
Wire Wire Line
	5000 4500 4800 4500
Wire Wire Line
	5000 4600 4800 4600
Wire Wire Line
	5000 4700 4800 4700
Text Label 4850 3300 0    50   ~ 0
A0
Text Label 4850 3400 0    50   ~ 0
A1
Text Label 4850 3500 0    50   ~ 0
A2
Text Label 4850 3600 0    50   ~ 0
A3
Text Label 4850 3700 0    50   ~ 0
A4
Text Label 4850 3800 0    50   ~ 0
A5
Text Label 4850 3900 0    50   ~ 0
A6
Text Label 4850 4000 0    50   ~ 0
A7
Text Label 4850 4100 0    50   ~ 0
A8
Text Label 4850 4200 0    50   ~ 0
A9
Text Label 4850 4300 0    50   ~ 0
A10
Text Label 4850 4400 0    50   ~ 0
A11
Text Label 4850 4500 0    50   ~ 0
A12
Text Label 4850 4600 0    50   ~ 0
A13
Text Label 4850 4700 0    50   ~ 0
A14
Entry Wire Line
	4700 3200 4800 3300
Entry Wire Line
	4700 3300 4800 3400
Entry Wire Line
	4700 3400 4800 3500
Entry Wire Line
	4700 3500 4800 3600
Entry Wire Line
	4700 3600 4800 3700
Entry Wire Line
	4700 3700 4800 3800
Entry Wire Line
	4700 3800 4800 3900
Entry Wire Line
	4700 3900 4800 4000
Entry Wire Line
	4700 4000 4800 4100
Entry Wire Line
	4700 4100 4800 4200
Entry Wire Line
	4700 4200 4800 4300
Entry Wire Line
	4700 4300 4800 4400
Entry Wire Line
	4700 4400 4800 4500
Entry Wire Line
	4700 4500 4800 4600
Entry Wire Line
	4700 4600 4800 4700
Wire Wire Line
	5000 5050 4900 5050
Text Label 4900 5050 0    50   ~ 0
~RD
Wire Wire Line
	5000 5150 4900 5150
Text Label 4900 5150 0    50   ~ 0
~WR
Wire Wire Line
	5000 5250 4750 5250
Text Label 4750 5250 0    50   ~ 0
~RAMEN
$Comp
L power:VCC #PWR019
U 1 1 5EB3084D
P 5550 3050
F 0 "#PWR019" H 5550 2900 50  0001 C CNN
F 1 "VCC" H 5567 3223 50  0000 C CNN
F 2 "" H 5550 3050 50  0001 C CNN
F 3 "" H 5550 3050 50  0001 C CNN
	1    5550 3050
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR017
U 1 1 5EB32548
P 4650 5300
F 0 "#PWR017" H 4650 5150 50  0001 C CNN
F 1 "VCC" H 4667 5473 50  0000 C CNN
F 2 "" H 4650 5300 50  0001 C CNN
F 3 "" H 4650 5300 50  0001 C CNN
	1    4650 5300
	1    0    0    -1  
$EndComp
Wire Wire Line
	4650 5300 4650 5350
Wire Wire Line
	4650 5350 5000 5350
$Comp
L power:GND #PWR020
U 1 1 5EB37534
P 5550 5600
F 0 "#PWR020" H 5550 5350 50  0001 C CNN
F 1 "GND" H 5555 5427 50  0000 C CNN
F 2 "" H 5550 5600 50  0001 C CNN
F 3 "" H 5550 5600 50  0001 C CNN
	1    5550 5600
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR021
U 1 1 5EB3BBF0
P 7350 5600
F 0 "#PWR021" H 7350 5350 50  0001 C CNN
F 1 "GND" H 7355 5427 50  0000 C CNN
F 2 "" H 7350 5600 50  0001 C CNN
F 3 "" H 7350 5600 50  0001 C CNN
	1    7350 5600
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR022
U 1 1 5EB3BF7A
P 7350 2400
F 0 "#PWR022" H 7350 2250 50  0001 C CNN
F 1 "VCC" H 7367 2573 50  0000 C CNN
F 2 "" H 7350 2400 50  0001 C CNN
F 3 "" H 7350 2400 50  0001 C CNN
	1    7350 2400
	1    0    0    -1  
$EndComp
Wire Wire Line
	6800 3000 6450 3000
Wire Wire Line
	6800 3100 6450 3100
Wire Wire Line
	6800 3200 6450 3200
Wire Wire Line
	6800 3300 6450 3300
Text Label 6650 3000 0    50   ~ 0
A0
Text Label 6650 3100 0    50   ~ 0
A1
Text Label 6650 3200 0    50   ~ 0
A2
Text Label 6650 3300 0    50   ~ 0
A3
Wire Wire Line
	6800 4500 6500 4500
Text Label 6500 4500 0    50   ~ 0
~UARTEN
NoConn ~ 11900 2450
$Comp
L z80comp2:z80glue U5
U 1 1 5EB69DED
P 3850 1550
F 0 "U5" H 4000 2100 50  0000 C CNN
F 1 "z80glue" H 3600 900 50  0000 C CNN
F 2 "" H 3800 1650 50  0001 C CNN
F 3 "" H 3800 1650 50  0001 C CNN
	1    3850 1550
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR013
U 1 1 5EB7EC44
P 3850 750
F 0 "#PWR013" H 3850 600 50  0001 C CNN
F 1 "VCC" H 3867 923 50  0000 C CNN
F 2 "" H 3850 750 50  0001 C CNN
F 3 "" H 3850 750 50  0001 C CNN
	1    3850 750 
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR014
U 1 1 5EB872CE
P 3850 2350
F 0 "#PWR014" H 3850 2100 50  0001 C CNN
F 1 "GND" H 3855 2177 50  0000 C CNN
F 2 "" H 3850 2350 50  0001 C CNN
F 3 "" H 3850 2350 50  0001 C CNN
	1    3850 2350
	1    0    0    -1  
$EndComp
Wire Wire Line
	3300 1000 3100 1000
Wire Wire Line
	3300 1100 3100 1100
Wire Wire Line
	3300 1200 3100 1200
Wire Wire Line
	3300 1300 3100 1300
Wire Wire Line
	3300 1400 3100 1400
Wire Wire Line
	3300 1700 3100 1700
Wire Wire Line
	3300 1800 3100 1800
Text Label 3150 1000 0    50   ~ 0
A4
Text Label 3150 1100 0    50   ~ 0
A5
Text Label 3150 1200 0    50   ~ 0
A6
Text Label 3150 1300 0    50   ~ 0
A7
Text Label 3150 1400 0    50   ~ 0
A15
Text Label 3100 1700 0    50   ~ 0
~MREQ
Text Label 3100 1800 0    50   ~ 0
~IOREQ
Entry Wire Line
	3000 1500 3100 1400
Entry Wire Line
	3000 1400 3100 1300
Entry Wire Line
	3000 1300 3100 1200
Entry Wire Line
	3000 1200 3100 1100
Entry Wire Line
	3000 1100 3100 1000
Wire Wire Line
	4400 1500 4700 1500
Wire Wire Line
	4400 1300 4700 1300
Wire Wire Line
	4400 1400 4700 1400
Text Label 6500 1300 0    50   ~ 0
~UARTEN
Text Label 4400 1400 0    50   ~ 0
~RAMEN
Text Label 4400 1500 0    50   ~ 0
~ROMEN
Wire Wire Line
	6800 4400 6650 4400
Text Label 6650 4400 0    50   ~ 0
~WR
Entry Wire Line
	6350 2900 6450 3000
Entry Wire Line
	6350 3000 6450 3100
Entry Wire Line
	6350 3100 6450 3200
Entry Wire Line
	6350 3200 6450 3300
Entry Wire Line
	6400 4300 6500 4200
Entry Wire Line
	6400 4200 6500 4100
Entry Wire Line
	6400 4100 6500 4000
Entry Wire Line
	6400 4000 6500 3900
Entry Wire Line
	6400 3900 6500 3800
Entry Wire Line
	6400 3800 6500 3700
Entry Wire Line
	6400 3700 6500 3600
Entry Wire Line
	6400 3600 6500 3500
Wire Wire Line
	6800 2700 6650 2700
Text Label 6650 2700 0    50   ~ 0
~RST
Wire Bus Line
	6400 4400 6100 4400
Wire Bus Line
	6100 4400 6100 5850
Wire Bus Line
	6100 5850 4550 5850
Connection ~ 4550 5850
$Comp
L Device:Crystal_Small Y1
U 1 1 5ED63565
P 6550 5150
F 0 "Y1" V 6600 5100 50  0000 R CNN
F 1 "3.6864MHz" V 6700 5450 50  0000 R CNN
F 2 "" H 6550 5150 50  0001 C CNN
F 3 "~" H 6550 5150 50  0001 C CNN
	1    6550 5150
	0    -1   -1   0   
$EndComp
Wire Wire Line
	6550 5050 6800 5050
$Comp
L Device:C_Small C11
U 1 1 5ED7A206
P 6550 5400
F 0 "C11" H 6750 5400 50  0000 R CNN
F 1 "5pF" H 6750 5500 50  0000 R CNN
F 2 "" H 6550 5400 50  0001 C CNN
F 3 "~" H 6550 5400 50  0001 C CNN
	1    6550 5400
	1    0    0    1   
$EndComp
$Comp
L Device:C_Small C10
U 1 1 5ED7B05B
P 6350 5200
F 0 "C10" H 6550 5200 50  0000 R CNN
F 1 "15pF" H 6550 5300 50  0000 R CNN
F 2 "" H 6350 5200 50  0001 C CNN
F 3 "~" H 6350 5200 50  0001 C CNN
	1    6350 5200
	-1   0    0    1   
$EndComp
Wire Wire Line
	6350 5100 6350 5050
Wire Wire Line
	6350 5050 6550 5050
Wire Wire Line
	6550 5300 6550 5250
Connection ~ 6550 5250
Wire Wire Line
	7350 5600 6550 5600
Wire Wire Line
	6550 5600 6550 5500
Wire Wire Line
	6550 5600 6350 5600
Wire Wire Line
	6350 5600 6350 5300
$Comp
L Interface_UART:MAX232 U9
U 1 1 5EDB13AC
P 9300 3900
F 0 "U9" H 9500 2850 50  0000 C CNN
F 1 "MAX232" H 9300 4150 50  0000 C CNN
F 2 "" H 9350 2850 50  0001 L CNN
F 3 "http://www.ti.com/lit/ds/symlink/max232.pdf" H 9300 4000 50  0001 C CNN
	1    9300 3900
	1    0    0    -1  
$EndComp
Wire Bus Line
	4550 5850 4400 5850
Wire Wire Line
	7900 3500 8200 3500
Text Label 7900 3500 0    50   ~ 0
BANKSEL
Text Label 7900 3400 0    50   ~ 0
~RTSA
Text Label 7900 2700 0    50   ~ 0
~CTSA
Text Label 7900 4300 0    50   ~ 0
TXA
Text Label 7900 4400 0    50   ~ 0
RXA
Wire Wire Line
	8350 3400 8350 4200
Wire Wire Line
	8350 4200 8500 4200
Wire Wire Line
	7900 3400 8350 3400
Wire Wire Line
	8300 2700 8300 4600
Wire Wire Line
	8300 4600 8500 4600
Wire Wire Line
	7900 2700 8300 2700
$Comp
L power:GND #PWR026
U 1 1 5EE75D69
P 9300 5100
F 0 "#PWR026" H 9300 4850 50  0001 C CNN
F 1 "GND" H 9305 4927 50  0000 C CNN
F 2 "" H 9300 5100 50  0001 C CNN
F 3 "" H 9300 5100 50  0001 C CNN
	1    9300 5100
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR025
U 1 1 5EE7620F
P 9300 2700
F 0 "#PWR025" H 9300 2550 50  0001 C CNN
F 1 "VCC" H 9317 2873 50  0000 C CNN
F 2 "" H 9300 2700 50  0001 C CNN
F 3 "" H 9300 2700 50  0001 C CNN
	1    9300 2700
	1    0    0    -1  
$EndComp
$Comp
L Device:C C12
U 1 1 5EE7734D
P 8500 3150
F 0 "C12" H 8500 3250 50  0000 L CNN
F 1 "0.1uF" H 8400 2950 50  0000 L CNN
F 2 "" H 8538 3000 50  0001 C CNN
F 3 "~" H 8500 3150 50  0001 C CNN
	1    8500 3150
	1    0    0    -1  
$EndComp
$Comp
L Device:C C13
U 1 1 5EEBDB7E
P 10100 3150
F 0 "C13" H 9950 3250 50  0000 L CNN
F 1 "0.1uF" H 9850 3050 50  0000 L CNN
F 2 "" H 10138 3000 50  0001 C CNN
F 3 "~" H 10100 3150 50  0001 C CNN
	1    10100 3150
	-1   0    0    -1  
$EndComp
$Comp
L Device:C C14
U 1 1 5EEBE0D7
P 10300 3500
F 0 "C14" V 10250 3600 50  0000 L CNN
F 1 "0.1uF" V 10250 3200 50  0000 L CNN
F 2 "" H 10338 3350 50  0001 C CNN
F 3 "~" H 10300 3500 50  0001 C CNN
	1    10300 3500
	0    -1   1    0   
$EndComp
Wire Wire Line
	10150 3500 10100 3500
Wire Wire Line
	10100 3800 10150 3800
$Comp
L Device:C C15
U 1 1 5EEC6839
P 10300 3800
F 0 "C15" V 10250 3900 50  0000 L CNN
F 1 "0.1uF" V 10250 3500 50  0000 L CNN
F 2 "" H 10338 3650 50  0001 C CNN
F 3 "~" H 10300 3800 50  0001 C CNN
	1    10300 3800
	0    -1   1    0   
$EndComp
$Comp
L power:VCC #PWR029
U 1 1 5EEEF5D1
P 10650 3400
F 0 "#PWR029" H 10650 3250 50  0001 C CNN
F 1 "VCC" H 10667 3573 50  0000 C CNN
F 2 "" H 10650 3400 50  0001 C CNN
F 3 "" H 10650 3400 50  0001 C CNN
	1    10650 3400
	1    0    0    -1  
$EndComp
Wire Wire Line
	10650 3400 10650 3500
Wire Wire Line
	10650 3500 10450 3500
$Comp
L power:GND #PWR027
U 1 1 5EEF812A
P 10550 3850
F 0 "#PWR027" H 10550 3600 50  0001 C CNN
F 1 "GND" H 10555 3677 50  0000 C CNN
F 2 "" H 10550 3850 50  0001 C CNN
F 3 "" H 10550 3850 50  0001 C CNN
	1    10550 3850
	1    0    0    -1  
$EndComp
Wire Wire Line
	10450 3800 10550 3800
Wire Wire Line
	10550 3800 10550 3850
Wire Wire Line
	7900 4300 8250 4300
Wire Wire Line
	7900 4400 8500 4400
Wire Wire Line
	8250 4300 8250 4000
Wire Wire Line
	8250 4000 8500 4000
Text Label 8400 4000 0    50   ~ 0
TXA
Text Label 8350 4600 0    50   ~ 0
~CTSA
Text Label 8350 4200 0    50   ~ 0
~RTSA
Text Label 8350 4400 0    50   ~ 0
RXA
$Comp
L Connector:Conn_01x03_Male J1
U 1 1 5F03FE41
P 8550 4850
F 0 "J1" H 8600 4700 50  0000 R CNN
F 1 "SERB" H 8650 4600 50  0000 R CNN
F 2 "" H 8550 4850 50  0001 C CNN
F 3 "~" H 8550 4850 50  0001 C CNN
	1    8550 4850
	-1   0    0    -1  
$EndComp
Text Label 8000 4750 0    50   ~ 0
RXDB
Text Label 8000 4850 0    50   ~ 0
TXDB
Wire Wire Line
	7900 4650 7900 4850
Wire Wire Line
	7900 4850 8350 4850
Wire Wire Line
	7900 4550 7950 4550
Wire Wire Line
	7950 4550 7950 4750
Wire Wire Line
	7950 4750 8350 4750
$Comp
L power:GND #PWR024
U 1 1 5F09F89E
P 8300 4950
F 0 "#PWR024" H 8300 4700 50  0001 C CNN
F 1 "GND" H 8305 4777 50  0000 C CNN
F 2 "" H 8300 4950 50  0001 C CNN
F 3 "" H 8300 4950 50  0001 C CNN
	1    8300 4950
	1    0    0    -1  
$EndComp
$Comp
L Connector:DB9_Male J2
U 1 1 5F10B0BC
P 10850 4600
F 0 "J2" H 10800 5150 50  0000 L CNN
F 1 "SERA" H 10750 4050 50  0000 L CNN
F 2 "" H 10850 4600 50  0001 C CNN
F 3 " ~" H 10850 4600 50  0001 C CNN
	1    10850 4600
	1    0    0    1   
$EndComp
Wire Wire Line
	10100 4000 10250 4000
Wire Wire Line
	10100 4400 10550 4400
Wire Wire Line
	10250 4000 10250 4600
Wire Wire Line
	10250 4600 10550 4600
Wire Wire Line
	10100 4200 10200 4200
Wire Wire Line
	10200 4200 10200 4500
Wire Wire Line
	10200 4500 10550 4500
Wire Wire Line
	10100 4600 10150 4600
Wire Wire Line
	10150 4600 10150 4700
Wire Wire Line
	10150 4700 10550 4700
$Comp
L power:GND #PWR028
U 1 1 5F1878FB
P 10550 5050
F 0 "#PWR028" H 10550 4800 50  0001 C CNN
F 1 "GND" H 10555 4877 50  0000 C CNN
F 2 "" H 10550 5050 50  0001 C CNN
F 3 "" H 10550 5050 50  0001 C CNN
	1    10550 5050
	1    0    0    -1  
$EndComp
Wire Wire Line
	10550 5000 10550 5050
NoConn ~ 10550 4200
NoConn ~ 10550 4300
NoConn ~ 10550 4800
NoConn ~ 10550 4900
Text Label 10350 4400 0    50   ~ 0
SRTX
Text Label 10350 4500 0    50   ~ 0
SRTS
Text Label 10350 4600 0    50   ~ 0
STXA
Text Label 10350 4700 0    50   ~ 0
SCTS
Wire Bus Line
	3000 5850 4550 5850
Connection ~ 6550 5050
Connection ~ 6550 5600
Text Label 6600 5250 0    50   ~ 0
UCLK
Wire Wire Line
	6550 5250 6800 5250
Connection ~ 7350 5600
Wire Wire Line
	1300 4050 1000 4050
Text Label 1050 4050 0    50   ~ 0
~WAIT
$Comp
L Device:R R3
U 1 1 5F385448
P 850 4050
F 0 "R3" V 950 4050 50  0000 L CNN
F 1 "4.7k" V 850 3950 50  0000 L CNN
F 2 "" V 780 4050 50  0001 C CNN
F 3 "~" H 850 4050 50  0001 C CNN
	1    850  4050
	0    -1   -1   0   
$EndComp
Text Label 1150 3150 0    50   ~ 0
CLK
Text Label 1050 3550 0    50   ~ 0
~INT
Wire Wire Line
	1300 3550 1000 3550
$Comp
L power:VCC #PWR01
U 1 1 5F3DA2B2
P 700 3350
F 0 "#PWR01" H 700 3200 50  0001 C CNN
F 1 "VCC" H 700 3500 50  0000 C CNN
F 2 "" H 700 3350 50  0001 C CNN
F 3 "" H 700 3350 50  0001 C CNN
	1    700  3350
	1    0    0    -1  
$EndComp
$Comp
L Device:R R2
U 1 1 5F3D9F65
P 850 3550
F 0 "R2" V 950 3550 50  0000 L CNN
F 1 "4.7k" V 850 3450 50  0000 L CNN
F 2 "" V 780 3550 50  0001 C CNN
F 3 "~" H 850 3550 50  0001 C CNN
	1    850  3550
	0    -1   1    0   
$EndComp
Wire Wire Line
	700  3350 700  3450
Connection ~ 700  3550
Wire Wire Line
	700  3550 700  4050
$Comp
L Device:C C2
U 1 1 5F442E26
P 1850 7150
F 0 "C2" H 1900 7250 50  0000 L CNN
F 1 "0.1uF" H 1900 7050 50  0000 L CNN
F 2 "" H 1888 7000 50  0001 C CNN
F 3 "~" H 1850 7150 50  0001 C CNN
	1    1850 7150
	1    0    0    -1  
$EndComp
$Comp
L Device:C C3
U 1 1 5F444327
P 2150 7150
F 0 "C3" H 2200 7250 50  0000 L CNN
F 1 "0.1uF" H 2200 7050 50  0000 L CNN
F 2 "" H 2188 7000 50  0001 C CNN
F 3 "~" H 2150 7150 50  0001 C CNN
	1    2150 7150
	1    0    0    -1  
$EndComp
$Comp
L Device:C C4
U 1 1 5F444547
P 2450 7150
F 0 "C4" H 2500 7250 50  0000 L CNN
F 1 "0.1uF" H 2500 7050 50  0000 L CNN
F 2 "" H 2488 7000 50  0001 C CNN
F 3 "~" H 2450 7150 50  0001 C CNN
	1    2450 7150
	1    0    0    -1  
$EndComp
$Comp
L Device:C C6
U 1 1 5F444841
P 2750 7150
F 0 "C6" H 2800 7250 50  0000 L CNN
F 1 "0.1uF" H 2800 7050 50  0000 L CNN
F 2 "" H 2788 7000 50  0001 C CNN
F 3 "~" H 2750 7150 50  0001 C CNN
	1    2750 7150
	1    0    0    -1  
$EndComp
$Comp
L Device:C C7
U 1 1 5F444B0C
P 3050 7150
F 0 "C7" H 3100 7250 50  0000 L CNN
F 1 "0.1uF" H 3100 7050 50  0000 L CNN
F 2 "" H 3088 7000 50  0001 C CNN
F 3 "~" H 3050 7150 50  0001 C CNN
	1    3050 7150
	1    0    0    -1  
$EndComp
$Comp
L Device:C C8
U 1 1 5F444DC1
P 3350 7150
F 0 "C8" H 3400 7250 50  0000 L CNN
F 1 "0.1uF" H 3400 7050 50  0000 L CNN
F 2 "" H 3388 7000 50  0001 C CNN
F 3 "~" H 3350 7150 50  0001 C CNN
	1    3350 7150
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR010
U 1 1 5F4451AA
P 2600 7300
F 0 "#PWR010" H 2600 7050 50  0001 C CNN
F 1 "GND" H 2605 7127 50  0000 C CNN
F 2 "" H 2600 7300 50  0001 C CNN
F 3 "" H 2600 7300 50  0001 C CNN
	1    2600 7300
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR09
U 1 1 5F4467BC
P 2600 7000
F 0 "#PWR09" H 2600 6850 50  0001 C CNN
F 1 "VCC" H 2600 7150 50  0000 C CNN
F 2 "" H 2600 7000 50  0001 C CNN
F 3 "" H 2600 7000 50  0001 C CNN
	1    2600 7000
	1    0    0    -1  
$EndComp
Wire Wire Line
	1850 7000 2150 7000
Connection ~ 2150 7000
Wire Wire Line
	2150 7000 2450 7000
Connection ~ 2450 7000
Wire Wire Line
	2450 7000 2600 7000
Connection ~ 2600 7000
Wire Wire Line
	2600 7000 2750 7000
Connection ~ 2750 7000
Wire Wire Line
	2750 7000 3050 7000
Connection ~ 3050 7000
Wire Wire Line
	3050 7000 3350 7000
Wire Wire Line
	1850 7300 2150 7300
Connection ~ 2150 7300
Wire Wire Line
	2150 7300 2450 7300
Connection ~ 2450 7300
Wire Wire Line
	2450 7300 2600 7300
Connection ~ 2600 7300
Wire Wire Line
	2600 7300 2750 7300
Connection ~ 2750 7300
Wire Wire Line
	2750 7300 3050 7300
Connection ~ 3050 7300
Wire Wire Line
	3050 7300 3350 7300
$Comp
L Device:R R1
U 1 1 5F463EA2
P 850 3450
F 0 "R1" V 950 3450 50  0000 L CNN
F 1 "4.7k" V 850 3350 50  0000 L CNN
F 2 "" V 780 3450 50  0001 C CNN
F 3 "~" H 850 3450 50  0001 C CNN
	1    850  3450
	0    -1   -1   0   
$EndComp
Connection ~ 700  3450
Wire Wire Line
	700  3450 700  3550
Wire Wire Line
	1000 3450 1300 3450
Text Label 1050 3450 0    50   ~ 0
~NMI
$Comp
L 74xx:74LS05 U4
U 2 1 5F4A99C1
P 7300 6200
F 0 "U4" H 7300 6050 50  0000 C CNN
F 1 "74LS05" H 7400 6350 50  0000 C CNN
F 2 "" H 7300 6200 50  0001 C CNN
F 3 "http://www.ti.com/lit/gpn/sn74LS05" H 7300 6200 50  0001 C CNN
	2    7300 6200
	-1   0    0    1   
$EndComp
$Comp
L Diode:1N4148 D1
U 1 1 5F4AD646
P 8050 6200
F 0 "D1" H 8050 5984 50  0000 C CNN
F 1 "1N4148" H 8050 6075 50  0000 C CNN
F 2 "Diode_THT:D_DO-35_SOD27_P7.62mm_Horizontal" H 8050 6025 50  0001 C CNN
F 3 "https://assets.nexperia.com/documents/data-sheet/1N4148_1N4448.pdf" H 8050 6200 50  0001 C CNN
	1    8050 6200
	-1   0    0    1   
$EndComp
$Comp
L Device:R R8
U 1 1 5F4AE323
P 7750 6050
F 0 "R8" H 7680 6004 50  0000 R CNN
F 1 "4k7" H 7680 6095 50  0000 R CNN
F 2 "" V 7680 6050 50  0001 C CNN
F 3 "~" H 7750 6050 50  0001 C CNN
	1    7750 6050
	1    0    0    1   
$EndComp
$Comp
L power:VCC #PWR023
U 1 1 5F4B4193
P 7750 5900
F 0 "#PWR023" H 7750 5750 50  0001 C CNN
F 1 "VCC" H 7767 6073 50  0000 C CNN
F 2 "" H 7750 5900 50  0001 C CNN
F 3 "" H 7750 5900 50  0001 C CNN
	1    7750 5900
	1    0    0    -1  
$EndComp
Wire Wire Line
	7900 5250 7900 6200
Connection ~ 7900 6200
Wire Wire Line
	8200 6200 8500 6200
Text Label 8200 6200 0    50   ~ 0
UARTEN
Wire Wire Line
	7600 6200 7750 6200
Connection ~ 7750 6200
Wire Wire Line
	7750 6200 7900 6200
Wire Wire Line
	7000 6200 850  6200
Wire Wire Line
	850  6200 850  4250
Wire Wire Line
	850  4250 1000 4250
Wire Wire Line
	1000 4250 1000 4050
Connection ~ 1000 4050
Wire Wire Line
	4400 1200 4700 1200
Text Label 4400 1200 0    50   ~ 0
UARTEN
Wire Wire Line
	7900 5150 8050 5150
Text Label 7900 5150 0    50   ~ 0
~INT
Wire Wire Line
	8300 4950 8350 4950
Text Notes 3950 2300 0    50   ~ 0
GAL16V8: glue.pld
$Comp
L power:VCC #PWR04
U 1 1 5F6A583C
P 1300 5150
F 0 "#PWR04" H 1300 5000 50  0001 C CNN
F 1 "VCC" H 1317 5323 50  0000 C CNN
F 2 "" H 1300 5150 50  0001 C CNN
F 3 "" H 1300 5150 50  0001 C CNN
	1    1300 5150
	1    0    0    -1  
$EndComp
Wire Wire Line
	3300 1600 3100 1600
Text Label 3200 1600 0    50   ~ 0
~M1
Wire Wire Line
	3300 1900 3100 1900
Text Label 3100 1900 0    50   ~ 0
~RFSH
Wire Wire Line
	4400 1700 4700 1700
Text Label 4400 1700 0    50   ~ 0
~IACK
Wire Wire Line
	6800 4700 6600 4700
Text Label 6600 4700 0    50   ~ 0
~IACK
$Comp
L 74xx:74HC14 U1
U 6 1 5F7B3414
P 850 2400
F 0 "U1" V 650 2400 50  0000 L CNN
F 1 "74HC14" H 850 2250 50  0000 L CNN
F 2 "" H 850 2400 50  0001 C CNN
F 3 "http://www.ti.com/lit/gpn/sn74HC14" H 850 2400 50  0001 C CNN
	6    850  2400
	0    1    1    0   
$EndComp
Wire Wire Line
	850  2700 850  3150
Wire Wire Line
	850  3150 1300 3150
Wire Wire Line
	850  2100 850  1850
Text Label 850  2050 1    50   ~ 0
UCLK
Wire Wire Line
	1300 1650 1350 1650
$Comp
L power:VCC #PWR03
U 1 1 5F97FBA6
P 1300 1650
F 0 "#PWR03" H 1300 1500 50  0001 C CNN
F 1 "VCC" H 1300 1800 50  0000 C CNN
F 2 "" H 1300 1650 50  0001 C CNN
F 3 "" H 1300 1650 50  0001 C CNN
	1    1300 1650
	1    0    0    -1  
$EndComp
Wire Wire Line
	750  1350 750  1250
$Comp
L power:GND #PWR02
U 1 1 5F96EC3C
P 750 1350
F 0 "#PWR02" H 750 1100 50  0001 C CNN
F 1 "GND" H 755 1177 50  0000 C CNN
F 2 "" H 750 1350 50  0001 C CNN
F 3 "" H 750 1350 50  0001 C CNN
	1    750  1350
	1    0    0    -1  
$EndComp
Connection ~ 1200 1250
Wire Wire Line
	1150 1250 1200 1250
$Comp
L Switch:SW_Push SW1
U 1 1 5F933400
P 950 1250
F 0 "SW1" H 950 1535 50  0000 C CNN
F 1 "RESET" H 950 1444 50  0000 C CNN
F 2 "" H 950 1450 50  0001 C CNN
F 3 "~" H 950 1450 50  0001 C CNN
	1    950  1250
	1    0    0    -1  
$EndComp
Connection ~ 1850 1050
Wire Wire Line
	1500 1050 1850 1050
Wire Wire Line
	1500 950  1500 1050
Wire Wire Line
	1200 950  1500 950 
Wire Wire Line
	1200 1250 1350 1250
$Comp
L Device:R R5
U 1 1 5F901EA0
P 1200 1100
F 0 "R5" H 1270 1146 50  0000 L CNN
F 1 "1M" V 1200 1050 50  0000 L CNN
F 2 "" V 1130 1100 50  0001 C CNN
F 3 "~" H 1200 1100 50  0001 C CNN
	1    1200 1100
	1    0    0    -1  
$EndComp
NoConn ~ 1350 1450
$Comp
L power:VCC #PWR011
U 1 1 5F8CF8A4
P 2700 1150
F 0 "#PWR011" H 2700 1000 50  0001 C CNN
F 1 "VCC" H 2717 1323 50  0000 C CNN
F 2 "" H 2700 1150 50  0001 C CNN
F 3 "" H 2700 1150 50  0001 C CNN
	1    2700 1150
	1    0    0    -1  
$EndComp
Text Label 2350 1250 0    50   ~ 0
RST
Wire Wire Line
	2350 1250 2550 1250
Wire Wire Line
	2700 1850 1850 1850
Wire Wire Line
	2350 1650 2700 1650
Connection ~ 2700 1450
Wire Wire Line
	2700 1450 2700 1650
$Comp
L Device:C_Small C5
U 1 1 5F880397
P 2700 1750
F 0 "C5" H 2609 1704 50  0000 R CNN
F 1 "0.1uF" H 2609 1795 50  0000 R CNN
F 2 "" H 2700 1750 50  0001 C CNN
F 3 "~" H 2700 1750 50  0001 C CNN
	1    2700 1750
	1    0    0    1   
$EndComp
Wire Wire Line
	2350 1450 2700 1450
$Comp
L Device:R R6
U 1 1 5F8611FE
P 2700 1300
F 0 "R6" H 2750 1350 50  0000 L CNN
F 1 "9.1k" V 2700 1200 50  0000 L CNN
F 2 "" V 2630 1300 50  0001 C CNN
F 3 "~" H 2700 1300 50  0001 C CNN
	1    2700 1300
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR05
U 1 1 5F860836
P 1850 1050
F 0 "#PWR05" H 1850 900 50  0001 C CNN
F 1 "VCC" H 1867 1223 50  0000 C CNN
F 2 "" H 1850 1050 50  0001 C CNN
F 3 "" H 1850 1050 50  0001 C CNN
	1    1850 1050
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR06
U 1 1 5F860553
P 1850 1850
F 0 "#PWR06" H 1850 1600 50  0001 C CNN
F 1 "GND" H 1855 1677 50  0000 C CNN
F 2 "" H 1850 1850 50  0001 C CNN
F 3 "" H 1850 1850 50  0001 C CNN
	1    1850 1850
	1    0    0    -1  
$EndComp
$Comp
L Timer:LM555 U2
U 1 1 5F85DCB5
P 1850 1450
F 0 "U2" H 2000 1800 50  0000 C CNN
F 1 "LM555" H 1850 1450 50  0000 C CNN
F 2 "" H 1850 1450 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/lm555.pdf" H 1850 1450 50  0001 C CNN
	1    1850 1450
	1    0    0    -1  
$EndComp
$Comp
L 74xx:74LS05 U4
U 1 1 5F9CB042
P 2450 800
F 0 "U4" H 2450 650 50  0000 C CNN
F 1 "74LS05" H 2550 950 50  0000 C CNN
F 2 "" H 2450 800 50  0001 C CNN
F 3 "http://www.ti.com/lit/gpn/sn74LS05" H 2450 800 50  0001 C CNN
	1    2450 800 
	1    0    0    -1  
$EndComp
Wire Wire Line
	2150 800  2150 1050
Wire Wire Line
	2150 1050 2550 1050
Wire Wire Line
	2550 1050 2550 1250
Text Label 2850 800  0    50   ~ 0
~RST
$Comp
L Device:R R4
U 1 1 5FA0761C
P 1050 2700
F 0 "R4" H 1120 2746 50  0000 L CNN
F 1 "4.7k" V 1050 2600 50  0000 L CNN
F 2 "" V 980 2700 50  0001 C CNN
F 3 "~" H 1050 2700 50  0001 C CNN
	1    1050 2700
	1    0    0    -1  
$EndComp
Text Notes 1350 700  0    50   ~ 0
Power-on reset
Wire Wire Line
	2750 800  3000 800 
Wire Wire Line
	1050 2850 1300 2850
Wire Wire Line
	1050 2550 2000 2550
Connection ~ 2000 2550
Connection ~ 2700 1650
Connection ~ 1850 1850
$Comp
L Device:C_Small C1
U 1 1 5FC80F1C
P 1200 1700
F 0 "C1" H 1109 1654 50  0000 R CNN
F 1 "0.1uF" H 1109 1745 50  0000 R CNN
F 2 "" H 1200 1700 50  0001 C CNN
F 3 "~" H 1200 1700 50  0001 C CNN
	1    1200 1700
	1    0    0    1   
$EndComp
Wire Wire Line
	1200 1800 1200 1850
Wire Wire Line
	1200 1850 1850 1850
Wire Wire Line
	1200 1600 1200 1250
NoConn ~ 1300 5250
NoConn ~ 1300 4150
Wire Wire Line
	1300 3850 1100 3850
Text Label 1200 3850 0    50   ~ 0
~M1
Wire Wire Line
	1300 3950 1100 3950
Text Label 1100 3950 0    50   ~ 0
~RFSH
Connection ~ 4700 2600
Wire Bus Line
	4600 2600 4700 2600
Wire Bus Line
	4700 2600 6350 2600
Wire Bus Line
	3150 2600 4700 2600
Connection ~ 3000 2600
Wire Wire Line
	3300 2000 3100 2000
Wire Wire Line
	3300 2100 3100 2100
Text Label 3200 2000 0    50   ~ 0
~RD
Text Label 3200 2100 0    50   ~ 0
~WR
$Comp
L 74xx:74HC14 U1
U 5 1 5FEF66B9
P 5600 1300
F 0 "U1" H 5550 1450 50  0000 L CNN
F 1 "74HC14" H 5550 1150 50  0000 L CNN
F 2 "" H 5600 1300 50  0001 C CNN
F 3 "http://www.ti.com/lit/gpn/sn74HC14" H 5600 1300 50  0001 C CNN
	5    5600 1300
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C9
U 1 1 5FEF828C
P 5100 1100
F 0 "C9" H 5008 1054 50  0000 R CNN
F 1 "0.1uF" H 5008 1145 50  0000 R CNN
F 2 "" H 5100 1100 50  0001 C CNN
F 3 "~" H 5100 1100 50  0001 C CNN
	1    5100 1100
	-1   0    0    1   
$EndComp
$Comp
L Device:R R7
U 1 1 5FF0FBDF
P 4850 1300
F 0 "R7" V 4750 1250 50  0000 L CNN
F 1 "120R" V 4850 1200 50  0000 L CNN
F 2 "" V 4780 1300 50  0001 C CNN
F 3 "~" H 4850 1300 50  0001 C CNN
	1    4850 1300
	0    1    1    0   
$EndComp
$Comp
L power:VCC #PWR018
U 1 1 5FF26567
P 5100 1000
F 0 "#PWR018" H 5100 850 50  0001 C CNN
F 1 "VCC" H 5117 1173 50  0000 C CNN
F 2 "" H 5100 1000 50  0001 C CNN
F 3 "" H 5100 1000 50  0001 C CNN
	1    5100 1000
	1    0    0    -1  
$EndComp
$Comp
L 74xx:74HC14 U1
U 4 1 5FF485F0
P 6200 1300
F 0 "U1" H 6150 1450 50  0000 L CNN
F 1 "74HC14" H 6150 1150 50  0000 L CNN
F 2 "" H 6200 1300 50  0001 C CNN
F 3 "http://www.ti.com/lit/gpn/sn74HC14" H 6200 1300 50  0001 C CNN
	4    6200 1300
	1    0    0    -1  
$EndComp
Wire Wire Line
	6500 1300 6800 1300
Wire Wire Line
	5000 1300 5100 1300
Wire Wire Line
	5100 1200 5100 1300
Wire Bus Line
	6350 2600 6350 3200
Wire Bus Line
	3000 1100 3000 2600
Wire Bus Line
	6400 3400 6400 4400
Wire Bus Line
	4550 3600 4550 5850
Wire Bus Line
	3000 4650 3000 5850
Wire Bus Line
	3150 2600 3150 4800
Wire Bus Line
	4700 2600 4700 4600
Wire Bus Line
	3000 2600 3000 4250
Connection ~ 5100 1300
Wire Wire Line
	5100 1300 5300 1300
Text Notes 5350 950  0    50   ~ 0
Delay the UART ~CS~ for a few ns to bring the\ntiming between R/~W~ and ~CS~ within spec\n(this might have been a red herring and not necessary)
$EndSCHEMATC
