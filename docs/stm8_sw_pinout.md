

 Circuit:
 * Ethernet shield attached with SPI to pins 10, 11, 12, 13 + 9 for reset
 * Output for relays on pins 3, 4 See relays.h tab
 * switch pins Connected to switch directly 14(A0), 15(A1), 16(A2), A3(17)
 A6(20),A7(21) Set to pullup, A6 and A7 can't be pullup on some chips
 *
 * ================This one is us=======================
 * + switch controller network             2,3,4       +
 * =====================================================
 *
 * //  1-wire                                5,6,7
 * //  SPISerial Peripheral Interface        (8,9 select 2 SPI slaves,can be any
 pins)10,11,12,13
 * //  GPIO output, relay, Led, etc.     8, D3(3), 9, A3(17) Over lap with
 above.
 * //  I2C                            A4(18), A5(19)
 * //  leave D3,9 to last to test pwm. Going to need PWM multiplex or second
 arduino or mega.
 * //  A6 & A7 are analogRead(); only, Can't use pinMode(A6,INPUT_PULLUP). Need
 to add a pull up resistor in hardware.


stm8s003f3 – 20-pin TSSOP20
-----------------------------------------------------------
STM8 pin   #   Arduino  PCB                         Notes
-----------------------------------------------------------
PD4        1   D13      Light Switch 3 (SW3)   TIM2_CH1, ADC_ETR
PD5        2   D14/A3   UART1_TX, AIN5
PD6        3   D15/A4   UART1_RX, AIN6, TIM1_CH1
NRST       4            Reset
OSCIN      5            Crystal input
OSCOUT     6            Crystal output
VSS        7            Ground
VCAP       8            1.8 V regulator capacitor
VDD        9            +5 V supply
PA3        10  D2/SS    SPI_NSS, TIM2_CH3, 1wire temp through solder jumper
PB5        11  D3/SDA   I2C                 I²C SDA, TIM1_BKIN
PB4        12  D4/SCL   I2C                 I²C SCL
PC3        13  D5       HA network in       TIM1_CH3, TLI, TIM1_CH1N
PC4        14  D6/A0    HA network out      CLK_CCO, TIM1_CH4, AIN2, TIM1_CH2N
PC5        15  D7/SCK   SPI                 SPI_SCK, TIM2_CH1
PC6        16  D8/MOSI  SPI                 SPI_MOSI, TIM1_CH1
PC7        17  D9/MISO  SPI                 SPI_MISO, TIM1_CH2
PD1        18  D10      SWIM debug interface
PD2        19  D11/A1   Light Switch 1 (SW1)        AIN3, TIM2_CH3
PD3        20  D12/A2   Light Switch 2 (SW2)            AIN4, TIM2_CH2, BEEP
-----------------------------------------------------------


Analog pins (Arduino A0–A4):
    A0 → PC4 / AIN2 (pin 14)
    A1 → PD2 / AIN3 (pin 19)
    A2 → PD3 / AIN4 (pin 20)
    A3 → PD5 / AIN5 (pin 2)
    A4 → PD6 / AIN6 (pin 3)

   I²C pins: PB4=SCL, PB5=SDA.
   SPI pins: PC5=SCK, PC6=MOSI, PC7=MISO, PA3=SS.
   UART1: TX=PD5, RX=PD6.

1-wire
   PA3/D2 temp through solder jumper
   Other wire header/plug connector gos to JP


