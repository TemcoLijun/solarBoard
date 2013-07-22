#include <16F1824.h>
#device adc=10  //Tell device to use 10-bit ADC
#fuses INTRC_IO,NOWDT,NOPROTECT,NOMCLR
// note BROWNOUT AND PUT GOES TOGETHER.  Need to activate both to enable brownout
#use delay(clock = 16000000)  //Adjusts the accuracy of the delay functions
#use standard_io(A)
#use standard_io(c)

//CAPACITIVE MODULE RELATIVE REGISTERS DEFINE
//#BYTE PIR1=0X0C
//#BIT  RCIF=PIR1.5 
////////////////////////////////////

#BYTE ANSELA   = 0x18C//127
   #BIT  ANSA0  = ANSELA.0 
   #BIT  ANSA1  = ANSELA.1
   #BIT  ANSA2  = ANSELA.2
   #BIT  ANSA4  = ANSELA.4

/*
#BYTE ANSELB   = 0x18D//127
   #BIT  ANSB4  = ANSELB.4 
   #BIT  ANSB5  = ANSELB.5
   #BIT  ANSB6  = ANSELB.6
   #BIT  ANSB7  = ANSELB.7
*/   
#BYTE ANSELC   = 0x18E//127
   #BIT  ANSC0  = ANSELC.0 
   #BIT  ANSC1  = ANSELC.1
   #BIT  ANSC2  = ANSELC.2
   #BIT  ANSC3  = ANSELC.3


#BYTE CPSCON0  =0x1E// 331
   #BIT  T0XCS    = CPSCON0.0
   #BIT  CPSOUT   = CPSCON0.1
   #BIT  CPSRNG0  = CPSCON0.2
   #BIT  CPSRNG1  = CPSCON0.3
   #BIT  CPSRM    = CPSCON0.6
   #BIT  CPSON    = CPSCON0.7

#BYTE CPSCON1  = 0x1F//332
   #BIT  CPSCH0   = CPSCON0.0
   #BIT  CPSCH1   = CPSCON0.1
   #BIT  CPSCH2   = CPSCON0.2
   #BIT  CPSCH3   = CPSCON0.3



#BYTE INTCON = 0x0B//93
   #BIT TMR0IF = INTCON.2
   #BIT TMR0IE = INTCON.5
   #BIT PEIE = INTCON.6
   #BIT GIE = INTCON.7

#BYTE OPTION_REG = 0x95//187
   #BIT PS0 = OPTION_REG.0
   #BIT PS1 = OPTION_REG.1
   #BIT PS2 = OPTION_REG.2
   #BIT PSA = OPTION_REG.3
   #BIT TMR0SE = OPTION_REG.4
   #BIT TMR0CS = OPTION_REG.5

#BYTE T1CON = 0x18//197
   #BIT TMR1ON = T1CON.0

   #BIT T1SYNC = T1CON.2
   #BIT T1OSCEN = T1CON.3
   #BIT T1CKPS0 = T1CON.4
   #BIT T1CKPS1 = T1CON.5
   #BIT TMR1CS0 = T1CON.6
   #BIT TMR1CS1 = T1CON.7

#BYTE TRISA    = 0x8C//126
   #BIT  TRISA2   = TRISA.2 
#BYTE TRISC    = 0x8E//126   

#BYTE T1GCON   = 0x19//198
   #BIT TMR1GE = T1GCON.7

#BYTE FVRCON = 0x117
   #BIT CDAFVR0 = FVRCON.2
   #BIT CDAFVR1 = FVRCON.3
   #BIT FVREN = FVRCON.7
   
#BYTE DACCON0 = 0x118
   #BIT DACNSS = DACCON0.0
   #BIT DACPSS0 = DACCON0.2
   #BIT DACPSS1 = DACCON0.3
   #BIT DACOE = DACCON0.5
   #BIT DACEN = DACCON0.7
   
#BYTE DACCON1 = 0x119
   #BIT DACR0 = DACCON1.0
   #BIT DACR1 = DACCON1.1
   #BIT DACR2 = DACCON1.2   
   #BIT DACR3 = DACCON1.3
   #BIT DACR4 = DACCON1.4

//#define SS           PIN_A1
//#define SCL          PIN_A5
#define POWER       PIN_A4
#define TEMP_AD     PIN_C0
//////////////////////////
////// for SPI///////////
//////callon/////////////
/////////////////////////
/***************write*************/
#ifndef SEND_ONE
   #define SEND_ONE 1
#endif

#define xFREQ_315M
#define xFREQ_433M
#define xFREQ_868M
#define FREQ_915M

#if SEND_ONE
#define GDO0         PIN_C1
#define SPI_CS       PIN_C2
#define SPI_CLK      PIN_C3
#define SPI_MISO     PIN_C4
#define SPI_MOSI     PIN_C5
#else
#define GDO0  PIN_C4
#define SPI_MOSI     PIN_A5
#define SPI_CS       PIN_A4
#define SPI_CLK      PIN_C2
#define SPI_MISO     PIN_C1
#endif
/**************LED******************/
#define LED_RED        PIN_A0
//#define LED_GREEN      PIN_C0

//define the register
#define IOCFG2    0x00
#define IOCFG1    0x01
#define IOCFG0    0x02
#define FIFOTHR   0x03
#define SYNC1     0x04
#define SYNC0     0x05
#define PKTLEN    0x06
#define PKTCTRL1  0x07
#define PKTCTRL0  0x08
#define ADDR      0x09
#define CHANNR    0x0A
#define FSCTRL1   0x0B
#define FSCTRL0   0x0C
#define FREQ2     0x0D
#define FREQ1     0x0E
#define FREQ0     0x0F
#define MDMCFG4   0x10
#define MDMCFG3   0x11
#define MDMCFG2   0x12
#define MDMCFG1   0x13
#define MDMCFG0   0x14
#define DEVIATN   0x15
#define MCSM2     0x16
#define MCSM1     0x17
#define MCSM0     0x18
#define FOCCFG    0x19
#define BSCFG     0x1A
#define AGCCTRL2  0x1B
#define AGCCTRL1  0x1C
#define AGCCTRL0  0x1D
#define WORREVT1  0x1E
#define WORREVT0  0x1F
#define WORCTRL   0x20
#define FREND1    0x21
#define FREND0    0x22
#define FSCAL3    0x23
#define FSCAL2    0x24
#define FSCAL1    0x25
#define FSCAL0    0x26
#define RCCTRL1   0x27
#define RCCTRL0   0x28
#define FSTEST    0x29
#define PTEST     0x2A
#define AGCTEST   0x2B
#define TEST2     0x2C
#define TEST1     0x2D
#define TEST0     0x2E

#define SRES      0x30
#define SFSTXON   0x31
#define SXOFF     0x32
#define SCAL      0x33
#define SRX       0x34
#define STX       0x35
#define SIDLE     0x36
#define SAFC      0x37
#define SWOR      0x38
#define SPWD      0x39
#define SFRX      0x3A
#define SFTX      0x3B
#define SWORRST   0x3C
#define SNOP      0x3D

#define PARTNUM   0x30
#define VERSION   0x31
#define FREQEST   0x32
#define LQI       0x33
#define RSSI      0x34
#define MARCSTATE 0x35
#define WORTIME1  0x36
#define WORTIME0  0x37
#define PKTSTATUS 0x38
#define VCO_VC_DAC  0x39
#define TXBYTES   0x3A
#define RXBYTES   0x3B
#define PATABLE   0x3E
#define TX_FIFO   0x3F
#define RX_FIFO   0x3F

#define data_len 8
#define sampling_time 20
#define temp_type 4
#define PIR_type 2
#define light_type 1
//#define _BSLECT_  

/////////////////////PARAMETER RF////////////////////

unsigned char Rbuffer[ data_len ] = {0};
int16 Wbuffer[ 3 ] = {0}; //we defined 3 number in array,0 for temp,1 for pir,2 for light
unsigned char PA_config[8] = {0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0};
unsigned char leng;

/////////////////////PARAMETER TEMP////////////////////
int16 data_sum;
int16 temp_buf;
int16 caliberate_temp = 0;

////////////////////////////////////////

//#define CO2_RX   PIN_C3
//#define CO2_TX   PIN_C1

  


#define HUM_DEFAULT_TABLE_START_ADR  0x10
#define HUM_USER_TABLE_START_ADR     0x40
#define SOFT_BAUDRATE  0xd0
#define TABLE_UPDATE    1

#define ITEM_CO2     50
#define PIC_VERSION 18 //UPDATE PIC VERSION FROM 14 TO 1
#define HIGH_START 2 // set so that it jumps at every second relatively
// the calibration of SERIAL_TIMEOUT_SECONDS is about 1 = 1.2seconds
// if set to 250, delay about 5 mins
#define SERIAL_TIMEOUT_SECONDS 10

#define INTS_PER_SECOND          147
#define T1_COUNT_MIN             50
#define T1_COUNT_MAX             255
#define T1_COUNT_STANDARD        50
#define HUM_OVERFLOW             500

//#define HUM_RH_TAB_START_ADD     0x40   //10POINT  10*4 = 40 BYTE A1A2A3A4
#define HUM_RH_MAX_COUNT         0x10   //          : A1A2 TICOUNT A3 A4 RH*100
#define Tem_offset               0x70



#define GRP 4
#define SAMPLE_CYCLE  13

#define ADJ_NUM   6000

//typedef  unsigned char     BYTE;

typedef struct S_RF_SETTINGS{
    BYTE sFSCTRL1;   // Frequency synthesizer control.
    BYTE sFSCTRL0;   // Frequency synthesizer control.
    BYTE sFREQ2;     // Frequency control word, high byte.
    BYTE sFREQ1;     // Frequency control word, middle byte.
    BYTE sFREQ0;     // Frequency control word, low byte.
    BYTE sMDMCFG4;   // Modem configuration.
    BYTE sMDMCFG3;   // Modem configuration.
    BYTE sMDMCFG2;   // Modem configuration.
    BYTE sMDMCFG1;   // Modem configuration.
    BYTE sMDMCFG0;   // Modem configuration.
    BYTE sCHANNR;    // Channel number.
    BYTE sDEVIATN;   // Modem deviation setting (when FSK modulation is enabled).
    BYTE sFREND1;    // Front end RX configuration.
    BYTE sFREND0;    // Front end RX configuration.
    BYTE sMCSM0;     // Main Radio Control State Machine configuration.
    BYTE sFOCCFG;    // Frequency Offset Compensation Configuration.
    BYTE sBSCFG;     // Bit synchronization Configuration.
    BYTE sAGCCTRL2;  // AGC control.
   BYTE sAGCCTRL1;  // AGC control.
    BYTE sAGCCTRL0;  // AGC control.
    BYTE sFSCAL3;    // Frequency synthesizer calibration.
    BYTE sFSCAL2;    // Frequency synthesizer calibration.
   BYTE sFSCAL1;    // Frequency synthesizer calibration.
    BYTE sFSCAL0;    // Frequency synthesizer calibration.
    BYTE sFSTEST;    // Frequency synthesizer calibration control
    BYTE sTEST2;     // Various test settings.
    BYTE sTEST1;     // Various test settings.
    BYTE sTEST0;     // Various test settings.
    BYTE sFIFOTHR;   // RXFIFO and TXFIFO thresholds.
    BYTE sIOCFG2;    // GDO2 output pin configuration
    BYTE sIOCFG0;    // GDO0 output pin configuration
    BYTE sPKTCTRL1;  // Packet automation control.
    BYTE sPKTCTRL0;  // Packet automation control.
    BYTE sADDR;      // Device address.
    BYTE sPKTLEN;    // Packet length.
} RF_SETTINGS;

#ifdef FREQ_915M
// 915m
RF_SETTINGS rfSettings = {
    0x0A,   // FSCTRL1   Frequency synthesizer control.
    0x00,   // FSCTRL0   Frequency synthesizer control.
    0x0C,   // FREQ2     Frequency control word, high byte.
    0x1D,   // FREQ1     Frequency control word, middle byte.
    0x89,   // FREQ0     Frequency control word, low byte.
    0x2D,   // MDMCFG4   Modem configuration.
    0x3B,   // MDMCFG3   Modem configuration.
    0x73,   // MDMCFG2   Modem configuration.
    0x22,   // MDMCFG1   Modem configuration.
    0xF8,   // MDMCFG0   Modem configuration.
    0x00,   // CHANNR    Channel number.
    0x00,   // DEVIATN   Modem deviation setting (when FSK modulation is enabled).
    0xB6,   // FREND1    Front end RX configuration.
    0x10,   // FREND0    Front end TX configuration.
    0x18,   // MCSM0     Main Radio Control State Machine configuration.
    0x1D,   // FOCCFG    Frequency Offset Compensation Configuration.
    0x1C,   // BSCFG     Bit synchronization Configuration.
    0xC7,   // AGCCTRL2  AGC control.
    0x00,   // AGCCTRL1  AGC control.
    0xB0,   // AGCCTRL0  AGC control.
    0xEA,   // FSCAL3    Frequency synthesizer calibration.
    0x2A,   // FSCAL2    Frequency synthesizer calibration.
    0x00,   // FSCAL1    Frequency synthesizer calibration.
    0x1F,   // FSCAL0    Frequency synthesizer calibration.
    0x59,   // FSTEST    Frequency synthesizer calibration.
    0x88,   // TEST2     Various test settings.
    0x31,   // TEST1     Various test settings.
    0x0B,   // TEST0     Various test settings.
    0x07,   // FIFOTHR   RXFIFO and TXFIFO thresholds.
    0x29,   // IOCFG2    GDO2 output pin configuration.
    0x06,   // IOCFG0D   GDO0 output pin configuration. 
    0x04,   // PKTCTRL1  Packet automation control.
    0x05,   // PKTCTRL0  Packet automation control.
    0x00,   // ADDR      Device address.
    0xFF    // PKTLEN    Packet length.
}; 
#endif
#ifdef FREQ_433M
// 433m
RF_SETTINGS rfSettings = {
    0x0C,   // FSCTRL1   Frequency synthesizer control.
    0x00,   // FSCTRL0   Frequency synthesizer control.
    0x10,   // FREQ2     Frequency control word, high byte.
    0xA7,   // FREQ1     Frequency control word, middle byte.
    0x62,   // FREQ0     Frequency control word, low byte.
    0x2D,   // MDMCFG4   Modem configuration.
    0x3B,   // MDMCFG3   Modem configuration.
    0x13,   // MDMCFG2   Modem configuration.
    0x22,   // MDMCFG1   Modem configuration.
    0xF8,   // MDMCFG0   Modem configuration.
    0x00,   // CHANNR    Channel number.
    0x62,   // DEVIATN   Modem deviation setting (when FSK modulation is enabled).
    0xB6,   // FREND1    Front end RX configuration.
    0x10,   // FREND0    Front end TX configuration.
    0x18,   // MCSM0     Main Radio Control State Machine configuration.
    0x1D,   // FOCCFG    Frequency Offset Compensation Configuration.
    0x1C,   // BSCFG     Bit synchronization Configuration.
    0xC7,   // AGCCTRL2  AGC control.
    0x00,   // AGCCTRL1  AGC control.
    0xB0,   // AGCCTRL0  AGC control.
    0xEA,   // FSCAL3    Frequency synthesizer calibration.
    0x2A,   // FSCAL2    Frequency synthesizer calibration.
    0x00,   // FSCAL1    Frequency synthesizer calibration.
    0x1F,   // FSCAL0    Frequency synthesizer calibration.
    0x59,   // FSTEST    Frequency synthesizer calibration.
    0x88,   // TEST2     Various test settings.
    0x31,   // TEST1     Various test settings.
    0x09,   // TEST0     Various test settings.
    0x07,   // FIFOTHR   RXFIFO and TXFIFO thresholds.
    0x29,   // IOCFG2    GDO2 output pin configuration.
    0x06,   // IOCFG0D   GDO0 output pin configuration. 
    0x04,   // PKTCTRL1  Packet automation control.
    0x05,   // PKTCTRL0  Packet automation control.
    0x00,   // ADDR      Device address.
    0xFF    // PKTLEN    Packet length.
};
#endif
// 868m
#ifdef FREQ_868M
RF_SETTINGS rfSettings = {
    0x0C,   // FSCTRL1   Frequency synthesizer control.
    0x00,   // FSCTRL0   Frequency synthesizer control.
    0x21,   // FREQ2     Frequency control word, high byte.
    0x62,   // FREQ1     Frequency control word, middle byte.
    0x76,   // FREQ0     Frequency control word, low byte.
    0x2D,   // MDMCFG4   Modem configuration.
    0x3B,   // MDMCFG3   Modem configuration.
    0x13,   // MDMCFG2   Modem configuration.
    0x22,   // MDMCFG1   Modem configuration.
    0xF8,   // MDMCFG0   Modem configuration.
    0x00,   // CHANNR    Channel number.
    0x62,   // DEVIATN   Modem deviation setting (when FSK modulation is enabled).
    0xB6,   // FREND1    Front end RX configuration.
    0x10,   // FREND0    Front end TX configuration.
    0x18,   // MCSM0     Main Radio Control State Machine configuration.
    0x1D,   // FOCCFG    Frequency Offset Compensation Configuration.
    0x1C,   // BSCFG     Bit synchronization Configuration.
    0xC7,   // AGCCTRL2  AGC control.
    0x00,   // AGCCTRL1  AGC control.
    0xB0,   // AGCCTRL0  AGC control.
    0xEA,   // FSCAL3    Frequency synthesizer calibration.
    0x2A,   // FSCAL2    Frequency synthesizer calibration.
    0x00,   // FSCAL1    Frequency synthesizer calibration.
    0x1F,   // FSCAL0    Frequency synthesizer calibration.
    0x59,   // FSTEST    Frequency synthesizer calibration.
    0x88,   // TEST2     Various test settings.
    0x31,   // TEST1     Various test settings.
    0x09,   // TEST0     Various test settings.
    0x07,   // FIFOTHR   RXFIFO and TXFIFO thresholds.
    0x29,   // IOCFG2    GDO2 output pin configuration.
    0x06,   // IOCFG0D   GDO0 output pin configuration. 
    0x04,   // PKTCTRL1  Packet automation control.
    0x05,   // PKTCTRL0  Packet automation control.
    0x00,   // ADDR      Device address.
    0xFF    // PKTLEN    Packet length.
};
#endif

#ifdef FREQ_315M
RF_SETTINGS rfSettings = {
    0x0C,   // FSCTRL1   
    0x00,   // FSCTRL0   
    0x0C,   // FREQ2     
    0x1D,   // FREQ1     
    0x89,   // FREQ0     
    0x2D,   // MDMCFG4
    0x3B,   // MDMCFG3   Modem configuration.
    0x03,   // MDMCFG2   Modem configuration.
    0x22,   // MDMCFG1   Modem configuration.
    0xF8,   // MDMCFG0   Modem configuration.
    0x00,   // CHANNR    Channel number.
    0x62,   // DEVIATN   Modem deviation setting (when FSK modulation is enabled).
    0xB6,   // FREND1    Front end RX configuration.
    0x10,   // FREND0    Front end TX configuration.
    0x18,   // MCSM0     Main Radio Control State Machine configuration.
    0x1D,   // FOCCFG    Frequency Offset Compensation Configuration.
    0x1C,   // BSCFG     Bit synchronization Configuration.
    0xC7,   // AGCCTRL2  AGC control.
    0x00,   // AGCCTRL1  AGC control.
    0xB0,   // AGCCTRL0  AGC control.
    0xEA,   // FSCAL3    Frequency synthesizer calibration.
    0x2A,   // FSCAL2    Frequency synthesizer calibration.
    0x00,   // FSCAL1    Frequency synthesizer calibration.
    0x1F,   // FSCAL0    Frequency synthesizer calibration.
    0x59,   // FSTEST    Frequency synthesizer calibration.
    0x88,   // TEST2     Various test settings.
    0x31,   // TEST1     Various test settings.
    0x0B,   // TEST0     Various test settings.
    0x07,   // FIFOTHR   RXFIFO and TXFIFO thresholds.
    0x29,   // IOCFG2    GDO2 output pin configuration.
    0x06,   // IOCFG0D   GDO0 output pin configuration. 
    0x04,   // PKTCTRL1  Packet automation control.
    0x05,   // PKTCTRL0  Packet automation control.
    0x00,   // ADDR      Device address.
    0xFF    // PKTLEN    Packet length.
};
#endif
//unsigned char filter_read_counter = 0;
//unsigned char current_channel_set = 3;








//*****************TEMP********************************//
int16 const temp_table[20] =
{25,   39,  61, 83, 102, 113, 112, 101, 85, 67, 51, 38, 28, 21, 15, 11, 8, 6 , 5, 19}; // note the lastest value is orignal of ADC value 


//*****************FUNC*********************************//
void initial ( void );
//////////////////////TEMP////////////////////////////////////////
unsigned INT16 Temp_filter_reading( unsigned char type );
unsigned int16 look_up_table(int16 count);
/////////////////////PIR//////////////////////////////////////////

/////////////////////LIGHT SENSOR////////////////////////////////

//////////////////////RF//////////////////////////////////////////
void SPI_write_data( unsigned int8 Regaddr, unsigned int8 Dvalue );
unsigned int8 SPI_read_data( unsigned int8 Regaddr );
void SPI_initial( void );
void SPI_end( void );
void SPI_write_strobe( unsigned int8 strobe );
void SPI_powerup_reset( void );
void SPI_reset_cc11( void );
void SPI_config( void );
unsigned int8 SPI_read_status( unsigned int8 Regaddr );
void SPI_write_burst(unsigned char Regaddr, unsigned char *Wbuf, unsigned char count);
void SPI_read_burst(unsigned char Regaddr, unsigned char *Rbuf, unsigned char count);

unsigned char Read_packet( unsigned char *Rbuffer, unsigned char *count );
void Write_packet( unsigned char *Wbuffer, unsigned char size );
void writeRfSettings( void );
