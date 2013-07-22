//#include <16F722.h>
//#device adc = 8 // use 8-bit ADC
#include <16F882.h>
#device adc = 10 // use 8-bit ADC
#fuses INTRC_IO,NOWDT,NOPROTECT,NOMCLR//BORV21//,NOBROWNOUT //PUT ,BROWNOUT,Internal RC Osci, Watchdog, No Protect, No Memory Clear;
#BUILD(nosleep)
#use delay(clock = 8000000)  //Adjusts the accuracy of the delay functions
#use standard_io(A)
#use standard_io(c)
//#use fast_io(b)
//#use fast_io(c)
#if 0
#define PWM_H  5
#define TOTAL_PWM_DUTY  10
#define TIMER2_POSTSCALE   1
#define TIMER2_START_VALUE  198
#define TIMER2_PERIOD   200 

#define PIC_VERSION     20 // 19 and up- extra byte 0x88 for set relay
//rev 20 SDA pin changed, also add extra check byte
/*
#define CPURESET        PIN_C4//used for main CPU reset 
#define SCL             PIN_C3  //used for I2C CLOCK
#define SDA             PIN_B7  //used for I2C DATA
#define RELAY1          PIN_C0
#define RELAY2          PIN_C1
#define RELAY3          PIN_C2
#define RELAY4          PIN_C5
#define RELAY5          PIN_C6

#define TEM_SENSOR      PIN_B0 

#define back_light      PIN_A7

#define AI1             PIN_A0 
#define AI2             PIN_A1
#define AI3             PIN_A2
#define AI4             PIN_A5
#define AI5             PIN_B2
#define AI6             PIN_B3
#define AI7             PIN_B1
#define AI8             PIN_B4

 
#define test            PIN_A4
#define CHANGE            1
#define NOT_CHANGE        0
*/
#define HIGH_START 400 // set so that it jumps at every second relatively
// the calibration of SERIAL_TIMEOUT_SECONDS is about 1 = 1.2seconds
// if set to 250, delay about 5 mins
#define SERIAL_TIMEOUT_SECONDS  4000//250

#define avg  49 //polling times between twice comunications ,it can up to 999,not test for after 999


int8 i2c_byte;
int1 serial_receive_watchdog ;
unsigned char serial_receive_timeout;
unsigned int16 high_count ;
unsigned int32 data_core;
unsigned int16 sampling_counter;

unsigned int16 analog_input_data[16];

int1 com_int_flag = 0;

int8 i2c_address;
//unsigned int16 analog_in[8];

int1 t_flag;
//unsigned int8 relay_duty;
//unsigned int16 heartbeat;
unsigned int8 relay_status; //indicator relay statue,open or close
int1 status_relay1 = 0;
int1 status_relay2 = 0;
int1 status_relay3 = 0;
int1 status_relay4 = 0;
int1 status_relay5 = 0;
int1 lcd_backlight;

int1 status_relay1_buff = 0;
int1 status_relay2_buff = 0;
int1 status_relay3_buff = 0;
int1 status_relay4_buff = 0;
int1 status_relay5_buff = 0;

int8 pwm_duty_couter = 0;


int8 relay_status_buff = 0;
int1 relay_change_flag = CHANGE; //0:not need change 1: need c
int8 relay_on_off_counter = 0;



void i2c_write_version();
short i2c_get_ack();
void i2c_give_ack();
void i2c_write( unsigned long ch );
unsigned char i2c_read();
void poll_i2c_bus();
void i2c_write_data();
void refresh_relay();
void set_relay();
void i2c_write_relay();
unsigned int16 filter_reading( );

void read_analog_input(int ix);

#endif
/*******************************************************************************
 SPI
 */
 void initial(void);
void SPI_write_data( BYTE Regaddr, BYTE Dvalue );
BYTE SPI_read_data( BYTE Regaddr );
void SPI_initial( void );
void SPI_write_strobe( BYTE strobe );
void SPI_powerup_reset( void );
void SPI_reset_cc11( void );
void SPI_config( void );
BYTE SPI_read_status( BYTE Regaddr );
void SPI_write_burst(BYTE Regaddr, BYTE *Wbuf, BYTE count);
void SPI_read_burst(BYTE Regaddr, BYTE *Rbuf, BYTE count);

//BYTE Read_packet( BYTE *Rbuffer, BYTE *count );
unsigned char Read_packet( unsigned char *Rbuffer, unsigned char *count );
void Write_packet( BYTE *Wbuffer, BYTE size );
void writeRfSettings( void);

#define GDO0         PIN_C6
#define SPI_CS       PIN_C2
#define SPI_CLK      PIN_C0
#define SPI_MOSI     PIN_C1
#define SPI_MISO     PIN_C5

#define LED_RED        PIN_A1
#define LED_GREEN      PIN_A0

/*

#define GDO0         PIN_C1
#define SPI_CS       PIN_C2
#define SPI_CLK      PIN_C3
#define SPI_MOSI     PIN_C5
#define SPI_MISO     PIN_C4

#define LED_RED        PIN_A1
#define LED_GREEN      PIN_A0
*/
//#define FREQ_915M

#define DATA_LEN 8


BYTE Rbuffer[ DATA_LEN ] = {0};
BYTE Wbuffer[ 2 ] = {0xaa,0x55};
BYTE PA_config[8] = {0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0};

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

//#ifdef FREQ_915M
// 915m
RF_SETTINGS rfSettings = {
    0x0C,   // FSCTRL1   Frequency synthesizer control.
    0x00,   // FSCTRL0   Frequency synthesizer control.
    0x23,   // FREQ2     Frequency control word, high byte.
    0x31,   // FREQ1     Frequency control word, middle byte.
    0x3B,   // FREQ0     Frequency control word, low byte.
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
    0x06,   // IOCFG0D   GDO0 output pin configuration. Refer to SmartRF� Studio User Manual for detailed pseudo register explanation.
    0x04,   // PKTCTRL1  Packet automation control.
    0x05,   // PKTCTRL0  Packet automation control.
    0x00,   // ADDR      Device address.
    0xFF    // PKTLEN    Packet length.
}; 
//#endif

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
