//#include <16F722.h>
//#device adc = 8 // use 8-bit ADC
#include <16F882.h>
#device adc = 10 // use 8-bit ADC
#fuses INTRC_IO,NOWDT,NOPROTECT,NOMCLR, BROWNOUT, PUT,BORV21 // Internal RC Osci, Watchdog, No Protect, No Memory Clear;
#use delay(clock = 4000000)  //Adjusts the accuracy of the delay functions
#use standard_io(A)
#use fast_io(b)
#use fast_io(c)

#define PIC_VERSION     22 // 19 and up- extra byte 0x88 for set relay
//rev 20 SDA pin changed, also add extra check byte

#define CPURESET        PIN_C4//used for main CPU reset 
#define SCL             PIN_C3  //used for I2C CLOCK
#define SDA             PIN_B7  //used for I2C DATA

#if 1
#define RELAY1          PIN_C0
#define RELAY2          PIN_C1
#define RELAY3          PIN_C2
#define RELAY4          PIN_C5
#define RELAY5          PIN_C6
#endif

#define TEM_SENSOR      PIN_B0 
#define LIGHT_SENSOR    PIN_B5

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

#define HIGH_START 400 // set so that it jumps at every second relatively
// the calibration of SERIAL_TIMEOUT_SECONDS is about 1 = 1.2seconds
// if set to 250, delay about 5 mins
#define SERIAL_TIMEOUT_SECONDS  4000//250

#define avg  49 //polling times between twice comunications ,it can up to 999,not test for after 999
#define OCC

int8 i2c_byte;
int1 serial_receive_watchdog ;
unsigned char serial_receive_timeout;
unsigned int16 high_count ;
unsigned int32 data_core;
unsigned int16 sampling_counter;

unsigned int16 analog_input_data[16];
#ifdef OCC
unsigned int16 occ_data[3];
#endif
int1 com_int_flag = 0;

int8 i2c_address;
//unsigned int16 analog_in[8];

int1 t_flag;
//unsigned int8 relay_duty;
//unsigned int16 heartbeat;
unsigned int8 relay_status; //indicator relay statue,open or close
int1 status_relay1;
int1 status_relay2;
int1 status_relay3;
int1 status_relay4;
int1 status_relay5;
int1 lcd_backlight;



void initial(void);
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


#define GDO0         PIN_C6
#define SPI_CS       PIN_C2
#define SPI_CLK      PIN_C0
#define SPI_MOSI     PIN_C1
#define SPI_MISO     PIN_C5

#define LED_RED        PIN_A1
#define LED_GREEN      PIN_A0

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

#define DATA_LEN 8

unsigned char Rbuffer[ DATA_LEN ] = {0};
BYTE Wbuffer[ 2 ] = {0xaa,0x55};
BYTE PA_config[8] = {0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0};

