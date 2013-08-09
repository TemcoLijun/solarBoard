
//REV 3_1 is used for 16f882, i2c potocol is 16 bits,and change I2C  potocal to fit hum and clock function
//REV 3 is used for 16f882, i2c potocol is 10 bits

//REV 2_1 is used for 16f722, i2c potocol is 16 bits
//REV 2 is used for 16f722, i2c potocol is 10 bits




#include "pic.h"
#ignore_warnings 203




#INT_RB
void external_isr()
{
  com_int_flag = 1;
  //output_low(RELAY2);
  poll_i2c_bus();
  //output_c(0xee | status_relay1);

}

//////////////////////////////
void main(void)
{
  int i;
  int8 number;
  BYTE read_count ;
  t_flag=0;
  data_core=0;
  sampling_counter=0;
  output_low(test);  //debug eeprom issue

  //Iniatialize relay when power up
  //output_c(0xef);

  initial();

  //status_relay1=0;
  //status_relay2=0;
  //status_relay3=0;
  //status_relay4=0;
  //status_relay5=0;
  lcd_backlight=0;

  for(i=0; i<16; i++)
    analog_input_data[i]=0;

   occ_data[0]= 23;
   occ_data[1]= 45;
   occ_data[2]= 67;
  while(1)
  {

    //   status_relay1=~status_relay1;
    //   output_high(test);
    //   delay_ms(1000);
    //   output_low(test);
    //   delay_ms(1000);


  //  refresh_relay();
 //   delay_us(10);


    for(i=0; i<10; i++)
      read_analog_input(i);
  //for cc1101    
    read_count = 6;
    if( Read_packet(Rbuffer, &read_count))
    {
      for(number = 0; number < 3; number++)
      {
         occ_data[number] = ( int16)(Rbuffer[number*2]*256)+Rbuffer[number*2+1];
      }
 /*     if( Rbuffer[0] == 0xaa )
      {
         occ_data[0]++;
         Rbuffer[0] = 0;
      }
      if(Rbuffer[1] == 0x55)
      {
         occ_data[1]++;
         occ_data[2]++;
         Rbuffer[1] = 0;
      }*/
    }
    //for reset
   high_count--;
    if (high_count == 0)    // once it is zero then check for timeout every 1.57s
    {
      high_count = HIGH_START;
      serial_receive_timeout--;  

      if (serial_receive_timeout == 0)
      {
        serial_receive_timeout = SERIAL_TIMEOUT_SECONDS;
        // if nothing yet from I2C, then we reset chip
        if (serial_receive_watchdog == 0)
        {
          output_high(CPURESET); // reset the 8051 chip
          delay_ms (200);    // short delay
          output_low(CPURESET);  // back to normal runing of the 8051 chip
          delay_ms (200);    // short delay
        }

        serial_receive_watchdog = 0;
      }
    }

  }
}

void initial(void)
{
  setup_oscillator(OSC_4MHZ);

  // Port Directions
  set_tris_b(0xff);
  //set_tris_c(0x88);
  set_tris_c (0xe8);


  output_high(CPURESET); 


  SETUP_ADC_PORTS(sAN12);
  SETUP_ADC_PORTS(sAN0);
  SETUP_ADC_PORTS(sAN1);
  SETUP_ADC_PORTS(sAN2);
  SETUP_ADC_PORTS(sAN4);
  SETUP_ADC_PORTS(sAN8);
  SETUP_ADC_PORTS(sAN9);
  SETUP_ADC_PORTS(sAN10);
  SETUP_ADC_PORTS(sAN11);
  SETUP_ADC_PORTS(sAN13);

  SETUP_ADC_PORTS(VSS_VREF); //using VREF for adc intput referenece
  SETUP_ADC(ADC_CLOCK_DIV_8);//SETUP ADC CLOCK SOURCE  
  // set_adc_channel(12);       

  // SETUP_TIMER_1( T1_INTERNAL | T1_DIV_BY_8 );
  // SET_TIMER1(0);//about 524ms overflow

  input(TEM_SENSOR);
  input(LIGHT_SENSOR);
  input(AI1);
  input(AI2);
  input(AI3);
  input(AI4);
  input(AI5);
  input(AI6);
  input(AI7);
  input(AI8);
 

  ENABLE_INTERRUPTS(GLOBAL);
  ENABLE_INTERRUPTS(INT_RB7);   // int for I2C

  high_count = HIGH_START;
  serial_receive_watchdog = 0;
  serial_receive_timeout = SERIAL_TIMEOUT_SECONDS; 
  SPI_initial();
  delay_ms(100);
  output_low(CPURESET);


}

#define LO 0 // Low
#define HI 1 // High
#define XX 2 // Don't Care

#define CL_AS_EXPECTED(s) ((s == XX) || (INPUT(SCL) == s))
#define DA_AS_EXPECTED(s) ((s == XX) || (INPUT(SDA) == s))

unsigned int waitCLcount;
unsigned int waitDAcount;

char waitCL(char state, unsigned int maxcount, char DAstate)
{
  waitCLcount = 0;
  while(1)
  {
    if( ! DA_AS_EXPECTED(DAstate) )
      return 0;

    if( input(SCL)==state )
      break;

    waitCLcount++;
    if (waitCLcount >= maxcount)
      return 0;
  }

  return 1;
}


char waitDA(char state, unsigned int maxcount, char CLstate)
{
  waitDAcount = 0;
  while(1)
  {
    if( ! CL_AS_EXPECTED(CLstate) )
      return 0;

    if( input(SDA)==state )
      break;

    waitDAcount++;
    if (waitDAcount >= maxcount)
      return 0;
  }

  return 1;
}
///////////////////////////////////////////////////////////////////////////////
void poll_i2c_bus()
{

  if( ! DA_AS_EXPECTED(LO) )
    return;

  if( ! CL_AS_EXPECTED(HI) )
    return;

  if( ! waitCL(LO,50,LO) )
    return;
  //analog_input_data[1] = waitCLcount;  // Debug, To measure I2C iterations

  if( ! waitCL(HI,50,XX) )
    return;
  //analog_input_data[2] = waitCLcount;  // Debug, To measure I2C iterations

  if( ! DA_AS_EXPECTED(HI) )
    return;

  // watch for 2nd start condition
  if( ! waitDA(LO,50,HI) )
    return;
  //analog_input_data[3] = waitDAcount;  // Debug, To measure I2C iterations

  if( ! waitCL(LO,50,XX) )
    return;
  //analog_input_data[4] = waitCLcount;  // Debug, To measure I2C iterations

  // Read the address being broadcast
  i2c_address = i2c_read();  

  if (i2c_address == 0xc1)
    return;

  if (i2c_address == 0xc2)
    return;

  if (i2c_address == 0xd3)
    return;

  if (i2c_address == 0xd4)
    return;                     


  // If address matches, send ACK and write data
  if (i2c_address == 0xb1)    //main cpu CMD, for data require
  {
    serial_receive_watchdog = 1;
    i2c_write_data();
  }
  else if (i2c_address == 0xb2)//main cpu CMD, for PIC version requirement
  {
    serial_receive_watchdog = 1;
    i2c_write_version();
  }
  #if 1
  else if (i2c_address == 0xb3) //main cpu CMD, relays control 
  {
    serial_receive_watchdog = 1;
    i2c_give_ack();
    
    relay_status = i2c_read();
    i2c_give_ack();

    i2c_write(relay_status & 0x00ff);
      if (!i2c_get_ack())
       return;   
       
    i2c_byte = i2c_read();    
    if(i2c_byte == 0x88)
      set_relay();
  }
#endif 
  waitCL(HI,50,XX);
}

////////////////////////////////////////////////////////////////////////////////
// 8-bit read
unsigned char i2c_read()
{
  unsigned char i, data1 = 0;

  for( i=0; i<8; i++ )
  {
    if( ! waitCL(HI,50,XX) )
      return 0;

    // Read bit from data line
    data1 = ( data1 << 1 ) | input(SDA);

    if( ! waitCL(LO,50,XX) )
      return 0;
  }

  return data1;
}

////////////////////////////////////////////////////////////////////////////////
// 16-bit write
void i2c_write( unsigned int16 ch )
{
  int8 i = 16;

  // restart the watchdog
  // restart_wdt();

  set_tris_b(0x7f);

  do
  {
    // Write bit to data line
    if ( ch & 0x8000 )
      output_high(SDA);
    else
      output_low(SDA);

    if( ! waitCL(HI,50,XX) )
    {
      goto i2c_write_exit;
    }
    //analog_input_data[5] = waitCLcount;  // Debug, To measure I2C iterations


    ch<<=1;

    if( ! waitCL(LO,50,XX) )
    {
      goto i2c_write_exit;
    }
    //analog_input_data[6] = waitCLcount;  // Debug, To measure I2C iterations

  } while( --i != 0 );

i2c_write_exit:
  output_high(SDA);
  set_tris_b(0xff);

}

////////////////////////////////////////////////////////////////////////////////
// MDF 12/01/04 - changed so that data line is not changed while clock is high
void i2c_give_ack()
{
  set_tris_b(0x7f);

  output_low(SDA);

  if( ! waitCL(HI,50,XX) )
  {
    goto give_ack_end;
  }

  if( ! waitCL(LO,50,XX) )
  {
    goto give_ack_end;
  }

give_ack_end:
  output_high(SDA);
  set_tris_b(0xff);
//input(SDA);
  return;

}
////////////////////////////////////////////////////////////////////////////////
short i2c_get_ack()
{
  short ack;

  if( ! waitCL(HI,50,XX) )
  {
    return 0;
  }
  //analog_input_data[7] = waitCLcount;  // Debug, To measure I2C iterations

  ack = ! input(SDA);

  if( ! waitCL(LO,50,XX) )
  {
    return 0;
  }
  //analog_input_data[8] = waitCLcount;  // Debug, To measure I2C iterations

  if( ! waitDA(HI,50,LO) )
  {
    return 0;
  }

  return ack;
}
////////////////////////////////////////////////////////////////////////////////
// MDF 12/01/04
void i2c_write_version()
{

  // Write data to the bus and receive ACK each time
  i2c_give_ack();
  i2c_write(PIC_VERSION);

  if (!i2c_get_ack())
    return;

  i2c_write(0x69);

  return;
}
////////////////////////////////////////////////////////////////////////////////
void i2c_write_relay()
{

  // Write data to the bus and receive ACK each time
  i2c_give_ack();
  i2c_write(relay_status);

  if (!i2c_get_ack())
    return;

  i2c_write(0x96);

  return;
}
////////////////////////////////////////////////////////////////////////////////
void i2c_write_data()
{
  unsigned int16 analog_buf;
  unsigned int16 checksum = 0;
  unsigned char i;

  i2c_give_ack();

  for(i=0;i<9;i++)                  //send analog input data( internal Temp + 8 channels)
  {
    analog_buf = analog_input_data[i] & 0xffff;
    checksum += analog_buf;

    if( i>0 )
    {
      if( !i2c_get_ack())
        return;
    }

    i2c_write(analog_buf);  
  }

  analog_buf=0;       //sent co2 data high 8 bits
  checksum += analog_buf;
  if (!i2c_get_ack())
    return;
  i2c_write(analog_buf);

  analog_buf = analog_input_data[9];       //sent co2 data low 8 bits
  checksum += analog_buf;
  if (!i2c_get_ack())
    return;
  i2c_write(analog_buf);
  
  #ifdef OCC
  analog_buf = occ_data[0];       //sent co2 data low 8 bits
  checksum += analog_buf;
  if (!i2c_get_ack())
    return;
  i2c_write(analog_buf);
  analog_buf = occ_data[1];       //sent co2 data low 8 bits
  checksum += analog_buf;
  if (!i2c_get_ack())
    return;
  i2c_write(analog_buf);
  analog_buf = occ_data[2];       //sent co2 data low 8 bits
  checksum += analog_buf;
  if (!i2c_get_ack())
    return;
  i2c_write(analog_buf);
  #endif
  

  i2c_get_ack();

  checksum = checksum & 0xffff;
  i2c_write(checksum);

  return;

}

void refresh_relay()
{
/*  output_bit( RELAY1, !status_relay1);
  output_bit( RELAY2, !status_relay2);
  output_bit( RELAY3, !status_relay3);
  output_bit( RELAY4, !status_relay4);
  output_bit( RELAY5, !status_relay5);*/
  output_bit( back_light, !lcd_backlight);
}

void set_relay()
{
/*  status_relay1=(int1)(relay_status&0x01);
  status_relay2=(int1)((relay_status>>1)&0x01);
  status_relay3=(int1)((relay_status>>2)&0x01);
  status_relay4=(int1)((relay_status>>3)&0x01);
  status_relay5=(int1)((relay_status>>4)&0x01); */
  lcd_backlight=(int1)((relay_status>>5)&0x01); 
}

unsigned int16 filter_reading( )
{
  unsigned int16 AI;

  unsigned int16 adc_buffer;

  data_core=0;
  while(sampling_counter<=avg)
  {
    com_int_flag = 0;
    adc_buffer = read_adc();
    if(com_int_flag == 0)
    {
      data_core += adc_buffer;
      sampling_counter++;
    }
  }

  AI = (data_core/sampling_counter);
  data_core = 0;
  sampling_counter = 0;

  return AI ;
}   

BYTE CONST MuxChannels[10]= {12,0,1,2,4,8,9,10,11,13};


void read_analog_input(char ix)
{
  unsigned int16 v;

  set_adc_channel(MuxChannels[ix]); 
  delay_us(10);

  v = filter_reading();

  disable_interrupts(GLOBAL);
  //if( ix == 0 )  // Debug, To measure I2C iterations 
  analog_input_data[ix] = v;
  enable_interrupts(GLOBAL);

  delay_us(20);
}


/*******************************************************************************************
 SPI
 */
 //SPI initial
void SPI_initial( void )
{
   
   output_high(SPI_CLK); //make the clk and mosi equal to 0 
   output_low(SPI_MOSI);
   output_high(SPI_CS);// = 1;
   delay_ms( 5 );
   
//   setup_spi( SPI_MASTER| SPI_L_TO_H| SPI_CLK_DIV_16 | SPI_XMIT_L_TO_H | SPI_SAMPLE_AT_END);
   
   
   SPI_powerup_reset();//we need open
   delay_us( 1 );
   writeRfSettings( );
   SPI_write_burst(PATABLE,PA_config,8);//here config the patable register, we choose the 0xc0
   output_low(SPI_CLK);// = 0;
   SPI_write_strobe( SIDLE );
   SPI_write_strobe( SCAL );
 //  SPI_write_strobe( SFRX );
   output_high(SPI_MISO);//SPI_MISO = 1;
  // delay_us( 1 );   
  // output_low(SPI_CS);// = 0;
}

//SPI Write data func
void SPI_write_byte( int8 Wdata )
{
   unsigned int8 i;
   for( i=0; i<8; i++ )
   {
      if( Wdata & 0x80 )
         output_high(SPI_MOSI);// = 1;
      else 
         output_low(SPI_MOSI);// = 0;
      
     // output_high(SPI_CLK);// = 0;
       output_low(SPI_CLK);
    //  delay_us( 2 );
      Wdata <<= 1;
      delay_us( 2 );
    //  output_low(SPI_CLK);// = 1;
    output_high(SPI_CLK);
   }
}
//SPI read data func
unsigned int8 SPI_read_byte( void )
{
   unsigned int8 i;
   unsigned int8 Rdata = 0;
  // output_high( SPI_MISO );//when this line free set high
   //while(input(SPI_MISO));
//   output_low(SPI_CS);
   for( i=0; i<8; i++ )
   {
   //  output_high(SPI_CLK);
   output_low(SPI_CLK);
     delay_us( 3 );
     Rdata <<= 1;
     delay_us( 3 );
     if( input(SPI_MISO) == 1 )
     Rdata |= 0x01;
     else
     Rdata &= 0xFE;
     delay_us( 3 );
   //  Rdata <<= 1;
   //  output_low(SPI_CLK);
   output_high(SPI_CLK);
     delay_us( 3 );
    // SPI_CLK = 1;           
   }
   return Rdata;
}

void SPI_write_data( BYTE regAddr, BYTE data)
{
   output_low( SPI_CS);
   while( input( SPI_MISO));
   SPI_write_byte( regAddr & 0x7f);
   delay_us(1);
   SPI_write_byte( data);
   output_high( SPI_CS);
}

BYTE SPI_read_data( BYTE regAddr)
{
   BYTE data;
   output_low( SPI_CS);
   while( input( SPI_MISO));
   SPI_write_byte( regAddr | 0x80);
   //if( spi_data_is_in())
   delay_us( 1 );
   data = SPI_read_byte( );
   output_high( SPI_CS );
   delay_us( 1 );
   return data;
}

//read data from the cc1101 register
BYTE SPI_read_status( BYTE regAddr )
{
   BYTE stat;
   output_low( SPI_CS );
   while( input( SPI_MISO ));
   SPI_write_byte( Regaddr | 0xc0 );
   delay_us( 1 );
  // if( spi_data_is_in())
   stat = SPI_read_byte( );
   output_high( SPI_CS );
    delay_us( 1 );
   return stat;
}

//set the strober register 
void SPI_write_strobe( BYTE strobe )
{
   output_low( SPI_CS );
   while( input( SPI_MISO ));
   SPI_write_byte( strobe );
    delay_us( 1 );
   output_high( SPI_CS );
}

//write the burst
void SPI_write_burst(BYTE regAddr, BYTE *Wbuf, BYTE count)
{
   BYTE i;
   output_low( SPI_CS);
   while( input( SPI_MISO ));
   SPI_write_byte( regAddr | 0x40 );
   for(i=0; i<count;i++)
   {
     SPI_write_byte( Wbuf[i] ); 
     delay_us( 5 );
   }
   output_high( SPI_CS );
}

//Read the burst
void SPI_read_burst(BYTE regAddr, BYTE *Rbuf, BYTE count)
{
   BYTE i;
   output_low( SPI_CS);
   while( input( SPI_MISO));
   SPI_write_byte( regAddr | 0xc0);
   for( i=0; i<count; i++)
   {
      Rbuf[i] = SPI_read_byte();
   }
   output_high( SPI_CS);
}

//RESET CC1101 chip
void SPI_reset_cc11( void )
{
 //  SPI_write_strobe( SIDLE );
// output_high( LED_GREEN);
   output_low( SPI_CS );
   while( input( SPI_MISO ));
//   output_high( LED_GREEN);
   SPI_write_byte( SRES );
   delay_us( 1 );
   output_high( SPI_CS );
}
//power on reset 
void SPI_powerup_reset( void )
{
   output_low( SPI_CS );
   delay_us( 2 );
   output_high( SPI_CS );
    delay_us( 41 );
   SPI_reset_cc11();
}


//write the packet to cc1101
void Write_packet( BYTE *Wbuffer, BYTE size )
{
   SPI_write_strobe( SIDLE );
  // delay_ms(10);
   SPI_write_strobe( SFTX );
 //  delay_ms(10); 
   SPI_write_data( TX_FIFO, size );
   SPI_write_burst( TX_FIFO,Wbuffer,size ); 
   SPI_write_strobe( STX );
 //  delay_ms(10); //output_high( LED_RED );
//   output_low( LED_GREEN ); //for test
   while(!input(GDO0)); // too fast
   while(input(GDO0));
 //  delay_ms(10);
  
  while((SPI_read_status(TXBYTES)&0x7F) != 0x00 )
  {
      SPI_write_strobe( SIDLE );
      return;
  }
   SPI_write_strobe( SFRX );
  // SPI_write_strobe( SIDLE );
}
unsigned char Read_packet( unsigned char *Rbuffer, unsigned char *count )
{
   unsigned char state[2];
   unsigned char size = 0;
   unsigned char i = (*count)* 4;
//   SPI_write_strobe( SFRX );
   SPI_write_strobe( SRX );
   delay_ms(5);
 //  output_high( LED_RED );delay_ms(30);
 //  delay_ms(30);
   if(SPI_read_status(RXBYTES)==0)
   {
    //  output_high( LED_GREEN );//
      return 0;      
   }
   while(input(GDO0))
   {
      --i;
      if(i<1)
      {
         SPI_write_strobe( SFRX );
         SPI_write_strobe(SIDLE);
         return 0;
      }
   }
//   output_high( LED_GREEN );
   if(SPI_read_status(RXBYTES))
   {
      size = SPI_read_data( RX_FIFO );
      if(size <= *count)
      {
         SPI_read_burst( RX_FIFO,Rbuffer,size ); 
         *count = size;
         SPI_read_burst( RX_FIFO,state,2 );
     //    output_high( LED_GREEN );//
         SPI_write_strobe( SFRX );
         SPI_write_strobe( SRX );//add this command at here for test
         return (state[1]&0x80);
      }
   }
   else
   {
      *count = size;
      SPI_write_strobe(SIDLE);
      SPI_write_strobe( SFRX );
      return 0;
   }
}

void writeRfSettings( void)
{
   SPI_write_data( FSCTRL1, 0x0C);
   SPI_write_data( FSCTRL0, 0x00);
   SPI_write_data( FREQ2, 0x23);
   SPI_write_data( FREQ1, 0x31);
   SPI_write_data( FREQ0, 0x3B);
   SPI_write_data( MDMCFG4, 0x2D);
   SPI_write_data( MDMCFG3, 0x3B);
   SPI_write_data( MDMCFG2, 0x13);
   SPI_write_data( MDMCFG1, 0x22);
   SPI_write_data( MDMCFG0, 0xF8);
   SPI_write_data( CHANNR, 0x00);
   SPI_write_data( DEVIATN, 0x62);
   SPI_write_data( FREND1, 0xB6);
   SPI_write_data( FREND0, 0x10);
   SPI_write_data( MCSM0, 0x18);
   SPI_write_data( FOCCFG, 0x1D);
   SPI_write_data( BSCFG, 0x1C);
   SPI_write_data( AGCCTRL2, 0xC7);
   SPI_write_data( AGCCTRL1, 0x00);
   SPI_write_data( AGCCTRL0, 0xB0);
   SPI_write_data( FSCAL3, 0xEA);
   SPI_write_data( FSCAL2, 0x2A);
   SPI_write_data( FSCAL1, 0x00);
   SPI_write_data( FSCAL0, 0x1F);
   SPI_write_data( FSTEST, 0x59);
   SPI_write_data( TEST2, 0x88);
   SPI_write_data( TEST1, 0x31);
   SPI_write_data( TEST0, 0x09);
   SPI_write_data( FIFOTHR, 0x07);
   SPI_write_data( IOCFG2, 0x29);
   SPI_write_data( IOCFG0, 0x06);
   SPI_write_data( PKTCTRL1, 0x04);
   SPI_write_data( PKTCTRL0, 0x05);
   SPI_write_data( ADDR, 0x00);
   SPI_write_data( PKTLEN, 0xFF);

}
