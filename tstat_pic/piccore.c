
//REV 3_1 is used for 16f882, i2c potocol is 16 bits,and change I2C  potocal to fit hum and clock function
//REV 3 is used for 16f882, i2c potocol is 10 bits

//REV 2_1 is used for 16f722, i2c potocol is 16 bits
//REV 2 is used for 16f722, i2c potocol is 10 bits




#include "pic.h"
#ignore_warnings 203


void writeRfSettings( RF_SETTINGS *pRfSettings);
//////////////////////////////
void main(void)
{
   BYTE Rval;
   BYTE read_count ;
   initial(); 
   delay_ms(10);
   int8 leng = 0;
   
 //  setup_wdt( WDT_2304MS);
   
   while(1)
   {
      #if 1
     //write the data into the FIFO buffer 
     if( 0 )//WR_flg )
     {
      output_high( LED_GREEN );
   //   output_high( LED_RED );
        delay_ms( 30 ); 
      leng = 2;
      Write_packet( Wbuffer, leng );
      output_low( LED_GREEN );
      delay_ms( 30 );
      }
    
     else    //read the data from the FIFO buffer
     {  
      read_count = 2;
  //    output_high( LED_RED );//delay_ms( 100 );
         if( Read_packet(Rbuffer, &read_count))//sizeof( Rbuffer )))
         {
        //     restart_wdt();
      //     output_high( LED_GREEN );delay_ms( 30 );  output_low( LED_GREEN );//delay_ms( 100 );
      //      for(recv_count = 0; recv_count < 8; recv_count++ )
       //     {
               if( Rbuffer[0] == 0xaa )
               {
                  output_high( LED_GREEN );
                  delay_ms( 200 );
                  output_low( LED_GREEN );
             //     delay_ms( 200 );
                  Rbuffer[0] = 0;
               }
               if(Rbuffer[1] == 0x55)
                {
                  output_high( LED_RED );
                  delay_ms( 200 );
                  output_low( LED_RED );
              //    delay_ms( 200 );
                  Rbuffer[1] = 0;
               }
               else
               {                                                        
             //     output_high( LED_RED );
                  output_low( LED_GREEN );
               }
            }
            else
            {
               output_low( LED_RED );
            }
          //  delay_ms( 100 );   
          //  output_low( LED_RED );
      //   }      
     }
   #endif
   #if 0
  //    output_high( LED_RED );
  //    output_high( LED_GREEN );
      SPI_write_data( CHANNR, 0x04 );
   //   SPI_write_burst( ADDR,Wbuffer,4 );
   //    delay_us(2);
       Rval = SPI_read_data( CHANNR );
   //   SPI_read_burst( ADDR,Rbuffer,4 );
    //  SPI_end();
       delay_us( 2 );

      if( Rval == 0x04 )
      {
         output_low( LED_GREEN );
   //      output_low( LED_RED );
        // delay_us( 100 );
  //       delay_ms( 30 );
         output_high( LED_GREEN );
    //     output_high( LED_RED );
       //  delay_us( 100 );
//         delay_ms( 30 );
         Rval = 0;
      }
      if( Rval == 0x00 )
      {
        output_low( LED_RED );
        // delay_us( 100 );
        delay_ms( 30 );
         output_high( LED_RED );
       //  delay_us( 100 );
         delay_ms( 30 );
         Rval = 0;
      }
      else
      {
      //   delay_ms( 1000 );
         output_low( LED_GREEN );
         output_low( LED_RED );
      //   delay_ms( 1000 );
      }
   #endif
   }
}

void initial(void)
{
  setup_oscillator(OSC_8MHZ);
//  SETUP_TIMER_0 (T0_DIV_4) ;
  SETUP_TIMER_1( T1_INTERNAL | T1_DIV_BY_8 );
   SET_TIMER1(0);//about 524ms overflow
  
// SETUP_ADC (ADC_CLOCK_DIV_32) ;
//  SETUP_ADC_PORTS (VSS_VDD) ;
  
  set_tris_c (0x60); //set up  bit5 ,bit1 of c port input 10/12
//  set_tris_a (0x20); //set up  bit5 of a port input
   
//  set_tris_c (0x44);
  SPI_initial();

//  output_high( LED_GREEN);
/*
  // Port Directions
  set_tris_b(0xff);
  set_tris_c(0x88);


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

  SETUP_ADC_PORTS(VSS_VREF); //using VREF for adc intput referenece
  SETUP_ADC(ADC_CLOCK_DIV_8);//SETUP ADC CLOCK SOURCE  
  // set_adc_channel(12);       

  // SETUP_TIMER_1( T1_INTERNAL | T1_DIV_BY_8 );
  // SET_TIMER1(0);//about 524ms overflow

  input(TEM_SENSOR);
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
  delay_ms(100);
  output_low(CPURESET);
*/
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
   
   
   SPI_powerup_reset();
   delay_us( 1 );
   writeRfSettings( &rfSettings);
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
/*
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
*/
void writeRfSettings( RF_SETTINGS *pRfSettings)
{
   SPI_write_data( FSCTRL1, pRfSettings->sFSCTRL1);
   SPI_write_data( FSCTRL0, pRfSettings->sFSCTRL0);
   SPI_write_data( FREQ2, pRfSettings->sFREQ2);
   SPI_write_data( FREQ1, pRfSettings->sFREQ1);
   SPI_write_data( FREQ0, pRfSettings->sFREQ0);
   SPI_write_data( MDMCFG4, pRfSettings->sMDMCFG4);
   SPI_write_data( MDMCFG3, pRfSettings->sMDMCFG3);
   SPI_write_data( MDMCFG2, pRfSettings->sMDMCFG2);
   SPI_write_data( MDMCFG1, pRfSettings->sMDMCFG1);
   SPI_write_data( MDMCFG0, pRfSettings->sMDMCFG0);
   SPI_write_data( CHANNR, pRfSettings->sCHANNR);
   SPI_write_data( DEVIATN, pRfSettings->sDEVIATN);
   SPI_write_data( FREND1, pRfSettings->sFREND1);
   SPI_write_data( FREND0, pRfSettings->sFREND0);
   SPI_write_data( MCSM0, pRfSettings->sMCSM0);
   SPI_write_data( FOCCFG, pRfSettings->sFOCCFG);
   SPI_write_data( BSCFG, pRfSettings->sBSCFG);
   SPI_write_data( AGCCTRL2, pRfSettings->sAGCCTRL2);
   SPI_write_data( AGCCTRL1, pRfSettings->sAGCCTRL1);
   SPI_write_data( AGCCTRL0, pRfSettings->sAGCCTRL0);
   SPI_write_data( FSCAL3, pRfSettings->sFSCAL3);
   SPI_write_data( FSCAL2, pRfSettings->sFSCAL2);
   SPI_write_data( FSCAL1, pRfSettings->sFSCAL1);
   SPI_write_data( FSCAL0, pRfSettings->sFSCAL0);
   SPI_write_data( FSTEST, pRfSettings->sFSTEST);
   SPI_write_data( TEST2, pRfSettings->sTEST2);
   SPI_write_data( TEST1, pRfSettings->sTEST1);
   SPI_write_data( TEST0, pRfSettings->sTEST0);
   SPI_write_data( FIFOTHR, pRfSettings->sFIFOTHR);
   SPI_write_data( IOCFG2, pRfSettings->sIOCFG2);
   SPI_write_data( IOCFG0, pRfSettings->sIOCFG0);
   SPI_write_data( PKTCTRL1, pRfSettings->sPKTCTRL1);
   SPI_write_data( PKTCTRL0, pRfSettings->sPKTCTRL0);
   SPI_write_data( ADDR, pRfSettings->sADDR);
   SPI_write_data( PKTLEN, pRfSettings->sPKTLEN);
}

/*
//read the packet from cc1101
BYTE Read_packet( BYTE *Rbuffer, BYTE *count )
{
   BYTE state[2];
   BYTE size = 0;
   BYTE i = (*count)* 4;
//   SPI_write_strobe( SFRX );
   SPI_write_strobe( SRX );
   delay_ms(5);
 //  output_high( LED_RED );delay_ms(30);
 //  delay_ms(30);
   if(SPI_read_status(RXBYTES)==0)
   {
      output_high( LED_GREEN );
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
}*/

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
/*
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

*/



