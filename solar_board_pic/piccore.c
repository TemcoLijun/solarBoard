/////////////////////////////////////////////////////////////////////////
/*
CONFIGURATION BITS:
Oscillator            Internal RC Clockout
Watchdog Timer         ON
Power Up Timer         Off
Master Clear Enable      Internal
Brown Out Detect      On
Code Protect          Off
Data EE Read Protect   Off
*/
/////////////////////////////////////////////////////////////////////////
//#include <16F1824.h>
#include "pic.h"
#ignore_warnings 203

////////////////////////////////////////


//void writeRfSettings( RF_SETTINGS *pRfSettings);
//#INT_TIMER0
/*
void clock_base0_isr()
{
   set_timer0 (56) ;

      pre_flash ++ ;
      cur_flash ++;
   //   if(pre_flash == 1)
  //    {
      //   pre_flg = !pre_flg;
      if(led_flg )
        {
         output_high (pwm);
        }
        else
        {
        output_low (pwm);  
        }
   //   // pre_flash = 0;
   //   }
   //   if(cur_flash > 40)
      {    
       //  cur_flg = !cur_flg;
       // cur_flash = 0;
    //  }
 // clear_interrupt (INT_TIMER0);
}*/
///////////////////////////////////////
void main( void )
{  
   unsigned char  read_count ;//= 64;
   unsigned char Rval;
   initial(); 
 //  output_high( SDA );
   //DISABLE_INTERRUPTS (INT_TIMER0) ;
   delay_ms(10);
   while(1)
   {  
      temp_buf = look_up_table(Temp_filter_reading( temp_type ));
      Wbuffer[0] = temp_buf + caliberate_temp; //we have not make caliberate at here.
      Wbuffer[1] = Temp_filter_reading( PIR_type );
      Wbuffer[2] = Temp_filter_reading( light_type );
   #if 1
     //write the data into the FIFO buffer    
     if( 1 )//WR_flg )
     {
   //   output_high( LED_RED );
   //   output_high( LED_RED );
        delay_ms( 30 ); 
      leng = 2;
      Write_packet( Wbuffer, leng );
   //   output_low( LED_RED );
      if( Wbuffer[1] > 250)
      {
         output_high( LED_RED );
         delay_ms(200);
         output_low( LED_RED);
      }
      delay_ms( 30 );
      }
    
     else    //read the data from the FIFO buffer
     {  
      read_count = 2;
   //  output_high( LED_RED );delay_ms( 100 );
         if( Read_packet(Rbuffer, &read_count))//sizeof( Rbuffer )))
         {
      //     output_high( LED_RED );delay_ms( 30 );  output_low( LED_RED );
      //      for(recv_count = 0; recv_count < 8; recv_count++ )
       //     {
               if( Rbuffer[0] == 0xaa )
               {
             //     output_high( LED_GREEN );
                  delay_ms( 200 );
              //    output_low( LED_GREEN );
                  delay_ms( 200 );
                  Rbuffer[0] = 0;
               }
               if(Rbuffer[1] == 0x55)
                {
                  output_high( LED_RED );
                  delay_ms( 200 );
                  output_low( LED_RED );
                  delay_ms( 200 );
                  Rbuffer[1] = 0;
               }
               else
               {                                                        
                  output_low( LED_RED );
            //      output_low( LED_GREEN );
               }
            }
          //  delay_ms( 100 );   
          //  output_low( LED_RED );
      //   }      
     }
     #endif
     #if 0                 
       SPI_write_data( CHANNR, 0x04 );
   //   SPI_write_burst( ADDR,Wbuffer,4 );
   //    delay_us(2);
       Rval = SPI_read_data( CHANNR );
   //   SPI_read_burst( ADDR,Rbuffer,4 );
    //  SPI_end();
       delay_us( 2 );

      if( Rval == 0x00 )
      {
         output_low( LED_GREEN );
   //      output_low( LED_RED );
        // delay_us( 100 );
        delay_ms( 30 );
         output_high( LED_GREEN );
    //     output_high( LED_RED );
       //  delay_us( 100 );
         delay_ms( 100 );
         Rval = 0;
      }
      if( Rval == 0x04 )
      {
        output_low( LED_RED );
        // delay_us( 100 );
         delay_ms( 30 );
         output_high( LED_RED );
       //  delay_us( 100 );
         delay_ms( 100 );
         Rval = 0;
      }
      else
      {
         output_low( LED_GREEN );
         output_low( LED_RED );
      }
      
         #endif
    }
}
//this function use to read the value from sensor
unsigned INT16 Temp_filter_reading( unsigned char type )
{
   unsigned int16 temp_read,min,max;
   unsigned int16 temp_ai;
   unsigned char sampling_counter;
   DISABLE_INTERRUPTS (INT_TIMER0) ;
   data_sum = 0;
   set_adc_channel( type ); //we open this channel for read the temp value
   for(sampling_counter = 0; sampling_counter < sampling_time; sampling_counter++ )
   {
      temp_read = read_adc();
      data_sum += temp_read;
      if( sampling_counter == 0)
      {
         min = temp_read;
         max = temp_read;
      }
      else
      {
         min = min < temp_read ? min : temp_read;
         max = max > temp_read ? max : temp_read;
      }
   }

   temp_read = (data_sum - min - max) / (sampling_time - 2);
   enable_interrupts (INT_TIMER0);
   sampling_counter = 0;
   temp_ai = (unsigned int16)temp_read&0x3ff;
   return temp_ai ;
}
//when we get the adc value that we need caculate it and turn ever the TEMP
unsigned int16 look_up_table(int16 count)
{                                        
    int16 val, work_var;
    int16 index = 19;
       
    work_var = temp_table[index]; 
    if(work_var > count)
    {
        val = index * 100; // >= max range, so use the last point of array
        return ((unsigned int16)val);                                                         
    }                         
 
    do                        
    {
        index--;
        work_var += temp_table[index];    //check which range match current value:L14 + (L13 - L14) + (L12-L13) + ... + [L (x - 1) - Lx] + [Lx - L (x + 1) ] = L (x + 1)
        if(work_var > count)    //can check relative document in : Z:\Designs\Temperature\Curves\ThermistorCurves_VoltageCalcs.xls
        {
            val = (work_var - count) * 100;    //get the difference value between current value and nearest default point
            val /= temp_table[index];
            if(index >= 4)
            {
                val += ((index - 4) * 100);    //get nearest temperature point value
                val &= 0x7fff;
            }
            else
            {
                val += index * 100;
                val = 400 - val;
                val |= 0x8000;
            }                     
            return ((unsigned int16)val);        
        }                 
    } while(index);                                                                      

    val = index * 100; 
    return (( unsigned int16)val);  
}  

///////////////////////////////////////////////////////
//
//           initial
//
///////////////////////////////////////////////////////
void initial ( void )
{
   //Set Chip oscillator to 16MHz
   
   setup_oscillator (OSC_16MHZ) ;
 //  SETUP_TIMER_0 (T0_DIV_8) ;
 //  set_timer0 (56); // time0 = 400us // TIMER0_OVERFLOW_TIME = 256 - (time / 4 / 16000000 / frequence)
//   enable_interrupts (INT_TIMER0);
   // enable_interrupts (INT_RA5);
 //  enable_interrupts (GLOBAL) ;
   // SET_TRIS_A (0X3F) ;
   // SET_TRIS_c (0X3F) ;
  // INPUT (ANALOG_INPUT1) ;
   SETUP_ADC (ADC_CLOCK_DIV_32) ;
   SETUP_ADC_PORTS (VSS_VDD) ; //Range from 0-VDD
   
   SETUP_ADC_PORTS (sAN1) ; //for light sensor 
   SETUP_ADC_PORTS (sAN2) ; // for PIR
   SETUP_ADC_PORTS (sAN4) ; //THERM NTC
   
 //  set_adc_channel (2) ;
 //here for the RF
   set_tris_c (0x12); //set up  bit5 ,bit1 of c port input 10/12
   set_tris_a (0x20); //set up  bit5 of a port input
   output_high (POWER) ; //POWER ON
   SPI_initial();
  // delay_ms (5) ;
   
}



//simulate SPI with I/O port 
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
      
  //    output_high(SPI_CLK);// = 0;
      output_low(SPI_CLK);// = 1;
     // delay_us( 2 );
      Wdata <<= 1;
     // delay_us( 2 );
   //   output_low(SPI_CLK);// = 1;    
      output_high(SPI_CLK);// = 0;
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
    // output_high(SPI_CLK);
    output_low(SPI_CLK);// = 0;
     delay_us( 1 );
     Rdata <<= 1;
     
     if( input(SPI_MISO) == 1 )
     Rdata |= 0x01;
     else
     Rdata &= 0xFE;
     delay_us( 1 );
   //  Rdata <<= 1;
   //  output_low(SPI_CLK);
   output_high(SPI_CLK);// = 0;
     delay_us( 1 );
    // SPI_CLK = 1;           
   }
   return Rdata;
}
//SPI initial
void SPI_initial( void )
{
   output_high(SPI_CLK); //make the clk and mosi equal to 0 
   output_low(SPI_MOSI);
 //  delay_us( 1 );
//   output_low(SPI_CS);// = 0;
 //  output_low(SPI_CLK);// = 0;
   output_high(SPI_CS);// = 1;
   delay_ms( 5 );
   SPI_powerup_reset();
   delay_us( 1 );
 //  writeRfSettings( &rfSettings  );
   writeRfSettings(  );
   SPI_write_burst(PATABLE,PA_config,8);//here config the patable register, we choose the 0xc0
   output_low(SPI_CLK);// = 0;
   SPI_write_strobe( SIDLE );
   SPI_write_strobe( SCAL );
 //  SPI_write_strobe( SFRX );
   output_high(SPI_MISO);//SPI_MISO = 1;
  // delay_us( 1 );   
  // output_low(SPI_CS);// = 0;   
   
}

//write data to the cc1101 value
void SPI_write_data( unsigned int8 Regaddr, unsigned int8 Dvalue )
{
   output_low( SPI_CS );
   while( input( SPI_MISO ));
   Regaddr &= 0X7F;
   SPI_write_byte( Regaddr );
   delay_us( 1 );
   SPI_write_byte( Dvalue );
    delay_us( 1 );
   output_high( SPI_CS );
}

//read data from the cc1101 value
unsigned int8 SPI_read_data( unsigned int8 Regaddr )
{
   unsigned int8 temp;
   output_low( SPI_CS );
   while( input( SPI_MISO ));
   SPI_write_byte( Regaddr | 0x80 );
   delay_us( 1 );
   temp = SPI_read_byte( );
   output_high( SPI_CS );
    delay_us( 1 );
   return temp;
}
//read data from the cc1101 register
unsigned int8 SPI_read_status( unsigned int8 Regaddr )
{
   unsigned int8 temp;
   output_low( SPI_CS );
   while( input( SPI_MISO ));
   SPI_write_byte( Regaddr | 0xc0 );
   delay_us( 1 );
   temp = SPI_read_byte( );
   output_high( SPI_CS );
    delay_us( 1 );
   return temp;
}
//set the strober register 
void SPI_write_strobe( unsigned int8 strobe )
{
   output_low( SPI_CS );
   while( input( SPI_MISO ));
   SPI_write_byte( strobe );
    delay_us( 1 );
   output_high( SPI_CS );
}
//write the burst
void SPI_write_burst(unsigned char Regaddr, unsigned char *Wbuf, unsigned char count)
{
   unsigned char i;
   output_low( SPI_CS );
   while( input( SPI_MISO ));
   SPI_write_byte( Regaddr | 0x40 );
   for(i=0; i<count;i++)
   {
     SPI_write_byte( Wbuf[i] ); 
     delay_us( 5 );
   }
   output_high( SPI_CS );
}
//read the burst
void SPI_read_burst(unsigned char Regaddr, unsigned char *Rbuf, unsigned char count)
{
   unsigned char i;
   output_low( SPI_CS );
   while( input( SPI_MISO ));
   SPI_write_byte( Regaddr | 0xc0 );
   for(i=0;i<count;i++)
   {
      Rbuf[i] = SPI_read_byte( );
   }
   output_high( SPI_CS );
}
//RESET CC1101 chip
void SPI_reset_cc11( void )
{
 //  SPI_write_strobe( SIDLE );
   output_low( SPI_CS );
   while( input( SPI_MISO ));
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
//spi config 
void SPI_config( void )
{
  SPI_write_data( IOCFG2, 0x24 ); //0x29
  SPI_write_data( IOCFG0, 0x06 ); 
  
  SPI_write_data( FIFOTHR, 0x07 ); 
 // SPI_write_data( SYNC1, 0xd3 ); 
 // SPI_write_data( SYNC0, 0x91 ); 
  
  SPI_write_data( PKTLEN, 0xff ); 
  SPI_write_data( PKTCTRL1, 0x04 );
  SPI_write_data( PKTCTRL0, 0x05 );
  
  SPI_write_data( ADDR, 0x00 ); //we set 01 as a write, set 00 as a read
  SPI_write_data( CHANNR, 0x00 );
  
  SPI_write_data( FSCTRL1, 0x0c);//0x06
  SPI_write_data( FSCTRL0, 0x00 );
  
  SPI_write_data( FREQ2, 0x23 );//0x21
  SPI_write_data( FREQ1, 0x31 );//0x62 
  SPI_write_data( FREQ0, 0x3B );//0x76 
  
  SPI_write_data( MDMCFG4, 0x2D );
  SPI_write_data( MDMCFG3, 0x3B );
  SPI_write_data( MDMCFG2, 0x13 );//0x13
  SPI_write_data( MDMCFG1, 0x22 );
  SPI_write_data( MDMCFG0, 0xf8 );
  
  SPI_write_data( DEVIATN, 0x62 );
//  SPI_write_data( MCSM2, 0x07 );
//  SPI_write_data( MCSM1, 0x00 );//0x30
  SPI_write_data( MCSM0, 0x18 );
  
  SPI_write_data( FOCCFG, 0x1D );//0x16
  SPI_write_data( BSCFG, 0x1c );
  
  SPI_write_data( AGCCTRL2, 0xC7 );
  SPI_write_data( AGCCTRL1, 0x00 );
  SPI_write_data( AGCCTRL0, 0xB0 );
  
 // SPI_write_data( WORREVT1, 0x87 );
 // SPI_write_data( WORREVT0, 0x6b );
  //SPI_write_data( WORCTRL, 0xfb );
  
  SPI_write_data( FREND1, 0xB6 );
  SPI_write_data( FREND0, 0x10 );
  
  SPI_write_data( FSCAL3, 0xEA );//0xe9
  SPI_write_data( FSCAL2, 0x2a );
  SPI_write_data( FSCAL1, 0x00 );//0x00
  SPI_write_data( FSCAL0, 0x1F );//0x1f
  
//  SPI_write_data( RCCTRL1, 0x41 );
//  SPI_write_data( RCCTRL0, 0x00 );o
  SPI_write_data( FSTEST, 0x59 );
//  SPI_write_data( PTEST, 0x7f );
//  SPI_write_data( AGCTEST, 0x3f );
  SPI_write_data( TEST2, 0x88 );//0x81
  SPI_write_data( TEST1, 0x31 );//0x35
  SPI_write_data( TEST0, 0x09 );
}
*/
//read the packet from cc1101
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

//write the packet to cc1101
void Write_packet( unsigned char *Wbuffer, unsigned char size )
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
 /* 
  while((SPI_read_status(TXBYTES)&0x7F) != 0x00 )
  {
      SPI_write_strobe( SIDLE );
      return;
  }*/
   SPI_write_strobe( SFRX );
  // SPI_write_strobe( SIDLE );
}

void writeRfSettings( void )
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
