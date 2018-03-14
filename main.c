///////////////////////////////////////////////////////////////////////////
////                         EX_SLAVE.C                                ////
////                                                                   ////
////  This program uses the PIC in I2C slave mode to emulate the       ////
////  24LC01 EEPROM. You can write to addresses 00h to 0Fh with it.    ////
////                                                                   ////
////  This program is to be used in conjunction with the ex_extee.c    ////
////  sample.  Use the "#include <2402.C>" or "#include <2401.c>".     ////
////  Only 16 bytes of address space are implemented, however.         ////
////                                                                   ////
////  If using a compiler version before 2.639 add "*0x14 = 0x3E;" to  ////
////  the begining of main(), and add "NOFORCE_SW" as the last         ////
////  parameter in the #use i2c directive.                             ////
////                                                                   ////
////  Jumpers:                                                         ////
////     PCM,PCH    pin C7 to RS232 RX, pin C6 to RS232 TX             ////
////                                                                   ////
////  This example will work with the PCM and PCH compilers.  The      ////
////  following conditional compilation lines are used to include a    ////
////  valid device for each compiler.  Change the device, clock and    ////
////  RS232 pins for your hardware if needed.                          ////
///////////////////////////////////////////////////////////////////////////
////        (C) Copyright 1996,2003 Custom Computer Services           ////
//// This source code may only be used by licensed users of the CCS    ////
//// C compiler.  This source code may only be distributed to other    ////
//// licensed users of the CCS C compiler.  No other use,              ////
//// reproduction or distribution is permitted without written         ////
//// permission.  Derivative programs created using this software      ////
//// in object code form are not restricted in any way.                ////
///////////////////////////////////////////////////////////////////////////




#include <18F2480.h>
//#include <18F248.h>
#fuses HS,NOPROTECT,NOLVP,NOWDT //#fuses XT,NOWDT,NOPROTECT,NOLVP,PUT,BROWNOUT
#device 18F2480*=16, ADC=8 // ADC 10 bits
//#device HIGH_INTS=true
#use delay(clock=20000000)
#use rs232(baud=9600, xmit=PIN_C6, rcv=PIN_C7)
#use fast_io(c)
#include <can-18xxx8.c>
#include <LCD_TM_S.c>


BYTE address = 0x00, buffer[0x10];
BYTE incoming = 0x00, state;
static unsigned int A0, A1, A2, T,L,H,C, M1, M2, NoB, NoBSet,SetV;
float volt, volts;
int16 ms;

/*
#int_timer2 
void isr_timer2(void) {
  // ms++; //keep a running timer that increments every milli-second
}
/*
/*
#INT_SSP
void ssp_interupt ()
{
  

   state = i2c_isr_state();

   if(state < 0x80)                     //Master is sending data
   {
      incoming = i2c_read();
      if(state == 1)                     //First received byte is address
         address = incoming;
      if(state == 2)                     //Second received byte is data
         buffer[address] = incoming;
   }
   if(state == 0x80)                     //Master is requesting data
   {
     //i2c_write(buffer[address]);
      i2c_write('B');
   }
   if(state > 0x80)
   {
  // i2c_write(address);
   i2c_write('C');
   //i2c_write(buffer[address]);
   }
}
*/
#INT_EXT 
void ext1_isr(void) { 
  switch(C)
  {
   case 1:
  
      L=A0;
      break;
   case 2:
   
      H=A0;
      break;
   case 3:
   
      M1=A0;
      break;
   case 4:
   
      M2=A0;
      break;
   
   case 5:
   
      NoBSet=NoB;
      break;
 
  }
}    

#INT_EXT1 
void ext1_isr1(void) { 

C++;
if(C>9){
    C=0;}

//   if (input(PIN_B0)==0) { //Possibly just slightly 'clearer' than using not 
//     EXT_INT_EDGE(0,L_TO_H); 
//      C++;
//       if(C>4){
//          C=0;} 
//       output_low(PIN_B1); 
//   } 
//    else { 
//     EXT_INT_EDGE(0,H_TO_L); 
      
//      output_high(PIN_B1); 
 // } 

} 


void main ()
{
   struct rx_stat rxstat;
   int32 rx_id;  // id can bu receiver
   int in_data[8];// number of buytes for receiver 
   int rx_len;
  // output_bit(PIN_B4,0);
//send a request (tx_rtr=1) for 8 bytes of data (tx_len=8) from id 24 (tx_id=24)
   int out_data[8];
   int32 tx_id=24;
   int1 tx_rtr=0;// gia tri cu la 1
   int1 tx_ext=0;//gia tri cu la 0
   int tx_len=8;
   int tx_pri=3;
   int i;

   for (i=0;i<8;i++) {
      out_data[i]=0;
      in_data[i]=0;
   }

union conv{
float f;
int8 b[4];
};
union conv temp, press, hu, temp_sht, hum_sht, temp_31855, press_rsc;
   printf("\r\n\r\nCCS CAN EXAMPLE\r\n");
 
  // setup_timer_2(T2_DIV_BY_4,79,16);   //setup up timer2 to interrupt every 1ms if using 20Mhz clock
  setup_timer_2(T2_DIV_BY_16, 249, 1);   //24,1
          //   The cycle time will be (1/clock)*4*t2div*(period+1)
          //   In this program clock=10000000 and period=127 (below)
          //   For the three possible selections the cycle time is:
          //     (1/10000000)*4*1*128 =  51.2 us or 19.5 kHz
  // enable_interrupts(INT_TIMER2);   //enable timer2 interrupt
   
   //EXT_INT_EDGE(0,L_TO_H); 
   clear_interrupt(INT_EXT); 
   enable_interrupts(INT_EXT);// B0
   set_tris_c(0x00);
   EXT_INT_EDGE(1,H_TO_L); 
   clear_interrupt(INT_EXT1); 
   enable_interrupts(INT_EXT1);// B1
   output_low(PIN_C2);
   setup_ccp1(CCP_PWM);  // PIN_C2 AS CCP PWM
   enable_interrupts(GLOBAL);       //enable all interrupts (else timer2 wont happen)
  // enable_interrupts(INT_SSP);

   setup_adc(ADC_CLOCK_DIV_16); //ADC_CLOCK_DIV_8
   setup_adc_ports(AN0_TO_AN2); //ALL_ANALOG
   
   //setup_ccp1(CCP_PWM_PLUS_3); 
   can_init();
   lcd_init();
   printf("\r\nRunning...");
   C=0;
   // the initial value 
   
   L = 28;
   H = 40;
   M1 = 50;
   M2 = 50;
   NoBSet = 1;
   // end of the initial value
  // output_float(PIN_B0); //Makes pin B3 Input
  // output_float(PIN_B1);
  // output_bit(PIN_B0,1);
  
   while (TRUE) {
   
    printf("\r\n state on i2c slave: %02x",state);
    printf("\r\n incoming from master i2c: %02x",buffer[address]);
  
    set_adc_channel(0); // EasyPIC Board: Read ADC 2 for PWM fan
    delay_ms(20);
    A0= read_adc();
  
  
    switch(C){
     
       case 0:  // Auto mode
               // lcd_init();  // reset lcd
              //  delay_ms(10);
              
                set_adc_channel(2); // EasyPIC Board: Read ADC 2 for PWM fan
                delay_ms(20);
                A2= read_adc();
                volt = 0.073333*A2 + 0.03333;
                
                set_adc_channel(1); // EasyPIC Board: Read ADC 2 for PWM fan
                delay_ms(20);
                A1= read_adc();
                volts = 0.073333*A1 + 0.03333;
                
                lcd_gotoxy(2,1);
                printf(lcd_putc,"      ");
                lcd_gotoxy(1,1);
                printf(lcd_putc,"T%3.1f", temp_31855.f);
                
               
                lcd_gotoxy(8,1);
                printf(lcd_putc,"    ");
                lcd_gotoxy(7,1);
                printf(lcd_putc,"B%3.1f", volt);
               // lcd_gotoxy(7,1);
               // printf(lcd_putc,"M:%u", A0);
                
               
                
                lcd_gotoxy(2,2);
                printf(lcd_putc,"      ");
                
                
                lcd_gotoxy(1,2);
                printf(lcd_putc,"L:%u", L);
                
                lcd_gotoxy(7,2);
                printf(lcd_putc,"    ");
                
                lcd_gotoxy(6,2);
                printf(lcd_putc,"H:%u", H);
                
                lcd_gotoxy(14,2);
                printf(lcd_putc,"   ");
                
                lcd_gotoxy(12,2);
                printf(lcd_putc,"P:%u", T);    
                
                // main program for automatic mode
                 output_bit(PIN_B5,0); // turn off battery
                 output_bit(PIN_C3,0); // turn off solar
                 output_bit(PIN_C4,0); // turn off charger
                  
                 if(volt < 11.9)
                 {
                  output_bit(PIN_B5,0); // turn off battery
                  output_bit(PIN_C4,1); // turn on solar
                  /*
                  if(volts<14)
                  {
                  output_bit(PIN_C3,1); // turn on solar directly 
                  }
                  else
                  output_bit(PIN_C4,0); // turn off solar direct to blower for protect 
                  */
                 }
                 
                 if(volt >12.9)
                 {
                  output_bit(PIN_B5,1); // turn on battery
                  output_bit(PIN_C4,0); // turn off solar
                 }
                 
                if(T>100)
                      {
                      T=100;
                      }
       
                if(temp_31855.f<L) // A1: low level
                {
                   if(press_rsc.f<2)
                   {
                   set_pwm1_duty(50);
                   }
                      /*
                      if(T>30)
                         {
                           T--;
                        
                         }  
                       else 
                         {
                         T = 30;
                         }
                      delay_ms(500);    
                      */
                      T=1;
                      set_pwm1_duty(T);
                }
                if(temp_31855.f>H)// A2: high level
                {
                /*
                if(T<100)
                      {
                      T++;
                      }
                   
                delay_ms(50);
                */
                T=100;
                set_pwm1_duty(T);
                }
                 if((temp_31855.f>L)&&(temp_31855.f<H)) 
                {
                T=8;
                set_pwm1_duty(T);
                }
               
                // end main program for automacit mode
                
                break;
                
      
      case 1: // Set low value 
                //lcd_init();  // reset lcd
               // delay_ms(10);
                            
                
                  lcd_gotoxy(1,1);
                  printf(lcd_putc,"Set L           ");
                  
                  lcd_gotoxy(4,2);
                  printf(lcd_putc,"      ");
                  
                  lcd_gotoxy(1,2);
                  printf(lcd_putc,"A1:%U", A0);
                  
                  lcd_gotoxy(10,2);
                  printf(lcd_putc,"        ");
                  lcd_gotoxy(10,2);
                  printf(lcd_putc,"L:%u", L);
              
               break;
       case 2: // Set high value 
                //lcd_init();  // reset lcd
                //delay_ms(10);
                            
                
                  lcd_gotoxy(1,1);
                  printf(lcd_putc,"Set H           ");
                  
                  lcd_gotoxy(4,2);
                  printf(lcd_putc,"      ");
                  
                  lcd_gotoxy(1,2);
                  printf(lcd_putc,"A0:%U", A0);
                  
                  lcd_gotoxy(12,2);
                  printf(lcd_putc,"     ");
                  
                  lcd_gotoxy(10,2);
                  printf(lcd_putc,"H:%u", H);
              
               break;
       case 3: // Set manual value for blow 1
                  // lcd_init();  // reset lcd
                  // delay_ms(10);
                             
                   set_adc_channel(0); // EasyPIC Board: Read ADC 2 for PWM fan
                   delay_ms(20);
                   A0= read_adc();
                  lcd_gotoxy(1,1);
                  printf(lcd_putc,"  Manual Blow 1 ");
                  lcd_gotoxy(4,2);
                  printf(lcd_putc,"   ");
                  lcd_gotoxy(1,2);
                  printf(lcd_putc,"A0:%U", A0);
                  
                  lcd_gotoxy(11,2);
                  printf(lcd_putc,"     ");
                  
                  lcd_gotoxy(8,2);
                  printf(lcd_putc,"M1:%u", M1);
                  
                  // manual program
                  set_pwm1_duty(M1);  // BLOW 1
                  output_bit(PIN_B5,0);
                  // end manual program
               break;  
       case 4: // Set manual value for blow 2
                 // lcd_init();  // reset lcd
                 // delay_ms(10);
                         
                
                  lcd_gotoxy(1,1);
                  printf(lcd_putc,"  Manual Blow 2 ");
                  
                  lcd_gotoxy(4,2);
                  printf(lcd_putc,"   ");
                  
                  lcd_gotoxy(1,2);
                  printf(lcd_putc,"A0:%U", A0);
                  
                  lcd_gotoxy(11,2);
                  printf(lcd_putc,"     ");
                  
                  lcd_gotoxy(8,2);
                  printf(lcd_putc,"M2:%u", M2);
              
               break;     
        case 5: // Choose No of Blows
                  // lcd_init();  // reset lcd
                  // delay_ms(10);
                                   
                   if((A0>0)&&(A0<60))
                   {
                     NoB = 1; // Blow 1
                      
                   }
                   if((A0>=60)&&(A0<120))
                   {
                     NoB = 2;
                      
                   }
                   if((A0>120))
                   {
                   NoB = 3;
                    
                   }
                   
                  lcd_gotoxy(1,1);
                  printf(lcd_putc,"Set Blows       ");
                  
                  lcd_gotoxy(5,2);
                  printf(lcd_putc,"   ");
                  
                  lcd_gotoxy(1,2);
                  printf(lcd_putc,"NoB:%U", NoB);
                  
                  lcd_gotoxy(15,2);
                  printf(lcd_putc,"   ");
                  
                  lcd_gotoxy(8,2);
                  printf(lcd_putc,"NoBSet:%U", NoBSet);
              
               break;  
       case 6: // Display BME280 id = 0x23 & id = 0x25
              // lcd_init();  // reset lcd
              // delay_ms(10);
              
               lcd_gotoxy(3,1);
               printf(lcd_putc,"     ");
                  
               lcd_gotoxy(1,1);
               printf(lcd_putc,"T:%3.1f", temp.f);
               
               lcd_gotoxy(10,1);
               printf(lcd_putc,"    ");
               
               lcd_gotoxy(8,1);
               printf(lcd_putc,"P:%3.1f", press.f);
               
               lcd_gotoxy(3,2);
               printf(lcd_putc,"    ");
               
               lcd_gotoxy(1,2);
               printf(lcd_putc,"H:%3.1f", hu.f);
               lcd_gotoxy(8,2);
               printf(lcd_putc,"      BME");
               break;
       case 7: // Display SHT
              // lcd_init();  // reset lcd
              // delay_ms(10);
               
               lcd_gotoxy(7,1);
               printf(lcd_putc,"     ");
               
               lcd_gotoxy(1,1);
               printf(lcd_putc,"SHT T:%3.1f", temp_sht.f);
               lcd_gotoxy(12,1);
               printf(lcd_putc,"0C");
               lcd_gotoxy(14,1);
               printf(lcd_putc,"   ");
               
               lcd_gotoxy(7,2);
               printf(lcd_putc,"     ");
               
               lcd_gotoxy(1,2);
               printf(lcd_putc,"SHT H:%3.1f", hum_sht.f);
               lcd_gotoxy(12,2);
               printf(lcd_putc,"RH   ");
               lcd_gotoxy(14,2);
               printf(lcd_putc,"   ");
               break;
               
       case 8: // Display Max31855
       
              // lcd_init();  // reset lcd
              // delay_ms(10);
               
               lcd_gotoxy(7,1);
               printf(lcd_putc,"        ");
               
               lcd_gotoxy(1,1);
               printf(lcd_putc,"MAX T:%3.1f", temp_31855.f);
               lcd_gotoxy(1,2);
               printf(lcd_putc,"                ");
               break;
       case 9://  Display RSC
              // lcd_init();  // reset lcd
              // delay_ms(10);
               lcd_gotoxy(7,1);
               printf(lcd_putc,"         ");
               lcd_gotoxy(1,1);
               printf(lcd_putc,"RSC P:%3.1f", press_rsc.f);      
               lcd_gotoxy(1,2);
               printf(lcd_putc,"                ");
               break;
      
                  
                  
              
                
              
             
    }
    //printf("\r\n adc chanel 2: %02x",A0);
  
    
 
      if(can_kbhit() )   //if data is waiting in buffer... //if ( can_kbhit() )
      { 
      // output_bit(PIN_B4,1);
          // printf("haaa");
         if(can_getd(rx_id, &in_data[0], rx_len, rxstat)) { //...then get data from buffer
            //printf("haha");
             if(rx_id ==21)
            {
            
            press_rsc.b[0] = in_data[0];
            press_rsc.b[1] = in_data[1];
            press_rsc.b[2] = in_data[2];
            press_rsc.b[3] = in_data[3];
            
            printf("Pressure RSC ID:21 = %f ",press_rsc.f);
            }
            if(rx_id ==23)
            {
          //  output_bit(PIN_B4,1);
            temp.b[0] = in_data[0];
            temp.b[1] = in_data[1];
            temp.b[2] = in_data[2];
            temp.b[3] = in_data[3];
            press.b[0] = in_data[4];
            press.b[1] = in_data[5];
            press.b[2] = in_data[6];
            press.b[3] = in_data[7];
            printf("Temperature BME280 ID:23 = %f ",temp.f);
            printf("Pressure BME280 ID:23 = %f ",press.f);
            }
             if(rx_id ==25)
            {
            
            hu.b[0] = in_data[0];
            hu.b[1] = in_data[1];
            hu.b[2] = in_data[2];
            hu.b[3] = in_data[3];
            
            printf("Humidity BME280 ID:25 = %f ",hu.f);
            }
            if(rx_id ==27)
            {
            temp_sht.b[0] = in_data[0];
            temp_sht.b[1] = in_data[1];
            temp_sht.b[2] = in_data[2];
            temp_sht.b[3] = in_data[3];
            
            hum_sht.b[0] = in_data[4];
            hum_sht.b[1] = in_data[5];
            hum_sht.b[2] = in_data[6];
            hum_sht.b[3] = in_data[7];
            
            printf("Temperature SHT_75 ID:27 = %f ",temp_sht.f);
            printf("Pressure SHT75 ID:27 = %f ",hum_sht.f);
            
            }
             if(rx_id ==29)
            {
            
            temp_31855.b[0] = in_data[0];
            temp_31855.b[1] = in_data[1];
            temp_31855.b[2] = in_data[2];
            temp_31855.b[3] = in_data[3];
            
            printf("Temperature MAX31855 ID:29 = %f ",temp_31855.f);
            }
            
            printf("\r\nGOT: BUFF=%U ID=%LU LEN=%U OVF=%U ", rxstat.buffer, rx_id, rx_len, rxstat.err_ovfl);
            printf("FILT=%U RTR=%U EXT=%U INV=%U", rxstat.filthit, rxstat.rtr, rxstat.ext, rxstat.inv);
            printf("\r\n    DATA = ");
            for (i=0;i<rx_len;i++) {
               printf("%X ",in_data[i]);
            }
            printf("\r\n");
           
         }
         
         else {
            printf("\r\nFAIL on GETD\r\n");
         }
           
      }

      //every two seconds, send new data if transmit buffer is empty
     /*
      if ( can_tbe() && (ms > 2000))
      {
         ms=0;
         i=can_putd(tx_id, out_data, tx_len,tx_pri,tx_ext,tx_rtr); //put data on transmit buffer
         if (i != 0xFF) { //success, a transmit buffer was open
            printf("\r\nPUT %U: ID=%LU LEN=%U ", i, tx_id, tx_len);
            printf("PRI=%U EXT=%U RTR=%U\r\n   DATA = ", tx_pri, tx_ext, tx_rtr);
            for (i=0;i<tx_len;i++) {
               printf("%X ",out_data[i]);
            }
            printf("\r\n");
            
         }
         else { //fail, no transmit buffer was open
            printf("\r\nFAIL on PUTD\r\n");
         }
      }
      */

    delay_ms(200);
     
   }
}
