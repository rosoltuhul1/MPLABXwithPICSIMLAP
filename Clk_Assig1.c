/*
 * Author: Fatimah Abu Reesh
 *
 * Created on April 12, 2023, 10:28 PM
 */


// CONFIG1H
#pragma config OSC = XT         // Oscillator Selection bits (XT oscillator)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 3         // Brown Out Reset Voltage bits (Minimum setting)

// CONFIG2H
#pragma config WDT = ON         // Watchdog Timer Enable bit (WDT enabled)
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = PORTC   // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = ON      // PORTB A/D Enable bit (PORTB<4:0> pins are configured as analog input channels on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = ON         // Single-Supply ICSP Enable bit (Single-Supply ICSP enabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-003FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (004000-007FFFh) not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (008000-00BFFFh) not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (00C000-00FFFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-003FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (004000-007FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (008000-00BFFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (00C000-00FFFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (004000-007FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (008000-00BFFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (00C000-00FFFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot Block (000000-0007FFh) not protected from table reads executed in other blocks)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.
#include <xc.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "my_ser.h"
#include "my_adc.h"
#include "lcd_x8.h"
#include <xc.h>
#include <stdbool.h>
#define _XTAL_FREQ   4000000UL 
#define STARTVALUE  3036
int state = 1;
float temp1=0.0;
int count = 0;
int clkmode=0; //0 for setup mode 1 for normal mode
int setting=0; //0 for secondes, 1 for min 2 for hours
int countFlag = 0;
int clockingMode = 0;
float seconds=00.0;
float minutes=00.0;
float hours=00.0;
int readanaloge=0;
float countseconds=00.0;
float countminutes=00.0;
float counthours=00.0;
float tseconds = 00.00;
float tminutes=00.0;
float thours=00.0;
float tempf;
float spf;
int hf=0;   //////Flag Heater
int cf=0;   //////Flag Cooler
char Buffer[32];
unsigned char Serial[5];
int i=0;
int stateFlag=0;
int TimerF=0;
int clkF=0;
unsigned char time[10];





void reloadTimer0(void)
{  
 
    TMR0H = (unsigned char) ((STARTVALUE >>  8) & 0x00FF);
    TMR0L =  (unsigned char)(STARTVALUE & 0x00FF );   
}






// switch(mode){
//        case 0: //normal mode 
//             lcd_gotoxy(1, 3);
//            sprintf(NOR, "NORMAL          " );
//            lcd_puts(NOR);
//            break;
//        case 1: //setup mode 
//            if(mode1==0){
//                 lcd_gotoxy(1, 3);
//               sprintf(S, "SETUP  seconds"  ); 
//               lcd_puts(S);
//            }
//            else if(mode1==1){
//                 lcd_gotoxy(1, 3);
//                 sprintf(M, "SETUP  minutes"  ); 
//                  lcd_puts(M);
//            }
//            else{
//                 lcd_gotoxy(1, 3);
//               sprintf(H, "SETUP  hours       "  );  
//                lcd_puts(H);
//            }
//            break;
//    } 
 
 
void Timer0_isr(void)
{

    PORTCbits.RC1=0;
    INTCONbits.TMR0IF=0;
    
   
    if(clockingMode == 1){
       if(seconds== 59.0)
        {
            seconds=00.0;
            if(minutes ==59.0)
            {
                minutes=00.0;
                if(hours == 23.0)
                    hours=00.0;
                else
                    hours ++;
            }
            else
                minutes ++;
            }
            else
            {
                seconds ++;
            }
            if ((hours==23.0) &&(minutes==59.0)&&(seconds==59.0)){
                seconds=0;
                minutes=0;
                hours=0;
            }
    }
    else if (clockingMode == 0){
         seconds=seconds;
         minutes=minutes;
         hours=hours;
    }
    if (clkF==1){
        
        if(countseconds== 00.0)
        {
            countseconds=59.0;
            if(countminutes ==00.0)
            {
                countminutes=59.0;
                if(counthours == 00.0)
                    counthours=9.0;
                else
                    counthours --;
            }
            else
                countminutes --;
            }
            else
            {
                countseconds --;
            }
        
        
        
        
        
 //        m= check_validation();     
//             s= check_validation();    
//             sprintf(str2, " %d:%d:%d \n",h, m,s); 
//             send_string_no_lib(str2 );  
//             hours=h; 
//             minutes=m;
//             seconds=s;
         
         
        
         
            if ((counthours==0.0) &&(countminutes==0.0)&&(countseconds==0.0)){
                countseconds=0;
                countminutes=0;
                counthours=0;
                clkF=0;
                PORTCbits.RC5=0;
                
                //display
                
                char LCD [64];
                char LCDP [100];
                lcd_putc('\f');
                lcd_gotoxy(1, 1);
                sprintf(LCD, "%2.0f:%2.0f:%2.0f ",counthours,countminutes,countseconds);
                lcd_puts(LCD);

               

           

      
                lcd_puts(LCDP);
                
                hours = 0;
                minutes = 0;
                seconds = 0;
                
                int buzzer=0;
                while(buzzer<8){
                    PORTCbits.RC1=!PORTCbits.RC1;
                    delay_ms(500);
                    buzzer++;
                }
                //seconds+=1;
                delay_ms(16000);
                lcd_putc('\f');
                state = 0; 
            }
        }
        else{
            counthours=counthours;
            countminutes=countminutes;
            countseconds=countseconds;
        }
    
    
    
    TimerF=1;
    reloadTimer0();
}



void EXT_Int1_isr(void)
{
    INTCON3bits.INT1IF=0;
    if(clkmode==0 && state==0) {
          delay_ms(100);
            state=1;}
            else if(clkmode==0){
                clkmode=1;
       
        if(state==1){
            delay_ms(100);
             clockingMode = 1;
                state=2;
            }
        else if(state==2){
            delay_ms(100);
             clockingMode = 0;
            state=1;  
        }
        
            }
    else {
                clkmode=0;
                delay_ms(100);
                state=1;
                   clockingMode = 0;
    }

    
}

void EXT_Int2_isr(void)
{
    
    INTCON3bits.INT2IF=0;
    if(clkmode==0)
    {
        setting=setting+1;
        if(setting==3){
            setting=0;
        }
        state=1;
    }
    delay_ms(100);
    
}





void SetTime()
{
    char sec1,sec2;
    char min1,min2;
    char hr1,hr2;
     
     
     sec2=(time[7]-48);
     sec1=(time[6]-48)*10;
    
     min1=(time[3]-48)*10;
     min2=(time[4]-48);
     hr1=(time[0]-48)*10;
     hr2=(time[1]-48);
     
     countseconds=sec1+sec2;
     countminutes=min1+min2;
     counthours=hr2+hr1;
}


void RX_isr(void){
    CLRWDT();
     unsigned char Recievedchar = RCREG; 
    
     if(stateFlag==0){
        if(Recievedchar=='<'){
            stateFlag=1;
            i=0;

        }
     }
     else if(stateFlag==1){
        if(Recievedchar=='>'){
            stateFlag=0;
 
        }
        else{
    
      Serial[i]=Recievedchar;
 
      i=i+1;
        }
   }
   PIR1bits.RCIF=0;  
    }



void EXT_Int0_isr(void){
     INTCONbits.INT0IF=0;
    if(readanaloge==0){
        readanaloge=1;
    }
    else{
        readanaloge=0;
    }
     delay_ms(100);
}


void __interrupt(high_priority)highIsr(void)
{
    if (INTCON3bits.INT1IF)EXT_Int1_isr();
    else if (INTCONbits.TMR0IF) Timer0_isr();
    else if (INTCONbits.INT0IF) EXT_Int0_isr();
    else if (INTCON3bits.INT2IF) EXT_Int2_isr();
    else if(PIR1bits.RCIF) RX_isr();
}



void delay_ms(unsigned int n)
{
    int i;
    for (i=0; i < n; i++){
         __delaywdt_ms(1) ; 
    }
}


void setupPorts(void)
{
    ADCON0 =0;
    ADCON1 = 0x0C; //3analog input 0000 1100
    TRISB = 0xFF; // all pushbuttons are inputs
    TRISC = 0x80; // RX input , others output
    PORTC =0;
    TRISA = 0xFF; // All inputs
    TRISD = 0x00; // All outputs
    TRISE= 0x00;  // All outputs
    
}


float read_and_display_Temp( void ){ 

CLRWDT();
temp1 = 100.0* read_adc_voltage((unsigned char) '2');
if(state==1 || state==2){
 sprintf(Buffer,"T=%2.2f%c", temp1);
   lcd_gotoxy(10, 1);
   lcd_puts(Buffer);     
   
}
   return temp1;
}


void main(void) {
     sprintf(Serial, "00000");
     
    if(count == 0) {
        clkF = 0;
        count++;
    }
   // setupSerial();
    setupPorts();
    init_adc_no_lib();
    INTCON = 0; // disable interrupts first, then enable the ones u want
    
    INTCON = 0;
    RCONbits.IPEN =0;
    INTCONbits.TMR0IE=1;
      INTCONbits.INT0E=1;
    
    INTCON2 = 0;
    
    INTCON3 = 0;
    INTCON3bits.INT1IE = 1;
        INTCON3bits.INT2IE = 1;
    INTCON2bits.INTEDG1 = 1;
    T0CON=0X80;
    PIE1 = 0;
    PIR1 = 0;
    IPR1 = 0;
    PIE2 = 0;
    PIE2 = 0;
    PIR2 = 0;
    IPR2 = 0;
    PIE1bits.RCIE=1;
  //  PIE1bits.TXIE=1;
    char LCD[64];
    char LCDP[64];
    char dum='0';
    lcd_init();
    lcd_putc('\f');
    INTCONbits.GIEH = 1;  // enable global interrupt bits
    INTCONbits.GIEL = 1;
      BAUDCONbits.BRG16 = 0;
    TXSTA = 0;
    SPBRG = 25;
    SPBRGH = 0;
    TXSTAbits.BRGH = 1; //baud rate high speed option
    TXSTAbits.TXEN = 1; //	;enable transmission
      RCSTA = 0; // ;SERIAL RECEPTION W/ 8 BITS,,
    RCSTAbits.CREN = 1; //;enable reception
    RCSTAbits.SPEN = 1;
    dum = RCREG; //, W        ; clear the receiver buffer      
    dum = RCREG; //,W         ; clear the receiver buffer
    
    lcd_init();
    lcd_send_byte(0, 1);
    while(1)
    {
        CLRWDT();  
        read_and_display_Temp();
          spf = read_adc_voltage(0);
          tempf =read_adc_voltage(1);
    
        if(PORTAbits.RA5 && !((counthours==0.0) &&(countminutes==0.0)&&(countseconds==0.0)) && clkF == 1){
            state = 6;
        }
        
         while(PORTAbits.RA5 ==0){
            delay_ms(100);
            hf=1; 
        }
         
        if(hf==1){
             hf=0;
            if(PORTCbits.RC5 ==0){
            delay_ms(100);
            PORTCbits.RC5=1; 
            }
            else{
                delay_ms(100);
                PORTCbits.RC5=0;
            }
        }

        while(PORTBbits.RB5 ==0){
            delay_ms(100);
            cf=1; 
        }
        if(cf ==1){
            cf=0; 
            if(PORTCbits.RC2 ==1){
            delay_ms(100);
            PORTCbits.RC2 =0;   
            }
            else{
                delay_ms(100);
            PORTCbits.RC2 =1;
            }  
        }
    


        switch (state){
            case 1:
                lcd_gotoxy(1, 1);
                sprintf(LCD, "%2.0f:%2.0f:%2.0f ",hours,minutes,seconds);
                lcd_puts(LCD);

               if(PORTCbits.RC5 == 1){
       lcd_gotoxy(10, 2);
       sprintf(LCD, " H:ON ");
       lcd_puts(LCD);
         
      
       }
       else{
          
        lcd_gotoxy(10, 2);
        sprintf(LCD, " H:OFF ");
        lcd_puts(LCD);
     
       }
         
       if(PORTCbits.RC2 == 1){
       lcd_gotoxy(1, 2);
       sprintf(LCD, " C:ON   ");
       lcd_puts(LCD);
         
      
       }
       else{
          
        lcd_gotoxy(1, 2);
        sprintf(LCD, " C:OFF   ");
        lcd_puts(LCD);
     
       }
                
                lcd_gotoxy(1, 3);
                sprintf(LCD, "Setup=>");
                lcd_puts(LCD);

                lcd_gotoxy(10, 3);
               

                switch(setting){
                    case 0:
                        sprintf(LCDP, "seconds");
                        break;

                    case 1:
                        sprintf(LCDP, "minutes");
                        break;

                    case 2:
                        sprintf(LCDP, "hours  ");
                        break;

    
                }
                 lcd_puts(LCDP);
                 if(readanaloge==1){
                  sprintf(Buffer,"A0=%2.2f  ", spf);
                  lcd_gotoxy(1,4);
                  lcd_puts(Buffer); 
        
                  sprintf(Buffer,"A1=%2.2f", tempf);
                   lcd_gotoxy(10,4);
                   lcd_puts(Buffer); 
                 }
                 else{
                 lcd_gotoxy(1, 4);
                sprintf(LCDP, "Eman & Hadeel");
                 lcd_puts(LCDP);}
                if(! PORTBbits.RB3){
                    if(clkmode ==0 )
                    {
                        if(setting==0){
                        if(seconds== 59.0 && minutes<59.0 ){
                            minutes ++;
                            seconds=00.0;
                        }
                        else if(hours <23.0 && minutes== 59.0 && seconds==59 ){
                            hours++;
                            minutes=00.0;
                            seconds =00.0;
                        }
                        else if(hours==23 && minutes==59 && seconds == 59 ){
                            hours=0;
                            minutes=0;
                            seconds=0;
                        }
                        else
                            seconds++;
                    }
                        else if(setting ==1)
                    {
                        if(minutes < 59.0){
                            minutes ++;
                            //seconds=00.0;
                        } 
                        else if(minutes==59 &&hours<23){
                            hours++;
                            minutes=0;
                        }
                        else if(minutes==59 && hours==23){
                            hours=0;
                            minutes=0;
                            
                        }
                            
                    }
                        else {
                        if(hours<23){
                            hours++;
                
                        }
                        else{
                            hours=0;
                            
                        }
                            
                    }

                    
                    
                    
                        delay_ms(250);
                        while(! PORTBbits.RB3){
                            
                        }
                    }
                    
                }
                if(! PORTBbits.RB4){
                  if(clkmode ==0 )
                    {
                        if(setting==0){
                        if(seconds>0){
                            seconds--;
                        }
                        else if(seconds==0&& minutes>0  ){
                            
                            minutes--;
                            seconds=59;
                        }
                        else if(hours>0 && minutes==0 && seconds == 0 ){
                            hours--;
                            minutes=59;
                            seconds=59;
                        }
                        else if(hours==0 && minutes==0 && seconds==0){
                            seconds=59;
                            minutes=59;
                            hours=23;
                        }
                    }
                        else if(setting == 1)
                    {
                        if(minutes >0){
                            minutes --;
                            //seconds=00.0;
                        } 
                        else if(minutes==0 &&hours>0){
                            hours--;
                            minutes=59;
                            
                        }
                        else if(minutes==0&& hours==0){
                            hours=23;
                            minutes=59;
                            
                        }
                            
                    }
                        else {
                        if(hours>0){
                            hours--;
                
                        }
                        else if(hours==0){
                            hours=23;
                            
                        }
                            
                    }

                    
                    
                    
                        delay_ms(250);
                        while(! PORTBbits.RB4){
                            
                        }
                  }
                  
            }
                 
                
                break;
            case 2:
                clockingMode = 1;
                
                if((clkF == 1) && countFlag==0){
                    countFlag++;
                    if(thours>=counthours)
                    hours += (thours - counthours);
                    if(tminutes>=countminutes)
                    minutes += (tminutes - countminutes);
                    if(tseconds>=tseconds)
                    seconds += (tseconds - tseconds);
                    
                }
                
                lcd_gotoxy(1, 1);
                sprintf(LCD, "%2.0f:%2.0f:%2.0f ",hours,minutes,seconds);
                lcd_puts(LCD);

                              if(PORTCbits.RC4 == 1){
       lcd_gotoxy(10, 2);
       sprintf(LCD, " H:ON ");
       lcd_puts(LCD);
         
      
       }
       else{
          
        lcd_gotoxy(10, 2);
        sprintf(LCD, " H:OFF ");
        lcd_puts(LCD);
     
       }
         
       if(PORTCbits.RC2 == 1){
       lcd_gotoxy(1, 2);
       sprintf(LCD, " C:ON   ");
       lcd_puts(LCD);
         
      
       }
       else{
          
        lcd_gotoxy(1, 2);
        sprintf(LCD, " C:OFF   ");
        lcd_puts(LCD);
     
       }
                
                lcd_gotoxy(1, 3);
                sprintf(LCD, "Normal             ");
                lcd_puts(LCD);
                 if(readanaloge==1){
                  sprintf(Buffer,"A0=%2.2f  ", spf);
                  lcd_gotoxy(1,4);
                  lcd_puts(Buffer); 
        
                  sprintf(Buffer,"A1=%2.2f", tempf);
                   lcd_gotoxy(10,4);
                   lcd_puts(Buffer); 
                 }
                 else{
                 lcd_gotoxy(1, 4);
                sprintf(LCDP, "Fatimah Housnia");
                 lcd_puts(LCDP);}

                break;
                
            default:
                break;
        }
       // char res[32]="                                ";
        if(Serial[0]=='R'){
            if(Serial[1]=='t'){
                sprintf(LCD, "%2.0f:%2.0f:%2.0f ",hours,minutes,seconds);
              send_string_no_lib((unsigned char*)LCD);
                 send_string_no_lib((unsigned char*)"\r\n");
            }
            else if(Serial[1]=='A' && Serial[2]=='0'){
                sprintf(LCD, "Analog0=> %2.0f ",spf);
              send_string_no_lib((unsigned char*)LCD);
                 send_string_no_lib((unsigned char*)"\r\n");    
            }
            else if(Serial[1]=='A' && Serial[2]=='1'){
              sprintf(LCD, "Analog1=> %2.0f ",tempf);
              send_string_no_lib((unsigned char*)LCD);
                 send_string_no_lib((unsigned char*)"\r\n");    
            }
            else if(Serial[1]=='A' && Serial[2]=='2'){
              sprintf(LCD, "Analog2=> %2.0f ",temp1);
              send_string_no_lib((unsigned char*)LCD);
                 send_string_no_lib((unsigned char*)"\r\n");    
            }
            else if(Serial[1]=='H'){
                if(PORTCbits.RC4 == 1){
                      send_string_no_lib((unsigned char*)"Heater TURNED ON \r\n");
                }
                else{
                      send_string_no_lib((unsigned char*)"Heater SWITCHED OFF \r\n");
                }
              
               
            }
            else if(Serial[1]=='C'){
              if(PORTCbits.RC2 == 1){
                      send_string_no_lib((unsigned char*)"Cooler TURNED ON \r\n");
                }
                else{
                      send_string_no_lib((unsigned char*)"Cooler SWITCHED OFF \r\n");
                }
            }
            else if(Serial[1]=='D'){
                int portd[8]={0,0,0,0,0,0,0,0};
                portd[0]=PORTDbits.RD0;
                portd[1]=PORTDbits.RD1;
                portd[2]=PORTDbits.RD2;
                portd[3]=PORTDbits.RD3;
                portd[4]=PORTDbits.RD4;
                portd[5]=PORTDbits.RD5;
                portd[6]=PORTDbits.RD6;
                portd[7]=PORTDbits.RD7;
                  sprintf(LCD, "%d,%d,%d,%d,%d,%d,%d,%d",portd[0],portd[1],portd[2],portd[3],portd[4],portd[5],portd[6],portd[7]);
              send_string_no_lib((unsigned char*)LCD);
                 send_string_no_lib((unsigned char*)"\r\n");
                
            }
        }
        else if(Serial[0]=='W'){
            if(Serial[1]=='H' && Serial[2]=='V'&& Serial[3]=='1'){
                PORTCbits.RC5=1;
            }
            else if(Serial[1]=='H' && Serial[2]=='V'&& Serial[3]=='0'){
                PORTCbits.RC5=0;
            }
            else if(Serial[1]=='C' && Serial[2]=='V'&& Serial[3]=='0'){
                PORTCbits.RC2=0;
            } 
            else if(Serial[1]=='C' && Serial[2]=='V'&& Serial[3]=='1'){
                PORTCbits.RC2=1;
            }
            else if(Serial[1]=='0' && Serial[2]=='V' && Serial[3]=='0'){
                PORTDbits.RD0=0;
            }
             else if(Serial[1]=='0' && Serial[2]=='V' && Serial[3]=='1'){
                PORTDbits.RD0=1;
            }
             else if(Serial[1]=='1' && Serial[2]=='V' && Serial[3]=='0'){
                PORTDbits.RD1=0;
            }
             else if(Serial[1]=='1' && Serial[2]=='V' && Serial[3]=='1'){
                PORTDbits.RD1=1;
            }
             else if(Serial[1]=='2' && Serial[2]=='V' && Serial[3]=='0'){
                PORTDbits.RD2=0;
            }
             else if(Serial[1]=='2' && Serial[2]=='V' && Serial[3]=='1'){
                PORTDbits.RD2=1;
            }
             else if(Serial[1]=='3' && Serial[2]=='V' && Serial[3]=='0'){
                PORTDbits.RD3=0;
            }
             else if(Serial[1]=='3' && Serial[2]=='V' && Serial[3]=='1'){
                PORTDbits.RD3=1;
            }
             else if(Serial[1]=='4' && Serial[2]=='V' && Serial[3]=='0'){
                PORTDbits.RD4=0;
            }
             else if(Serial[1]=='4' && Serial[2]=='V' && Serial[3]=='1'){
                PORTDbits.RD4=1;
            }
             else if(Serial[1]=='5' && Serial[2]=='V' && Serial[3]=='0'){
                PORTDbits.RD5=0;
            }
             else if(Serial[1]=='5' && Serial[2]=='V' && Serial[3]=='1'){
                PORTDbits.RD5=1;
            }
             else if(Serial[1]=='6' && Serial[2]=='V' && Serial[3]=='0'){
                PORTDbits.RD6=0;
            }
             else if(Serial[1]=='6' && Serial[2]=='V' && Serial[3]=='1'){
                PORTDbits.RD6=1;
            }
             else if(Serial[1]=='7' && Serial[2]=='V' && Serial[3]=='0'){
                PORTDbits.RD7=0;
            }
             else if(Serial[1]=='7' && Serial[2]=='V' && Serial[3]=='1'){
                PORTDbits.RD7=0;
            }
            
        }
         for(int j=0;j<5;j++){
                Serial[j]='0';     
                 }
    }
   
    return;
}