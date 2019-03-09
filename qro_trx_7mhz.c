        ///////////////////////////////////////////////////////////////////
       //  Double DDS (AD9834, AD9835) for 7MHz HiPerformance TRX with  //
      //               ATMega644AP  and Line LCD 4x20                  //
     ///////////////////////////////////////////////////////////////////
    //                                                               //
   //  Compiler:         GCC (GNU AVR C-Compiler)                   //
  //  Autor:            Peter Rachow                               //
 //  Last modification: 2019-01-14                                //
///////////////////////////////////////////////////////////////////

  ////////////////////
 //   PORT USAGE   //
////////////////////

//O U T P U T 

//DDS 1 SPI interface
//PD0: FSYNC 
//PD1  SCLK 
//PD2  DATA 

//DDS 2 SPI interface
//PC4 FSYNC
//PC5 SDATA
//PC6 SCLK

//PD3 AGC fast/slow

//LCD 4-Bit parallel interface
//PD4 LCD RS 
//PD5 LCD RW
//PD6 LCD E
//PD7 LCD Backlight

//PC0 LCD D0
//PC1 LCD D1
//PC2 LCD D2
//PC3 LCD D3

//I N P U T 
//ADC0 Voltage control
//ADC1 S-Meter
//ADC2 PWR-Meter
//ADC3 Keys
//ADC4 Temp sensor 0
//ADC5 TX/RX detect
//ADC6 Temp sensor 1

//PB1, PB2 Rotary encoder

//  EEPROM  //////////////////////////////////////////////////
// Bytes:
// 0:39: VFO frequencies of VFO 0:9 in blocks by 4 bytes
// 127: Last VFO selected
// 128: Backlight set
// 129: AGC fast=0;slow=1
//////////////////////////////////////////////////////////////

#define F_CPU 8000000

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/eeprom.h>
#include <util/delay.h>

int main(void);
long menu(long, long);

///////////////////////
//  SPI DDS1  AD9834 //
///////////////////////
#define DDS1_PORT PORTD
#define DDS1_DDR DDRD
#define DDS1_FSYNC 1 //green
#define DDS1_SCLK 2  //blue
#define DDS1_SDATA 4 //white

void spi1_start(void);
void spi1_send_bit(int);
void spi1_stop(void);
void set_frequency1(long);
long scan_memories(void);

///////////////////////
//  SPI DDS2 AD9835  //
///////////////////////
#define DDS2_PORT PORTC
#define DDS2_DDR  DDRC
#define DDS2_FSNYC 16 //green PC4
#define DDS2_SDATA 32 //PC5
#define DDS2_SCLK 64  //PC6

void spi2_send_word(unsigned int);
void set_frequency2(long);

  ///////////////////
 //  LCD-Display  //
///////////////////
#define LCD_DDR DDRC         //DDR for LCD D0:D3
#define LCD_PORT PORTC      //Port for LCD D0:D3
#define RS_PORT PORTD      //Port for LCD RS line
#define E_PORT PORTD      //Port for LCD E line
#define RW_PORT PORTD    //Port for LCD RW line
#define LCD_D0 1   //yellow
#define LCD_D1 2   //darkgreen
#define LCD_D2 4   //white
#define LCD_D3 8   //darkblue
#define LCD_RS 16  //darkblue
#define LCD_RW 32  //white
#define LCD_E  64  //green
#define LCD_BL 128 //violet

#define ICONSOLID 0x10
#define ICONOFF   0x00

//LCD hardware based functions
void lcd_write(char, unsigned char);
void lcd_write(char, unsigned char);
void lcd_init(void);
void lcd_cls(void);
void lcd_line_cls(int);
void lcd_putchar(int, int, unsigned char);
void lcd_putstring(int, int, char*);
int lcd_putnumber(int, int, long, int, int, char, char);
void defcustomcharacters(void);
int lcd_check_busy(void);

//LCD display for radio
void s_meter(int);
void show_frequency(long); 
void show_txfrequency(long);
void show_pa_temp(int, int);
void show_voltage(int);
void show_vfo(int, int);
void show_tuning_step(int, int);
void show_sideband(int, int);
void lcd_set_icon(int, int);
void lcd_set_batt_icon(int);
void lcd_setbacklight(int);
void show_data(long, int, int, int, int, int, int, int);
void show_msg(char*);
void show_agc(int, int);

//ADC
//ADC Channels
//ADC0: s-meter
int get_adc(int);
int get_keys(void);
int get_temp(int);

//Interfrequency options
#define IFOPTION 0

#if (IFOPTION == 0)
    //9MHz Filter 9XMF24D (box73.de)
    #define INTERFREQUENCY 9000000
    #define IF_LSB 8998500
    #define IF_USB 9001500
    long fbfo[] = {IF_LSB, IF_USB}; 
#endif  
  
#if (IFOPTION == 1)
    //10.695MHz Filter 10M04DS (ex CB TRX "President Jackson")
    #define INTERFREQUENCY 10695000 //fLSB 10692100, fUSB 10697700
    #define IF_LSB 10691900
    #define IF_USB 10697900
    long fbfo[] = {IF_LSB, IF_USB}; 
#endif  

#if (IFOPTION == 2)
    //10.7MHz Filter 10MXF24D (box73.de)
    #define INTERFREQUENCY 10700000 
    #define IF_LSB 10697720
    #define IF_USB 10702220
    long fbfo[] = {IF_LSB, IF_USB}; 
#endif  

//Radio data, tuning, frequencies etc.
#define MAXSTEPS 6
#define STDTSTEP 2
int sideband = 0;
int tuning = 0;
int tuningstep[] = {10, 50, 100, 500, 1000, 5000};
int cur_tstep = STDTSTEP;
int cur_vfo = 0;
long f_vfo[10];  //Max. 10 VFOs
int split = 0;
int agc = 0; //fast

void set_agc(int);

//EEPROM
void store_frequency(long, int);
long load_frequency(int);
void store_last_vfo(int); 
int load_last_vfo(void); 
int is_mem_freq_ok(long);

//Timer 1
unsigned long runseconds = 0;

//Backlight
int light = 1;

  /////////////////
 //  SPI DDS 1  //
/////////////////
void spi1_start(void)
{
	DDS1_PORT |= DDS1_SCLK;      //SCLK hi
    DDS1_PORT &= ~(DDS1_FSYNC);  //FSYNC lo
}

void spi1_stop(void)
{
	DDS1_PORT |= DDS1_FSYNC; //FSYNC hi
}

void spi1_send_bit(int sbit)
{
    if(sbit)
	{
		DDS1_PORT |= DDS1_SDATA;  //SDATA hi
	}
	else
	{
		DDS1_PORT &= ~(DDS1_SDATA);  //SDATA lo
	}
	
	DDS1_PORT |= DDS1_SCLK;     //SCLK hi
    DDS1_PORT &= ~(DDS1_SCLK);  //SCLK lo
}

void set_frequency1(long f)
{

    double fword0;
    long fword1, x;
    int l[] = {0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    int m[] = {0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, t1;
    
    //x = 268435456 / fClk
    //fword0 = (double) 3.579139413 * f; // 75MHz
    fword0 = (double) 2.440302361  * (f + INTERFREQUENCY - 1500); //fClk = 110.000.900 MHZ
    //fword0 = (double) 5.36870912  * f; //fClk = 50MHz
    
    fword1 = (long) fword0;

    //Transfer frequency word to byte array
    x = (1 << 13);      //2^13
    for(t1 = 2; t1 < 16; t1++)
    {
		if(fword1 & x)
	    {
			l[t1] = 1;
	    }
	    x >>= 1;
    }
    
    x = (1L << 27);  //2^27
    for(t1 = 2; t1 < 16; t1++)
    {
	    if(fword1 & x)
	    {
	        m[t1] = 1;
	    }
	    x >>= 1;
    }
    ////////////////////////////////////////
    
    //Transfer to DDS
    //Send start command
    spi1_start();
    for(t1 = 15; t1 >= 0; t1--)
    {
       spi1_send_bit(0x2000 & (1 << t1));
    }
    spi1_stop();
        
    //Transfer frequency word	
    //L-WORD
    spi1_start();
    for(t1 = 0; t1 < 16; t1++)
    {
       spi1_send_bit(l[t1]);
    }
    spi1_stop();
	
	//M-WORD
	spi1_start();
    for(t1 = 0; t1 < 16; t1++) 
    {
       spi1_send_bit(m[t1]);
    }
    spi1_stop();
}

  /////////////////
 //  SPI DDS 2  //
/////////////////
void spi2_send_word(unsigned int sbyte)
{
    unsigned int t1, x = 32768;
    for(t1 = 0; t1 < 16; t1++)
    {
        if(sbyte & x)
        {
            DDS2_PORT |= DDS2_SDATA; 
        }
        else
        {
            DDS2_PORT &= ~(DDS2_SDATA);  
        }
        
        DDS2_PORT |= DDS2_SCLK;  
        DDS2_PORT &= ~(DDS2_SCLK); 
        
        x = x >> 1;
    }
    DDS2_PORT |= DDS2_SDATA; 
}

void set_frequency2(long f)
{
    long fxtal = 50000000;  //fXtal in MHz
    double fword0;
    long fword1;
    int t1;

    unsigned char xbyte[] = {0x33, 0x22, 0x31, 0x20};    
    fword0 = (double) f / fxtal;
    fword1 = (long) (fword0 * 0xFFFFFFFF);

    //Send 4 * 16 Bit to DDS
    for(t1 = 0; t1 < 4; t1++)
    {
        DDS2_PORT &= ~(DDS2_FSNYC); 
        spi2_send_word((xbyte[t1] << 8) + (((fword1 >> ((3 - t1) * 8))) & 0xFF));
        DDS2_PORT |= DDS2_FSNYC; 
    }

    //End of sequence
    DDS2_PORT &= ~(DDS2_FSNYC); 
    spi2_send_word(0x8000);
    DDS2_PORT &= ~(DDS2_FSNYC); 
}

long scan_memories(void)
{
	int t1;
	long ftmp;
	int key = 0;
	long time1 = 0;
	int runsecs2 = 0;
	
	lcd_putstring(3, 3, "(QUIT)  (SEL)");
	
	while(!key)
	{
	    for(t1 = 0; t1 < 10; t1++)
	    {
		    ftmp = load_frequency(t1);
		    if(is_mem_freq_ok(ftmp))
		    {
			    set_frequency1(ftmp);
			    show_frequency(ftmp);
			    lcd_putstring(2, 0, "VFO");
	            lcd_putchar(2, 3, t1 + 0x80);
	            time1 = runseconds;
	            runsecs2 = runseconds;
	            lcd_putnumber(2, 5, runseconds - time1, -1, -1, 'l', 0);
	            while(time1 + 3 > runseconds)
	            {
			        key = get_keys();
			        if(runsecs2 != runseconds)
			        {
						lcd_putnumber(2, 5, runseconds - time1, -1, -1, 'l', 0);
						runsecs2 = runseconds;
					}	
		    	    switch(key)
		            {
			            case 1: return 0;
			                    break;
			            case 2: cur_vfo = t1;
			                    return cur_vfo;
		            }
		        }
		     }       
	    }
	}    
	return 0;
}			        
			
			
		

  ///////////////////////////////
 //  L   C   D   Module 4x20  //
///////////////////////////////

// Write CMD or DATA to LCD
void lcd_write(char lcdmode, unsigned char value)
{
    int t1;
    
    while(lcd_check_busy());  //Check busy flag
    
	LCD_DDR |= 0x0F;          //Set DDR data lines as output
	RW_PORT &= ~(LCD_RW);     //Set RW to write operation, i. e. =0
	
    E_PORT &= ~(LCD_E);       //E=0
    if(!lcdmode)
	{
        RS_PORT &= ~(LCD_RS); //CMD
	}	
    else
	{
        RS_PORT |= LCD_RS;    //DATA
	}	
    
    //HI NIBBLE    
    E_PORT |= LCD_E; //E = 1
    for(t1 = 0; t1 < 4; t1++)
	{
	    if(((value & 0xF0) >> 4) & (1 << t1))
	    {
	       LCD_PORT |= (1 << t1);      
	    }
        else	
	    {
           LCD_PORT &= ~(1 << t1);     
	    }  
	}	
	E_PORT &= ~(LCD_E);
	
	//LO NIBBLE
	E_PORT |= LCD_E;
	for(t1 = 0; t1 < 4; t1++)
	{
	    if(value  & (1 << t1))
	    {
	       LCD_PORT |= (1 << t1);      
	    }
        else	
	    {
           LCD_PORT &= ~(1 << t1);     
	    }  
	}
    E_PORT &= ~(LCD_E);
    _delay_ms(1);
}


//Send one char to LCD
void lcd_putchar(int row, int col, unsigned char ch)
{
	//lcd_write(0, 0x80 + col + row * 0x40);
    lcd_write(0, 0x80 + col + row * 0x20);
    lcd_write(1, ch);
}


//Print out \0-terminated string on LCD
void lcd_putstring(int row, int col, char *s)
{
    unsigned char t1 = col;

    while(*(s))
	{
        lcd_putchar(row, t1++, *(s++));
	}	
}

//Clear LCD
void lcd_cls(void)
{
    lcd_write(0, 1);
}

//Init LCD
void lcd_init(void)
{
	int t1;
		   
    // Basic settings of LCD
    // 4-Bit mode, 2 lines, 5 pixels width matrix
    lcd_write(0, 0x28);
        
    //4-line mode
    lcd_write(0, 0x2C); //RE=1
    lcd_write(0, 0x09);
    lcd_write(0, 0x28); //RE=0
        
    //Switch icons off
    lcd_write(0, 0x2C); //RE=1
    lcd_write(0, 0x40); //Set SEGRAM addr to 0x00
    for(t1 = 0; t1 < 15; t1++)
    {
		lcd_write(1, 0);
	}	
    lcd_write(0, 0x28); //RE=0
            
    // Display on, Cursor off, Blink off 
    lcd_write(0, 0x0C);
    
    // No display shift, no cursor move
    lcd_write(0, 0x04);
    
}

//Set icon by number, -1 switches allicons off
void lcd_set_icon(int icon, int status)
{
		
	//Set icon
    lcd_write(0, 0x2C); //RE=1
    lcd_write(0, 0x40); //Set SEGRAM addr to 0x00
    lcd_write(0, 0x40 + icon); //Select icon addr
	lcd_write(1, status); //Activate icon in given mode
	lcd_write(0, 0x28); //RE=0
}	

//Set icon by number, -1 switches allicons off
void lcd_set_batt_icon(int volts1)
{
	int v[] = {0x10, 0x18, 0x1C, 0x1E, 0x1F};
	int t1;
 
    for(t1 = 11; t1 < 15; t1++)
	{
	    if(volts1 >= t1 * 10 && volts1 < (t1 + 1) * 10)
		{
		    //Set icon
            lcd_write(0, 0x2C); //RE=1
            lcd_write(0, 0x40); //Set SEGRAM addr to 0x00
            lcd_write(0, 0x40 + 0x0F); //Select icon addr
	        lcd_write(1, v[t1 - 10]); //Activate icon in given mode
            lcd_write(0, 0x28); //RE=0
        }   
    }   
}	

int lcd_check_busy(void)
{
	unsigned char value;
	
	LCD_DDR &= ~(0x0F);    //LCD_PORT data line bits D0:D3 to rx mode
	   
    RW_PORT |= LCD_RW;     //Read operation => RW=1
	
	RS_PORT &= ~(LCD_RS); //CMD => RS=0: for busy flag
	
	//Read data
	//Hi nibble
	E_PORT |= LCD_E;          //E=1
    _delay_us(1);       
	value = (PIND & 0x0F) << 4;
    E_PORT &= ~(LCD_E);       //E=0	
		
	//Lo nibble
	E_PORT |= LCD_E;          //E=1
    _delay_us(1);       
	value += (PIND & 0x0F);
    E_PORT &= ~(LCD_E);       //E=0	
		
	LCD_DDR |= 0x0F;         //Set DDR 0:3 as output
	
	return (value >> 8) & 1;
}  


//Set backlight
void lcd_setbacklight(int duty_cycle)
{
	double dc = 255 - (duty_cycle * 2.55);
	
	OCR2A = (int) dc;

}	

//Define chars
void defcustomcharacters(void)
{
    int i1;
    unsigned char adr=64;

    unsigned char customchar[]={0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,   // #0
		                        0x04, 0x0A, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00,   // #1 ° sign
		                        0x00, 0x00, 0x00, 0x07, 0x04, 0x04, 0x04, 0x04,   // #2 ,-
		                        0x00, 0x00, 0x00, 0x1C, 0x04, 0x04, 0x04, 0x04,   // #3 -,
		                        0x04, 0x04, 0x04, 0x1C, 0x00, 0x00, 0x00, 0x00,   // #4 -'
		                        0x04, 0x04, 0x04, 0x07, 0x00, 0x00, 0x00, 0x00,   // #5 '-
		                        0x00, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x00, 0x00,   // #6 -
		                        0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04};  // #7 |
    lcd_write(0, 0);
    lcd_write(1, 0);

    //Send data to CGRAM in lcd
    for (i1 = 0; i1 < 64; i1++)
    {
        lcd_write(0, adr++);
        lcd_write(1, customchar[i1]);
    }
}

//Write an n-digit number (int or long) to LCD
int lcd_putnumber(int row, int col, long num, int digits, int dec, char orientation, char showplussign)
{
    char cl = col, minusflag = 0;
    unsigned char cdigit[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, digitcnt = 0;
    long t1, t2, n = num, r, x = 1;

    if(num < 0)
    {
        minusflag = 1;
        n *= -1;
    }

    /* Stellenzahl automatisch bestimmen */
    if(digits == -1)
    {
        for(t1 = 1; t1 < 10 && (n / x); t1++)
		{
            x *= 10;
		}	
        digits = t1 - 1;
    }

    if(!digits)
    {
        digits = 1;
    }
    
    for(t1 = digits - 1; t1 >= 0; t1--)
    {
        x = 1;
        for(t2 = 0; t2 < t1; t2++)
            x *= 10;
        r = n / x;
        cdigit[digitcnt++] = r + 48;

        if(t1 == dec) 
            cdigit[digitcnt++] = 46;
        n -= r * x;
    }

    digitcnt--;
    t1 = 0;

    /* Ausgabe */
    switch(orientation)
    {
        case 'l':   cl = col;
                    if(minusflag)
                    {
                        lcd_putchar(row, cl++, '-');
                        digitcnt++;
                    }	 
		            else
		            {
		                if(showplussign)
			            {
			                lcd_putchar(row, cl++, '+');
                            digitcnt++;
			            } 
                    }	
			
                    while(cl <= col + digitcnt)                       /* Linksbuendig */
		            {
                        lcd_putchar(row, cl++, cdigit[t1++]);
					}	
                    break;

        case 'r':   t1 = digitcnt;                              /* Rechtsbuendig */
                    for(cl = col; t1 >= 0; cl--)              
					{
                        lcd_putchar(row, cl, cdigit[t1--]);
                        if(minusflag)	
						{
                            lcd_putchar(row, --cl, '-');
                        }
					}	
    }
	
    if(dec == -1)
	{
        return digits;
	}	
    else
	{
        return digits + 1;	
	}	
}	

//CLS for one LCD line
void lcd_line_cls(int ln)
{
    int t1;
	
	for(t1 = 0; t1 < 15; t1++)
	{
	    lcd_putchar(1, t1, 32);
	}
}	



  ///////////////////////////////
 //   TRX display functions   //
///////////////////////////////

//Display current frequency divide by 100 with
//dec. separator
void show_frequency(long f)
{
	int row = 2, col = 11;
	lcd_putnumber(row, col, f / 100, -1, 1, 'l', 0);
	lcd_putstring(row, col + 6, "kHz");
}	

void show_txfrequency(long f) //Show TX freq for split mode
{
	int row = 2, col = 0;
	if(f > 0)
	{
	    lcd_putnumber(row, col, f / 100, -1, 1, 'l', 0);
	    lcd_putstring(row, col + 6, "kHz");
	}
	else
	{
		lcd_putstring(row, col, "           ");
	}	
	
}	

void show_voltage(int v10)
{
	int row = 0, col = 15;
	lcd_putchar(row, col + lcd_putnumber(row, col, v10, -1, 1, 'l', 0), 'V');
}	

void show_vfo(int vfo,  int showmode)
{
	int row, col;
		
    if(showmode)
	{
		row = 2, col = 2;
	}
	else
	{	
		row = 0, col = 10;
	}   
	
	lcd_putstring(row, col, "VFO");
	lcd_putchar(row, col + 3, vfo + 0x80);
	
}	

void show_tuning_step(int ts, int showmode)
{
	int row, col;
	
	char *tunstep[] = {"10Hz ", "50Hz ", "100Hz", "500Hz", "1kHz ", "5kHZ "};
		
	if(showmode)
	{
		row = 2, col = 7;
	}
	else
	{	
		row = 0, col = 0;
	}   
	lcd_putstring(row, col, "      ");
	lcd_putstring(row, col, tunstep[ts]); 
}

void show_sideband(int sb, int showmode)
{
	int row, col;
	
	char *sidebandstr[] = {"LSB", "USB"};
	    
	if(showmode)
	{
		row = 2, col = 9;
	}
	else
	{	
		row = 0, col = 6;
	}   
		
	lcd_putstring(row, col, sidebandstr[sb]);    
}

void show_pa_temp(int temp, int sensor)
{
	int row = 1, col = 0;
	char unitstr[] = {1, 'C', 0};
	
	lcd_putstring(row, col + lcd_putnumber(row, col, temp, -1, 1, 'l', 0), unitstr);
	lcd_putchar(row, col + 6, sensor + 16);
}
	
//Show meter value
void s_meter(int value)
{
	int c[] = {32, 212, 211, 210, 209, 208};
	int v, t1;
	int maxs = 35, row = 3, col = 0;
	
	//Clear meter
	if(value == -1)
	{
		for(t1 = 0; t1 < 7; t1++)	 
		{
			lcd_putchar(row, col + t1, 32);
		}
		return;
	}
			
	v = value;
	if(v > maxs)
	{
		v = maxs;
	}	
	
	//Full blocks first
	for(t1 = 0; t1 < v / 5; t1++)
	{
		lcd_putchar(row, col + t1, 208);
	}	
	
	//Remaining char
	if(col + t1 < 20)
	{
	     lcd_putchar(row, col + t1, c[v - (v / 5) * 5]);
	     lcd_putchar(row, col + t1 + 1, 32);
	}     
}

void show_msg(char *msgtxt)
{
	if(msgtxt[0] == 0)
	{
		lcd_putstring(2, 0, "       ");
		return;
	}
		
	lcd_putstring(2, 0, msgtxt);	
}	

void show_agc(int agcset, int showmode)
{
   	int row, col;
	
	if(showmode)
	{
		row = 2, col = 8;
	}
	else
	{	
		row = 1, col = 8;
	}   
	
	switch(agcset)
	{
	    case 0: lcd_putstring(row, col, "Fast");
	            break;
	    case 1: lcd_putstring(row, col, "Slow");
	            break;
	 }  
}		        
	
//Show all radio data at once
void show_data(long freq, int t_step, int side_band, int pa_temp, int sensor, int vfo, int vdd, int agc_set)
{
    show_frequency(freq);  
    show_tuning_step(t_step, 0);
    show_sideband(side_band, 0);
    show_pa_temp(pa_temp, sensor);
    show_vfo(vfo, 0);    
    show_voltage(vdd);
    lcd_set_batt_icon(vdd);
    show_agc(agc_set, 0);
}

  //////////////////////
 //    A   D   C     //
//////////////////////
//Read ADC value
int get_adc(int adc_channel)
{
	int adc_val = 0;
	
	//ADC config and ADC init
    ADCSRA = (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1); //Activate ADC, Prescaler=64

	ADMUX = (1 << REFS0) + adc_channel;     //Read ADC channel with Vref=VCC
	
    _delay_ms(3);
	
    ADCSRA |= (1<<ADSC);
    while(ADCSRA & (1<<ADSC));
	_delay_ms(3);
	
	ADCSRA |= (1<<ADSC);
    while(ADCSRA & (1<<ADSC));
	
	adc_val = ADCL;
    adc_val += ADCH * 256;   
	
	ADCSRA &= ~(1<<ADEN); //Deactivate ADC
	
	return adc_val;
}	

//Read keys via ADCx
int get_keys(void)
{

    int key_value[] = {127, 109};
    int t1;
    int adcval = get_adc(3);
    long ti0, ti1; //Time stamps for detecting push duration
    long runsecsold0 = runseconds;
    long runsecsold1 = runseconds;
           
    //TEST display of ADC value 
    //lcd_putstring(2, 0, "    ");
    //lcd_putnumber(2, 0, adcval, -1, -1, 'l', 0);    
    ti0 = runseconds;		
    for(t1 = 0; t1 < 2; t1++)
    {
        if(adcval > key_value[t1] - 3 && adcval < key_value[t1] + 3)
        {
			
			while(adcval < 512)
			{
			    adcval = get_adc(3);
			    if(runsecsold1 != runseconds)
			    {
					lcd_putnumber(3, 0, runseconds - runsecsold0, -1, -1, 'l', 0);
					runsecsold1 = runseconds;
				}	
			}    	
			ti1 = runseconds - ti0;
            if(ti1 > 1)
            {
		         return(t1 + 1 + 128);
	        }
	        else
	        {
	            return t1 + 1;
	        }    
        }
    }
    	
    return 0;
}

//Measure temperature of final amplifier
//Sensor = KTY81-210
int get_temp(int sensor)
{
    double temp;
	double rt, rv = 5100, ut;
	double r0 = 1630; //Resistance of temperature sensor at 0°C
	double m = 17.62;   //slope of temperature sensor in Ohms/K
	
	//Calculate voltage from ADC value
	if(!sensor)
	{
	    ut = (double)get_adc(4) * 5 / 1024;
	}   
	else
	{
	    ut = (double)get_adc(6) * 5 / 1024;
	}   
	
	//Calculate thermal resistor value from ADC voltage ut
	rt = rv / (5 / ut - 1);
	
	//Calculate temperature from rt
	temp = 10 * ((rt - r0) / m);
		
	return (int)(temp);
}	

//////////////////////
//   E  E  P  R  O  M
//////////////////////
void store_frequency(long f, int memplace)
{
    long hiword, loword;
    unsigned char hmsb, lmsb, hlsb, llsb;
	
    int start_adr = memplace * 4;
    
	cli();
    hiword = f >> 16;
    loword = f - (hiword << 16);
    hmsb = hiword >> 8;
    hlsb = hiword - (hmsb << 8);
    lmsb = loword >> 8;
    llsb = loword - (lmsb << 8);

    while(!eeprom_is_ready());
    eeprom_write_byte((uint8_t*)start_adr, hmsb);

    while(!eeprom_is_ready());
    eeprom_write_byte((uint8_t*)start_adr + 1, hlsb);

    while(!eeprom_is_ready());
    eeprom_write_byte((uint8_t*)start_adr + 2, lmsb);

    while(!eeprom_is_ready());
    eeprom_write_byte((uint8_t*)start_adr + 3, llsb);
    
    sei();	
}

long load_frequency(int memplace)
{
    long rf;
    unsigned char hmsb, lmsb, hlsb, llsb;
    int start_adr = memplace * 4;
		
    cli();
    hmsb = eeprom_read_byte((uint8_t*)start_adr);
    hlsb = eeprom_read_byte((uint8_t*)start_adr + 1);
    lmsb = eeprom_read_byte((uint8_t*)start_adr + 2);
    llsb = eeprom_read_byte((uint8_t*)start_adr + 3);
	sei();
	
    rf = (long) 16777216 * hmsb + (long) 65536 * hlsb + (unsigned int) 256 * lmsb + llsb;
		
	return rf;
	
}

void store_last_vfo(int vfo)
{
	eeprom_write_byte((uint8_t*)128, vfo);
}

int load_last_vfo(void)
{
	return eeprom_read_byte((uint8_t*)128);
}

////////////////////////////////////////////
//    Store or recall Frequency handling //
//////////////////////////////////////////
int is_mem_freq_ok(long f)
{
	if(f >= 6900000 && f <= 7300000)
	{
		return 1;
	}	
	else
	{
		return 0;
	}		
}	

  /////////////////////////
 // INTERRUPT HANDLERS  //
/////////////////////////
//Rotary encoder
ISR(INT2_vect)
{ 
    int gray = (PINB & 0x06) >> 1;           // Read PB1 and PB2
	int state = (gray >> 1) ^ gray;         // Convert from Gray code to binary

    if((state == 0) || state == 2) 
    {
		tuning = 1;
	}
	
	if((state == 1) || state == 3) 
    {
		tuning = -1;
	}	
} 

//Timer1
ISR(TIMER1_COMPA_vect)
{
    runseconds++;
}

  /////////////////////////
 //     USER CONTROL    //
/////////////////////////
//Return values:
//a) Frequency (long) for split mode, or
//b) VFO (0..9) as MEM scan result selected by user
//c) -1 in all other cases
long menu(long f, long fsplit)
{
	int menu_pos = cur_tstep;
	int key = 0;
	long ftmp;
	char *scan_str[] = {"OFF", "ON "};
	int x_vfo;
	
	lcd_cls();
	lcd_putstring(0, 6, "* Menu *");
	lcd_putstring(1, 4, "Tuning step");
	lcd_putstring(3, 3, "(NEXT)   (OK)");
			
	///////////////
	//Tuning step//
	///////////////
	while(get_keys());
	show_tuning_step(menu_pos, 1);
	while(key == 0)
	{
		//Change step
        if(tuning <= -1)
		{
		    if(menu_pos < MAXSTEPS - 1)
		    {
				menu_pos++;
			}	
			else
			{
				menu_pos = 0;
			}	
			show_tuning_step(menu_pos, 1);
			tuning = 0;
		}

		if(tuning >= 1)  
		{    
		    if(menu_pos > 0)
		    {
				menu_pos--;
			}
			else
			{
				menu_pos =  MAXSTEPS - 1;
			}		
			show_tuning_step(menu_pos, 1);
			tuning = 0;
		}			    
		key = get_keys();
	}	
		
	while(get_keys());
		
	switch(key)
	{
	    case 1:    break; //Go to next item!
	            
	    case 2:    cur_tstep = menu_pos;
	               return -1;         
	               break;
	               
	    case 129:  return -1;
	               break;
 	    
	}   
	
	  ///////////////
	 //  A  G  C  //
	///////////////
	menu_pos = agc;
	key = 0;
	
	lcd_cls();
	lcd_putstring(0, 6, "* Menu *");
	lcd_putstring(1, 8, "AGC");
	lcd_putstring(3, 3, "(NEXT)   (OK)");
	
	show_agc(menu_pos, 1);
		
	while(key == 0)
	{
		////Change sideband
        if(tuning <= -1)
		{
		    if(menu_pos < 1)
		    {
				menu_pos++;
			}	
			else
			{
				menu_pos = 0;
			}	
			show_agc(menu_pos, 1);
			tuning = 0;
		}

		if(tuning >= 1)  
		{    
		    if(menu_pos > 0)
		    {
				menu_pos--;
			}
			else
			{
				menu_pos =  1;
			}		
			show_agc(menu_pos, 1);
			tuning = 0;
		}			    
		key = get_keys();
	}	
	
	agc = menu_pos;
	set_agc(agc);
	
	while(!eeprom_is_ready());
	eeprom_write_byte((uint8_t*)129, agc);
	
	while(get_keys());
		
	switch(key)
	{
	    case 1:     break; //Go to next item!
	            
	    case 2: 
	    case 129:   return -1;
	}            
                 
		
	///////////////
	// Sideband  //
	///////////////
	menu_pos = sideband;
	key = 0;
	
	lcd_cls();
	lcd_putstring(0, 6, "* Menu *");
	lcd_putstring(1, 6, "Sideband");
	lcd_putstring(3, 3, "(NEXT)   (OK)");
	
	show_sideband(menu_pos, 1);
	while(key == 0)
	{
		////Change sideband
        if(tuning <= -1)
		{
		    if(menu_pos < 1)
		    {
				menu_pos++;
			}	
			else
			{
				menu_pos = 0;
			}	
			set_frequency2(fbfo[menu_pos]);
			show_sideband(menu_pos, 1);
			tuning = 0;
		}

		if(tuning >= 1)  
		{    
		    if(menu_pos > 0)
		    {
				menu_pos--;
			}
			else
			{
				menu_pos =  1;
			}		
			set_frequency2(fbfo[menu_pos]);
			show_sideband(menu_pos, 1);
			tuning = 0;
		}			    
		key = get_keys();
	}	
	
	while(get_keys());
		
	switch(key)
	{
	    case 1:     set_frequency2(fbfo[sideband]);
			        show_sideband(sideband, 1);
	                break; //Go to next item!
	            
	    case 2:     sideband = menu_pos;
	                set_frequency2(fbfo[menu_pos]);
			        show_sideband(menu_pos, 1);
	                return -1;
	                break;
	    case 129:   return -1;
	    
	}            
        
	///////////////
	//     VFO   //
	///////////////
	
	//Load VFO to current freq register
	store_frequency(f, cur_vfo);
	store_last_vfo(cur_vfo);
	
	menu_pos = cur_vfo;
	key = 0;
	
	lcd_cls();
	lcd_putstring(0, 6, "* Menu *");
	lcd_putstring(1, 5, "Select VFO");
	lcd_putstring(3, 3, "(NEXT)   (OK)");
	show_vfo(menu_pos, 1);
	ftmp = load_frequency(menu_pos);
    show_frequency(ftmp);
    				
	while(key == 0)
	{
		//Change VFO
        if(tuning <= -1)
		{
		    if(menu_pos < 9)
		    {
				menu_pos++;
			}	
			else
			{
				menu_pos = 0;
			}	
			show_vfo(menu_pos, 1);
			
			//Set freq if value OK!
			ftmp = load_frequency(menu_pos);
			if(is_mem_freq_ok(ftmp))
			{
				set_frequency1(ftmp);
				show_frequency(ftmp);
				lcd_putchar(2, 8, 32);
			}	
			else
			{
				lcd_putchar(2, 8, '!');
			}	
			tuning = 0;
		}

		if(tuning >= 1)  
		{    
		    if(menu_pos > 0)
		    {
				menu_pos--;
			}
			else
			{
				menu_pos =  9;
			}		
			show_vfo(menu_pos, 1);
			//Set freq if value OK!
			ftmp = load_frequency(menu_pos);
			if(is_mem_freq_ok(ftmp))
			{
				set_frequency1(ftmp);
				show_frequency(ftmp);
				lcd_putchar(2, 8, 32);
			}	
			else
			{
				lcd_putchar(2, 8, ' ');
			}	

			tuning = 0;
		}			    
		key = get_keys();
	}	
	
	while(get_keys());
		
	switch(key)
	{
	    case 1: set_frequency1(f_vfo[cur_vfo]); //Restore old VFO and QRG
				show_frequency(f_vfo[cur_vfo]);
	            break; //Go to next item!
	            
	    case 2: cur_vfo = menu_pos;
	            store_last_vfo(cur_vfo);
	            ftmp = load_frequency(cur_vfo);
			    if(is_mem_freq_ok(ftmp))
			    {
					f_vfo[cur_vfo] = ftmp;
				    set_frequency1(ftmp);
				    show_frequency(ftmp);
			    }	
			    else
			    {
					f_vfo[cur_vfo] = f;
				    set_frequency1(f);
				    show_frequency(f);
			    }	
			    show_vfo(cur_vfo, 0);
	            return -1;
	     
	     case 129: return -1;       
	}            
	
	//Store current QRG to VFO memory
	store_frequency(f, cur_vfo);
	store_last_vfo(cur_vfo);
	
	menu_pos = cur_vfo;
	key = 0;
	
	lcd_cls();
	lcd_putstring(0, 6, "* Menu *");
	lcd_putstring(1, 5, "Save QRG");
	lcd_putstring(3, 3, "(NEXT)   (OK)");
	show_vfo(menu_pos, 1);
	ftmp = f;
    show_frequency(ftmp);
    				
	while(key == 0)
	{
		//Change VFO
        if(tuning <= -1)
		{
		    if(menu_pos < 9)
		    {
				menu_pos++;
			}	
			else
			{
				menu_pos = 0;
			}	
			show_vfo(menu_pos, 1);
			
			//Set freq if value OK!
			ftmp = load_frequency(menu_pos);
			if(is_mem_freq_ok(ftmp))
			{
				set_frequency1(ftmp);
				show_frequency(ftmp);
				lcd_putchar(2, 8, 32);
			}	
			else
			{
				lcd_putchar(2, 8, '!');
			}	
			tuning = 0;
		}

		if(tuning >= 1)  
		{    
		    if(menu_pos > 0)
		    {
				menu_pos--;
			}
			else
			{
				menu_pos =  9;
			}		
			show_vfo(menu_pos, 1);
			//Set freq if value OK!
			ftmp = load_frequency(menu_pos);
			if(is_mem_freq_ok(ftmp))
			{
				set_frequency1(ftmp);
				show_frequency(ftmp);
				lcd_putchar(2, 8, 32);
			}	
			else
			{
				lcd_putchar(2, 8, ' ');
			}	

			tuning = 0;
		}			    
		key = get_keys();
	}	
	
	while(get_keys());
		
	switch(key)
	{
	    case 1: set_frequency1(f_vfo[cur_vfo]); //Restore old VFO and QRG
				show_frequency(f_vfo[cur_vfo]);
	            break; //Go to next item!
	            
	    case 2: cur_vfo = menu_pos;
	            store_last_vfo(cur_vfo);
	            f_vfo[cur_vfo] = f;
				set_frequency1(f);
				show_frequency(f);
	            return -1;
	    case 129: return -1;        
	}            
	
	
	///////////////
	// BACKLIGHT //
	///////////////
	menu_pos = light;
	key = 0;
	
	lcd_cls();
	lcd_putstring(0, 6, "* Menu *");
	lcd_putstring(1, 5, "Backlight");
	lcd_putstring(3, 3, "(NEXT)   (OK)");
	
	while(key == 0)
	{
		lcd_putchar(2, 9 + lcd_putnumber(2, 9, menu_pos, -1, -1, 'l', 0), '%');;
		
		//Set light by changing PWM duty cycle
        if(tuning <= -1)
		{
		    if(menu_pos < 100)
		    {
				menu_pos++;
			}	
			else
			{
				menu_pos = 20;
				lcd_putstring(2, 9, "     ");
			}	
			lcd_setbacklight(menu_pos);
			tuning = 0;
		}

		if(tuning >= 1)  
		{    
		    if(menu_pos > 20)
		    {
				menu_pos--;
			}
			else
			{
				menu_pos =  100;
			}		
			if(menu_pos == 99);
			{
				lcd_putstring(2, 9, "     ");
			}	
			lcd_setbacklight(menu_pos);
			tuning = 0;
		}			    
		key = get_keys();
		light = menu_pos;
	}	
	eeprom_write_byte((uint8_t*)128, light);
	while(get_keys());
	
	if(key == 129)
	{
		return -1;
	}	
	
	///////////////
	//   Split   //
	///////////////
	menu_pos = split;
	key = 0;
	
	lcd_cls();
	lcd_putstring(0, 6, "* Menu *");
	lcd_putstring(1, 7, "Split");
	lcd_putstring(3, 3, "(NEXT)   (OK)");
	
	while(key == 0)
	{
		if(menu_pos == 0)
		{
			lcd_putstring(2, 8, "OFF");
		}
		else	
		{
			lcd_putstring(2, 8, "ON ");
		}
		
		//Set split mode
        if(tuning <= -1)
		{
		    if(menu_pos < 1)
		    {
				menu_pos++;
			}	
			else
			{
				menu_pos = 0;
			}	
			tuning = 0;
		}

		if(tuning >= 1)  
		{    
		    if(menu_pos > 0)
		    {
				menu_pos--;
			}
			else
			{
				menu_pos =  1;
			}		
			tuning = 0;
		}	
		
		key = get_keys();
	}	
	while(get_keys());
	
	switch(key)
	{
	    case 1: break; //Go to next item!
	            
	    case 2: split = menu_pos;
	            break;
	    case 129: return -1;         
	   
	}   

	//Set split TX frequency
	if(split)
	{
		lcd_cls();
	    lcd_putstring(0, 6, "* Menu *");
	    lcd_putstring(1, 2, "SEL Split Freq.");
	    lcd_putstring(3, 3, "(NEXT)   (OK)");
	
	    if(is_mem_freq_ok(fsplit))
	    {
		    ftmp = fsplit;
		}
		else
		{
			ftmp = f;
		}	    
		show_txfrequency(ftmp);    	
		key = 0;
		
		while(key == 0)
		{
		    //Set TX freq
            if(tuning <= -1)
		    {
		        ftmp += tuningstep[cur_tstep];
		        set_frequency1(ftmp);    		 
	            show_txfrequency(ftmp);    	
			    tuning = 0;
		    }

		    if(tuning >= 1)  
		    {    
		        ftmp -= tuningstep[cur_tstep];
		        set_frequency1(ftmp);    		 
	            show_txfrequency(ftmp);    	
			    tuning = 0;
		    }	
	 	    key = get_keys();
	    }	
	    while(get_keys());
	}    
		
	switch(key)
	{
	    case 1: break; //Go to next item or return
	            return 0;
	            
	    case 2: if(split)
	            {
					return ftmp;
				}
				else
				{
					return -1;
				}	
		case 129: return -1;		
	}   
	
	while(get_keys());
	
	///////////////
	//  MEM SCAN //
	///////////////
	menu_pos = 0;
	key = 0;
	
	lcd_cls();
	lcd_putstring(0, 6, "* Menu *");
	lcd_putstring(1, 6, "MEM SCAN");
	lcd_putstring(3, 3, "(NEXT)   (OK)");
	
	while(key == 0)
	{
		lcd_putstring(2, 9, scan_str[menu_pos]);
		
		//Set light by changing PWM duty cycle
        if(tuning <= -1)
		{
		    if(menu_pos < 1)
		    {
				menu_pos++;
			}	
			else
			{
				menu_pos = 0;
				
			}	
			lcd_putstring(2, 9, scan_str[menu_pos]);
			tuning = 0;
		}

		if(tuning >= 1)  
		{    
		    if(menu_pos > 0)
		    {
				menu_pos--;
			}
			else
			{
				menu_pos =  1;
			}		
			
			lcd_putstring(2, 9, scan_str[menu_pos]);
			tuning = 0;
		}			    
		key = get_keys();
	}	
	
	while(get_keys());
	
	lcd_putstring(2, 9, "   ");
	
	switch(key)
	{
	    case 1: break; //Go to next item or return
	            return -1;
	            
	    case 2: x_vfo = scan_memories(); //Scan memories, return selected VFO number
	            ftmp = load_frequency(x_vfo);
			    if(is_mem_freq_ok(ftmp))
			    {
					cur_vfo = x_vfo;
					f_vfo[cur_vfo] = ftmp;
				    set_frequency1(ftmp);
				    show_frequency(ftmp);
			    }	
			    else
			    {
					f_vfo[cur_vfo] = f;
				    set_frequency1(f);
				    show_frequency(f);
			    }	
			    show_vfo(cur_vfo, 0);
			    break;
		case 129: return -1; 	    
	}   
	
	
	///////////////
	//   LO LSB  //
	///////////////
	menu_pos = 0;
	key = 0;
	
	lcd_cls();
	lcd_putstring(0, 6, "* Menu *");
	lcd_putstring(1, 6, "LO LSB");
	lcd_putstring(3, 3, "(NEXT)   (OK)");
	lcd_putnumber(2, 0, fbfo[0], -1, -1, 'l', 0);    			
	set_frequency2(fbfo[0]);
	
	while(!key)
	{
	    if(tuning <= -1)
		{
			fbfo[0] += tuningstep[cur_tstep];
		    set_frequency2(fbfo[0]);
		    lcd_putnumber(2, 0, fbfo[0], -1, -1, 'l', 0);    			
		    tuning = 0;
		}

		if(tuning >= 1)  
		{   
			fbfo[0] -= tuningstep[cur_tstep];
		    set_frequency2(fbfo[0]);
		    lcd_putnumber(2, 0, fbfo[0], -1, -1, 'l', 0);    			
		    tuning = 0;
		}		
		key = get_keys();
	}	
	
	while(get_keys());
	
	if(key == 129)
	{
		return -1;
	}	
	
	///////////////
	//   LO USB  //
	///////////////
	menu_pos = 0;
	key = 0;
	
	lcd_cls();
	lcd_putstring(0, 6, "* Menu *");
	lcd_putstring(1, 6, "LO USB");
	lcd_putstring(3, 3, "(NEXT)   (OK)");
    lcd_putnumber(2, 0, fbfo[1], -1, -1, 'l', 0);    			
    set_frequency2(fbfo[1]);
    
	while(!key)
	{
	    if(tuning <= -1)
		{
			fbfo[1] += tuningstep[cur_tstep];
		    set_frequency2(fbfo[1]);
		    lcd_putnumber(2, 0, fbfo[1], -1, -1, 'l', 0);    			
		    tuning = 0;
		}

		if(tuning >= 1)  
		{   
			fbfo[1] -= tuningstep[cur_tstep];
		    set_frequency2(fbfo[1]);
		    lcd_putnumber(2, 0, fbfo[1], -1, -1, 'l', 0);    			
		    tuning = 0;
		}		
		key = get_keys();
	}	
	
	while(get_keys());
		
	set_frequency2(fbfo[sideband]); //Restore correct sideband
	
	return 0;
}

  /////////////
 //  A G C  //
/////////////
void set_agc(int agcset)
{
	if(agcset)
	{
		DDRD |= (1 << 3);
		PORTD &= ~(1 << 3);  
    }
    else
    {
		DDRD &= ~(1 << 3);  //Switch PIN to tri-state mode
    }   
}
    
int main()
{
	
	//ADC
	int adcval, adcvalold = 0;	
	int adccnt = 0;
	
	//Saving current VFO etc.
	long runsecondsold0 = 0;
	long last_freq = 0;
	
	//Timer counter compare
	long runsecondsold1 = 0;
		 
	//Voltage check
	long runsecondsold2 = 0;
	double volts0;
	int volts1, volts1_old = 0;
	
	//Meter clear
	long runsecondsold4 = 0;
	
	//TX/RX detect
    int tx = 0;
    int tx_old = 0;
    
    //PA Temp
    int cur_sensor = 0;
    long runsecondsold3 = 0;
    
    //Split
    long f_split = 0;
    
    //Returning from menu
    long ret_value = 0;
    
    PORTA = 0x08; // Pullup for key reading
    PORTB = 0x06; //Tuning optical encoder PD1, PD2 (INT2)
   
    //OUTPUT PORT C
    DDRC = 0x0F;       //LCD D0:D3
    DDS2_DDR |=  0xF0; //DD2 FSYNC, SDATA, SCLK
    
    //OUTPUT PORT D	
	DDS1_DDR = 0x07; //DDS1 FSYNC, SDATA, SCLK
    DDRD |= 0xF0;    //LCD RS,RW, E, BACKLIGHT
        
    //Switch AGC fast/slow
    DDRD |= (1 << 3);
        
    _delay_ms(100);       

    //Interrupt definitions for rotary encoder attached to PD2 and PD3
    EIMSK = (1 << INT2);; //Activate INT2 only
	EICRA = (1 << ISC20);   // Trigger INT2 on pin change
	
	// Timer 1 as counter for seconds
    TCCR1A = 0;             // normal mode, no PWM
    TCCR1B = (1 << CS12) | (1<<WGM12);   // Prescaler = /256 based on system clock 8MHz => 31250 inc per s
                                         // and enable reset of counter register
	OCR1AH = (31250 >> 8);
    OCR1AL = (31250 & 0x00FF);
	TIMSK1 |= (1<<OCIE1A);
	
	// Timer 2 PWM for display light
    TCCR2A |= (1<<COM2A1) | (1<<COM2A0) | (1<<WGM21) | (1<<WGM20);
    TCCR2B |= (1<<CS20); // No prescale
            				
	//DDS2 init => Set AD9835 to sleepmode
    DDS2_PORT &= ~(DDS2_FSNYC); 
    spi2_send_word(0xF800);
    DDS2_PORT |= DDS2_FSNYC; 
	set_frequency2(fbfo[sideband]);
    DDS2_PORT &= ~(DDS2_FSNYC); 
    //AD9835 wake up from sleep
    spi2_send_word(0xC000);	
    DDS2_PORT &= ~(DDS2_FSNYC); 
    
    _delay_ms(100);
             
    //Load last frequency used if possile         
    cur_vfo = load_last_vfo();
    if(cur_vfo < 0 || cur_vfo > 9)
    {
		cur_vfo = 0;
	}
	f_vfo[cur_vfo] = load_frequency(cur_vfo);
	if(!is_mem_freq_ok(f_vfo[cur_vfo]))
	{
		f_vfo[cur_vfo] = 7123000;
	}	
	
	//Set VFO		
	set_frequency1(f_vfo[cur_vfo]);
	_delay_ms(10);
	set_frequency1(f_vfo[cur_vfo]);
			  	 
    //Display TRX data
    lcd_init();
    lcd_cls();
    defcustomcharacters();
    
    //Show voltage
    volts0 = (double) get_adc(0) * 5 / 1024 * 3 * 10;
    volts1 = (int) volts0;  	
    volts1_old = volts1;
    
    //Display data
    show_data(f_vfo[cur_vfo], cur_tstep, sideband, get_temp(cur_sensor) / 10, cur_sensor, cur_vfo, volts1, agc);
            
    //Set disp light
    light = eeprom_read_byte((uint8_t*)128); 
    if(light < 20 || light > 100)
    {
		light = 70;
	}	
    lcd_setbacklight(light);
    
    //AGC settings
    agc = eeprom_read_byte((uint8_t*)129); 
    if(agc < 0 || agc > 1)
    {
		agc = 0;
	}	
    set_agc(agc);
    show_agc(agc, 0);
        
    sei();
      	
    split  = 0;  	
	for(;;) 
	{
		//Tune VFO frequency
        if(tuning <= -1)
		{
	   	    runsecondsold0 = runseconds;
		    f_vfo[cur_vfo] += tuningstep[cur_tstep];
		    set_frequency1(f_vfo[cur_vfo]);    		 
	        show_frequency(f_vfo[cur_vfo]);    		
	        tuning = 0;
		    
	        runsecondsold1 = runseconds;
		}

		if(tuning >= 1)  
		{   
			runsecondsold0 = runseconds;
		    f_vfo[cur_vfo] -= tuningstep[cur_tstep];
		    set_frequency1(f_vfo[cur_vfo]);    		 
	        show_frequency(f_vfo[cur_vfo]);    		
	        tuning = 0;
		    runsecondsold1 = runseconds;		
		}			    
		
		
		//Save current VFO all 120 secs if not idle
		if((runseconds > runsecondsold0 + 120) && (last_freq != f_vfo[cur_vfo]))
		{
			store_frequency(f_vfo[cur_vfo], cur_vfo);
	        store_last_vfo(cur_vfo);
			last_freq = f_vfo[cur_vfo];
			runsecondsold0 = runseconds;
			runsecondsold4 = runseconds;
			show_msg("SAVED. ");
		}	 
		
		//Reset tuning step after 3 seconds idle
		if((runseconds > runsecondsold1 + 3) && (cur_tstep != STDTSTEP))
		{
			 cur_tstep = STDTSTEP;
			 show_tuning_step(cur_tstep, 0);
			 runsecondsold1 = runseconds;
		}	 
		
		if(!tx)
		{
		    //Show S value, hardware integration		
		    adcval = get_adc(1) << 1;
		    s_meter(adcval);
		}    
		else
		{
		    //Show PWR value, software integration
		    adcval += (get_adc(2) >> 3);
		    adccnt++;
		    
		    if(adccnt > 4) //Take 4 values and display
		    {
		        if(adcval < adcvalold)
		        {
		            s_meter(-1);
		        }    
		        s_meter(adcval >> 2);
		        adcvalold = adcval;
		        adcval = 0;
		        adccnt = 0;
		     }   
		}    
							
		//Show voltage, update each 10 secs
        if(runseconds > runsecondsold2 + 10)
        {
			volts0 = (double) get_adc(0) * 5 / 1024 * 3 * 10;
		    volts1 = (int) volts0;
   		    if(volts1 != volts1_old)
		    {
    	        show_voltage(volts1);
	     		volts1_old = volts1;
		    }	
            lcd_set_batt_icon(volts1);
		    runsecondsold2 = runseconds;
		}   	
		
		//Show PA temp every 5 secs
		if(runseconds > runsecondsold3 + 5)
		{
		    show_pa_temp(get_temp(cur_sensor) / 10, cur_sensor);
		    runsecondsold3 = runseconds;
		    if(cur_sensor)
		    {
				cur_sensor = 0;
			}
			else	
			{
				cur_sensor = 1;
			}
		}    
		
		//Clear meter every 3 second
        if(runseconds > runsecondsold4 + 3)
        {
			show_msg("");
			runsecondsold4 = runseconds;
		}   	

		
		if(PINA & (1 << 5)) //TX, cause PINA5 is 1
		{
			if(!tx)
			{
				lcd_set_icon(0x0E, 0x10);
				s_meter(-1);    //Clear meter
				tx = 1;
			}	
		}
		else
		{
			if(tx)
			{
			    lcd_set_icon(0x0E, 0);
			    s_meter(-1);    //Clear meter
			    tx = 0;
			}    
		}	
		
		//Switch VFO freq if SPLIT
		if(tx != tx_old)
		{
			if(split && tx)
			{
				set_frequency1(f_split);
				show_frequency(f_split);
				tx_old = tx;
			}	
			
			if(split && !tx)
			{
				set_frequency1(f_vfo[cur_vfo]);
				show_frequency(f_vfo[cur_vfo]);
				tx_old = tx;
			}	
		}	
		
		if(get_keys() == 1) //Menu call
		{
			s_meter(-1);    //Clear meter
			
			ret_value = menu(f_vfo[cur_vfo], f_split);
			
			//Is return value a freq for split mode?
			if(is_mem_freq_ok(ret_value))
			{
				lcd_cls();
				f_split = ret_value;
				show_txfrequency(f_split);
			}
			else
			{
				lcd_cls();
				split = 0;
				show_txfrequency(-1);	
			}	
				
			runsecondsold1 = runseconds;
			show_data(f_vfo[cur_vfo], cur_tstep, sideband, get_temp(cur_sensor) / 10, cur_sensor, cur_vfo, volts1, agc);
		}	
	}

    return 0;
}
