
# define F_CPU 8000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <math.h>
#include <avr/interrupt.h>


const unsigned int maxUnits=2;//Maximum number of units used by this base unit.
unsigned int  dataArray[maxUnits+5][21];
unsigned int  unitArray[100];

volatile unsigned int currentLineNum=1, cursor=0x00,currentLine1=1, inCommandMenu=0, currentIndex=1;
unsigned int timerCount=0,allDeleted=1,unitsUsed=0;

void COM_WRITE(int ,unsigned int , unsigned int , unsigned int , unsigned int , unsigned int , unsigned int);
void COM_STATUS(unsigned int&);
void COM_READ(unsigned int,  unsigned int&);
void COM_FLUSH_TX();
void COM_FLUSH_RX();
void COM_RX_MODE();
void COM_INIT();
void COM_READ_PAYLOAD(unsigned int&);
void COM_WRITE_PAYLOAD(unsigned int);
void COM_TX(unsigned int);
void COM_READ_LARGE(int numBytes, unsigned int regValue, unsigned int&,unsigned int&,unsigned int&,unsigned int&,unsigned int&);

void lcd_char(char);
void lcd_command(char);
void lcd_init();

void keypad_init();
void keypad_get(unsigned int&);
void numsel(int);


void BU_CURSOR_DOWN();
void BU_CURSOR_UP();
void BU_PRINT_LINE(unsigned int);
void BU_SET_LINE(unsigned int);
void BU_SET_LINE_VAR(unsigned int);
void BU_CURSOR_DOWN_CMD();
void BU_CURSOR_UP_CMD();
void BU_CMD_MENU(unsigned int);
void FOR_DELAY();
void BU_INC_POSINLINE(unsigned int);
void BU_CLEAR_LINE(unsigned int);

void SPI_init(){
	// Set MOSI,SS and SCK, CE output
	DDRB= 0x2E;
	// Enable SPI, Master, set clock rate fck/4
	SPCR = (1<<SPE)|(1<<MSTR);
	PORTB |= (1 << PORTB2); 	//set SS[CSN] high
	PORTB &= ~(1<< PORTB1);//set CE to be 0;
}

void COM_INIT(){
	unsigned int temp;
	//Write to  CONFIG reg:mask all interrupts, Power up , CRC 1 byte, TX mode
	//Write to  reg, 0x3A: pwr_up=1,
	COM_READ(0x00,temp);
	COM_WRITE(1,0x00,0x7A,0,0,0,0);
	COM_READ(0x00,temp);
	COM_WRITE(1,0x04,0xFF,0,0,0,0);//retransmit delay to max, number of attempts to max
	COM_WRITE(5,0x0A,0xBB,0x89,0x87,0x86,0x85);//Write RX addr
	COM_WRITE(5,0x10,0xFF,0x12,0x13,0x14,0x15);//Write TX addr
	COM_WRITE(1,0x06,0x26,0,0,0,0);//Change message rate
	COM_WRITE(1,0x05,0x10,0,0,0,0);//Change RF channel	
}

void COM_STATUS(unsigned int &status){
	//send CMD word
	PORTB &= ~(1 << PORTB2);//Start transmission, set CSN low
	SPDR =0xFF;
	while(!(SPSR & (1<<SPIF)));// Wait for transmission to complete
	PORTB |= (1 << PORTB2); //set CSN high
	//recieve data
	status=SPDR;
}

void COM_READ(unsigned int regValue,  unsigned int &dataRead){
	PORTB &= ~(1 << PORTB2);//set CSN low
	SPDR =regValue;
	while(!(SPSR & (1<<SPIF)));
	SPDR=0xFE;//junk data
	while(!(SPSR & (1<<SPIF)));
	dataRead=SPDR;
	PORTB |= (1 << PORTB2); //set CSN high
	};

void COM_READ_LARGE(int numBytes, unsigned int regValue, unsigned int &byte1,unsigned int &byte2,unsigned int &byte3,unsigned int &byte4,unsigned int &byte5){
	PORTB &= ~(1 << PORTB2);//set CSN low
	SPDR =regValue;
	while(!(SPSR & (1<<SPIF)));
	if(numBytes >=1){
		SPDR=0xFE;//junk data
		while(!(SPSR & (1<<SPIF)));
		byte1=SPDR;
	}
	if(numBytes >=2){
		SPDR=0xFE;//junk data
		while(!(SPSR & (1<<SPIF)));
		byte2=SPDR;
	}
	if(numBytes >=3){
		SPDR=0xFE;//junk data
		while(!(SPSR & (1<<SPIF)));
		byte3=SPDR;
	}
	if(numBytes >=4){
		SPDR=0xFE;//junk data
		while(!(SPSR & (1<<SPIF)));
		byte4=SPDR;
	}
	if(numBytes == 5){
		SPDR=0xFE;//junk data
		while(!(SPSR & (1<<SPIF)));
		byte5=SPDR;
	}
	PORTB |= (1 << PORTB2); //set CSN high
}	

void COM_WRITE(int numBytes,unsigned int regValue, unsigned int byte1, unsigned int byte2, unsigned int byte3, unsigned int byte4, unsigned int byte5){
	regValue=regValue | 0x20;
	PORTB &= ~(1 << PORTB2);
	SPDR =regValue;
	while(!(SPSR & (1<<SPIF)));
	//Transmit bytes, LSB to MSB
	if(numBytes >=1){
		SPDR =byte1;
		while(!(SPSR & (1<<SPIF)));
	}
	if(numBytes >=2){
		SPDR =byte2;
		while(!(SPSR & (1<<SPIF)));
	}
	if(numBytes >=3){
		SPDR =byte3;
		while(!(SPSR & (1<<SPIF)));
	}
	if(numBytes >=4){
		SPDR =byte4;
		while(!(SPSR & (1<<SPIF)));
	}
	if(numBytes == 5){
		SPDR =byte5;
		while(!(SPSR & (1<<SPIF)));
	}
	PORTB |= (1 << PORTB2);
	
}

void COM_WRITE_PAYLOAD(unsigned int payload){//MUST BE IN TX MODE, 8 bit message.
	PORTB &= ~(1 << PORTB2);//set CSN low
	SPDR =0xA0;
	while(!(SPSR & (1<<SPIF)));
		//Transmit bytes, LSB to MSB
		SPDR =payload;
		while(!(SPSR & (1<<SPIF)));
		PORTB |= (1 << PORTB2); //set CSN high
	
}

void COM_READ_PAYLOAD(unsigned int &payload){//MUST be in RX mode. Read payload from RX FIFO
	unsigned int status,temp;
	PORTB &= ~(1<< PORTB1);//set CE to be 0;
	PORTB &= ~(1 << PORTB2);//set CSN low
	SPDR =0x61;
	while(!(SPSR & (1<<SPIF)));
	payload=SPDR;
	PORTB |= (1 << PORTB2); //set CSN high
	
	//Reset RX_DR Pin via RX_DR bit in status
	temp= status | 0x40;//sets RX_DR to 1
	COM_WRITE(1,0x07,temp,0,0,0,0);
	
	//set CE high to re-enter active RX mode
	PORTB |= (1 << PORTB1); 	//set CE to be 1	
}

void COM_FLUSH_TX(){
	PORTB &= ~(1 << PORTB2);//set CSN low
	SPDR =0xE1;
	while(!(SPSR & (1<<SPIF)));
	PORTB |= (1 << PORTB2); //set CSN high
}

void COM_FLUSH_RX(){
	PORTB &= ~(1 << PORTB2);//set CSN low
	SPDR =0xE2;
	while(!(SPSR & (1<<SPIF)));
	PORTB |= (1 << PORTB2); //set CSN high
}

void COM_TX(unsigned int payload){//8 bit message
	unsigned int temp,status,wait=1,i;
	PORTB &= ~(1<< PORTB1);//SET CE to be 0
	
	//Set CONGIF bit PRIM_RX to low.
	COM_READ(0x00, temp);
	temp &= 0xFE;
	COM_WRITE(1,0x00,temp,0,0,0,0);
	//Write payload
	COM_WRITE_PAYLOAD(payload);
	//Step3:Pulse CE for at least 10 seconds
	PORTB |= (1<< PORTB1);//set CE to be 1;
	for(i=0;i<20;i++){
		_delay_us(1);
	}
	PORTB &= ~(1<< PORTB1);//set CE to be 0;	
	//Check for ACK (TX_DS=1 is successful) in status register, If not received( TX_DS=0 and MAX_RT =1) flush payload and reset TX_DS and MAX_RT by writing 1.
	do 
	{
		COM_STATUS(status);//Read Status Register
		temp=status & 0x30;
		if(temp == 0x20){//TX_DS=1,Clear TX_DS.
			wait=currentLineNum;
			wait=0;
			temp= status | 0x20;//sets TX_DS=1 to clear
			COM_WRITE(1,0x07,temp,0,0,0,0);
		}
		else if(temp == 0x10){//MAX_RT=1 =(
			//Ideally, input something to deal with this error. Instead, Flush payload and clear status
			wait=currentLineNum;
			wait=0;
			COM_FLUSH_TX();
			temp= status | 0x10;//sets MAX_RT to 1
			COM_WRITE(1,0x07,temp,0,0,0,0);
		}
	} while (wait==1);
}

void COM_RX_MODE(){//Enters RX mode, enable 
	unsigned int temp;
	//Set payload length in RX_PW_PX register
	COM_WRITE(1,0x11,0x01,0,0,0,0);//Enables datapipe0, sets length to 8.
	COM_WRITE(1,0x02,0x01,0,0,0,0);
	//Set CONGIF bit PRIM_RX to 1. Set RX_IRQ enabled.
	COM_READ(0x00, temp);
	temp |= 0x01;
	temp &= 0x3F;
	COM_WRITE(1,0x00,temp,0,0,0,0);
	PORTB |= (1<< PORTB1);//set CE to be 1 This enters active RX mode.
}



void lcd_init()
{
	DDRC = 0x3F; 
	lcd_command(0x33); //Initialize LCD Driver
	lcd_command(0x32); //Four bit mode
	lcd_command(0x2C);//(0x2C); //2 Line Mode
	lcd_command(0x0C); //Display On, Cursor Off, Blink Off. Change to 0x0F if blink/cursor is desired
	lcd_command(0x01); //Clear Screen, Cursor Home
	lcd_command(0x06); //cursor move right, no screen shift.
	
	
}
void lcd_command(char cmd)
{
	unsigned int inCMD=0;
	if(inCommandMenu == 1){
		inCMD=1;
	}
	inCommandMenu=1;
	
	char temp = cmd;
	PORTC=0;//RS =0, E=0
	_delay_ms(2);
	cmd = ( (cmd & 0xF0) >> 4) | 0x20; //Write Upper Nibble (RS=0) E --> 1
	PORTC = cmd;
	_delay_ms(2);
	cmd ^= 0x20; //E --> 0
	PORTC = cmd;
	_delay_ms(2);
	cmd = temp;
	cmd = ( (cmd & 0x0F) ) | 0x20; //Write Lower Nibble (RS=0) E --> 1
	PORTC = cmd;
	_delay_ms(2);
	cmd ^= 0x20; //E --> 0
	PORTC = cmd;
	_delay_ms(2);
	if(inCMD==0){
		inCommandMenu=0;
	}
}
void lcd_char(char data)
{
	unsigned int inCMD=0;
	if(inCommandMenu == 1){
		inCMD=1;
	}
	inCommandMenu=1;
	char temp = data;
	PORTC = 0x10;//RS =1
	_delay_ms(1);

	data = ( (data & 0xF0) >> 4) | 0x30; //Write Upper Nibble (RS=1) E --> 1
	PORTC = data;
	_delay_ms(1);

	data ^= 0x20; // E --> 0
	PORTC = data;
	_delay_ms(1);

	data = temp;
	data = ( (data & 0x0F) ) | 0x30; //Write Lower Nibble (RS=1) E --> 1
	PORTC = data;
	_delay_ms(1);

	data ^= 0x20; //E --> 0
	PORTC = data;
	_delay_ms(1);
	if(inCMD==0){
		inCommandMenu=0;
	}

}

void keypad_init(){
	DDRD = 0x07;//D3:0 as outputs for 3x4 keypad;
	
}

void keypad_get(unsigned int &key){
	unsigned int done=0;
	unsigned int inCMD=0;
	if(inCommandMenu == 1){
		inCMD=1;
	}
	inCommandMenu=1;
	//check for up,down, sel, back
	//Check for 1,4,7,9
	if(done==0){
		_delay_us(3000);
		PORTD |= (1 << PORTD0);
		_delay_us(1000);
		
		if((PINB & 0x40)==0x40){//up, B6. A on 1x4
			key=0x0D;
			done=1;
		}
		else if((PINB & 0x80)==0x80){//down, B7. B on 1x4
			key=0x0E;
			done=1;
		}
		else if((PIND & 0x80)==0x80){//sel, D7. C on 1x4
			key=0x0F;
			done=1;
		}
		else if((PINB & 0x01)==0x01){//back , B0. D on 1x4
			key=0xF0;
			done=1;
		}
		//END OF 1x4
		
		if((PIND & 0x40)==0x40){//(PIND & (1<<PD6)){//12
			done=1;
			key=0x01;
		}
		else if((PIND & 0x20)==0x20){//(PIND & (1<<PD5)){//9
			done=1;
			key=0x02;
		}
		else if((PIND & 0x10)==0x10){//(PIND & (1<<PD4)){//6
			done=1;
			key=0x03;
		}
		else if((PIND & 0x08)==0x08){//(PIND & (1<<PD3)){//3
			done=1;
			key=0x00;
		}
		_delay_us(1000);
		PORTD &= ~(1 << PORTD0);
		_delay_us(3000);
	}//END done==0 for  D0
	//Check for 2,5,8,11
	if(done == 0){
			_delay_us(3000);
			PORTD |= (1 << PORTD1);
			_delay_us(1000);
			if((PIND & 0x40)==0x40){//(PIND & (1<<PD6)){//11
				done=1;
				key=0x04;
			}
			else if((PIND & 0x20)==0x20){//(PIND & (1<<PD5)){//8
				done=1;
				key=0x05;
			}
			else if((PIND & 0x10)==0x10){//(PIND & (1<<PD4)){//5
				done=1;
				key=0x06;
			}
			else if((PIND & 0x08)==0x08){//(PIND & (1<<PD3)){//2
				done=1;
				key=0x00;
			}
			_delay_us(1000);
			PORTD &= ~(1 << PORTD1);
			_delay_us(3000);
	}//end done==0 for D1

	//Check for 3,6,9,12
	if(done == 0){
			_delay_us(3000);
			PORTD |= (1 << PORTD2);
			_delay_us(1000);
			if((PIND & 0x40)==0x40){//(PIND & (1<<PD6)){//12
				done=1;
				key=0x07;
			}
			else if((PIND & 0x20)==0x20){//(PIND & (1<<PD5)){//9
				done=1;
				key=0x08;
			}
			else if((PIND & 0x10)==0x10){//(PIND & (1<<PD4)){//6
				done=1;
				key=0x09;
			}
			else if((PIND & 0x08)==0x08){//(PIND & (1<<PD3)){//3
				done=1;
				key=0x00;
			}
			_delay_us(1000);
			PORTD &= ~(1 << PORTD2);
			_delay_us(3000);
	}//end done==0 for D2
	
	if(done==0){//null state
		key=0xFF;
	}
	if(inCMD==0){
		inCommandMenu=0;
	}
}
	
void BU_ENTER_NEW(unsigned int line){
	unsigned int inCMD=0;
	if(inCommandMenu == 1){
		inCMD=1;
	}
	inCommandMenu=1;
	lcd_char(' ');
	BU_INC_POSINLINE(line);
	numsel(dataArray[line][1]);
	numsel(dataArray[line][2]);
	lcd_char('.');
	lcd_char('E');
	lcd_char('n');
	lcd_char('t');
	lcd_char('e');
	lcd_char('r');
	lcd_char(' ');
	lcd_char('n');
	lcd_char('e');
	lcd_char('w');
	lcd_char(' ');
	if(inCMD==0){
		inCommandMenu=0;
	}
}	
	
void BU_PRINT_LINE(unsigned int index){
	/*
	//Turn off blink. Offset by 1. Print line. Turn on Blink.
	//
	*/
	unsigned int count=3,temp=1,inCMD=0;
	if(inCommandMenu == 1){
		inCMD=1;
	}
	inCommandMenu=1;
	lcd_command(0x0C);//blink off
	if(dataArray[index][15]!=0){
		lcd_char(' ');
		numsel(dataArray[index][1]);
		numsel(dataArray[index][2]);
	do//print line
	{
		temp=dataArray[index][count];
		count++;
		if((temp !=0xF0)){
			if(temp<16){
				numsel(temp);
			}
			else{
				lcd_char((char)temp);
			}
		}
		else
		{
			count--;
		}
	} while ((temp != 0xF0) && !(count >=11));
		while(count<12){
			count++;
			lcd_char(' ');
		}
		numsel(dataArray[index][19]);numsel(dataArray[index][20]);lcd_char(' ');numsel(dataArray[index][16]);numsel(dataArray[index][17]);lcd_char(':');numsel(dataArray[index][18]);numsel(0);
	}
	else{//If at last line, and not initlaized. Check to make sure last line not already passed.Ensure units are avaliable
		if((!(dataArray[index-1][15]==0) || (allDeleted==1)) && (unitsUsed <maxUnits)){
			allDeleted=0;
			BU_ENTER_NEW(index);
		}
		}
		
	if(inCMD==0){
		inCommandMenu=0;
	}
	lcd_command(0x0F);//blink on
}

void BU_CLEAR_LINE(unsigned int line){
	unsigned int inCMD=0;
	if(inCommandMenu == 1){
		inCMD=1;
	}
	inCommandMenu=1;
	unsigned int temp=0;
	BU_SET_LINE_VAR(line);
	for(temp=0;temp<20;temp++){
		lcd_char(' ');
	}
	if(inCMD==0){
		inCommandMenu=0;
	}
}

void BU_INC_POSINLINE(unsigned int line){
	unsigned int temp2,temp3;
	unsigned int inCMD=0;
	if(inCommandMenu == 1){
		inCMD=1;
	}
	inCommandMenu=1;
	temp2=dataArray[line-1][1];//ten
	temp3=dataArray[line-1][2];//one
	
	if(temp3==9){
		temp3=0;
		if(temp2==9){
			temp2=0;
		}
		else{
			temp2++;
		}
	}
	else{
		temp3++;
	}
	dataArray[line][2]=(temp3);
	dataArray[line][1]=(temp2);
	if(inCMD==0){
		inCommandMenu=0;
	}
	
}
void BU_SET_LINE(unsigned int line){
	/*
	//Entered when user selects line to add new unit to.
	//Blink on, enter desired inputs, enter. 
	//Should have capability to backspace
	//TODO:Implement optional alphabet select.
	*/
	unsigned int count=4;
	unsigned int key=0xFF,done=0,temp2,temp3,s1=0xFF,s2=0xFF,done2=0;
	unsigned int inCMD=0;
	if(inCommandMenu == 1){
		inCMD=1;
	}
	inCommandMenu = 1;
	lcd_command(0x0C);//blink off
	lcd_command(0x01);//Clear Screen
	lcd_char(' ');lcd_char('E');lcd_char('n');lcd_char('t');lcd_char('e');lcd_char('r');lcd_char(' ');lcd_char('I');lcd_char('D');lcd_char(':');//numsel(tb);numsel(ta);//last 2 debug
	BU_SET_LINE_VAR(2);
	dataArray[line][0]=' ';
	BU_INC_POSINLINE(line);
	dataArray[line][3]='.';
	lcd_char(' ');
	lcd_command(0x0F);//turn on blink
	do //Poll keypad for input, if valid input, Put input into array. If array full, stop.
	{
		do 
		{
			keypad_get(key);
			_delay_ms(250);
			if((key !=0xF0) && (key != 0x0F)&&(key != 0x0D) && (key != 0xFF) && (count<11)){//if select,back, up, down not pressed, take key as tentative input.
				done=1;
			}
			else if(key==0x0D){//if back pressed, decrement count
				if(count>=5){//if count within range if backspaced, do stuff. Do nothing if invalid.
					count--;
					lcd_command(0x0C);//blink 0ff
					lcd_command(0x80 | 0x40| (count-3));
					lcd_char(' ');
					lcd_command(0x80 | 0x40| (count-3));
					lcd_command(0x0F);//blink on
				}
			}
		} while (done==0);//wait until key pressed and not backspace operation. If select pressed, exit
		if((count <11) && (key != 0x0E)&&(key != 0x0D)&&(key != 0x0F)&&(key != 0xF0) && (key !=0xFF)){//select not pressed, input key pressed into array and print
			done=0;
			dataArray[line][count]=key;
			numsel((int)key);
			count++;
		}
		else if((key==0x0E) && (count>4)){//select pressed, exit.v set unitArray, unitNumber
			dataArray[line][count]=0xF0;//key set to zero, used to detect end of inputted ID
			done2=1;
		}
		else{
			done=0;
			key=0xFF;
		}
	} while (done2==0);
	lcd_command(0x01);
	lcd_char('E');lcd_char('n');lcd_char('t');lcd_char('e');lcd_char('r');lcd_char(' ');lcd_char('P');lcd_char('a');lcd_char('r');lcd_char('t');lcd_char('y');
	BU_SET_LINE_VAR(2);
	lcd_char('S');lcd_char('i');lcd_char('z');lcd_char('e');lcd_char(':');
	done=0;
	done2=0;
	count=6;
	key=0xFF;
	do //Poll keypad for input, if valid input, Put input into array. If array full, stop.
	{
		do
		{		
			keypad_get(key);
			_delay_ms(250);
			if((key !=0xF0) && (key != 0x0F)&&(key != 0x0D) && (key != 0xFF)){//if select,back, up, down not pressed, take key as tentative input.
				done=1;
			}
			else if(key==0x0D){//if back pressed, decrement count
				if(count>=7){//if count within range for valid backspaced, do stuff. Do nothing if invalid.
					count--;
					if(count==7){
						s2=0xFF;
					}
					lcd_command(0x0C);//blink 0ff
					lcd_command(0x80 | 0x40| count-1);
					lcd_char(' ');
					lcd_command(0x80 | 0x40| count-1);
					lcd_command(0x0F);//blink on
				}
			}
		} while (done==0);//wait until key pressed
		if((count <8) && (key != 0x0E)&&(key != 0x0D)&&(key != 0x0F)&&(key != 0xF0) && (key !=0xFF)){//select not pressed, input key pressed and print
			done=0;
			if(count==6){				
				if(key !=0){			
				s1=key;
				numsel((int)key);
				count++;	
				}
			}
			else if(count==7){
				s2=key;
				numsel((int)key);
				count++;
			}
			
		}	
		else if((key==0x0E) && (count > 6)){//select pressed, exit.v set unitArray, unitNumber
			if(s2==0xFF){//If s2 not used, set s1 as ones input	
				dataArray[line][19]=0;
				dataArray[line][20]=s1;
			}
			else{//Else set s1 as tens, s2 as ones
				dataArray[line][19]=s1;
				dataArray[line][20]=s2;
			}
			done2=1;
			temp3=0;
			count=1;
			do //Scan unitArray for available unit. Set as unit number when unused unit found. Transmit place in line to  unit.
			{
				if(unitArray[count]==0){
					unitArray[count]=1;
					dataArray[line][15]=count;
					unitsUsed++;
					temp2=dataArray[line][1]*10+dataArray[line][2];
					COM_WRITE(5,0x10,count,0x12,0x13,0x14,0x15);//Set TX write addr based on unit number
					COM_TX(temp2);//send position in line
					COM_FLUSH_TX();
					COM_TX(temp2);//send position in line
					COM_FLUSH_TX();
					temp3=1;
					
				}
				count++;
			} while (temp3==0);
		}
		
	} while (done2==0);
	if(inCMD==0){
		inCommandMenu=0;
	}
}
void BU_SET_LINE_VAR(unsigned int line){
	unsigned int inCMD=0;
	if(inCommandMenu == 1){
		inCMD=1;
	}
	inCommandMenu=1;
	if(line==1){
		cursor=0x00;
		currentLineNum=1;
	}
	else if (line==2){
		cursor=0x40;
		currentLineNum=2;
	}
	else if(line==3){
		cursor=0x14;
		currentLineNum=3;
	}
	else if(line==4){
		cursor=0x54;
		currentLineNum=4;
	}
	lcd_command(0x80 | cursor);
	if(inCMD==0){
		inCommandMenu=0;
	}
	}
		
void BU_CURSOR_UP(){
	/*
	//Shift cursor up by one. If at top of display, shift all lines down by 1. If at top of array, no action
	//If at top, no action
	*/
	unsigned int inCMD=0;
	if(inCommandMenu == 1){
		inCMD=1;
	}
	inCommandMenu=1;
	lcd_command(0x0C);//blink off
	if(cursor==0x00){//at top, shift all lines up
		if(currentLine1!=1){//if not at top
			currentLine1 -= 4;
			currentIndex -= 4;
			BU_CLEAR_LINE(1);
			BU_CLEAR_LINE(2);
			BU_CLEAR_LINE(3);
			BU_CLEAR_LINE(4);
			BU_PRINT_LINE(currentLine1);
			BU_SET_LINE_VAR(2);
			BU_PRINT_LINE(currentLine1+1);
			BU_SET_LINE_VAR(3);
			BU_PRINT_LINE(currentLine1+2);
			BU_SET_LINE_VAR(4);
			BU_PRINT_LINE(currentLine1+3);
			BU_SET_LINE_VAR(1);
		}
	}
	else if (cursor == 0x40){
		cursor=0x00;
		currentLineNum=1;
		currentIndex--;
	}
	else if(cursor == 0x14){
		cursor=0x40;
		currentLineNum=2;
		currentIndex--;
	}
	else if(cursor == 0x54){
		cursor=0x14;
		currentLineNum=3;
		currentIndex--;
	}
	lcd_command(0x80 | cursor);//shift cursor to new line, or back to start of line1
	lcd_command(0x0F);//blink on
	if(inCMD==0){
		inCommandMenu=0;
	}
}

void BU_CURSOR_DOWN(){
	/*
	//Shift cursor down by one. If at bottom of display, shift all lines up by 1. If at bot of array, no action
	//
	*/
	unsigned int temp;
	unsigned int inCMD=0;
	if(inCommandMenu == 1){
		inCMD=1;
	}
	inCommandMenu=1;
	temp=dataArray[currentIndex+1][15];
	lcd_command(0x0C);//blink off
	if(cursor==0x00){//at top, shift all lines up
		if((dataArray[currentIndex][15]!=0) && (currentIndex < maxUnits)	 ){
		cursor=0x40;
		currentLineNum=2;
		currentIndex++;
		}
	}
	else if (cursor == 0x40){
			if((dataArray[currentIndex][15]!=0)&& (currentIndex < maxUnits)	){
				cursor=0x14;
				currentLineNum=3;
				currentIndex++;
			}
		}
	else if(cursor == 0x14){
			if((dataArray[currentIndex][15]!=0)&& (currentIndex < maxUnits)	){
				cursor=0x54;
				currentLineNum=4;
				currentIndex++;
			}
		}
	else if(cursor == 0x54){
		temp=dataArray[currentLine1+4][15];
			if(((temp !=0) || (dataArray[currentLine1+3][15] != 0))&& (currentIndex < maxUnits)){
				currentLine1+=4;
				currentIndex++;
				BU_CLEAR_LINE(1);
				BU_CLEAR_LINE(2);
				BU_CLEAR_LINE(3);
				BU_CLEAR_LINE(4);
			BU_SET_LINE_VAR(1);
			BU_PRINT_LINE(currentLine1);
			BU_SET_LINE_VAR(2);
			BU_PRINT_LINE(currentLine1+1);
			BU_SET_LINE_VAR(3);
			BU_PRINT_LINE(currentLine1+2);
			BU_SET_LINE_VAR(4);
			BU_PRINT_LINE(currentLine1+3);
			BU_SET_LINE_VAR(1);	
			}
	}
	lcd_command(0x80 | cursor);//shift cursor to new line, or back to start of line1
	lcd_command(0x0F);//blink on
	if(inCMD==0){
		inCommandMenu=0;
	}
	
}

void timerInit(){
	TCCR1B = TCCR1B | (1 << WGM12);//set bit 3(WGM12) for Timer1 Ctrl Reg B to 1, enabling CTC mode
	TCCR1B = TCCR1B | (1 << CS10);//set prescaler divider of timer1 to 64 if both
	TCCR1B = TCCR1B | (1 << CS12);//set prescale divider of timer 1 to 256 if just this
	OCR1A = 39062;//30;   //set counter endpoint
}

void BU_DELETE_LINE(unsigned int index){
	/*
	//Delete line. Shift data up until at end of used data.
	//
	*/
	unsigned int temp,count;//oldLine;
	unsigned int inCMD=0;
	if(inCommandMenu == 1){
		inCMD=1;
	}
	inCommandMenu = 1;
	temp=dataArray[index+1][15];
	unitArray[dataArray[index][15]]=0;
	unitsUsed--;
	if((dataArray[index+1][15]==0) && currentIndex==1){
		allDeleted=1;
	}
	if(temp==0){
		for(count=0;count<=20;count++){//delete last line
			dataArray[index][count]=0;
		}
	}
	else{
		while(temp !=0){//shift dataArray up by 1
			for(count=0;count<=19;count++){
				if(count==1){//change position in queue by -1. Since dataArray[index+1] is being shifted down to dataArray[index], leave number untouched.
					count+=2;
				}
				else{//swap lower array into higher
					dataArray[index][count]=dataArray[index+1][count];
				}
			}// END FOR(temp++) loop
			index++;
			temp=dataArray[index+1][15];
		}//END OF WHILE

		for(count=0;count<=20;count++){//delete last line
			dataArray[index][count]=0;
		}
	}//END OF ELSE
	if(inCMD==0){
		inCommandMenu=0;
	}
	
}

void FOR_DELAY(){
	unsigned int temp1=0;
	for(temp1=0; temp1<1000;temp1++){
		_delay_ms(1);
	}
}

void BU_CMD_MENU(unsigned int index){
	/*
	//Displays CMD menu for user navigation.
	//Upon exit, display first line.
	//Have option to select command, of back out of cmd menu.
	*/
	unsigned int tempLine,temp=0,temp1=0,temp2=0,temp3=0,count=4;
	inCommandMenu=1;
	tempLine=currentLineNum;
	lcd_command(0x01);//clear screen.
	temp = dataArray[index][15];
	if(temp !=0){
		//inCommandMenu=1;
		temp3=floor((temp)%10);//ones
		temp2=temp-temp3;//tens
		
		//Print available commands if unit already active, then return cursor to line 2.
		lcd_char('I');lcd_char('D');lcd_char(':');		
			do//print 4 to 14 of dataArray[index][#]
			{
				temp=dataArray[index][count];
				count++;
				if((temp !=0xF0)){
					if(temp<16){
						numsel(temp);
					}
					else{
						lcd_char((char)temp);
					}
				}
			} while ((temp != 0xF0) && !(count >=12));	
		
		lcd_char(' ');lcd_char('U');lcd_char('n');lcd_char('i');lcd_char('t');lcd_char(':');numsel(temp2);numsel(temp3);
		BU_SET_LINE_VAR(2);
		lcd_char(' ');lcd_char('1');lcd_char('.');lcd_char('P');lcd_char('a');lcd_char('g');lcd_char('e');
		BU_SET_LINE_VAR(3);
		lcd_char(' ');lcd_char('2');lcd_char('.');lcd_char('D');lcd_char('e');lcd_char('l');lcd_char('e');lcd_char('t');lcd_char('e');
		BU_SET_LINE_VAR(4);
		lcd_char(' ');lcd_char('3');lcd_char('.');lcd_char('C');lcd_char('a');lcd_char('l');lcd_char('l');lcd_char(' ');lcd_char('t');lcd_char('o');lcd_char(' ');lcd_char('d');lcd_char('e');lcd_char('s');lcd_char('k');
		temp=0;
		temp1=0xFF;
		BU_SET_LINE_VAR(2);
		while(temp==0){//acts as main loop of CMD function
			keypad_get(temp1);
			_delay_ms(250);
			if(temp1==0x0E){//select pressed, evaluate based on current cursor
					if(cursor == 0x40){//Page, TODO:add confirmation
						temp2=dataArray[index][15];
						COM_WRITE(5,0x10,temp2,0x12,0x13,0x14,0x15);//Set TX write addr based on unit number
						COM_TX(0);//send page command
						COM_FLUSH_TX();
						COM_TX(0);
						COM_FLUSH_TX();
						
						temp2=dataArray[index][1]*10+dataArray[index][2];
						temp3=index;
						temp=dataArray[temp3][15];
						while((temp!=0) && !(temp3>maxUnits)){
							COM_WRITE(5,0x10,temp3,0x12,0x13,0x14,0x15);
							COM_TX(0x9B+temp2);//send decrement command
							COM_FLUSH_TX();
							temp3++;
							temp=dataArray[temp3][15];
						}
						temp=1;
						temp3=currentLineNum;
						BU_DELETE_LINE(index);				
					}
					else if(cursor == 0x14){//delete, TODO:add confirmation
						temp2=dataArray[index][15];
						temp2=dataArray[index][1]*10+dataArray[index][2];				
						temp3=index;
						temp=dataArray[temp3][15];
						while((temp!=0) && !(temp3>maxUnits)){							
							COM_WRITE(5,0x10,temp3,0x12,0x13,0x14,0x15);
							COM_TX(0x9B+temp2);//send decrement command
							COM_FLUSH_TX();
							temp3++;
							temp=dataArray[temp3][15];
						}
						temp=1;
						BU_DELETE_LINE(index);
					}
					else if(cursor == 0x54){//call to desk, TODO:add confirmation
						temp=1;//exit while loop
						temp2=dataArray[index][15];
						COM_WRITE(5,0x10,temp2,0x12,0x13,0x14,0x15);//Set TX write addr based on unit number
						COM_TX(0xFE);//send toDesk command
						COM_FLUSH_TX();
					}
			}
			else if(temp1==0x0D){//back hit, return to main display
				temp=1;//exits loop
			}
			else if(temp1==0xF0){//up hit, cursor up
				BU_CURSOR_UP_CMD();
			}
			else if(temp1==0x0F){//down hit, cursor down
				BU_CURSOR_DOWN_CMD();
			}
		}//end of while temp==0
	}//end of if
	else{
		BU_SET_LINE(currentIndex);//input patron data
	}
	//Reprint main display
	BU_CLEAR_LINE(1);
	BU_CLEAR_LINE(2);
	BU_CLEAR_LINE(3);
	BU_CLEAR_LINE(4);
	BU_SET_LINE_VAR(1);
	BU_PRINT_LINE(currentLine1);
	BU_SET_LINE_VAR(2);
	BU_PRINT_LINE(currentLine1+1);
	BU_SET_LINE_VAR(3);
	BU_PRINT_LINE(currentLine1+2);
	BU_SET_LINE_VAR(4);
	BU_PRINT_LINE(currentLine1+3);
	BU_SET_LINE_VAR(tempLine);
	inCommandMenu=0;
}

void BU_CURSOR_UP_CMD(){//Move cursor up, line 1 not accessible
	unsigned int inCMD=0;
	if(inCommandMenu == 1){
		inCMD=1;
	}
	inCommandMenu = 1;
	 if(cursor == 0x14){
		BU_SET_LINE_VAR(2);
	}
	else if(cursor == 0x54){
		BU_SET_LINE_VAR(3);
	}
	if(inCMD==0){
		inCommandMenu=0;
	}
}
void BU_CURSOR_DOWN_CMD(){//Move cursor down, line 5 and below not accessible unless more commands added
	unsigned int inCMD=0;
	if(inCommandMenu == 1){
		inCMD=1;
	}
	inCommandMenu = 1;
	if(cursor == 0x40){
		BU_SET_LINE_VAR(3);
	}
	else if(cursor == 0x14){
		BU_SET_LINE_VAR(4);
	}
	if(inCMD==0){
		inCommandMenu=0;
	}
}
	for(unsigned int count=1; count<=2;count++){
	dataArray[count][0]= ' ';
	unitArray[count]=1;
	BU_INC_POSINLINE(count);
	dataArray[count][3]='.';
	dataArray[count][4]=(char)('0'+count);
	dataArray[count][5]=(char)('0'+count);
	dataArray[count][6]=(char)('0'+count);
	dataArray[count][7]=(char)('0'+count);
	dataArray[count][8]=(char)('0'+count);
	dataArray[count][9]=(char)('0'+count);
	dataArray[count][10]=(char)('0'+count);
	dataArray[count][11]=(char)('0'+count);
	dataArray[count][12]=(char)('0'+count);
	dataArray[count][13]=(char)('0'+count);
	dataArray[count][14]=' ';
	dataArray[count][15]=count;
	dataArray[count][16]=0;
	dataArray[count][17]=0;
	dataArray[count][18]=0;
	dataArray[count][19]=0;
	dataArray[count][20]=0;
	}
}
	unsigned int temp;
	unsigned int key=0xFF;
	unsigned int count=1,cycle=1,testMode=1;
	while(1){
		if(testMode==1){
		_delay_ms(1000);
		if(cycle==1){
			cycle++;
			COM_WRITE(5,0x10,0x02,0x12,0x13,0x14,0x15);
			//COM_READ(0x00,temp);
			COM_TX(count);
			COM_FLUSH_TX();
			count++;
			if(count==100){
				count=0;
			}
			
		}else{
			cycle=1;
			COM_WRITE(5,0x10,0x01,0x12,0x13,0x14,0x15);//order all units below paged unit to decrement line number
			//COM_READ(0x00,temp);
			COM_TX(count);//send decrement command
			COM_FLUSH_TX();
			count++;
			if(count==100){
				count=0;
			}
		}
		}//testMode if
		else{
		keypad_get(key);
		_delay_ms(250);
		if(!(key == 0xFF)){
			
			lcd_command(0x01);
			//BU_SET_LINE_VAR(3);
			numsel((int)key);
			
			if(key != 0xF0){
				COM_READ(0x00,temp);
				COM_TX(key);
				COM_FLUSH_TX();
				//	_delay_ms(500);
			}
			else{
				COM_READ(0x00,temp);
				COM_TX(0xFE);
				COM_FLUSH_TX();
				
			}
			
			lcd_char('G');

		}//END KEYPAD IF
		else if((key != 0xFF)){
			numsel(key);
		}
		}
		}//end else testMode
}

int main(void)
{
	unsigned int key=0xFF;
	SPI_init();
	COM_INIT();
	COM_FLUSH_TX();
	lcd_init();
	keypad_init();

	dataArray[0][0]= ' ';
	dataArray[0][1]=0;
	dataArray[0][2]=0;
	dataArray[0][3]=0;
	dataArray[0][4]=0;
	dataArray[0][5]=0;
	dataArray[0][6]=0;
	dataArray[0][7]=0;
	dataArray[0][8]=0;
	dataArray[0][9]=0;
	dataArray[0][10]=0;
	dataArray[0][11]=0;
	dataArray[0][12]=0;
	dataArray[0][13]=0;
	dataArray[0][14]=0;
	dataArray[0][15]=0;
	dataArray[0][16]=0;
	dataArray[0][17]=0;
	dataArray[0][18]=0;
	dataArray[0][19]=0;
	dataArray[0][20]=0;
	
	currentLine1=1;
	currentIndex=1;
	lcd_command(0x0F);
	BU_CLEAR_LINE(1);
	BU_CLEAR_LINE(2);
	BU_CLEAR_LINE(3);
	BU_CLEAR_LINE(4);
	
	BU_SET_LINE_VAR(1);
	BU_PRINT_LINE(1);
	BU_SET_LINE_VAR(1);
	
	
	sei();
	timerInit();
	TIMSK1 =TIMSK1 | (1<< OCIE1A);//enable timer1 interrupt
	
	while(1)
	{
		key=currentLineNum;
		keypad_get(key);
		_delay_ms(250);
		if(key==0x0E){//If select pressed, enter CMD menu. If on "enter new", proceed into Setline via CMD menu
			BU_CMD_MENU(currentIndex);
		}
		else if(key==0xF0){//up pressed, cursor up
			BU_CURSOR_UP();
		}
		else if(key==0x0F){//down pressed, cursor down
			BU_CURSOR_DOWN();
		}
	}//END OF WHILE
}//END OF MAIN

void numsel(int a){//used to print numbers to lcd
	if(a==0){//print zero
		lcd_char(0x30);
	}
	if(a==1){//print 1
		lcd_char(0x31);
	}
	if(a==2){//print 2
		lcd_char(0x32);
	}
	if(a==3){//print 3
		lcd_char(0x33);
	}
	if(a==4){//print 4
		lcd_char(0x34);
	}
	if(a==5){//print zero
		lcd_char(0x35);
	}
	if(a==6){//print zero
		lcd_char(0x36);
	}
	if(a==7){//print zero
		lcd_char(0x37);
	}
	if(a==8){//print zero
		lcd_char(0x38);
	}
	if(a==9){//print zero
		lcd_char(0x39);
	}
	if(a==10){//print A
		lcd_char(0x41);
	}
	if(a==11){//print B
		lcd_char(0x42);
	}
	if(a==12){//print C
		lcd_char(0x43);
	}
	if(a==13){//print D
		lcd_char(0x44);
	}
	if(a==14){//print E
		lcd_char(0x45);
	}
	if(a==15){//print F
		lcd_char(0x46);
	}
}

ISR(TIMER1_COMPA_vect){//print time interrupt, increment timers every ten seconds
	/*
	//Increment all active unit timers every ten seconds.
	//Print timers if on base menu OR only if not doing stuff?
	//
	*/
	unsigned int tens,ones,tenths,count=1,onCount=0,temp,oldLine,temp3;
	oldLine=currentLineNum;
	temp=dataArray[1][15];
	timerCount++;
	while(temp!=0){//if data is active, and in first four lines, print.
		if(timerCount==2){
		tens=dataArray[count][16];
		ones=dataArray[count][17];
		tenths=dataArray[count][18];
		//increment timer
		tenths++;
		if(tenths>=6){
			tenths=0;
			ones++;
		}
		if(ones>=6){
			ones=0;
			tens++;
		}
		if(tens>=6){
			tens=0;
		}
		if((count>=currentLine1) &&(count<=currentLine1+4) && !(inCommandMenu == 1)){//print if on screen, and not in command menu
			if(onCount==0){
				lcd_command(0x80 | 0x00+ 15);//increment to position 15
			}
			else if(onCount==1){
				lcd_command(0x80 | 0x40+ 15);//increment to position 15
			}
			else if(onCount==2){
				lcd_command(0x80 | 0x14+ 15);//increment to position 15
			}
			else if(onCount==3){
				lcd_command(0x80 | 0x54+ 15);//increment to position 15
			}
			numsel((int)tens);
			numsel((int)ones);
			lcd_char(':');
			numsel((int)tenths);
			lcd_char('0');
			onCount++;
			BU_SET_LINE_VAR(oldLine);
		}
		
		dataArray[count][16]=tens;
		dataArray[count][17]=ones;
		dataArray[count][18]=tenths;
		}//end if
		count++;
		temp=dataArray[count][15];//check to see if next data position in queue is enabled
		
	}//end while
	if(timerCount>=2){
		timerCount=0;
	}
}
