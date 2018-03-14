// Lcd 16x2 in 4 bit mode 
// LCD.C file diver 
// As defined in the following structure the pin connection is as follows:
// RB4 RS
// RB5 Enable
// RB0 DB4
// RB1 DB5
// RB2 DB6
// RB3 DB7
//
// LCD pins B0-B3 are not used and RW is not used.
#define rs PIN_C0      //B4
//rw connect gnd
#define enabled PIN_C1 //B5
#define DB4 PIN_B0 // B0
#define DB5 PIN_B1 // B1
#define DB6 PIN_B7 // B2
#define DB7 PIN_B6 // N3
char const lcd_type=2;
char const LCD_INIT_STRING[4] = {0x20 | (lcd_type << 2), 0xc, 1, 6};
//=======================================
//=======================================
void make_out_data(char buffer_data)
{ 
output_bit(DB4,bit_test(buffer_data,0));
output_bit(DB5,bit_test(buffer_data,1));
output_bit(DB6,bit_test(buffer_data,2));
output_bit(DB7,bit_test(buffer_data,3));
}
//===================================
//===================================
void lcd_send_nibble(char buffer_nibble)
{ make_out_data(buffer_nibble);
delay_us(10);
output_high(enabled);
delay_us(10);
output_low(enabled);
}
//====================================
//====================================
void lcd_send_byte( char address, char n )
{
output_low(rs); //rs= = 0;
delay_ms(1);
output_bit(rs,address);//lcd.rs = address;
delay_us(20);
delay_us(20);
output_low(enabled);//lcd.enable = 0;
lcd_send_nibble(n >> 4);
lcd_send_nibble(n & 0xf);
}
//===============================================
//===============================================
void lcd_init()
{ char i;
//set_tris_lcd(LCD_WRITE);
output_low(rs); //lcd.rs = 0;
//output_low(rw); //lcd.rw = 0;
output_low(enabled); //lcd.enable = 0;
delay_ms(200);
for(i=1;i<=3;++i)
{
lcd_send_nibble(3);
delay_ms(10);
}
lcd_send_nibble(2);
for(i=0;i<=3;++i)
lcd_send_byte(0,LCD_INIT_STRING[i]);
}
//================================================
//================================================
void lcd_gotoxy( char x, char y)
{char address;
switch(y) {
case 1 : address=0x80;break;
case 2 : address=0xc0;break;
case 3 : address=0x94;break;
case 4 : address=0xd4;break;
}
address=address+(x-1);
lcd_send_byte(0,0x80|address);
}
//=================================================
//=================================================
void lcd_putc( char c) {
switch (c) {
case '\f' : lcd_send_byte(0,1);
delay_ms(2); break;
case '\n' : lcd_gotoxy(1,2); break;
case '\b' : lcd_send_byte(0,0x10); break;
default : lcd_send_byte(1,c); break;
}
}
//====================================================
void LCD_Command(int cm);
void LCD_ShiftLeft(void);
void LCD_ShiftRight(void);
void LCD_MoveRight(char p);
void LCD_MoveLeft(char p);
void LCD_String(char*s,int dly);

void LCD_Command(int cm){
lcd_send_byte(0,cm);
}

void LCD_ShiftLeft(void){
lcd_send_byte(0,0x18);
}
void LCD_ShiftRight(void){
lcd_send_byte(0,0x1C);
}
void LCD_MoveRight(char p){
char i;
for(i=0;i<p;i++){
LCD_ShiftRight();
delay_ms(100);
}
}
void LCD_MoveLeft(char p){
char i;
for(i=0;i<p;i++){
LCD_ShiftLeft();
delay_ms(100);
}
}
/*
void strcpy(char*s1,char*s2) {
while(*s1++ == *s2++);
}
*/
void LCD_String(char*s, int dly) {
while(*s!=0) {
lcd_putc(*s++);
delay_ms(dly);
}
}
