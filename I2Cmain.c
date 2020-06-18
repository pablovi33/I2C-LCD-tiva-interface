
//*** PROGRAMA QUE CONFIGURA EL PERIFERICO I2C ***
//*** EN EL TIVA TM4C1294 COMO MAESTRO Y ESCLAVO EL RTC DS1307***

//Librerias
#include<stdint.h>
#include<stdbool.h>

//REGISTROS DE RELOJ
#define SYSCTL_RCGCGPIO_R       (*((volatile uint32_t *)0x400FE608)) //Reloj del puerto
#define SYSCTL_RCGCI2C_R        (*((volatile uint32_t *)0x400FE620)) //Reloj de I2C
#define SYSCTL_PRGPIO_R        (*((volatile uint32_t *)0x400FEA08)) //Bandera de "Peripherial Ready"

//REGISTROS DEL PUERTO B
#define GPIO_PORTB_DATA_R   (*((volatile uint32_t *)0x400593FC)) //Para los datos del puerto
#define GPIO_PORTB_DIR_R    (*((volatile uint32_t *)0x40059400)) //Para seleccionar función
#define GPIO_PORTB_AFSEL_R  (*((volatile uint32_t *)0x40059420)) //Para seleccionar función alterna
#define GPIO_PORTB_ODR_R    (*((volatile uint32_t *)0x4005950C)) //Para activar el Open Drain
#define GPIO_PORTB_DEN_R    (*((volatile uint32_t *)0x4005951C)) //Para activar función digital
#define GPIO_PORTB_PCTL_R   (*((volatile uint32_t *)0x4005952C)) //Para el control del puerto

//REGISTROS DEL MÓUDLO I2C
#define I2C0_MSA_R              (*((volatile uint32_t *)0x40020000)) //I2C Master Slave Adress
#define I2C0_MCS_R              (*((volatile uint32_t *)0x40020004)) //I2C Master Control Status
#define I2C0_MDR_R              (*((volatile uint32_t *)0x40020008)) //I2C Master Data Register
#define I2C0_MTPR_R             (*((volatile uint32_t *)0x4002000C)) //I2C Master Time Period
#define I2C0_MCR_R              (*((volatile uint32_t *)0x40020020)) //I2C Master Configuration Register

/*
El registro I2C Master Control/Status (I2C_MCS_R) tiene:
-Modo READ-ONLY DATUS: los 7 bits menos significativos son:
    7:Clock Time Out Error  6:BUS BUSY      5:IDLE
    4:Arbitration Lost      3:DataAck       2:AdrAck
    1:Error                 0:CONTROLLER BUSY

-Modo WRITE-ONLY CONTROL_ Los 6 bits menos significativos son:
    6:BURST    5:QuickCommand  4:High Speed Enable
    3:ACK      2:STOP          1:START
    0:RUN
*/

#define I2C_MCS_ACK 0x00000008 //Transmmitter Acknowledge Enable
#define I2C_MCS_DATACK 0x00000008 // Data Acknowledge Enable
#define I2C_MCS_ADRACK 0x00000004 // Acknowledge Address
#define I2C_MCS_STOP 0x00000004 // Generate STOP
#define I2C_MCS_START 0x00000002 // Generate START
#define I2C_MCS_ERROR 0x00000002 // Error
#define I2C_MCS_RUN 0x00000001 // I2C Master Enable
#define MAXRETRIES 5 // number of receive attempts before giving up

//**DirecciOn de pantalla LCD
int AdrePCF8574 =0x027;///Dirección del RTC DS1307

// Commandos
#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT 0x10
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80

// flags for display entry mode
#define LCD_ENTRYRIGHT 0x00
#define LCD_ENTRYLEFT 0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

// flags for display on/off control
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00

// flags for display/cursor shift
#define LCD_DISPLAYMOVE 0x08
#define LCD_CURSORMOVE 0x00
#define LCD_MOVERIGHT 0x04
#define LCD_MOVELEFT 0x00

// flags for function set
#define LCD_8BITMODE 0x10
#define LCD_4BITMODE 0x00
#define LCD_2LINE 0x08
#define LCD_1LINE 0x00
#define LCD_5x10DOTS 0x04
#define LCD_5x8DOTS 0x00

// flags for backlight control
#define LCD_BACKLIGHT 0x08
#define LCD_NOBACKLIGHT 0x00

#define En B00000100  // Enable bit
#define Rw B00000010  // Read/Write bit
#define Rs B00000001  // Register select bit

//segundo set de definiciones
#define LCD_FIRST_ROW         0x80
#define LCD_SECOND_ROW        0xC0
#define LCD_THIRD_ROW         0x94
#define LCD_FOURTH_ROW        0xD4
#define LCD_CURSOR_OFF        0x0C
#define LCD_UNDERLINE_ON      0x0E
#define LCD_BLINK_CURSOR_ON   0x0F
#define LCD_MOVE_CURSOR_LEFT  0x10
#define LCD_MOVE_CURSOR_RIGHT 0x14
#define LCD_TURN_ON           0x0C
#define LCD_TURN_OFF          0x08
#define LCD_SHIFT_LEFT        0x18
#define LCD_SHIFT_RIGHT       0x1E

#define LCD_TYPE              2 // 0 -> 5x7 | 1 -> 5x10 | 2 -> 2 lines

unsigned char RS, i2c_add, BackLight_State = LCD_BACKLIGHT;

/**
 * This is the driver for the Liquid Crystal LCD displays that use the I2C bus.
 *
 * After creating an instance of this class, first call begin() before anything else.
 * The backlight is on by default, since that is the most likely operating mode in
 * most cases.
 */

/*El cAlculo del Time Period Register (TPR) se especifica en la página 1284
 Asumiendo un reloj de 16 MHz y un modo de operación estándar (100 kbps):
*/

int TPR = 7;
//int i2c_add;

// Variables para manejo de rutinas
uint8_t error;
uint32_t i;

// DefiniciOn de apuntadores para direccionar
#define NVIC_ST_CTRL_R          (*((volatile unsigned long *)0xE000E010))  //apuntador del registro que permite configurar las acciones del temporizador
#define NVIC_ST_RELOAD_R        (*((volatile unsigned long *)0xE000E014))  //apuntador del registro que contiene el valor de inicio del contador
#define NVIC_ST_CURRENT_R       (*((volatile unsigned long *)0xE000E018))  //apuntador del registro que presenta el estado de la cuenta actual

// Definición de constantes para operaciones
#define NVIC_ST_CTRL_COUNT      0x00010000  // bandera de cuenta flag
#define NVIC_ST_CTRL_CLK_SRC    0x00000004  // Clock Source
#define NVIC_ST_CTRL_INTEN      0x00000002  // Habilitador de interrupción
#define NVIC_ST_CTRL_ENABLE     0x00000001  // Modo del contador
#define NVIC_ST_RELOAD_M        0x00FFFFFF  // Valor de carga del contador

//**********FUNCION Inizializa el SysTick *********************
void SysTick_Init(void){
  NVIC_ST_CTRL_R = 0;                   // Desahabilita el SysTick durante la configuración
  NVIC_ST_RELOAD_R = NVIC_ST_RELOAD_M;  // Se establece el valor de cuenta deseado en RELOAD_R
  NVIC_ST_CURRENT_R = 0;                // Se escribe al registro current para limpiarlo

  NVIC_ST_CTRL_R = 0x00000001;         // Se Habilita el SysTick y se selecciona la fuente de reloj
}

//*************FUNCION Tiempo de retardo utilizando wait.***************
// El parametro de retardo esta en unidades del reloj interno/4 = 4 MHz (250 ns)
void SysTick_Wait(uint32_t retardo){
    NVIC_ST_RELOAD_R= retardo-1;   //nUmero de cuentas por esperar
    NVIC_ST_CURRENT_R = 0;
    while((NVIC_ST_CTRL_R&0x00010000)==0){//espera hasta que la bandera COUNT sea valida
    }
   } //

// Tiempo de retardo utilizando wait
// 4000000 igual a 1 s

//*** FunciOn que inicializa los relojes, el GPIO y el I2C0 ***
void I2C_Init(void){
    //CONFIGURACIÓN DE LOS RELOJ
    SYSCTL_RCGCI2C_R |= 0x0001; // Activamos el reloj de I2C0 [I2C9 I2C8 I2C7 ... I2C0]<--Mapa de RCGCI2C
    SYSCTL_RCGCGPIO_R |= 0x0002; // Activamos el reloj GPIO_PORTB mientras se activa el reloj de I2C0
    while((SYSCTL_PRGPIO_R&0x0002) == 0){};//Espero a que se active el reloj del puerto B

    //CONFIGURACIÓN DE LOS GPIOS
    /*Acorde con la tabla "Signals by function" de la p. 1808:
     el PIN 2 del puerto B (PB2) es el I2C0SCL del I2C0, y
     el PIN 3 del puerto B (PB3) es el I2C0SDA del I2C0
    */
    GPIO_PORTB_AFSEL_R |= 0x0C; // Activo la funciOn alterna del PB2 y PB3
    GPIO_PORTB_ODR_R |= 0x08;   // Activo el OPEN DRAIN para el PB3, ya que el PB2 ya tiene uno por preconfig.
    GPIO_PORTB_DIR_R |= 0x0C;   //Activo al PB2 y al PB3 como OUTPUT
    GPIO_PORTB_DEN_R |= 0x0C;   //Activo la función digital de PB3 y PB2
    /*
    AsI como el registro AFSEL indica que se ejecutarA una funciOn externa, en el registro PCTL
    debemos indicar QUE funciOn alterna se realizarA acorde con la tabla 26-5 de la p.1808 e indicarlo
     en el correspondiente PCMn (uno por cada bit del puerto) del registro PCTL
     */

    GPIO_PORTB_PCTL_R|=0x00002200;


    //CONFIGURACIÓN DEL MODULO I2C0
    I2C0_MCR_R = 0x00000010; // Habilitar funciOn MASTER para el I2C0
    I2C0_MTPR_R = TPR; // Se establece una velocidad estándar de 100kbps
}


int I2C_Master_Wait(){
    while(I2C0_MCS_R&0x00000001){}; //Espero a que la transmisiOn acabe
      if(I2C0_MCS_R&0x00000002==1){ //¿Hubo error?
          error=1;
          return error;
      };
      return 0;

}

void I2C_Master_Init(int SubAdd){

    while(I2C0_MCS_R&0x00000001){}; // wait for I2C ready
    //Para transmitir
    I2C0_MSA_R=(AdrePCF8574<<1)&0xFE; //Cargo la direcciOn de la pantalla e indico "SEND", es decir, el Slave va a recibir
    I2C0_MDR_R=SubAdd&0x0FF; //Envio la SubdirecciOn (rango 0-3)
    I2C0_MCS_R=(I2C_MCS_RUN|I2C_MCS_START); // Condición de START y corro


}

void I2C_Master_Write(int dato){
    I2C_Master_Wait();
    SysTick_Init();
    SysTick_Wait(300);//espera 75 uS
    I2C0_MDR_R=(dato); //Envio el dato
    I2C0_MCS_R=(I2C_MCS_RUN);
}

void I2C_Master_Stop(){
    I2C0_MCS_R=(I2C_MCS_STOP|I2C_MCS_RUN); //Inicio la ultima transmisiOn y STOP
    I2C_Master_Wait();
    SysTick_Init();
        SysTick_Wait(300);//espera 75 uS
}

//funcion que sintetiza proceso de habilitacion e interrupcion de comunicaciOn
void IO_Expander_Write(int dato)
{
    //(funcionan direcciones de 0 a 3)
    I2C_Master_Init(0x03);
    I2C_Master_Write(dato | LCD_BACKLIGHT);
    I2C_Master_Stop();
}

void LCD_Write_4Bit(unsigned char Nibble)
{
    Nibble |= RS; // Get The RS Value To LSB OF Data
  IO_Expander_Write(Nibble | 0x04);
  IO_Expander_Write(Nibble & 0xFB);
  SysTick_Init();
  SysTick_Wait(200);//espera 50 uS
}

void LCD_CMD(int CMD)
{
  RS = 0; // Command Register Select
  LCD_Write_4Bit(CMD & 0xF0);
  LCD_Write_4Bit((CMD << 4) & 0xF0);
}

void LCD_Init()
{
      SysTick_Init();
      SysTick_Wait(200);//espera 50 uS
      IO_Expander_Write(0x00);
      SysTick_Wait(120000);//espera 30 mS
      LCD_CMD(0x03);
       SysTick_Wait(20000);//espera 5 mS
       LCD_CMD(0x03);
       SysTick_Wait(20000);//espera 5 mS
       LCD_CMD(0x03);
      SysTick_Wait(20000);//espera 5 mS
      LCD_CMD(LCD_RETURNHOME);
      SysTick_Wait(20000);//espera 5 mS
      LCD_CMD(0x20 | (LCD_TYPE << 2));
      SysTick_Wait(200000);//espera 50 mS
      LCD_CMD(LCD_TURN_ON);
      SysTick_Wait(200000);//espera 50 mS
      LCD_CMD(LCD_CLEARDISPLAY);
      SysTick_Wait(200000);//espera 50 mS
      LCD_CMD(LCD_ENTRYMODESET | LCD_RETURNHOME);
      SysTick_Wait(200000);//espera 50 mS
}

void LCD_Set_Cursor(unsigned char ROW, unsigned char COL)
{
  switch(ROW)
  {
    case 2:
      LCD_CMD(0xC0 + COL-1);
      break;
    case 3:
      LCD_CMD(0x94 + COL-1);
      break;
    case 4:
      LCD_CMD(0xD4 + COL-1);
      break;
    // Case 1
    default:
      LCD_CMD(0x80 + COL-1);
  }
}

void LCD_Write_Char(char Data)
{
  RS = 1; // Data Register Select
  LCD_Write_4Bit(Data & 0xF0);
  LCD_Write_4Bit((Data << 4) & 0xF0);
}

void LCD_Write_String(char* Str)
{
  for(i=0; Str[i]!='\0'; i++)
    LCD_Write_Char(Str[i]);
}


//*** PROGRAMA PRINCIPAL ****

void main(){


    I2C_Init(); //FunciOn que inicializa los relojes, el GPIO y el I2C0

    //Inicializo Slave
    while(I2C0_MCS_R&0x00000001){}; // espera que el I2C estE listo
    LCD_Init();

    SysTick_Init();

    LCD_Set_Cursor(1, 1);
    LCD_Write_String("Hola mundos :D!!");
    //LCD_Write_Char('a');
    LCD_Set_Cursor(2, 1);
    LCD_Write_String("PabloVivarColina");

        SysTick_Wait(4000000);//espera 1S
        //LCD_CMD(LCD_CLEARDISPLAY);


        LCD_Set_Cursor(1, 1);
               LCD_Write_String("                ");
               //LCD_Write_Char('a');
               LCD_Set_Cursor(2, 1);
               LCD_Write_String("                ");

        LCD_Set_Cursor(1, 1);
        LCD_Write_String("Hola");
        //LCD_Write_Char('a');
        LCD_Set_Cursor(2, 1);
        LCD_Write_String("Nada");


}

