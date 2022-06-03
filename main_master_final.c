/*
 * Archivo: main_master.c
 * Dispositivo: PIC16F887
 * Compilador:  XC8, MPLABX v5.40
 * Autor: Jos? Fernando de Le?n Gonz?lez
 * Programa: Master del robot de pelea (Entradas anal?gicas y control de servos) 
 * 
 * Hardware:  potenci?metros de posicionamiento en RA0, RA1, RA2 y RA3, selector de esclavo en RA6 y RA7, 
 *            servomotores FUTABA S3003 en RC1 (miembro RA1) y RC2 (miembro LA1) bot?n MODO en RB0, 
 *            botones de acci?n y EDITAR/ACEPTAR en RB1 y RB2 & Joysticks en RA4 (Rueda derecha)
 *            y RA5 (rueda izquierda)
 * 
 * Creado: 25/05/22
 * ?ltima modificaci?n: 25/05/22
 */

// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT // Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdint.h>

/*------------------------------------------------------------------------------
 * Constantes
------------------------------------------------------------------------------*/
#define _XTAL_FREQ 500000       // Oscilador de 500 kHz
#define IN_MIN 0                // Valor minimo de entrada del potenciometro
#define IN_MAX 255              // Valor m?ximo de entrada del potenciometro
#define OUT_MIN 20               // Valor minimo de ancho de pulso de se?al PWM
#define OUT_MAX 80              // Valor m?ximo de ancho de pulso de se?al PWM

/*------------------------------------------------------------------------------
 * Variables
------------------------------------------------------------------------------*/
unsigned short CCPR_1 = 0;        // Variable para almacenar ancho de pulso al hacer la interpolaci?n lineal
unsigned short CCPR_2 = 0;        // Variable para almacenar ancho de pulso al hacer la interpolaci?n lineal

uint8_t pot_RA1 = 0;               // Variable para almacenar el valor a guardar de la posici?n del servo 1
uint8_t pot_RP2 = 0;               // Variable para almacenar el valor a guardar de la posici?n del servo 2
uint8_t pot_LA1 = 0;               // Variable para almacenar el valor a guardar de la posici?n del servo 3
uint8_t pot_LP2 = 0;               // Variable para almacenar el valor a guardar de la posici?n del servo 4

uint8_t mode_flag = 0;
uint8_t wheel_flag_r = 0;
uint8_t wheel_flag_l = 0;

uint8_t serial_data = 0;         // Variable para almacenar datos provenientes de 
uint8_t pot_elect = 0;           // Variable para escoger pot
uint8_t pot_interfaz1 = 0;
uint8_t pot_interfaz2 = 0;
uint8_t pot_interfaz3 = 0;
uint8_t pot_interfaz4 = 0;

uint8_t save1_flag = 0;
uint8_t save2_flag = 0;
/*------------------------------------------------------------------------------
 * Prototipos de funciones
------------------------------------------------------------------------------*/
void setup (void);                  // Prototipo de funci?n SETUP

unsigned short interpole(uint8_t value, uint8_t in_min, uint8_t in_max, 
            unsigned short out_min, unsigned short out_max);                // Prototipo de funci?n interpolaci?n

void move_servo(uint8_t servo, uint8_t interpole_val);           // Prototipo de funci?n para mover los servos

void send_data (uint8_t data);                  //Prototipo de funci?n para enviar informaci?n

uint8_t read_EEPROM(uint8_t address);           // Prototipo de funci?n para leer valores de la EEPROM

void write_EEPROM(uint8_t address, uint8_t data);  // Prototipo de funci?n para escribir valores de la EEPROM

/*------------------------------------------------------------------------------
 * Interrupciones
------------------------------------------------------------------------------*/
void __interrupt() isr (void){
    
    if (INTCONbits.RBIF)        // Fue interrupci?n IOCB?
    {
        if (!PORTBbits.RB0){
            mode_flag = mode_flag++;
            if (mode_flag >= 3)
                mode_flag = 0;
        }
   
        else if (!PORTBbits.RB1){
            if (mode_flag == 0){
                CCPR_1 = read_EEPROM(0x01);
                move_servo (1,CCPR_1);
                
                PORTDbits.RD1 = 0;
                send_data(read_EEPROM (0x02));
                __delay_ms(50);
                
                CCPR_2 = read_EEPROM(0x03);
                move_servo (2,CCPR_2);
                
                PORTDbits.RD1 = 1;
                send_data(read_EEPROM (0x04));
                __delay_ms(50);
            }
            else if (mode_flag == 1){
                if (save1_flag == 0){
                    save1_flag = 1;
                }                    
            }            
        }
        
        else if (!PORTBbits.RB2){
            if (mode_flag == 0){
                CCPR_1 = read_EEPROM(0x05);
                move_servo (1,CCPR_1);
                
                PORTDbits.RD1 = 0;
                send_data(read_EEPROM (0x06));
                __delay_ms(50);
                
                CCPR_2 = read_EEPROM(0x07);
                move_servo (2,CCPR_2);
                
                PORTDbits.RD1 = 1;
                send_data(read_EEPROM (0x08));
                __delay_ms(50);
            }
            else if (mode_flag == 1){
                if (save2_flag == 0){
                    save2_flag = 1;
                }  
            }    
        }
                
        INTCONbits.RBIF = 0;    // Limpiamos bandera de interrupci?n RBIF
    }
    
    if(PIR1bits.ADIF){                      // Fue interrupci?n del ADC?
        
        if (ADCON0bits.CHS == 0b0100){     // Verificamos que AN4 sea el canal seleccionado
                if (ADRESH <= 107){
                    wheel_flag_r = 1;
                }
                else if (ADRESH >= 147){
                    wheel_flag_r = 2;
                }
                else{
                    wheel_flag_r = 0;
                }    
            }
        else if(ADCON0bits.CHS == 0b0101){
                if (ADRESH <= 107){
                    wheel_flag_l = 1;
                }
                else if (ADRESH >= 147){
                    wheel_flag_l = 2;
                }
                else{
                    wheel_flag_l = 0;
                }    
            }
        if (mode_flag == 1){
            if(ADCON0bits.CHS == 0b0000){            // Verificamos que AN0 sea el canal seleccionado
                pot_RA1 = ADRESH;                      // Guardamos el valor de ADRESH en la variable pot_1
                move_servo (1,ADRESH);               // Mover servo RA1 (en el master)
            }
            else if(ADCON0bits.CHS == 0b0001){       // Verificamos que AN1 sea el canal seleccionado
                pot_RP2 = ADRESH;                      // Guardamos el valor de ADRESH en la variable pot_2
                PORTDbits.RD1 = 0;                   // Mover servo RP2 (en el slave por medio de SPI verificando el bit RD1)
                send_data(ADRESH);
                __delay_ms(50);                      // Esperamos que el slave reciba el dato y movilice el servo
            }
            else if (ADCON0bits.CHS == 0b0010){      // Verificamos que AN2 sea el canal seleccionado
                pot_LA1 = ADRESH;                      // Guardamos el valor de ADRESH en la variable pot_3
                move_servo (2,ADRESH);               // Mover servo LA1 (en el master)
            }
            else if (ADCON0bits.CHS == 0b0011){       // Verificamos que AN3 sea el canal seleccionado
                pot_LP2 = ADRESH;                      // Guardamos el valor de ADRESH en la variable pot_4
                PORTDbits.RD1 = 1;                   // Mover servo LP2 (en el slave por medio de SPI verificando el bit RD1)
                send_data(ADRESH);              
                __delay_ms(50);                      // Esperamos que el slave reciba el dato y movilice el servo
            }
            else{}
                
        }
        PIR1bits.ADIF = 0;                  // Limpiamos bandera de interrupci?n
    }
    
    if(PIR1bits.RCIF){          // Bandera de datos provenientes de comunicación seriañ
        serial_data = RCREG;    // Guardamos valor recibido en el arreglo mensaje
        
        if(mode_flag == 2){
            
            pot_elect = 0b11000000 & serial_data;
            
            if (pot_elect == 0){
                pot_interfaz1 = serial_data;
                move_servo (1 ,pot_interfaz1);
            }
            else if(pot_elect == 0b01000000){
                pot_interfaz2 = serial_data;
                PORTDbits.RD1 = 0;                   // Mover servo RP2 (en el slave por medio de SPI verificando el bit RD1)
                send_data(pot_interfaz2);
                __delay_ms(50);
            }
            else if(pot_elect == 0b10000000){
                pot_interfaz3 = serial_data;
                move_servo (2,pot_interfaz3); 
            }
            else if(pot_elect == 0b11000000){
                pot_interfaz4 = serial_data;
                PORTDbits.RD1 = 1;                   // Mover servo RP2 (en el slave por medio de SPI verificando el bit RD1)
                send_data(pot_interfaz4);
                __delay_ms(50);
            }
             __delay_ms(50);
        }
    }
    return;
}
/*------------------------------------------------------------------------------
 * Ciclo principal
------------------------------------------------------------------------------*/
void main(void) {
    setup();
    
    CCPR_1 = read_EEPROM(0x01);
    move_servo (1,CCPR_1);
                
    PORTDbits.RD1 = 0;
    send_data(read_EEPROM (0x02));
    __delay_ms(50);
                
    CCPR_2 = read_EEPROM(0x03);
    move_servo (2,CCPR_2);
                
    PORTDbits.RD1 = 1;
    send_data(read_EEPROM (0x04));
    __delay_ms(50);
    
    while(1){
        if (ADCON0bits.GO == 0) {          // Verificar si se debe hacer una conversi?n
        if (ADCON0bits.CHS == 0b0000)       
            ADCON0bits.CHS = 0b0001;       // Convertir el valor del potenci?metro RP2
        
        else if (ADCON0bits.CHS == 0b0001)
            ADCON0bits.CHS = 0b0010;       // Convertir el valor del potenci?metro LA1
        
        else if (ADCON0bits.CHS == 0b0010)
            ADCON0bits.CHS = 0b0011;       // Convertir el valor del potenci?metro LP2
        
        else if (ADCON0bits.CHS == 0b0011)
            ADCON0bits.CHS = 0b0100;       // Convertir el valor del Joystick 1 
        
        else if (ADCON0bits.CHS == 0b0100)
            ADCON0bits.CHS = 0b0101;       // Convertir el valor del Joystick 2
        else
            ADCON0bits.CHS = 0b0000;       // Convertir el valor del potenci?metro RA1 
        
        __delay_us(1000);
        ADCON0bits.GO = 1;
        
        }
        if (mode_flag == 0){
            PORTDbits.RD2 = 1;
            PORTDbits.RD3 = 0;
            PORTBbits.RB7 = 0;
            
        }
        
        else if (mode_flag == 1){
            PORTDbits.RD2 = 0;
            PORTDbits.RD3 = 1;
            PORTBbits.RB7 = 0;
        }
        else {
            PORTDbits.RD2 = 0;
            PORTDbits.RD3 = 0;
            PORTBbits.RB7 = 1;
            
        }
        
        if (wheel_flag_r == 1){

            PORTBbits.RB3 = 1;
            __delay_ms(10);
            PORTBbits.RB4 = 0;
            __delay_ms(10);
            /*PORTBbits.RB5 = 1;
            __delay_ms(10);
            PORTBbits.RB6 = 0;
            __delay_ms(10);*/
        }
        else if (wheel_flag_r == 2){

            PORTBbits.RB3 = 0;
            __delay_ms(10);
            PORTBbits.RB4 = 1;
            __delay_ms(10);
           /* PORTBbits.RB5 = 0;
            __delay_ms(10);
            PORTBbits.RB6 = 1;
            __delay_ms(10);*/
        }
        else{
            PORTBbits.RB3 = 0;
            __delay_ms(10);
            PORTBbits.RB4 = 0;
            __delay_ms(10);
            /*PORTBbits.RB5 = 0;
            __delay_ms(10);
            PORTBbits.RB6 = 0;
            __delay_ms(10);*/
        }
        
        if (wheel_flag_l == 1){

            PORTBbits.RB5 = 1;
            __delay_ms(10);
            PORTBbits.RB6 = 0;
            __delay_ms(10);
           /* PORTDbits.RD6 = 1;
            __delay_ms(10);
            PORTDbits.RD7 = 0;
            __delay_ms(10);*/
        }
        else if (wheel_flag_l == 2){
            PORTBbits.RB5 = 0;
            __delay_ms(10);
            PORTBbits.RB6 = 1;
            __delay_ms(10);
           /* PORTDbits.RD6 = 0;
            __delay_ms(10);
            PORTDbits.RD7 = 1;
            __delay_ms(10);*/
        }
        else{
            PORTBbits.RB5 = 0;
            __delay_ms(10);
            PORTBbits.RB6 = 0;
            __delay_ms(10);
            /*PORTDbits.RD6 = 0;
            __delay_ms(10);
            PORTDbits.RD7 = 0;
            __delay_ms(10);*/
        }
       /* if(mode_flag == 2){
            pot_interfaz1;
            pot_interfaz2;
            pot_interfaz3;
            pot_interfaz4;
        }*/
        
        if (save1_flag == 1){
        write_EEPROM(0x01, pot_RA1);
        __delay_ms(50);
        write_EEPROM(0x02, pot_RP2);
        __delay_ms(50);
        write_EEPROM(0x03, pot_LA1);
        __delay_ms(50);
        write_EEPROM(0x04, pot_LP2);
        __delay_ms(50);
        save1_flag = 0;
        }
        if (save2_flag == 1){
            write_EEPROM(0x05, pot_RA1);
                __delay_ms(50);
                write_EEPROM(0x06, pot_RP2);
                __delay_ms(50);
                write_EEPROM(0x07, pot_LA1);
                __delay_ms(50);
                write_EEPROM(0x08, pot_LP2);
                __delay_ms(50);
        }
    }

    return;
}
/*------------------------------------------------------------------------------
 * Configuraci?n
------------------------------------------------------------------------------*/
void setup(void){
    //Configuraci?n de los puertos
    ANSEL = 0b11111111;        // PORTA y PORTE como entrada anal?gica
    ANSELH = 0;                // I/O digitales
    
    TRISA = 0b11111111;        // PORTA como entrada
    PORTA = 0;                 // Limpiamos PORTA  
     
    TRISE = 0b1111;            // PORTE como entrada
    PORTE = 0;                 // limpiamos PORTE
    
    TRISD = 0b00000000;        // TRISD como salida
    PORTD = 0b00000000;        // RD0 habilitado (SS SLAVE I)
    
    // Configuraci?n de las IOCB
    TRISB = 0b00000111;         // RB0, RB1 & RB2 de PORTB como entradas
    PORTB = 0;
    
    OPTION_REGbits.nRBPU = 0;   // Habilitamos resistencias de pull-up del PORTB
    WPUBbits.WPUB0 = 1;         // Habilitamos resistencia de pull-up de RB0, RB1 & RB2
    WPUBbits.WPUB1 = 1;
    WPUBbits.WPUB2 = 1;
    
    INTCONbits.RBIE = 1;        // Habilitamos interrupciones del PORTB
    IOCBbits.IOCB0 = 1;         // Habilitamos IOCB en RB0, RB1 & RB2
    IOCBbits.IOCB1 = 1;
    IOCBbits.IOCB2 = 1;
    
    INTCONbits.RBIF = 0;        // Limpiamos bandera de interrupci?n de PORTB
    
    //Configuraci?n SPI MASTER de puertos
    TRISC = 0b10010000;         // -> SDI entrada, SCK y SD0 como salida
    PORTC = 0;
        
    // Configuraci?n reloj interno
    OSCCONbits.IRCF = 0b011;    // IRCF <2:0> 011 -> 500 kHz
    OSCCONbits.SCS = 1;         // Oscilador interno
    
    // Configuraci?n ADC
    ADCON0bits.ADCS = 0b01;     // Fosc/8
    ADCON1bits.VCFG0 = 0;       // VDD
    ADCON1bits.VCFG1 = 0;       // VSS
    ADCON0bits.CHS = 0b0000;    // Seleccionamos el AN0
    ADCON1bits.ADFM = 0;        // Justificado a la izquierda
    ADCON0bits.ADON = 1;        // Habilitamos modulo ADC
    __delay_us(40);             // Sample time
    
    // Configuraci?n PWM
    TRISCbits.TRISC2 = 1;       // RC2 -> CCP1 como entrada
    TRISCbits.TRISC1 = 1;       // RC1 -> CCP2 como entrada
    PR2 = 156;                  // periodo de 20 ms
    
    // Configuraci?n CCP
    CCP1CON = 0;                // Apagamos CCP1
    CCP1CONbits.P1M = 0;        // Modo single output
    
    CCP1CONbits.CCP1M = 0b1100; // Asignaci?n de modo a PWM1
    CCP2CONbits.CCP2M = 0b1100; // Asignaci?n de modo a PWM2
    
    CCPR1L = 155>>2;
    CCP1CONbits.DC1B = 155 & 0b11;    // Valor inicial del duty cycle PWM1
    
    CCPR2L = 155>>2;
    CCP2CONbits.DC2B0 = 155 & 0b01;
    CCP2CONbits.DC2B1 = 155 & 0b10;   //  Valor inicial del duty cycle PWM2
            
    
    PIR1bits.TMR2IF = 0;        // Limpiamos bandera de interrupcion del TMR2
    T2CONbits.T2CKPS = 0b11;    // prescaler 1:16
    T2CONbits.TMR2ON = 1;       // Encendemos TMR2
    while(!PIR1bits.TMR2IF);    // Esperar un ciclo del TMR2
    PIR1bits.TMR2IF = 0;        // Limpiamos bandera de interrupcion del TMR2 nuevamente
    
    TRISCbits.TRISC2 = 0;       // RC2 -> CCP1 como salida del PWM2
    TRISCbits.TRISC1 = 0;       // RC1 -> CCP2 como salida del PWM2
    
    // Configuraci?n del SPI (MASTER)
    
    // SSPCON <5:0>
    SSPCONbits.SSPM = 0b0000;   // -> SPI Maestro, Reloj -> Fosc/4 (250kbits/s)
    SSPCONbits.CKP = 0;         // -> Reloj inactivo en 0
    SSPCONbits.SSPEN = 1;       // -> Habilitamos pines de SPI
    // SSPSTAT<7:6>
    SSPSTATbits.CKE = 1;        // -> Dato enviado cada flanco de subida
    SSPSTATbits.SMP = 1;        // -> Dato al final del pulso de reloj
    SSPBUF = 0xFF;              // Enviamos un dato inicial
        
    // Configuracion interrupciones
    INTCONbits.PEIE = 1;        // Habilitamos int. de perifericos
    INTCONbits.GIE = 1;         // Habilitamos int. globales
    PIE1bits.RCIE = 1;          // Habilitamos int. UART
    PIE1bits.ADIE = 1;          // Habilitamos interrupcion de ADC
    
    PIR1bits.ADIF = 0;          // Limpiamos bandera de ADC
    PIR1bits.RCIF = 0;          // Limpiamos bandera de int. UART
    
    //Configuración comunicación serial
        //SYNC = 0, BRGH = 1, BRG16 = 1, SPBRG=25 <- Valores de tabla 12-5
    TXSTAbits.SYNC = 0;         // Comunicación ascincrona (full-duplex)
    TXSTAbits.BRGH = 1;         // Baud rate de alta velocidad 
    BAUDCTLbits.BRG16 = 1;      // 16-bits para generar el baud rate
    
    SPBRG = 25;
    SPBRGH = 0;                 // Baud rate ~9600, error -> 0.16%
    
    RCSTAbits.SPEN = 1;         // Habilitamos comunicación
    TXSTAbits.TX9 = 0;          // Utilizamos solo 8 bits
    TXSTAbits.TXEN = 1;         // Habilitamos transmisor
    RCSTAbits.CREN = 1;         // Habilitamos receptor
}

/*------------------------------------------------------------------------------
 * Funciones
------------------------------------------------------------------------------*/
//Funci?n para interpolar valores de entrada a valores de salida
unsigned short interpole(uint8_t value, uint8_t in_min, uint8_t in_max, 
                         unsigned short out_min, unsigned short out_max){
    
    return (unsigned short)(out_min+((float)(out_max-out_min)/(in_max-in_min))*(value-in_min));
}

//Funci?n para posicionar un servomotor en base al valor de un potenci?metro
void move_servo(uint8_t servo, uint8_t interpole_val) {
    
    unsigned short CCPR = 0;    
    if (servo == 1){
        CCPR = interpole(interpole_val, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX); // Valor de ancho de pulso
        CCPR1L = (uint8_t)(CCPR>>2);    // Guardamos los 8 bits mas significativos en CPR1L
        CCP1CONbits.DC1B = CCPR & 0b11; // Guardamos los 2 bits menos significativos en DC1B
    }
    else {
        CCPR = interpole(interpole_val, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX); // Valor de ancho de pulso
        CCPR2L = (uint8_t) (CCPR>>2);
        CCP2CONbits.DC2B0 = CCPR & 0b10;
        CCP2CONbits.DC2B1 = CCPR & 0b01;   
    }
    return;
}

//Funci?n para enviar informaci?n mediante SPI
void send_data (uint8_t data){

    PORTDbits.RD0 = 0;              // Habilitamos el esclavo para recibir datos
    SSPBUF = data;                  // Cargamos valor del contador al buffer
    while(!SSPSTATbits.BF){}        // Esperamos a que termine el envio
    PORTDbits.RD0 = 1;              // Deshabilitamos el esclavo hasta el siguiente env?o.

    return;
}

// Funci?n para leer valores de la EEPROM
uint8_t read_EEPROM(uint8_t address){
    EEADR = address;
    EECON1bits.EEPGD = 0;       // Lectura a la EEPROM
    EECON1bits.RD = 1;          // Obtenemos dato de la EEPROM
    return EEDAT;               // Regresamos dato 
}

//Funci?n para escribir valores en la EEPROM
void write_EEPROM(uint8_t address, uint8_t data){
    EEADR = address;
    EEDAT = data;
    EECON1bits.EEPGD = 0;       // Escritura a la EEPROM
    EECON1bits.WREN = 1;        // Habilitamos escritura en la EEPROM
    
    INTCONbits.GIE = 0;         // Deshabilitamos interrupciones
    INTCONbits.PEIE = 0;        // Deshabilitamos interrupciones de los puertos
    EECON2 = 0x55;      
    EECON2 = 0xAA;
    
    EECON1bits.WR = 1;          // Iniciamos escritura
    
    EECON1bits.WREN = 0;        // Deshabilitamos escritura en la EEPROM
    INTCONbits.RBIF = 0;
    INTCONbits.GIE = 1;         // Habilitamos interrupciones
    INTCONbits.PEIE = 1;        // Habilitamos interrupciones de los puertos
}