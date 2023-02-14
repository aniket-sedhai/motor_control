#include <msp430.h> 
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#define MAX_SUBSTRINGS 100
#define MAX_COMMAND_ARG 5
#define ARG_2 1
#define ARG_1 0

#define CLOCKWISE 1
#define COUNTER_CLOCKWISE 2
#define STOP 3

/**--GLOBAL VARIABLES DECLARATION--**/
char* Azimuth_Rotation = "AZ";
char* Stay = "SA";
char* return_curr_angle = "IP";

char RXData[20] = "";
char command[20] = "";
char* cmd = "";
unsigned int command_index = 0;
unsigned int K = 0;
int command_received = 0;
int command_not_executed = 0;
char* command_substrings[MAX_COMMAND_ARG];
float target_angle = 0;
float current_angle = 0;
//Closer than this means we can stop
static float ANGLE_THRESHOLD = 10.0f;
int rotation_state = 0;

/*--FUNCTION DECLARATIONS--*/
void init_LEDs(void);
void init_UART(int Rx, int Tx);
void UART_Transmit(char string[]);
void delay_ms(int ms);
void UART_Read_Command(void);
void initButton(void);
void init_GPIO(void);
int split_by_space(char *str, char *substrings[]);
int getRotationDir(void);


/**
 * main.c
 */
void main(void)
{
    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer



    init_UART(1,0);
    init_LEDs();
    init_GPIO();
//  initButton();
    // -- Setup IRQ A1 RXIFG

    UCA0IE |= UCRXIE;          // LOCAL ENABLE FOR A0 RXIFG

    __enable_interrupt();       //global en for maskables.... (GIE bit in SR)

    PM5CTL0 &= ~LOCKLPM5;           //TURN ON I/O
    while(1)
    {
        if (command_received)
        {
            split_by_space(cmd, command_substrings);
            command_received = 0;

            if (*command_substrings[ARG_1] == *Azimuth_Rotation)
            {
                rotation_state = 1;
                target_angle = atoi(command_substrings[ARG_2]);
            }
            else if (*command_substrings[ARG_1] == *Stay)
            {
                target_angle = current_angle;
                rotation_state = 0;
            }
            else if (*command_substrings[ARG_1] == *return_curr_angle)
            {
                rotation_state = 0;
            }

            command_not_executed = 1;

        }
        if(command_not_executed)
        {
            while(getRotationDir() != STOP && command_received == 0)
            {
                if(getRotationDir() == CLOCKWISE)
                {
                    //Drive MOTOR_DIR_PIN LOW
                    P1OUT &= ~BIT4;
                }
                else
                {
                    //Drive MOTOR_DIR_PIN HIGH
                    P1OUT |= BIT4;
                }
                float delay_amount = 0.1;
                if(abs(target_angle - current_angle)<40.0)
                {
                    delay_amount = 1;
                }
                // Create a pwm signal on the MOTOR_OUT_PIN
                P1OUT |= BIT3;
                delay_ms(100);
                P1OUT &= ~BIT3;
                delay_ms(100);

            }
        }

       P1OUT |= BIT0;
       delay_ms(1000);
       P1OUT &= ~BIT0;
       delay_ms(1000);
    }
}


#pragma vector = USCI_A0_VECTOR
__interrupt void EUSCI_A0_RX_ISR(void)
{
    char received_char;
    received_char = UCA0RXBUF;
    if (received_char == 10 || received_char == 13)
    {
        command_index = 0;
        RXData[K] = '\0';
        for (command_index = 0; command_index <= K; command_index++)
        {
            command[command_index] = RXData[command_index];
            RXData[command_index] = '\0';
        }
        cmd = &command[0];
        K = 0;
        command_received = 1;
    }
    else
    {
        RXData[K] = received_char;
        K++;
    }

    UCA0IFG &= ~UCRXIFG;
}

void init_UART(int Rx, int Tx)
{
    UCA0CTLW0 |= UCSWRST;           //put A0 into SW reset

    UCA0CTLW0 |= UCSSEL__SMCLK;     //BRCLK = SMCLK (want 115200 baud)
    UCA0BRW = 104;                    // prescalar = 104 => baud rate = 9600
    UCA0MCTLW = 0xD600;             //set modulation & low freq


    //-- setup ports
    if (Rx ==1)
        P1SEL0 |= BIT1;                 //P1.0 set function to UART A0 TX
    else
        P1SEL0 &= ~BIT1;                 //P1.0 set function to UART A0 TX
    if (Tx == 1)
        P1SEL0 |= BIT0;                 //P1.1 set function to UART A0 RX
    else
        P1SEL0 &= ~BIT0;                 //P1.0 set function to UART A0 TX

    UCA0CTLW0 &= ~UCSWRST;                    // Initialize eUSCI
}

int split_by_space(char *str, char *substrings[])
{
    int num_substrings = 0; // Number of substrings found
    char *token; // Pointer to the current substring

    // Split the string into substrings using strtok()
    token = strtok(str, " ");
    while (token != NULL && num_substrings < MAX_SUBSTRINGS)
    {
        substrings[num_substrings] = token;
        num_substrings++;
        token = strtok(NULL, " ");
    }

    return num_substrings;
}

int getRotationDir(void)
{
    float delta_angle = current_angle - target_angle;
    if (abs(delta_angle) < ANGLE_THRESHOLD || abs(delta_angle) > 360 - ANGLE_THRESHOLD)
        return STOP;
    if (delta_angle > 0)
    {
        if (delta_angle < 180)
        {
            return CLOCKWISE;
        }
        else
            return COUNTER_CLOCKWISE;
    }
    else
    {
        if (delta_angle > -180)
        {
            return COUNTER_CLOCKWISE;
        }
        else
            return CLOCKWISE;
    }
}

// Initializes LED on P1.0 and P4.0 for use
void init_GPIO(void)
{
    // P1.3 can be MOTOR_OUT_PIN
    P1DIR |= BIT3;
    P1OUT &= ~BIT0;

    // P1.4 can be the MOTOR_DIR_PIN
    P1DIR |= BIT4;
    P1OUT &= ~BIT4;
}


// Initializes LED on P1.0 and P4.0 for use
void init_LEDs(void)
{

    P1DIR |= BIT0;
    P1OUT |= BIT0;

    P4DIR |= BIT0;
    P4OUT &= ~BIT0;
}

// Initializes button 2 for use
void initButton(void)
{
  P1DIR &= ~BIT2;        // Set P1.1 as input
  P1REN |= BIT2;         // Enable pull-up/down resistor on P1.2
  P1OUT |= BIT2;         // Set pull-up resistor on P1.2
  P1IES |= BIT2;         // Set P1.1 to trigger on falling edge
  //P1IE |= BIT2;          // Enable interrupt on P1.2
}

#pragma vector = PORT1_VECTOR
__interrupt void Button1_2(void)
{
    if (!(P1IN & BIT2))
    {
        P4OUT |= BIT0;
        delay_ms(5000);
        P4OUT &= ~BIT0;
    }
    P1IFG &= ~BIT2; // clear P1.2 interrupt flag
}


// Delay function using ISR
void delay_ms(int ms) {
    // Setup Timer_A
    TA0CTL = TASSEL__ACLK | MC__STOP;  // Use ACLK, stop timer
    TA0CCR0 = 32768 / 1000 * ms;      // Set period to match desired delay
    TA0CCTL0 = CCIE;                  // Enable interrupt

    // Start Timer_A
    TA0CTL |= MC__UP;                 // Start timer in up mode
    __bis_SR_register(LPM3_bits + GIE);  // Enter low-power mode with interrupts enabled
}

// Timer_A interrupt service routine
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer_A_ISR(void) {
    TA0CTL = MC__STOP;  // Stop timer
    __bic_SR_register_on_exit(LPM3_bits);  // Exit low-power mode
}
