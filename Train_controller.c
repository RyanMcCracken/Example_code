/*Ryan McCracken Project 3 controller*/


/*
 Controller User Guide-----------------------------------------
 * Button1 = Tap - Nothing /Hold 1 second - switch to alternate address
 * Button2 = Unused
 * Potentiometer = Speed control, first quarter=reverse (0-max speed(255)), middle half = No speed, last quarter=forward (0-max speed(255))
 * Switch = Train address finder mode(TAFM) (flip switch wait until led3 holds on)/ Speed control mode(SCM) (led1 will flash if in this state)
 * Led1 = TFM - holds on previous value/SCM - defaults low on train1 and blinks according to address (3 blinks = address 3), defaults high on train 2
 * Led2 = TFM - Off = searching for train, On = address found/ SCM = nothing
 * Led3 = Both modes - On = short circuit on rails, Off = nothing
 */


// CONFIG
#pragma config FOSC = INTOSCIO  // Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA4/OSC2/CLKOUT pin, I/O function on RA5/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // MCLR Pin Function Select bit (MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = ON       // Brown Out Detect (BOR enabled)
#pragma config IESO = ON        // Internal External Switchover bit (Internal External Switchover mode is enabled)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is enabled)

#include <xc.h>

#define led1 RA5
#define led2 RA4
#define led3 RC3
#define find_mode RA2

//Define variables

unsigned short int count = 0;

volatile unsigned short int timer1 @0x0E;

unsigned char message1_buffer[5];
unsigned char message2_buffer[5];
unsigned char* write;
unsigned char* send;
volatile bit write_bit = 0;

volatile bit buffer1 = 0;
volatile bit buffer2 = 0;

unsigned char i = 0;
unsigned char k = 0;

unsigned short int Idraw = 0;
unsigned char address_display = 0;
unsigned short int five_hertz = 0;

volatile bit shorted_rails = 0;

volatile bit current_address = 0;

unsigned char direction = 0;
unsigned short int speed = 0;
unsigned char address = 0x03;
unsigned char data = 0;
unsigned char error = 0;

unsigned char sending_data[2][3];

unsigned char scan_address = 0x03;

volatile bit address_found = 0;

unsigned long timer1_clock = 0;

unsigned char bit_num = 0;
unsigned char byte_num = 0;

volatile bit button1 = 0;
volatile bit button2 = 0;

void write_m1(address, data, error)                                                             //writes to message 1 buffer exclusively when searching for trains
{
    for (unsigned char b = 0; b <= 40; b++) {                                                   //for length on message loop round to fill buffer                                       
        if (b <= 12) write_bit = 1;                                                             //preamble
        else if (b <= 13) write_bit = 0;                                                        //packet start
        else if (b <= 21) {                                                                     //address write
            if (((address >> (7 - (b - 14))) & 1) == 1)write_bit = 1;
            else write_bit = 0;
        } else if (b <= 22) write_bit = 0;                                                      //data start
        else if (b <= 30)                                                                       //data write
        {
            if (((data >> (7 - (b - 23))) & 1) == 1)write_bit = 1;
            else write_bit = 0;
        } else if (b <= 31) write_bit = 0;                                                       //error start
        else if (b <= 39) {
            if (((error >> (7 - (b - 32))) & 1) == 1)write_bit = 1;                              //once it finds a one, outputs ones for rest of byte
            else write_bit = 0;
        }

        i = (b / 8);                                                                            //chooses byte value based of the number of bits written
        k = (b - (i * 8));                                                                      //choose the bit value of the current byt based of the bits written

        if (write_bit) {
            message1_buffer[i] |= (1 << k);                                                     //writes a 1 to the kth bit of the ith byte
        } else {
            message1_buffer[i] &= (~(1 << k));                                                  //ands a byte of 1's where the bit to be wrote as zero is set to 1 and the compliment taken
        }
    }
    return;
}

void main() {
    //sets up inputs/outputs
    TRISC = 0x07;               //111000 0b00000111
    TRISA = 0x07;               //111000 0b00000111

    //sets up buttons for interrupt on change
    IOCA = 0x03;                //set up as interrupt on pin change
    RAIE = 1;                   //enable interrupt on pin change
    CMCON0 = 0x07;              //turns off comparators
    RAIF = 0;                   //clear interrupt flag

    //TIMER/Interrupt setup
    PS0 = 0;PS1 = 0;PS2 = 0;    //sets timer to 1:1 rate
    PSA = 1;                    //prescalar assigned to WDT rate
    T0CS = 0;                   //uses internal clock as source
    TMR0 = 0;                   //clear timer
    T0IE = 1;                   //enable timer0 interrupt
    T0IF = 0;                   //clear timer0 overflow interrupt flag
    GIE = 1;                    //enables all interrupts

    //sets up clock speed to 8 MHz with the internal oscillator used
    OSCCON = 0x71;

    T1CON = 0x01;               //setup timer 1

    //adc conversion intialise
    ANSEL = 0x30;               //set pin 10 and 9 as analog inputs
    ADFM = 1;                   //right justified 
    VCFG = 0;                   //Vdd reference bit
    ADON = 1;                   //set adc as operating
    ADCS2 = 0;ADCS1 = 0;ADCS0 = 1;  //select A/d conversion clock, want adc to be multpile of frequency, if Fosc = 4MHz, using rate 2MHz

    //timer 2
    PEIE = 1;                   //allows peripheral interrupts
    TMR2IE = 1;                 //enable timer 2 interrupt
    T2CON = 0x05;               //setup timer 2 on with prescalar of 4            

    //PWM
    CCP1CON = 0b10001100;       //half bridge, active high
    PWM1CON = 0x01;             //determines delay from pwm change

    //sets starting positions of pointers
    write = message1_buffer;    
    send = message2_buffer;

    //set default addresses so I don't have to recalibrate off of 0 (global)
    sending_data[0][0] = 0x03;
    sending_data[1][0] = 0x04;

    while (1) {
        if (find_mode == 0) {                                                               //if switch is set to speed control mode then write data else search addresses
            led2 = 0;                                                                       //reset led3 state after led3 was set high when address finder mode succeeded
            if ((buffer1 == 1) && (buffer2 == 1))                                           //if neither buffers have successfully been wrote wait
            {
                ;
            } else                                                                          //if ready write
            {

                if (buffer1 == 0)                                                           //if buffer1 ready write to buffer 1
                {
                    write = message1_buffer;                                                //sets pointer
                } else if (buffer2 == 0)                                                    //if buffer1 ready write to buffer 1
                {
                    write = message2_buffer;                                                //sets pointer
                }
                for (unsigned char b = 0; b <= 40; b++) {                                   //cycle through length of message to write to each bit
                    //3-byte packet format--------------------------------------------------------------------------------------------------------------------
                    //              |--preamble--|  |address | |--data-|  |-error--|         |--preamble--|   |address|   |--data-|   |-error-|
                    //packet format 1111 1111 1111 0000 0001 1001 1011 1100 1101 1001        1111 1111 1111 0 0000 0011 0 0110 1111 0 0110 1100 1
                    //                 packet start^      data^     error^       end^           packet start^      data^       error^        end^
                    //error = XOR of address and data---------------------------------------------------------------------------------------------------------
                    if (b <= 12) write_bit = 1;                                             //preamble
                    else if (b <= 13) write_bit = 0;                                        //packet start
                    else if (b <= 21) {                                                     //address write
                        if (((sending_data[current_address][0] >> (7 - (b - 14))) & 1) == 1)write_bit = 1;      //sending data[0=train1,1=train2][0=address,1=data,2=error]
                        else write_bit = 0;
                    } else if (b <= 22) write_bit = 0;                                      //data start
                    else if (b <= 30)                                                       //data write
                    {
                        if (((sending_data[current_address][1] >> (7 - (b - 23))) & 1) == 1)write_bit = 1;      //works by checking the value of the bit and setting write_bit to the value
                        else write_bit = 0;
                    }
                    else if (b <= 31) write_bit = 0;                                        //error start
                    else if (b <= 39) {
                        if (((sending_data[current_address][2] >> (7 - (b - 32))) & 1) == 1)write_bit = 1;
                        else write_bit = 0;
                    }

                    i = (b / 8);                                                            //determines byte to write to based of message position
                    k = (b - (i * 8));                                                      //determines bit to write to based of message position

                    if (write_bit) {                                                        //checks the value to be wrote to the buffer 
                        write[i] |= (1 << k);                                               //writes a 1 to the kth bit of the ith byte
                    } else {
                        write[i] &= (~(1 << k));                                            //shifts a 1 to the bit to be written, takes the compliment, ands with buffer.
                    }
                }
                if (write == message1_buffer) {                                             //tells interrupt that buffer 1 has been written and thats its ready to start writing another buffer
                    buffer1 = 1;
                }
                if (write == message2_buffer) {                                             //tells interrupt that buffer 1 has been written and thats its ready to start writing another buffer
                    buffer2 = 1;
                }
                CHS2 = 1;CHS1 = 0;CHS0 = 1;                                                 //sets adc to read the potentiometer
                asm("NOP;");
                asm("NOP;");
                asm("NOP;");
                asm("NOP;");
                asm("NOP;");                                                                //waits for it to be set
                asm("NOP;");
                asm("NOP;");
                asm("NOP;");
                asm("NOP;");
                GO_nDONE = 1;                                                               //starts reading
                while (GO_nDONE) {                                                          //wait for reading to be done
                    ;
                } 
                speed = ADRESL | (ADRESH << 8);                                             //save speed reading

                if (speed <= 255) {                                                         //if speed in first quarter                 
                    direction = 0;                                                          //set direction reverse
                    speed = (255 - speed);                                                  //set speed to vary from 0-255 based of distance from center
                } else if (speed >= 769)                                                    //if in top quarter
                {
                    direction = 1;                                                          //set direction forward
                    speed = (speed - 769);                                                  //set speed to vary from 0-255 based of distance from center
                } else {                                                                    //else must be in middle half 
                    speed = 0;                                                              //set speed to zero
                }
                
                //to write to data sending_data[current_address][1]
                sending_data[current_address][1] = 0b01000000;                              //writes 01 denoting that this packet is a speed/direction instruction
                if (direction != 0) {                                                       //if direction is 1 then write to the 3rd MSB
                    sending_data[current_address][1] |= 0b00100000;
                } else {                                                                    //if direction is 0 then write to the 3rd MSB  
                    sending_data[current_address][1] &= 0b11011111;
                }
                sending_data[current_address][1] &= 0b11110000;                             //clears any leftover speed settings
                sending_data[current_address][1] |= (speed & 0x0F);                         //writes the speed (0-255) into the 4 LSB
                sending_data[current_address][2] = sending_data[current_address][1];        //error = data
                sending_data[current_address][2] ^= sending_data[current_address][0];       //error XOR address, this is the equivalent of error = data XOR address

                //shorted rails check
                //if shorted turn off H-bridge and hold led3 on, check if no longer short circuited, if still short circuited wait and try again 
                CHS2 = 1;CHS1 = 0;CHS0 = 0;                                                 //sets adc to read current check resistor
                asm("NOP;");
                asm("NOP;");
                asm("NOP;");
                asm("NOP;");
                asm("NOP;");                                                                //waits for it to be set
                asm("NOP;");
                asm("NOP;");
                asm("NOP;");
                asm("NOP;");
                GO_nDONE = 1;                                                               //starts reading
                while (GO_nDONE) {                                                          //wait for reading to be done
                    ;
                }
                Idraw = ADRESL | (ADRESH << 8);                                             //save current measurement

                //if rails are short circuited then stop H-Bridge and periodically return function to test if short is fixed
                if (Idraw > 250)                                                            //if current over threshold denoting a short circuit                                        
                {
                    led3 = 1;                                                               //turn led3 on telling user about short circuit
                    shorted_rails = 1;                                                      //flag that rails are shorted
                    while (shorted_rails == 1) {                                            //repeat until rails are no longer shorted
                        TRISC4 = 1;TRISC5 = 1;                                              //turn RC4 and 5 to inputs to stop the H-bridge and therefore stop current draw
                        timer1_clock = 0;                                                   //reset timer1 counter
                        while (timer1_clock < 10000) {                                      //wait a delay, allowing users to fix short circuit
                            ;
                        }
                        TRISC4 = 0;TRISC5 = 0;                                              //turn RC4 and 5 back to outputs resuming normal H-bridge functionality
                        CHS2 = 1;CHS1 = 0;CHS0 = 0;                                         //sets adc to read current check resistor
                        asm("NOP;");
                        asm("NOP;");
                        asm("NOP;");
                        asm("NOP;");
                        asm("NOP;");                                                        //waits for it to be set
                        asm("NOP;");
                        asm("NOP;");
                        asm("NOP;");
                        asm("NOP;");
                        GO_nDONE = 1;                                                       //starts reading
                        while (GO_nDONE) {                                                  //wait for reading to be done
                            ;
                        }
                        Idraw = ADRESL | (ADRESH << 8);                                     //save current measurement

                        if (Idraw < 250)shorted_rails = 0;                                  //if no longer short circuited exit 
                    }
                    led3 = 0;                                                               //signal short has been fixed
                }
            }
        } else {                                                                            //if in find mode
            //find mode uses a sub routine write_m1 to test address, as a result used address/data/error are labeled quite literally
            address = 0;                                                                    //sets address as global to stop all trains and therefore any current draw
            data = 0b01100001;                                                              //sets data to an e-stop
            error = data;                                                                   //sets error as data for xor function
            error ^= address;                                                               //XOR address with error, making error = address XOR data
            write_m1(address, data, error);                                                 //calls sub routine to write e-stop at global address

            CHS2 = 1;CHS1 = 0;CHS0 = 0;                                                     //sets adc to read current check resistor
            asm("NOP;");
            asm("NOP;");
            asm("NOP;");
            asm("NOP;");
            asm("NOP;");                                                                    //waits for it to be set
            asm("NOP;");
            asm("NOP;");
            asm("NOP;");
            asm("NOP;");
            GO_nDONE = 1;                                                                   //starts reading
            while (GO_nDONE) {                                                              //wait for reading to be done
                ;
            }
            Idraw = ADRESL | (ADRESH << 8);                                                 //save current measurement

            while (Idraw > 144)                                                             //wait until no current across rails, in case trains have super capacitors that are still charging
            {
                CHS2 = 1;CHS1 = 0;CHS0 = 0;                                                 //sets adc to read current check resistor
                asm("NOP;");
                asm("NOP;");
                asm("NOP;");
                asm("NOP;");
                asm("NOP;");                                                                //waits for it to be set
                asm("NOP;");
                asm("NOP;");
                asm("NOP;");
                asm("NOP;");
                GO_nDONE = 1;                                                               //starts reading
                while (GO_nDONE) {                                                          //wait for reading to be done
                    ;
                }
                Idraw = ADRESL | (ADRESH << 8);                                             //save current measurement 
            }

            //once capacitors are charged and there is no current draw begin searching through addresses
            address_found = 0;                                                              //sets flag saying address not found
            scan_address = 0x03;                                                            //sets starting address
            while (address_found == 0) {                                                    //while address not found keep scanning
                if (scan_address == sending_data[(current_address + 1) % 2][0]) scan_address++; //skip over already used address as to not write to same address twice. (current_address + 1) % 2 sets 1 to 0 and 0 to 1

                address = scan_address;                                                     //sets address to the address currently being searched, (using variable allows us to write over address for e-stop later)
                data = 0b01111110;                                                          //motor to full speed forward
                error = data;                                                               //sets error as data for xor function
                error ^= address;                                                           //XOR address with error, making error = address XOR data
                write_m1(address, data, error);                                             //calls sub routine to write e-stop at global address

                timer1_clock = 0;                                                           //resets timer 1 clock
                while ((timer1_clock < 4000) | (Idraw > 144)) {                             //continue to sample the current for given sample time or until current exceeds threshold
                    CHS2 = 1;CHS1 = 0;CHS0 = 0;                                             //sets adc to read current check resistor
                    asm("NOP;");
                    asm("NOP;");
                    asm("NOP;");
                    asm("NOP;");
                    asm("NOP;");                                                            //waits for it to be set
                    asm("NOP;");
                    asm("NOP;");
                    asm("NOP;");
                    asm("NOP;");
                    GO_nDONE = 1;                                                           //starts reading
                    while (GO_nDONE) {                                                      //wait for reading to be done
                        ;
                    }
                    Idraw = ADRESL | (ADRESH << 8);                                         //save current measurement
                    if (Idraw >= 144) {                                                     //if current exceeds threshold, threshold is chosen specifically as value greater than current draw with no load while still being small enough to pick up a single train
                        address_found = 1;                                                  //flag that there is a train there
                    }
                }
                address = 0;                                                                //global
                data = 0b01100001;                                                          //e stop
                error = data;
                error ^= address;                                                           //creates error byte
                write_m1(address, data, error);                                             //writes e-stop to global address once testing has been complete

                if (scan_address > 0x05)scan_address = 0x03;                                //reset testing address if over limit
                else if (address_found == 0) scan_address++;                                //increment if address hasn't been found
            }
            sending_data[current_address][0] = scan_address;                                //save the successful address as the new address for the current train
            while (find_mode == 1) {                                                        //while still switched to finding mode
                led2 = 1;                                                                   //signal led2 to say that address has been found
                //short circuit check----------------------------------------------------------------------------------------------------------
                //since the code could technically be stuck here indefinitely (if the user doesn't flip back the switch) it is important to check that no short occurs here
                //as a result this code is identical to the previous short circuit check
                CHS2 = 1;CHS1 = 0;CHS0 = 0;                                                 //sets adc to read current check resistor
                asm("NOP;");
                asm("NOP;");
                asm("NOP;");
                asm("NOP;");
                asm("NOP;");                                                                //waits for it to be set
                asm("NOP;");
                asm("NOP;");
                asm("NOP;");
                asm("NOP;");
                GO_nDONE = 1;                                                               //starts reading
                while (GO_nDONE) {                                                          //wait for reading to be done
                    ;
                }
                Idraw = ADRESL | (ADRESH << 8);                                             //save current measurement

                //if rails are short circuited then stop H-Bridge and periodically return function to test if short is fixed
                if (Idraw > 250)                                                            //if current over threshold denoting a short circuit                                        
                {
                    led3 = 1;                                                               //turn led3 on telling user about short circuit
                    shorted_rails = 1;                                                      //flag that rails are shorted
                    while (shorted_rails == 1) {                                            //repeat until rails are no longer shorted
                        TRISC4 = 1;TRISC5 = 1;                                              //turn RC4 and 5 to inputs to stop the H-bridge and therefore stop current draw
                        timer1_clock = 0;                                                   //reset timer1 counter
                        while (timer1_clock < 10000) {                                      //wait a delay, allowing users to fix short circuit
                            ;
                        }
                        TRISC4 = 0;TRISC5 = 0;                                              //turn RC4 and 5 back to outputs resuming normal H-bridge functionality
                        CHS2 = 1;CHS1 = 0;CHS0 = 0;                                         //sets adc to read current check resistor
                        asm("NOP;");
                        asm("NOP;");
                        asm("NOP;");
                        asm("NOP;");
                        asm("NOP;");                                                        //waits for it to be set
                        asm("NOP;");
                        asm("NOP;");
                        asm("NOP;");
                        asm("NOP;");
                        GO_nDONE = 1;                                                       //starts reading
                        while (GO_nDONE) {                                                  //wait for reading to be done
                            ;
                        }
                        Idraw = ADRESL | (ADRESH << 8);                                     //save current measurement

                        if (Idraw < 250)shorted_rails = 0;                                  //if no longer short circuited exit 
                    }
                    led3 = 0;                                                               //signal short has been fixed
                }
            }
        }
    }
}

//Interrupt handler
void interrupt isr(void) {
    //interrupt exclusive variable definitons
    static unsigned char next_pr2;
    static unsigned char wptr = 0;
    
    //timer2 interrupt, occurs every time a PWM cycle finishes
    if (TMR2IF) {
        PR2 = next_pr2;                                                                     //sets the duty cycle for the current bit,
        //when writing a bit the new CCPR1L value will automatically delay itself until the PWM cycle has completed while the PR2 will change immediately
        //to work around this the CCPR1L is set and the next PR2 is saved and set on the next interrupt resulting in the CCPR1L and PR2 values aligning
        if (find_mode == 0)                                                                 //if in SCM use both buffers
        {
            wptr += 1;                                                                      //increments bit pointer
            if (wptr >= 39) {                                                               // sent last bit from the last buffer write
                wptr = 0;                                                                   // reset bit pointer
                if (buffer1 == 1 && buffer2 == 1) {                                         // ISR owns both buffers, so can switch
                    if (send == message1_buffer) {                                          // we just sent buffer1...
                        send = message2_buffer;                                             // point at other buffer
                        buffer1 = 0;                                                        // let go of buffer1
                    } else {                                                                // just sent buffer2...
                        send = message1_buffer;                                             // point at buffer1
                        buffer2 = 0;                                                        // release buffer 2
                    }
                }
            }
            // line up next bit to be sent
            if ((send[wptr >> 3]>>(wptr & 7))&1) {                                          // pick the wptr'th bit of send buffer, is it 1?
                CCPR1L = (56 / 2);      // set 58us mark time
                next_pr2 = 56;          // set 116us space time
            } else {                                                                        // it is a 0
                CCPR1L = 56;            // 116us mark
                next_pr2 = 112;         // 232us space
            }
        } else                                                                              //TFM use only buffer 1
        {
            wptr += 1;
            if (wptr >= 39) {                                                               // sent last bit from that last buffer write
                wptr = 0;                                                                   // reset write pointer
            }
                                                                                            // line up next bit to be sent
            if ((message1_buffer[wptr >> 3]>>(wptr & 7))&1) {                               // pick the wptr'th bit of send buffer, is it 1?
                CCPR1L = (56 / 2);      // set 58us mark time
                next_pr2 = 56;          // set 116us space time
            } else {                                                                        // it is a 0
                CCPR1L = 56;            // 116us mark
                next_pr2 = 112;         // 232us space
            }
        }
        TMR2IF = 0;                                                                         //clear interrupt flag
        return;                                                                             //leave interrupt
    }

    //if change on pin interrupt
    if (RAIF) {
        if (RA0 == 0) {                                                                     //if button 1 was pressed
            button1 = 1;                                                                    //flag that button1 is pressed                                                                     
        } else {                                                                            //if button1 is not pressed
            button1 = 0;                                                                    //clear flag
        }
        
        if (RA1 == 0) {                                                                     //if button 2 pressed
            asm("nop;");                                                                    //currently no function, NOP required or else code bugs
        }
        RAIF = 0;                                                                           //clear interrupt flag
        return;                                                                             //end interrupt
    }
    
    //timer0 interrupt 
    if (T0IF) {                                                                             //if timer0 interrupt
        timer1_clock++;                                                                     //increment timer1_clock for time keeping functions

        if (five_hertz < 2000)five_hertz++;                                                 //increments a five hertz counter
        else {                                                                              //occurs at a little under five hertz    
            //flash address on led1
            if (find_mode == 0)                                                             //if in SCM communicate address using led 1                                                      
            {
                if (address_display <= (sending_data[current_address][0] * 2)) {            //for the address value * 2 toggle led1 
                    if (led1) led1 = 0;
                    else led1 = 1;
                }
                if (address_display < 15)address_display++;                                 //increments timer to allow for pause after address flashes)
                else address_display = 1;                                                   //resets timer (setting to zero results in uneven blinks)
            }
            five_hertz = 0;                                                                 //resets five hertz counter
        }
        if (button1) {                                                                      //if button1 has been pressed start counting
            if (count <= 4000) count++;
        } else {                                                                            //if button1 is released check counter value
            if (count >= 4000) {                                                            //if over threshold perform hold function
                if (led1)led1 = 0;                                                          //toggles led1 to show which train is being written to
                else led1 = 1;
                if (current_address) current_address = 0;                                   //toggle which train is being written to
                else current_address = 1;
            }
            count = 0;                                                                      //resets counter for button1 held
        }
        T0IF = 0;                                                                           //clear timer 0 interrupt flag
        return;                                                                             //end interrupt
    }
    // should never get here
    while (1) {                                                                             //if unhandled interrupt occurs led2 will toggle rapidly.
        if (led2 == 1)led2 = 0; 
        else led2 = 1;
    }
}