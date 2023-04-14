//---------- PREPROCESSOR MACROS ----------
#define SW_BASE 0xFF200040
#define BUTTON_BASE 0xFF200050
#define HEX0_HEX3_BASE 0xFF200020
#define HEX4_HEX5_BASE 0xFF200030
#define I2C0_BASE 0xFFC04000 // Base address of the first I2C controller

#define TIMER_BASE_ADDR 0xFFFEC600
#define TIMER_LOAD (*((volatile int*)(TIMER_BASE_ADDR + 0x00)))
#define TIMER_COUNTER (*((volatile int*)(TIMER_BASE_ADDR + 0x04)))
#define TIMER_CONTROL (*((volatile int*)(TIMER_BASE_ADDR + 0x08)))
#define TIMER_INTERRUPT (*((volatile int*)(TIMER_BASE_ADDR + 0x0C)))

#define JP1_BASE 0xFF200060
#define ULTRASONIC (*((volatile int*)(JP1_BASE + 0x00)))
#define ULTRASONIC_CONTROL (*((volatile int*)(JP1_BASE + 0x04)))

#define ADC_BASE 0xFF204000

//---------- STRUCTURES ----------
typedef struct I2CStruct
{
	// "pad" Variables used to proper align varibles with there proper memory locations
		int control;			//0x00 Control
		int target; 			//0x04 Target
		int slave;				//0x08 Slave
		int pad0; 				//0x0C skip
		int data_cmd;			//0x10 Data Command
		int std_scl_hcnt;		//0x14 Standard Clock High Period
		int std_scl_lcnt;		//0x18 Standard Clock Low Period
		int fast_scl_hcnt;		//0x1C Fast Clock High Period
		int fast_scl_lcnt;		//0x20 Fast Clock Low Period
		int pad1;				//0x24 skip
		int pad2;				//0x28 skip
		int intr_status;		//0x2C
		int intr_mask;			//0x30		
		int raw_intr_status;	//0x34
		int rx_fifo_thr;		//0x38
		int tx_fifo_thr;		//0x3C
		int cmb_intr;			//0x40
		int rx_under_intr;		//0x44
		int rx_over_intr;		//0x48
		int tx_over_intr;		//0x4C
		int intr_read;			//0x50
		int tx_abort_intr;		//0x54
		int rx_done_intr;		//0x58
		int activity_intr;		//0x5C
		int stop_dtct_intr;		//0x60
		int start_dtct_intr;	//0x64
		int gen_call_intr;		//0x68
		int enable;				//0x6C Enable
		int status;				//0x70
		int tx_fifo_lvl;		//0x74 Transmit FIFO
		int rx_fifo_lvl;		//0x78 Receiver FIFO
		int sda_hold;			//0x7C
		int tx_abort_src;		//0x80
		int gen_slave_nack;		//0x84
		int dma_control;		//0x88
		int dma_tx_lvl;			//0x8C
		int rx_data_lvl;		//0x90
		int sda_setup;			//0x94
		int ack_gen_call;		//0x98
		int enable_status;		//0x9C Enable Status
		int ss_fs_supp;			//0xA0
} I2Cn;

//---------- GLOBAL POINTERS ----------
volatile unsigned int *const switchPointer = (unsigned int *)SW_BASE; // Switch pointer
volatile unsigned int *const buttonPointer = (unsigned int *)BUTTON_BASE; // Button pointer
volatile unsigned int *const hexPointer[2] = {HEX0_HEX3_BASE, HEX4_HEX5_BASE}; // Hex pointer array
volatile I2Cn* const I2C0Pointer = (I2Cn*)I2C0_BASE; // First I2C controller pointer

//---------- GLOBAL VARIABLES ----------
// Hex codes for each value 1, 2, 3 .. A, B, C .. F
unsigned char hexCode[16] = {0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7C, 0x7, 0x7F, 0x67, 0x77, 0x7C, 0x39, 0x5E, 0x79, 0x71};

// setDistance, currentDistance, vehicleSpeed
unsigned char displayValues[3] = {0, 0, 0};

unsigned char accelerometerData;

volatile unsigned int *const ADC_ptr = (unsigned int *)ADC_BASE;


/**
 * @brief starts the timer and counts down for a duration of time
 * 
 * @param duration_microseconds // duration of time timer counts down
 */
void timer_init(int duration_microseconds){
    int time_load_value = (duration_microseconds * 200) -1; 
    // max time is 21.47 s
    TIMER_LOAD = time_load_value;
    TIMER_CONTROL |= (1<<0); //turn on timer
    TIMER_CONTROL |= (1<<2); //enable interrupts

} 

/**
 * @brief checks if timer has timed out
 * 
 * @return true 
 * @return false 
 */
int timer_is_timed_out(){
    int interrupt_status = TIMER_INTERRUPT;
    if (interrupt_status == 1){
        return 1;
    }else{
        return 0;
    }
}

/**
 * @brief stops timer
 * 
 */
void timer_stop(){
    TIMER_CONTROL &= ~(1 << 0); //sets first bit to zero, masks by 0x1110
}

/**
 * @brief resets interrupt and stops timer
 * 
 */
void timer_reset_timeout(){
    TIMER_INTERRUPT = 1; //resets interrupt bit (kinda confusing, but you set to 1 not zero to reset)
    timer_stop();
}

/**
 * @brief checks the time passed
 * 
 * @param timeout_period 
 * @return uint32_t 
 */
int timer_time_passed(int timeout_period){
    int current_tick_count = TIMER_COUNTER; //gets tick value in timer
    int elapsed_ticks = (timeout_period/200) - current_tick_count; //converts time in microseconds to ticks, calculates tick delta
    int elapsed_time_us = elapsed_ticks*200;
    return elapsed_time_us;
}

int timer_is_timed_out2(int timeout_period, int timeout_threshold){
    int current_tick_count = TIMER_COUNTER; //gets tick value in timer
    int elapsed_ticks = (timeout_period/200) - current_tick_count; //converts time in microseconds to ticks, calculates tick delta
    int elapsed_time_us = elapsed_ticks*200;
	
	if(elapsed_time_us < timeout_threshold){
		return 1;
	}else{
		return 0;
	}
}

void ultrasonic_init(){
    ULTRASONIC_CONTROL |= (1<<0);
    ULTRASONIC_CONTROL |= (0<<1);
    
}


int calculate_distance(int time_microseconds){
    return time_microseconds * 0.034 /2;
}


void ultrasonic_set_high(){
    ULTRASONIC |=1;
}

void ultrasonic_set_low(){
    ULTRASONIC &= ~(1 << 0);
}

int ultrasonic_read(){
    return (ULTRASONIC & 0b10);
}

//---------- HEX DISPLAY FUNCTION ----------
// Display needed information on the dash (7-seg displays)
void displayHex(){
	
	unsigned char digitDisplay[6] = {0, 0, 0, 0, 0, 0};
	
    unsigned char i, j;
	
	for (i = 0; i < sizeof(digitDisplay) / sizeof(digitDisplay[0]); i++){
		j = i / 2;
		digitDisplay[i] = (displayValues[j] % 10);
		displayValues[j] = displayValues[j] / 10;
	}

    *hexPointer[0] = (((hexCode[digitDisplay[3]] << 8 | hexCode[digitDisplay[2]]) << 8 | hexCode[digitDisplay[1]]) << 8 | hexCode[digitDisplay[0]]);
    *hexPointer[1] = (hexCode[digitDisplay[5]] << 8 | hexCode[digitDisplay[4]]);
}

//---------- READ SWITCH FUNCTION ----------
// Poll for switch (enable/diable cruise control)
int checkSwitches(){
	*switchPointer &= 1;
	return (*switchPointer);
}


//---------- INITIALIZE I2C CONTROLLER ----------
void initI2C(){
    // Disable I2C0 controller
    // Setting Enable Register bit 0 to 0 (disable), bit 1 to 1 (abort transmissions)
	I2C0Pointer->enable = 0b10;

    // Checking if previous command was excecuted
    // Checking if Enable Status Register bit 0 is 0 (disabled) 
	while (I2C0Pointer->enable_status & 0x1){}
	
	// Set Control Register bits as 0b0110 0101 = 0x65 (High Speed, 7 Bit Addressing)
	/*
        0 : (nothing, bit reserved)
	    1 : disable slave
        1 : enable master restarts
        0 : 7-bit addressing mode master
        0 : 7-bit addressing mode slave
        1 : fast mode (with below)
        0 : fast mode (with above)
        1 : master enabled
    */     
    
	I2C0Pointer->control = 0b01100101;

    // Note: If using multiple targets (peripherals) put following code in read/write functions due to needing to specify device
        /* Target Values Set : ADXL345 Accelerometer Documantation

            Calculations:    
                period = 1/400khz = 2.5 us (Fast Mode)
                t_high > 0.6 us
                t_low > 1.3 us
                
                diff = period - (t_high + t_low)
                    = 2.5 - (0.6 + 1.3)
                diff = 0.6 us

                diff must be zero, therefore add 0.6 us to (t_high + t_low) (add 0.3 us to each)
                Therfore:
                        t_high = 0.9 us = 900 ns
                        t_low = 1.6 us = 1600 ns

                        period addresses are units of 10 ns

                        Fast Clock Low Period = 90
                        Fast Clock High Period = 160
        */

        I2C0Pointer->target = 0x53; // I2C, alt address: 0x53 (ALT ADDRESS pin is low)
        I2C0Pointer->fast_scl_hcnt = 90; // Set low period of clock (90 = 900 ns)
        I2C0Pointer->fast_scl_lcnt = 160; // Set high period of clock (160 = 1600 ns)
    
    // Enable I2C0 controller
	// Setting Enable Register bit 0 to 1 (enable), bit 1 to 0 (dont abort transmissions)
	I2C0Pointer->enable = 0b01;

	// Checking if previous command was excecuted
    // Checking if Enable Status Register bit 0 is 1 (I2C0 controller is enabled) 
	while ((I2C0Pointer->enable_status & 0x1) == 0){}
}

//---------- READ BYTE FROM I2C PERIPHERAL ----------
// Read at address "address" passed through
unsigned char readOverI2C(unsigned char address){

    /* Data Command Register
        Used to load the transmit queue
        10 bits in length: 0b0ccc xxxx xxxx
            x's -> Data sent
            c's -> Command Bits
    */

    // To Restart Device: 0b0ccc xxxx xxxx = 0100 0000 0000 = 0x400 (Control Bits = 0x4)
	I2C0Pointer->data_cmd = address + 0x400;
	
	// To Set Read Mode: 0b0ccc xxxx xxxx = 0001 0000 0000 = 0x100 (Control Bits = 0x1)
	I2C0Pointer->data_cmd = 0x100;
    
	// Waiting for the Recieve FIFO Register to have read data
	while (I2C0Pointer->rx_fifo_lvl == 0){}

	// Read byte of data from the Data Command Register
    // Reading from the Data Command Register removes the data from the Recieve FIFO Register
	return I2C0Pointer->data_cmd;
}

//---------- WRITE BYTE FROM I2C PERIPHERAL ----------
// Write to address "address" passed through
void writeOverI2C(unsigned char address, unsigned char value){
	// the data_cmd register is used to load the transmit queue
	// data_cmd uses 10 bits: [0ccc xxxx xxxx]
	//   x's are for data sent to device
	//   c's are command bits 
	// Command bits 0100 (i.e. 0x4) issues a restart before sending data
	I2C0Pointer->data_cmd = address + 0x400;
	// write again to add value to transmit queue to send to device
	I2C0Pointer->data_cmd = value;
	// this is a pretty general way of writing to I2C
}

//---------- INITIALIZE ACCELEROMETER I2C PERIPHERAL ----------
void initAccelerometer(){

    /* ADXL345 Accelerometer Documantation:

        Register 0x31 - DATA_FORMAT:
        |--------------------------------------------------------------|
        |    D7      D6        D5       D4     D3         D2      D1 D0|
        |SELF_TEST   SPI   INT_INVERT   0   FULL_RES    Justify   Range|
        |--------------------------------------------------------------|

        Register 0x2C - BW_RATE:
        |-----------------------------------|
        |D7  D6  D5      D4      D3 D2 D1 D0|
        |0   0   0   LOW_POWER      Rate    |
        |-----------------------------------|

        Register 0x2D - POWER_CTL:
        |----------------------------------------------------|
        |D7  D6   D5        D4         D3        D2     D1 D0|
        |0   0   Link   AUTO_SLEEP   Measure   Sleep   Wakeup|
        |----------------------------------------------------|
    */
	
    // Set full resolusion
    // Register: 0x31, Sending: 0b00001000 = 0x08
	writeOverI2C(0x31, 0x08);
	
    // Configure accelerometer for 200 Hz sampling
    // Register: 0x2C, Sending: 0b00001011 = 0x0B
	writeOverI2C(0x2C, 0x0B);
	
	// Configure accelerometer to start measuring
    // Register: 0x2D, Sending: 0b00001000 = 0x08
	writeOverI2C(0x2D, 0x08);
}


//---------- MAIN FUNCTION ----------
int main(void){
	
    displayHex();

    // Initialize I2C0 controller
    initI2C();

    /* ADXL345 Accelerometer Documantation
        Memory Map:
            Device ID(DEVID): 0x00
            DEVID holds a fixed value of 0xE5
    */
	if (readOverI2C(0x00) != 0xE5 ){
		// Dead loop
		while(1){}
	}

    // Initialize accelerometer with I2C
    initAccelerometer();
	
    int state = 0; //stores state of "state-machine"
    int timed_out = 1; //stores if timer has timed out
    int max_duration = 0xFFFFFFFF; // stores max timer duration
    int max_threshold = 0xFFFFFF;
    int time_of_flight = 0; // stores time of flight for detected distance
    int distance = 0; //stores distance calculation
    ultrasonic_init();
   
	// Main loop to keep program running
	while (1){
		// Second loop to check if switch is on
		if (checkSwitches()){
            // Read from potentiometer
            ADC_ptr[0] = 0x1; //Refresh channel
            // Read current ADC value (channel 0)
            volatile int value = ADC_ptr[0];
            //Only need lowest 12 bits
            value &= 0xFFF;
            // Set vehicle speed
            displayValues[1] = value * 100/4096 * 99/100;

            /*ADXL345 Accelerometer Documantation:

                Register 0x30 - INT_SOURCE:
                |-----------------------------------------------------------------------------------------------|
                |    D7            D6          D5          D4          D3           D2          D1         D0   |
                |DATA_READY   SINGLE_TAP   DOUBLE_TAP   Activity   Inactivity    FREE_FALL   Watermark   Overrun|
                |-----------------------------------------------------------------------------------------------|
            */

            // Register: 0x30, Reading: 0b10000000 = 0x08
            if (readOverI2C(0x30) >= 0x80){
                // Register: 0x32, Reading a byte from DATAX0
                accelerometerData = readOverI2C(0x32);
                displayValues[2] = accelerometerData;
		    }

            if(state == 0){
                    displayValues[1] = 0;
                    // sends trigger signal of ultrasonic
                    if(timer_is_timed_out()==1){
                    //waits for timer to finish 10us count
                        timer_reset_timeout();
                        ultrasonic_set_low();
                        state =1;
                    }else{
                    //sets counter to 10us count
                        ultrasonic_set_high();
                        timer_init(10);
                    }
                    
                }else if(state ==1){
                    displayValues[1] = 11;
                    // waits and listens for echo
                    if(ultrasonic_read() == 1){
                        time_of_flight = timer_time_passed(max_duration);
                        distance = calculate_distance(time_of_flight);
                        displayValues[0] = distance;
                        state = 0;
                    }

                    if(timer_is_timed_out2(max_duration, max_threshold)==1){
                        //no ultrasonic reading, reset
                        timed_out=1;
                        state =0;
                    }

                    if(timed_out==1){
                        //start timer count
                        timer_init(max_duration);
                        timed_out =0;
                    }
                }
            }
			displayHex();
		}
	return 0;
}