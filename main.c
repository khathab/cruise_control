#define SW_BASE 0xFF200040
#define BUTTON_BASE 0xFF200050
#define HEX0_HEX3_BASE 0xFF200020
#define HEX4_HEX5_BASE 0xFF200030
#define ADC_BASE 0xFF204000

volatile unsigned int *const switchPointer = (unsigned int *)SW_BASE;
volatile unsigned int *const buttonPointer = (unsigned int *)BUTTON_BASE;
volatile unsigned int *const hexPointer[2] = {HEX0_HEX3_BASE, HEX4_HEX5_BASE};
volatile unsigned int *const ADC_ptr = (unsigned int *)ADC_BASE;

unsigned char hexCode[16] = {0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7C, 0x7, 0x7F, 0x67, 0x77, 0x7C, 0x39, 0x5E, 0x79, 0x71};

// setDistance, currentDistance, vehicleSpeed
unsigned char displayValues[3] = {0, 0, 0};

// Function to display needed information on the dash (7-seg displays)
void DisplayHex(){
	
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

// Function to poll for switch (enable/diable cruise control)
int CheckSwitches(){
	*switchPointer &= 1;
	return (*switchPointer);
}

int acc(){
	int speed = 5*2;
	displayValues[2] = speed;
}

int main(void){
    DisplayHex();
	while (1){
		// Second loop to check if switch is on
		while (CheckSwitches()){
			displayValues[1] = 20;
			displayValues[2] = 60;

            // Read from potentiometer
            ADC_ptr[0] = 0x1; //Refresh channel
            // Read current ADC value (channel 0)
            volatile int value = ADC_ptr[0];
            //Only need lowest 12 bits
            value &= 0xFFF;
            // Set vehicle speed
			displayValues[0] = value * 100/4096 * 99/100;

			DisplayHex();
		}
	}
}