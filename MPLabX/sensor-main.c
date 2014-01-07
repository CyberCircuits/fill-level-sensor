/*
 * Bottling line Fill Level Detection.
 * -----------------------------------
 * 06 DEC 2013 Roving Dynamics Ltd (rodyne.com)
 * Free for unrestricted use under Lessor GPL Licence.
 */

#include <plib.h>

#define SystemClock        50000000L  // 50Mhz
#define DELAY_FACTOR_50MHZ 25         // 50Mhz/2million

#pragma config FSOSCEN = OFF  // See Help C32 configuration bits
#pragma config OSCIOFNC = OFF
#pragma config JTAGEN = OFF
#pragma config FPLLIDIV = DIV_2, FPLLMUL = MUL_20, FPLLODIV = DIV_1, FWDTEN = OFF //external 5Mhz XTAL   /  2  * 20  / 2  == 50Mhz clock
#pragma config POSCMOD = HS, FNOSC = PRIPLL, FPBDIV = DIV_2

#define TRUE		1
#define FALSE		0
#define ON			1			// Logical 1 (3.3V)
#define OFF			0			// Logical 0 (0V)
#define uint8   unsigned char
#define uint16  unsigned short
#define uint32  unsigned int

// Non-Volatile Memory does not exist on PIC32 but we can use spare flash at end, and can erase flash up to 1000 times before chip is toast!
#define NVM_END					(void*) (0xBD000000 + BMXPFMSZ) // End Address of PIC321xx FLASH Memory
#define NVM_START		    (void*) (NVM_END - 0x400)	       // Start address of last page is 1K back (Note 1024 page size on PIC32MX1xx Different for bigger pic32's )

// PIC32 Pin definitions and Macros - Note DONT USE RA4 until I figure out how to bloody configure it as digital IO! (See paragmas)
#define ANA								0												// Pin 2  Analog in from TSL1401
#define CLK(state)				LATBbits.LATB0 = state	// Pin 4  Digital Output
#define SI(state)					LATBbits.LATB1 = state	// Pin 5  Digital Output
#define OC_REJECT(state)	LATBbits.LATB6 = state  // Pin 16 Digital Output
#define OC_COUNT(state)		LATBbits.LATB5 = state	// Pin 14 Digital Output (was laser)
#define BACKLIGHT(state)	LATBbits.LATB12 = state // Pin 23 Digital Output
#define LED(state)				LATBbits.LATB8 = state	// Pin 17 Digital Output
#define LED_REJECT(state)	LATBbits.LATB9 = state	// Pin 18 Digital Output
#define BUZZER(state)			LATBbits.LATB3 = state	// Pin 7 Digital Output

// Definitions and variables specific to fill detection

#define MAXBRIGHTNESS   254			// TSL1401 has analog out, we do 10 bit ADC but we only need 8-bits so values above 254 are clipped.
#define MAXLINESPEED		500			// Fastest speed of bottling line in mm/Second
#define BOTTLENECKWIDTH 24			// width of a bottle neck in mm
#define BOTTLEGAP				130			// Gap between bottle necks in mm
#define MINGAP					25			// Minimum Gap width (in samples) .. note we cannot really have equivelent for max gap as there may be bottles missing
#define MAXSAMPLES			300     // Maximum samples we ever want in our accumulators before we consider it junk bottle neck passing at slowest line speed
#define NORMFILL				63			// TSL1401 has 128 pixels (0..127), we will manually position the sensor so the fill level is centre of the range
#define MARGIN					8				// Default inner margin for processing algorithm
#define BAUDRATE				115200  // Serial2 baud rate

uint8   CalData[128][32];				// used during calibration only
uint8   RawData[128];  			  	// Buffer for raw analog values from TSL1401
uint16  CalAvg[32];
uint32	AccumulatedData[128];  	// we average horizontal pixel band across bottle as it travels past sensor and put accumulated totals here (used with array below)
uint32	FinalData[128];					// to detect trailing edge of bottle we look for all pixels overexposed, however clear glass and empty bottles
																// may give false readings so once we get to an overexposed area we stop accumulating in this array so we can
																// go back to it if it proves to be the real deal, if not we copy the accumulated data over and wait for next
																// if proves to be the real edge (ie accumulator[] overexposed for samplecount x 2 we use this and average

uint16  SampleCount;					  // Number of samples in Accumulator
uint16  FinalSampleCount;			  // Number of samples in SavedAccumulator (where we stopped accumulating in anticipation of trailing edge detection)
uint8   SerialConnected=0;			// set when system received a byte
uint8   AccumulatorSaved;				// Flags accumulator above has been saved so we dont accidently overwrite the data with bad
uint8   min,max,avg;						// used in calibration showing how many passes we do to set calibration, min and max point calcs etc
uint8   CalGapPos;							// used in calibration to show which pos in CalData[] had the minimum sample exposure (so we know if the image data has bottle in front or not)
uint8   CalBotPos;							// used in calibration to show which pos in CalData[] had the maximum sample exposure (so we know if the image data has bottle in front or not)
uint8   AvgLevel[64];           // use this array to keep last 64 (good) levels
uint8   AvgIndex;								// which slot in array currently active
uint16  AvgDetFill;	  					// Find real average fill level (add up array and divide by 64)
uint32  BottleCount	=	0;
uint32  RejectCount	=	0;
uint32  ticker = 0;							// General timeout for bottling line movements (>10000 = line not moving)
uint16  exposure;								// Default exposure, how long the backlight is left on during the sample rate (eg 2000 = 2mS) min should be to easilly overexpose when no bottle in front
uint8   NVMChanged  = 0;				// If we overwrite the default NVM values then flag change to allow writing to happen (we dont write if nothing changed!)
uint32  *NVMPtr;
uint8   DetectedFillLevel;
uint8   MaxDownSlope,MinUpSlope;

// Non Volatile Memory: variables below can be overwritten at run time, We use last page of flash memory for this (NVM_START and NVM_END)
uint8   Tmin					= 12;   // Minimum samples we want in our accumulators before we consider it a representitive of a bottle neck passing at fastest line speed
uint8 	algorithm			= 75;   // Specifies Which algorithm to use to detect the fill level (allows a bit of in the field tweeking)
uint8   cap						= 36;   // Specifies the start of our processing area (everything above this is ignored in detection algorithm)
uint8   neck					= 100;  // Specifies the end of our processing area (everything below this is ignored in detection algorithm)
uint16  SampleRate		= 4000; // Sampling rate (eg 4000 = 4mS = 250 images/second or 1 sample every 2mm when bottling line moving at 500mm/Second )
uint16  RejOnTime			= 500;  // How long OC Reject signal stays ON in mS
uint8   tollerance		= 20;	  // Default tollerance (pixels above or below mid point of sensor, 63 is the mid-point of the sensor)

volatile uint16		BuzzerTimeout,RejTimeout; // counters used for open collector reject pin and buzzer volatile as changed in interrupt
volatile uint8		RxDataIn,RxDataOut;				// indexes for char read in from uART and char read out by user set by interrupts so make volatile
volatile uint8		RxBuffer[32];							// uart rx circular buffer


void DelayMicroseconds(uint32 uS) // Blocking wait of 1uS - Dont use core timer as it very occasionally overflows and can cause timing error
{
  uint32 z;
  for(z=0;z<uS;z++)
  {
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
  }
}

uint8 Serial2Available(void)
{
  return RxDataIn-RxDataOut; // return num of chars waiting in buffer pr zero if no chars waiting
}

uint8 Serial2Read(void) // Reads buffer. No checks here, you must use SerialDataAvailable before calling this...
{
  uint8 c;

  if(RxDataOut<RxDataIn) // check we actually have data first (if rxDataIn>buf we have overrun but we ignore this)
  {
    c = RxBuffer[RxDataOut++];
    if(RxDataOut==RxDataIn) // once up to date reset the buffer indexes back to the begining
    {
      RxDataOut=0;
      RxDataIn=0;
    }
    return c;
  }
  else
    return 0; // no data WTF, should have used serialDataAvaiable first!
}

int AnalogRead(char analogPIN)  // PORT A analog reads
{
  AD1CHS = analogPIN << 16;     // AD1CHS<16:19> controls which analog pin goes to the ADC
  AD1CON1bits.SAMP = 1;         // Begin sampling
  while( AD1CON1bits.SAMP );    // wait until acquisition is done
  while( ! AD1CON1bits.DONE );  // wait until conversion done
  return ADC1BUF0;              // result stored in ADC1BUF0
}

void Beep(uint16 mS) // timer1 interrupt (called 4000 times a second) and set buzzer off for amount of time indicated and return. (non-blocking)
{
  if(BuzzerTimeout==0) // only do this if buzzer/timer1 not already active
  {
    if(mS>3000) mS=3000;
    BuzzerTimeout = 4*mS;
  }
}

void Reject() // timer1 interrupt is called 4000 times a second. engages reject signal for amount of time indicated. (non-blocking)
{
  RejectCount++;
	Beep(100);
  if(RejTimeout==0)  RejTimeout = 4*RejOnTime;
}

void ClearFinalData() // Clear sample saved accumulator
{
	uint8 i;
	for(i=0; i<128; i++)
		FinalData[i]=0;
	FinalSampleCount=0;
	AccumulatorSaved=FALSE;
}

void ClearAll() // Clear all accumulators and turn off outputs
{
	uint8 i;
	for(i=0; i<128; i++)
		AccumulatedData[i]=0;
	SampleCount=0;
	ClearFinalData();
	OC_COUNT(OFF);
	LED(OFF);
}

void AquireImage() // Read TSL1401 chip into sample array
{
  uint8  i;
	uint16 d;
	asm("di");  // timing critical area, disable any interrupts!
	BACKLIGHT(ON);
	// 1. clock out crap as fast as possible (see TSL1401 documentation or parallax TLS1401 developer board is better) 2uS x 128 = 256uS
	SI(ON);
	for(i=0;i<129;i++)
	{
		CLK(ON);
		DelayMicroseconds(1);
		SI(OFF);
		CLK(OFF);
		DelayMicroseconds(1);
		if(i>exposure) BACKLIGHT(OFF);
	}
	// 2. Wait for exposure to finish if its greater than 128
  if(exposure>128) DelayMicroseconds(exposure-128);
	BACKLIGHT(OFF); // turn off the backlight
	// 3. Second pass, clock out the real data into raw buffer for processing and also average samples and put in sample array
	SI(ON);
	asm("ei");
	for(i=127; i>0; i--) // sensor is inverted 180 deg because of lens so top is bottom and bottom is top so do this backwards or turn chip around on pcb!
	{
		CLK(ON);
		DelayMicroseconds(1);
		SI(OFF);
		d = AnalogRead(ANA);
		if(d>MAXBRIGHTNESS)	RawData[i]=MAXBRIGHTNESS; else RawData[i]=d;  // 255 is Max value we can get from TSL1401, but we have 10 bit ADC in PIC32 so we need to clip
	  if(SampleCount<MAXSAMPLES && SampleCount>2) AccumulatedData[i] = AccumulatedData[i] + RawData[i]; // accumulate total light across width of bottle so we can get an avg
		CLK(OFF);
		DelayMicroseconds(1);
	}
	if(SampleCount<MAXSAMPLES) SampleCount++; // increment how many samples in accumulator
	CLK(ON);
	DelayMicroseconds(1);
	CLK(OFF);
	if(SampleRate - 1500 > exposure)
	  DelayMicroseconds(SampleRate - 1500 - exposure); // hard-code delay so we get correct sample rate
}

void Serial2Write(uint8 data) // write to UART directly and wait to finish (hangs execution until finished)
{
	if(SerialConnected) // dont write anything to serial until connection made from client
	{
    UARTSendDataByte(UART2, data);
    while(!UARTTransmissionHasCompleted(UART2));
	}
}

void SendCounters()
{
	Serial2Write(6);
	Serial2Write(99);
	Serial2Write(AvgDetFill&0xff);
  Serial2Write(MaxDownSlope);
	Serial2Write(SampleCount);
	Serial2Write(DetectedFillLevel);
	Serial2Write(exposure>>8);
	Serial2Write(exposure);
	Serial2Write(BottleCount>>16);
	Serial2Write(BottleCount>>8);
	Serial2Write(BottleCount);
	Serial2Write(RejectCount>>8);
	Serial2Write(RejectCount);
}

void DoSerial() // Serial Processing of any input data
{
	char c;
	uint8 i;
	if(Serial2Available())
	{
		SerialConnected=1;	// Client has connected so we can now send stuff.. (Set until POR as we dont have the equivelent for a disconnect)
		c = Serial2Read();
		if(c==3)	// 3. Raw image test (focus setup etc)
		{
			while(c!=1 && c!=9)
			{
				OC_COUNT(ON);
				AquireImage();
				Serial2Write(3);
				Serial2Write(99);
				for(i=0; i<128; i++)
					Serial2Write(RawData[i]);
				Serial2Write(99);
				DelayMicroseconds(100000);
				if(Serial2Available()) c = Serial2Read();
			}
		}
		if(c==4) // 4. end processing
		{
			PowerSaveSleep(); // END!
			while(1);
		}
		if(c==5)	// 5. Send fixed (NVM) settings
		{
			Serial2Write(5);
    	Serial2Write(99);
			Serial2Write(Tmin);
			Serial2Write(algorithm);
			Serial2Write(cap);
			Serial2Write(neck);
			Serial2Write(SampleRate>>8);
			Serial2Write(SampleRate);
			Serial2Write(RejOnTime>>8);
			Serial2Write(RejOnTime);
			Serial2Write(tollerance);
		}
		if(c==6) SendCounters();// 6. Send current settings
		if(c==7)	// 7. Receive new settings temporary until reset or permanent if written to NVM
		{
			Tmin=Serial2Read();
			algorithm=Serial2Read();
			cap=Serial2Read();
			neck=Serial2Read();
			SampleRate=Serial2Read()<<8 + Serial2Read();
			RejOnTime=Serial2Read()<<8 + Serial2Read();
			tollerance=Serial2Read();
			NVMChanged=1;
			Serial2Write(7);
		}
		if(c==8 && NVMChanged)  // 8. Save to NVM
		{
			if( (void*)(NVMPtr+8) >= NVM_END)  // used all the flash page so we have to erase whole page and start again (will never happen seriously!)
			{
				NVMErasePage(NVM_START);
				NVMPtr = NVM_START;
			}
			// Each variable is stored as a 32 bit integer which is the minimum size programable to flash NVMPtr will always be set at next free FLASH slot after POR or a write
			NVMWriteWord( (void*) (NVMPtr++) , Tmin );
			NVMWriteWord( (void*) (NVMPtr++) , algorithm );
			NVMWriteWord( (void*) (NVMPtr++) , cap );
			NVMWriteWord( (void*) (NVMPtr++) , neck );
			NVMWriteWord( (void*) (NVMPtr++) , SampleRate );
			NVMWriteWord( (void*) (NVMPtr++) , RejOnTime);
			NVMWriteWord( (void*) (NVMPtr++) , tollerance);
			NVMChanged = 0; // All good so unflag change
			Serial2Write(8);
		}
		if(c==9) SoftReset();		// 9. RESET Sensor!
	}
}

uint8 DoPassFail(uint8 calibrating) // find dark band to get fill-level, remember there may NOT be one if level outside sensor and foam will confuse reading
{
	uint8 y,y1,SampleMaxUpSlope;

	// mode=0 is calibration, mode!=0 is run

	MaxDownSlope = 0;
	max=0;
	min=0xff;
	// Average image data, find min and max inside area of interest between cap and neck and re-scale for better contrast and easier to view
 	for(y=0; y<128; y++)
		if(y>=cap && y<=neck)
		{
			FinalData[y]=FinalData[y]/FinalSampleCount; // average (FinalSampleCount>0 always)
			if(FinalData[y]>max) max=FinalData[y]; // find max
			if(FinalData[y]<min) min=FinalData[y]; // find min
		}

	// smooth data either side of cap and neck leaving what we are interested in and find max and min for scaling
	for(y=0; y<128; y++)
	{
		if(y<cap) FinalData[y]=FinalData[cap];
		if(y>neck) FinalData[y]=FinalData[neck];
	}
	
	// Re-scale the remaining image data from the new floor up to MAXBRIGHTNESS so we can have better contrast (and easier to view)
	if(max!=min) // stops DIV0 Trap!
		for(y=0; y<128; y++)
		{
			FinalData[y]=FinalData[y] - min;
			FinalData[y]=FinalData[y] * MAXBRIGHTNESS/(max-min);
		}

	// look for steepest slope down (usually top of foam) in processed image data
	for(y=cap+MARGIN; y<neck-MARGIN; y++)
	{
		if(FinalData[y-MARGIN]>FinalData[y]) // trap for young players! - remember we are dealing with bytes here that roll over if we subtract and go negitive!
		  if(FinalData[y-MARGIN]-FinalData[y]>MaxDownSlope)
				MaxDownSlope=FinalData[y-MARGIN]-FinalData[y]; //
	}

	// send out the image data if connected..
	SendCounters();
  Serial2Write(2);
  Serial2Write(99);
  for(y=0; y<128; y++)
 		Serial2Write(FinalData[y]);
	Serial2Write(99);

	if(MaxDownSlope>=algorithm) // if this max downward slope goes far enough down (ie > the algorithm value) then this is generally the top of foam layer
	{
		for(y=cap+MARGIN; y<neck-MARGIN; y++)
		{
  		if(FinalData[y-MARGIN]>FinalData[y])
	   		if(FinalData[y-MARGIN]-FinalData[y]==MaxDownSlope) // all good so far, *something* at the right level
				{
					// need to look for some sort of up slope up AFTER the downslope, otherwise we may be ALL foam!
					// this could vary a lot so in calibration `mode` we set the MinDownslope parameter to look for when running
					//
					for(y1=y; y1<neck-MARGIN; y1++) // start from point we've found and find biggest slope up for this sample
					{
						if(FinalData[y1]>FinalData[y1-MARGIN]) // remember above we are dealing with bytes here that roll over if we subtract and go negitive!
							if(FinalData[y1]-FinalData[y1-MARGIN]>SampleMaxUpSlope)
								SampleMaxUpSlope=FinalData[y1]-FinalData[y1-MARGIN]; //set it
					}

					if(calibrating) // were calibrating
					{
						if(SampleMaxUpSlope<MinUpSlope) MinUpSlope=SampleMaxUpSlope; // set it in calibration mode
						if(MinUpSlope<30) MinUpSlope=0; // if too low ignore
						return(y);
					}
					else // check it in run mode (we need to ensure this sample is reasonably the same)
					{
						if(MinUpSlope==0)
							return(y);
						else
						  if(SampleMaxUpSlope>MinUpSlope/4)	return(y);
					}
				}
		}
	}
	return(0); // nothing found so return fill level=0. Be careful trying further algorithms we may end up finding good levels where there are none! best a few false positives than a failed one get through!
}

uint8 FoundBottleEnd()
{
	uint8 DetCount=0;
	uint8 i;
	while(SampleCount<MAXSAMPLES) // loop here until bottle has passed the sensor and we have done our calculations or we have timed out
	{
		AquireImage();
		if(SampleCount>Tmin) // we have gone past the earliest bottle trailing edge at full speed so start to look for clear space to determine if bottle has passed
		{
			if(RawData[NORMFILL]>=MAXBRIGHTNESS && RawData[NORMFILL+4]>=MAXBRIGHTNESS) // overexposed.. we may have passed the edge..
			{
				if(!AccumulatorSaved) // first time we think we found the edge save it
				{
					for(i=0; i<128; i++)
						FinalData[i]=AccumulatedData[i];
					FinalSampleCount = SampleCount;
					AccumulatorSaved=TRUE;

				}
				if(DetCount++>10) return(TRUE); // count several over-exposures to determine if goe past edge
			}
			else
				ClearFinalData(); // false alarm we still have more bottle going past
		}
	}
	return(FALSE);
}

void WaitGapEnd() // wait here during gap while no bottle in front of sensor. (basically if nothing in front of sensor it will be constantly at max brightness)
{
	uint8 DetCount=0;
	ticker=0;
	do
	{
		if(ticker++>3000) // We may get stuck here, so after a few seconds with no bottle in front re-enable serial abd turn off open collectors and leds
		{
			if(ticker%10==0) Serial2Write(4);
			ClearAll();
			DoSerial();
			DelayMicroseconds(SampleRate*2);
			if(ticker>800000L) PowerSaveSleep(); // goto sleep if nothing happens for 3 hours (will need reset to wake up)
		}
		AquireImage();
		if( RawData[NORMFILL]<MAXBRIGHTNESS && RawData[NORMFILL+4]<MAXBRIGHTNESS && ticker>MINGAP ) DetCount++;
	}
	while(DetCount<2); // nothing between sensor and backlight must be able to cause overexposure!
  ClearAll(); // Reset Counters and accumulators
	// we need to know the line is actually moving before we process data, if we are coming out of the above loop too quick (or too slow) then
	// something is not right. so assume the bottle line is moving and bottle has passed in front of sensor ONLY if the time in this loop is
	// reasonable (note this time window should account for missing bottles!)
}

void GetSampleDataset() // fill sample image buffer with SampleRate*4x32 uS worth of data for analysis by calibration routine (conveyor must be moving)
{
	uint8 i,j;
	for(j=0;j<32;j++) // get enough sample data to cover both when there is a bottle in front and a gap (assumes line running or all 32 slots will be the same!)
	{
	  AquireImage();
		for(i=0;i<128;i++)
		  CalData[i][j]=RawData[i];
		DelayMicroseconds(SampleRate*4); // slow down so we get a wider sampling (need to catch both bottle in front and gap in sample data-set)
	}
  // scan points in image to get min max avg
	min=0xff;
	max=0;
	for(j=0;j<32;j++) // average only calculated 20 pixels either side of fill line
	{
		CalAvg[j]=0;
		for(i=43;i<83;i++)
			CalAvg[j]=CalAvg[j]+CalData[i][j];
		CalAvg[j]=CalAvg[j]/40;
		if(CalAvg[j]>max && j>0) max=CalAvg[j];
		if(CalAvg[j]<min && j>0) min=CalAvg[j];
	}
	CalBotPos=0;
	CalGapPos=0;
	for(j=0;j<32;j++)
	{
	  if(CalAvg[j]==min && CalBotPos==0 && j>0) CalBotPos=j; // bottle position
	  if(CalAvg[j]==max && CalGapPos==0 && j>0) CalGapPos=j; // gap position
	}
}

void CalibrateSensor() // sample whats happening to set the exposures and other calibration parameters automatically, only finish when calibrated
{
	uint16 i,pass;
	uint8 mode=0;

	exposure=2400; // Start the calibration at brightest
	pass=0;
	while(mode<10)
	{
		GetSampleDataset();
		LED(pass++%2 && pass%10>7);
		// Using data from position where bottle in front of sensor (ie min exposed) check algorithm can see 10 good avg min and max
		if(CalAvg[CalGapPos]>=MAXBRIGHTNESS && CalAvg[CalBotPos]>60 && CalAvg[CalBotPos]<210 )
			mode++;
		else
		{
			mode=0; // reset and half exposure rate
			if(exposure==2400) exposure=1200;
			else if(exposure==1200) exposure=600;
			else if(exposure==600) exposure=2400;
			if(pass>500) PowerSaveSleep(); // give up!
		}
 		Serial2Write(10);
 		Serial2Write(exposure>>8);
 		Serial2Write(exposure);
		Serial2Write(CalAvg[CalGapPos]);
		Serial2Write(CalAvg[CalBotPos]);
		Serial2Write(mode);
		DoSerial();
	}
	// before we exit calibration we must get a rolling average of 16 samples which must be in the range 60 - 68 to ensure its aligned so zero array

	pass=0;
	MinUpSlope=100; // Start at some possible value. See DoPassFail()
	for(i=0; i<16; i++) AvgLevel[i]=0;

	while(mode<30) // Check it and set avg fill level
	{
		WaitGapEnd(); // Wait for gap between bottles
		if(FoundBottleEnd()) // wait for bottle to pass
		{
  		LED(ON);
			DetectedFillLevel = DoPassFail(TRUE); // Do Level detection processing and return detected fill level (pass 0 to say were calibrating)
			if ( DetectedFillLevel>NORMFILL-tollerance && DetectedFillLevel<NORMFILL+tollerance ) // if inside tollerance..
			{
				// Recalulate average fill level (taking rolling average of last 16 good levels)
				if(AvgIndex++>=16) AvgIndex=0;
				AvgLevel[AvgIndex] = DetectedFillLevel;
				AvgDetFill = 0;
				for(i=0;i<16;i++)
					AvgDetFill = AvgDetFill + AvgLevel[i];
				AvgDetFill = AvgDetFill / 16;
				if(AvgDetFill>58 && AvgDetFill<70) return; // everything seems good so exit calibration
			}
		}
		DoSerial();
	}
}

void main()
{
	uint8 i;

	// Init PIC32
  DDPCONbits.JTAGEN = 0; // Turn off JTAG allows RA0, RA1, RA4, and RA5 to be configured
  SYSTEMConfigPerformance(SystemClock);

	// Init Digital pins
	mPORTASetPinsDigitalOut(0xffff);  // Reset all PORTA to Digital outputs as default
	LATA=0;													  // All OFF
	mPORTBSetPinsDigitalOut(0xffff);  // Reset all PORTB to Digital outputs as default
	mPORTBSetPinsDigitalIn(BIT_13 | BIT_14 | BIT_15);
	LATB=0;													 // All OFF
	CNPUB=0xe000;										 // RB13,14,15 enable weak pull-ups for keypad

	// Init analog pin RA1 as analog input (ouch a bit more complex!)
  TRISAbits.TRISA0 = 1;
	ANSELAbits.ANSA0 = 1;
  AD1CON1bits.ADON = 0;  // disable ADC to configure
  AD1CON1 = 0x00E0;		   // internal counter ends sampling and starts conversion (auto-convert), manual sample
  AD1CON2 = 0;			 		 // AD1CON2<15:13> set voltage reference to pins AVSS/AVDD
  AD1CON3 = 0x0f01; 		 // TAD = 4*TPB, acquisition time = 15*TAD
  AD1CON1bits.ADON = 1;  // enable ADC

  // Configure timer 1 interrupt for beeper,  SYS_CLK/256 = 195312 / 40 ~= 4882 times a second)
  OpenTimer1(T1_ON | T1_SOURCE_INT | T1_PS_1_256, 40);
  ConfigIntTimer1(T1_INT_ON | T1_INT_PRIOR_3);

  // init serial

	/*
  Serial2Init(9600);  // Set up virgin HC-06 Module to 115200 baud and rename it
	DelayMicroseconds(5000000);
	Serial2WriteStr("AT+NAMEFill-Sensor");
	DelayMicroseconds(2000000);
	Serial2WriteStr("AT+BAUD8");
	DelayMicroseconds(2000000);
  */

  // Set up the serial receive buffer indexes
  RxDataIn=0;
  RxDataOut=0;

  mPORTBSetPinsDigitalOut(BIT_10);  // Set pin 21 PB10(Tx) as output
  mPORTBSetPinsDigitalIn (BIT_11);  // Set pin 22 PB11(Rx) as input
  PPSUnLock;                        // Unlock to allow PIN Mapping
  PPSOutput(4, RPB10, U2TX);        // MAP Tx to RB10
  PPSInput (2, U2RX, RPB11);        // MAP Rx to RB11
  PPSLock;                          // lock to Prevent Accidental Mapping

  // Configure UART2 (whatever, rest of function copied straight from example code)
  UARTConfigure(UART2, UART_ENABLE_PINS_TX_RX_ONLY);
  UARTSetLineControl(UART2, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
  UARTSetDataRate(UART2, 50000000, BAUDRATE);
  UARTEnable(UART2, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX) );

  // Configure UART2 RX Interrupt (we dont enable TX interrupt unless a char waiting to go, so this is done in Serial2Write() ) //
  INTSetVectorPriority(INT_VECTOR_UART(UART2), INT_PRIORITY_LEVEL_5);
  INTEnable(INT_SOURCE_UART_RX(UART2), INT_ENABLED);
  // Configure for Multi-Vectored Interrupts //
  INTConfigureSystem(INT_SYSTEM_CONFIG_MULT_VECTOR);
  // Enable Interrupts //
  INTEnableInterrupts();

	// To retrieve last saved values from NVM we look for where the NVM record data ends and the erased flash begins, saved data is just before the 0xffffffff
	NVMPtr = NVM_START;
	while( *NVMPtr != 0xffffffff && (void*)NVMPtr < NVM_END)
		NVMPtr++;

	if( *(NVMPtr-8) != 0xffffffff) // check real data in the previous slot and if so retrieve stored values and overwrite defaults
	{
		Tmin					= *(NVMPtr-8);
		algorithm			= *(NVMPtr-7);
		cap						= *(NVMPtr-6);
		neck					= *(NVMPtr-5);
		SampleRate		= *(NVMPtr-4);
		RejOnTime			= *(NVMPtr-3);
		tollerance		= *(NVMPtr-3);
	}

	OC_REJECT(OFF);
 	Serial2Write(1);
	ClearAll();	
	CalibrateSensor();
  for(i=0;i<64;i++) AvgLevel[i]=64;

	while(TRUE) // DO LIVE LEVEL-DETECTION BEFORE CAPPPING MACHINE UNTIL POR
	{
		WaitGapEnd(); // Wait for gap between bottles
		if(FoundBottleEnd()) // wait for bottle to pass
		{
			DetectedFillLevel = DoPassFail(FALSE); // Do Level detection processing and return detected fill level, pass 1 to say were live
			if ( DetectedFillLevel<(AvgDetFill-tollerance) /*|| DetectedFillLevel>(AvgDetFill+tollerance)*/ )  //  Y=fill level: check if Y inside or outside tollerance either side of fill
				Reject();
			else
			{
				BottleCount++;
				OC_COUNT(ON);
				LED(ON);
				// Recalulate average fill level (taking rolling average of last 64 good levels should give us true average if they have not quite got level right)
				if(AvgIndex++>=64) AvgIndex=0;
				AvgLevel[AvgIndex] = DetectedFillLevel;
				AvgDetFill = 0;
				for(i=0;i<64;i++)
					AvgDetFill = AvgDetFill + AvgLevel[i];
				AvgDetFill = AvgDetFill / 64;
			}
			DoSerial();
		}
	}

}

void __ISR( _TIMER_1_VECTOR, IPL3) _T1Interrupt( void) // interrput code for the timer 1 (4000 x a second)
{
  if(BuzzerTimeout>0) BuzzerTimeout--;
  BUZZER(BuzzerTimeout && BuzzerTimeout%2); // sound @ 2Khz while active
  if(RejTimeout>0) RejTimeout--;
  OC_REJECT(RejTimeout); // engage OC Reject for given time
	LED_REJECT(RejTimeout);
  IFS0bits.T1IF = 0; // mark interrupt processed and allow next interrupt
}

void __ISR(_UART2_VECTOR, ipl5) IntUart2Handler(void) // UART 2 RX Interrupt Handler (Highest priority)
{
  if(IFS1bits.U2RXIF)  // RX char ready to read from UART into Rxbuffer
  {
    RxBuffer[RxDataIn] = (char)ReadUART2(); // Read data from Rx.
    if(RxDataIn<(sizeof(RxBuffer)-2)) RxDataIn++; // Make sure buffer is not full and move the RxBuffer Index to indicate a char available if we are at the buffer limit this indicates an error
    IFS1bits.U2RXIF=0; // All done, so Clear Interrupt flag to allow new interrupt
  }
} // end UART interrupt handler