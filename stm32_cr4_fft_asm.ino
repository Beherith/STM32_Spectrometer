// Real-time audio spectrum analyzer with LCD support
// By Beherith (mysterme at gmail dot com) license: Public Domain


//FFT stuff------------------------------------------------------------------
#define FFTLEN 1024
#include "cr4_fft_1024_stm32.h"
uint16_t data16[FFTLEN];
uint32_t data32[FFTLEN];
uint32_t y[FFTLEN];
uint16_t hammingwindow[FFTLEN/2];
uint16_t bins = FFTLEN;

//Display stuff:------------------------------------------------------------------
#include "Adafruit_ILI9341_STM.h"
#include "Adafruit_GFX_AS.h"
#include <SPI.h>
#define PORTRAIT 0
#define LANDSCAPE 1

// TOUCH IS UNUSED!
/*
#include <UTouch.h>
UTouch  myTouch( PB12, PB13, PB14, PB15, PA8);
#define  TOUCH_CALIB_X 0
#define  TOUCH_CALIB_Y 1
#define  TOUCH_CALIB_Z 2
*/
#define TFT_DC      PA0      //   (Green) 
#define TFT_CS      PA1      //   (Orange) 
#define TFT_RST     PA2      //   (Yellow)
// Hardware SPI1 on the STM32F103C8T6 *ALSO* needs to be connected and pins are as follows.
//
// SPI1_NSS  (PA4) (LQFP48 pin 14)    (n.c.)
// SPI1_SCK  (PA5) (LQFP48 pin 15)    (Brown)
// SPI1_MISO (PA6) (LQFP48 pin 16)    (White)
// SPI1_MOSI (PA7) (LQFP48 pin 17)    (Grey)
//
Adafruit_ILI9341_STM TFT = Adafruit_ILI9341_STM(TFT_CS, TFT_DC, TFT_RST); // Using 
#define TFT_LED        PA3     // Backlight, connect to 3.3v
const int16_t myWidth = 320;
const int16_t myHeight = 240 ;
int16_t currcolumn = 0;

int displayMode = 0; //0 = linear, 1=log, 2= squished linear

//DMA--------------------------------------------------------------------------
volatile static bool dma1_ch1_Active;
#include <libmaple/pwr.h>
#include <libmaple/scb.h>
#include <libmaple/rcc.h>
#include <libmaple/adc.h>
//Other stuff------------------------------------------------------------------

#define BOARD_LED PC13 //PB0
USBSerial serial_debug;
const int8_t analogInPin = PB0; // CONNECT YOUR ANALOG SOURCE HERE!
uint32_t tick =0;
const uint32_t sampleRate = 37650;
#define BTN1 PB12
#define BTN2 PB13

//TODO: Move these functions to header?

void init_hamming_window(uint16_t * windowtarget, int len){
  for(int i = 0;i<len/2; i++){ windowtarget[i] = (0.54 - 0.46 * cos((2 * i * 3.141592)/(len-1))) * 65536; }
}
  
void window(uint32_t * data, uint16_t * weights, int len, int scale){
  for(int i =0; i<len; i++){
    int weight_index = i;
    if( i > len/2 ) weight_index = len-i;
    data[i] = ((data[i] * scale * weights[weight_index]) >> 16) & 0xFFFF;
  }
}

uint16_t asqrt(uint32_t x) { //good enough precision, 10x faster than regular sqrt
  /*      From http://medialab.freaknet.org/martin/src/sqrt/sqrt.c
   *   Logically, these are unsigned. We need the sign bit to test
   *   whether (op - res - one) underflowed.
   */
  int32_t op, res, one;

  op = x;
  res = 0;
  /* "one" starts at the highest power of four <= than the argument. */
  one = 1 << 30;   /* second-to-top bit set */
  while (one > op) one >>= 2;
  while (one != 0) {
    if (op >= res + one) {
      op = op - (res + one);
      res = res +  2 * one;
    }
    res /= 2;
    one /= 4;
  }
  return (uint16_t) (res);
}

void fill(uint32_t * data, uint32_t value, int len){
  for (int i =0; i< len;i++) data[i]=value;
}

void fill(uint16_t * data, uint32_t value, int len){
  for (int i =0; i< len;i++) data[i]=value;
}

void real_to_complex(uint16_t * in, uint32_t * out, int len){
  for (int i = 0;i<len;i++) out[i]=in[i]*8;
}

void generate_squarewave_data(uint16_t * data, uint32_t period, uint32_t amplitude, int len){
  for (int i =0; i< len;i++){
    if ((i/(period/2)) & 1 ==1){
		data[i] = amplitude;
	}else{
		data[i]= 0;
    }
  }
}

void generate_sawtoothwave_data(uint16_t * data, uint32_t period, uint32_t amplitude, int len){
  for (int i =0; i< len;i++){
    data[i] = (i - period * (int (i/period))) * (amplitude/period);
  }
}

void setADCs ()
{
  rcc_set_prescaler(RCC_PRESCALER_ADC,RCC_ADCPRE_PCLK_DIV_8 );

  int pinMapADCin = PIN_MAP[analogInPin].adc_channel;
  adc_set_sample_rate(ADC1, ADC_SMPR_239_5); //~37.65 khz sample rate
 
  adc_set_reg_seqlen(ADC1, 1);
  ADC1->regs->SQR3 = pinMapADCin;
  ADC1->regs->CR2 |= ADC_CR2_CONT; // | ADC_CR2_DMA; // Set continuous mode and DMA
  ADC1->regs->CR2 |= ADC_CR2_SWSTART;

  //Serial.println("Calibrating ADC1");
  //adc_calibrate(ADC1); //TODO: Calibration doesnt ever return for some odd reason...

}
static void DMA1_CH1_Event() {
  dma1_ch1_Active = 0;
}

void adc_dma_enable(const adc_dev * dev) {
  bb_peri_set_bit(&dev->regs->CR2, ADC_CR2_DMA_BIT, 1);
}
int toggle_displayMode(){
  displayMode++;
  if (displayMode ==3) displayMode = 0;
  return displayMode;
}

void takeSamples()
{
  Serial.print("tick:");
  Serial.println(micros()-tick);
  tick = micros();
  
  real_to_complex(data16,data32,FFTLEN);//clear inputs
  // perform DMA, 
  dma_init(DMA1);
  dma_attach_interrupt(DMA1, DMA_CH1, DMA1_CH1_Event);

  adc_dma_enable(ADC1);
  dma_setup_transfer(DMA1, DMA_CH1, &ADC1->regs->DR, DMA_SIZE_16BITS, data16, DMA_SIZE_16BITS, (DMA_MINC_MODE | DMA_TRNS_CMPLT));// Receive buffer DMA
  dma_set_num_transfers(DMA1, DMA_CH1, FFTLEN );
  dma1_ch1_Active = 1;
  
  dma_enable(DMA1, DMA_CH1); // Enable the channel and start the transfer.
  uint32_t time_left_in_dma = micros();
  
  #define PARALLEL 0
  if (PARALLEL){
    perform_fft(data32, y, FFTLEN);
    drawFrequencySlice(y, FFTLEN);
    Serial.print( micros() - time_left_in_dma) ;
    Serial.println(" us left in DMA");
    while (dma1_ch1_Active){};   //Wait for the DMA to complete
    dma_disable(DMA1, DMA_CH1); //End of trasfer, disable DMA and Continuous mode.
  }else{
    while (dma1_ch1_Active){};    //Wait for the DMA to complete
    dma_disable(DMA1, DMA_CH1); //End of trasfer, disable DMA and Continuous mode.
    perform_fft(data32, y, FFTLEN);
    drawFrequencySlice(y, FFTLEN);
  }
  
  if (digitalRead(BTN1) == LOW){  //print everything to serial, very slow obviously
    printdataset(data32, FFTLEN, 0);
    printdataset(y, FFTLEN, sampleRate);
    toggle_displayMode();
  }
  
  if (digitalRead(BTN2) == LOW){ //Draw nice spectrum
    drawTimeFrequencyDomain(data32,y,FFTLEN); //TODO this uses the signal level data of the newer data!
  }
}

uint16 timer_set_period(HardwareTimer timer, uint32 microseconds) {
  if (!microseconds) {
    timer.setPrescaleFactor(1);
    timer.setOverflow(1);
    return timer.getOverflow();
  }

  uint32 cycles = microseconds * (72000000 / 1000000); // 72 cycles per microsecond

  uint16 ps = (uint16)((cycles >> 16) + 1);
  timer.setPrescaleFactor(ps);
  timer.setOverflow((cycles / ps) - 1 );
  return timer.getOverflow();
}

void selftest(){
  TFT.println("Selftesting");
	Serial.println("Performing self FFT test");
	generate_sawtoothwave_data(data16,64, 1337, FFTLEN);
  real_to_complex(data16, data32, FFTLEN);
  TFT.println("Generated wave");
	perform_fft(data32, y, FFTLEN);
	TFT.println("FFT done");
	drawTimeFrequencyDomain(data32,y,FFTLEN);
	printdataset(data32,FFTLEN,0);
	printdataset(y,FFTLEN,1024);
  TFT.println("Tests passed!");
	delay(2000);
}

void setup() {
  delay(500);
  Serial.begin(115200);
  Serial.println("Testing cr4_fft_1024_stm32");
  serial_debug.begin(9600);
  serial_debug.println("This is my debug port. Look at my port, my port is amazing!");

  //myTouch.InitTouch();
  //myTouch.setPrecision(PREC_EXTREME);
  
  // initialize the display
  TFT.begin();
  TFT.fillScreen(ILI9341_RED);
  TFT.setRotation(LANDSCAPE);
  TFT.setCursor(0,0);
  TFT.setTextColor(0xFFFF);
  TFT.setTextSize(2);

  //initialize FFT variables
  init_hamming_window(hammingwindow,FFTLEN);
  fill(y,0,FFTLEN);
  fill(data32,1,FFTLEN);
  fill(data16,1,FFTLEN);

  setADCs();
  TFT.println("ADCs set");
   
  //timer_set_period(Timer3, 1000);
  //analogWrite(PB15,100); //this doesnt work if serial is not connected
  
  selftest();
  TFT.println("Selftest complete");
  digitalWrite(BTN1, HIGH);
  pinMode(BTN1, INPUT_PULLUP);
  digitalWrite(BTN2, HIGH);
  pinMode(BTN2, INPUT_PULLUP);
}

void inplace_magnitude(uint32_t * target, uint16_t len){
  uint16_t * p16;
  for (int i=0;i<len;i++){
     int16_t real = target[i] & 0xFFFF;
     int16_t imag = target[i] >> 16;
     uint32_t magnitude = asqrt(real*real + imag*imag);
     target[i] = magnitude; 
  }
}

float bin_frequency(uint32_t samplerate, uint32_t binnumber, uint32_t len){
   return (binnumber*samplerate)/((float)len);
  }
  
uint16_t mag2color(uint32_t mag,uint32_t scale){
    //A sensible value for max MAG is about 4096, except for the DC component...
    scale = max(scale,256);
    mag = min(65535, (mag*1024)/scale);
    byte r,g,b;
    if (mag < 32){
      r=0;
      g=0;
      b=mag;
      }
    else if (mag < 1024){
      r = 0;
      g = mag >>3;
      b = 31;
    }
    else{
      r= mag >> 9;
      g = 63;
      b = 31;
      }
    return ( (r<<11)|(g <<6 )|(b )  );
    
  //RGB where R5G6B5
    
    return (uint16_t) mag;
    uint16_t color = 0;
    color = min(63, mag>>6);
    return color<<5;
  }
  
uint32_t perform_fft(uint32_t * indata, uint32_t * outdata,const int len){
	uint32_t timetaken = micros();
	//window(indata,hammingwindow,len,8); //scaling factor of 4 for 4095> 16 bits
	cr4_fft_1024_stm32(outdata,indata,len);
	inplace_magnitude(outdata,len);
	return micros() - timetaken;
}

uint32_t logbin(uint32_t bin, uint32_t len, uint32_t resolution);

uint32_t drawFrequencySlice(uint32_t * magdata, const int len){
  uint32_t maxvalue = 0;
  uint32_t timetaken = micros();
  
  for (int i =1; i< len/2; i++) maxvalue = max(maxvalue,magdata[i]); //dont count DC, dont count the second half
	
	TFT.drawFastVLine(currcolumn+1,0, myHeight, 0xFFFF); //Draw a white line leading before the spectrum
  
  if (displayMode == 0){
    for (int i= 0; i<(myHeight);i++){
      TFT.drawPixel(currcolumn,myHeight - i -1 ,mag2color((magdata[2*i]+magdata[2*i+1])/2,maxvalue));
    }
  }
  if (displayMode == 1){
    for (int xpos = 0; xpos < myHeight; xpos++){
      uint32_t collect = 0;
      uint32_t startbin =  pow(2.0, float(xpos)/26.666);
      uint32_t endbin = pow(2.0, float(xpos+1)/26.666);
      byte b = 0;
      for (int i=startbin; i<=endbin; i++){
        collect+=magdata[i];
        b++;
      } 
      collect = collect/b;
      TFT.drawPixel(currcolumn,myHeight-1 -xpos ,mag2color(collect,maxvalue));
    }
  }

  if (displayMode ==2){
    #define SCALE 128
    for (int i = 0;  i <SCALE;i++){
      TFT.drawPixel(currcolumn, (myHeight-1) - i,mag2color(magdata[i],maxvalue));  
    }
    
    for (int i = SCALE; i< myHeight; i++){
      uint32_t collect = 0;
      for (int bin =0; bin <4; bin++){
        collect = collect + magdata[SCALE + (i-SCALE)*4 + bin];
      }
      collect = collect/4;
      TFT.drawPixel(currcolumn, (myHeight-1) - i,mag2color(collect,maxvalue));  
    }
  }

	currcolumn++;
	if (currcolumn == myWidth) currcolumn =0;
  Serial.print("MaxBin = ");
  Serial.println(maxvalue);
	return micros() - timetaken;
}
uint32_t drawTimeDomain(uint32_t * data, int len){
  uint32_t timetaken = micros();
	uint32_t maxvalue = 0 ;
	for (int i =0; i< len; i++) maxvalue = max(maxvalue,data[i]);
	TFT.fillScreen(0x0000);
	TFT.setCursor(0,0);
	TFT.setTextColor(0xFFFF);
	TFT.setTextSize(2);
	TFT.print("Max = ");
	TFT.println(maxvalue, DEC);
	
	for (int i =0; i< len; i++) TFT.drawPixel((i*myWidth)/len, (data[i]*myHeight)/maxvalue, 0xFFFF);
	return micros() - timetaken;
}

void drawTimeFrequencyDomain( uint32_t * x, uint32_t * y, int len){
  //Draw the signal onto the top half of the screen
  TFT.fillScreen(0x0000);
  uint32_t maxvalue = 0 ;
  uint32_t minvalue = 65536;
  for (int i =0; i< len; i++){
    maxvalue = max(maxvalue,x[i]);
    minvalue = min(minvalue,x[i]);
  }
  TFT.fillScreen(0x0000);
  TFT.setCursor(0,0);
  TFT.setTextColor(0xFFFF);
  TFT.setTextSize(2);
  TFT.print('[');
  TFT.print(minvalue);
  TFT.print('-');
  TFT.print(maxvalue);
  TFT.println(']');
  
  for (int i =0; i< len; i++) TFT.drawPixel((i*myWidth)/len,  myHeight/2 - ((x[i]-minvalue)*myHeight/2)/(maxvalue-minvalue), 0xFFFF);

  //Draw the frequency bins from 1 to FFTLEN/2 (because the DC component is ignored, and the second half of the FFT contains the bins above the nyquist limit
  maxvalue = 0;
  for (int i =1; i< len/2; i++) maxvalue = max(maxvalue,y[i]); // dont draw the first bin, as that contains DC
  
  for (int i =1; i< len/2; i++) TFT.drawFastVLine((i*myWidth)/(len/2), myHeight - (y[i]*myHeight/2)/maxvalue, myHeight, 0xF0FF);  
  TFT.setCursor(0, myHeight/2);
  TFT.print("Max = ");
  TFT.println(maxvalue, DEC);
  }


void printdataset(uint32_t * data, int len, int samplerate){
	Serial.print("Printing dataset at ");
	Serial.println((long long unsigned int) data, HEX);
	if (samplerate > 0){
		Serial.println("Bin#	freq	mag");
		for (int i =0; i< len; i++){
			Serial.print(i);
			Serial.print("	");
			Serial.print(bin_frequency(samplerate,i,len));
			Serial.print("	");
			Serial.println(data[i]);
		}
	}else{
		Serial.println("i	value");
		for (int i =0; i< len; i++){
			Serial.print(i);
			Serial.print("	");
			Serial.println(data[i]);
		}
	} 
}

void loop() { 
 while(1){ // loop seems to run only once, hence the while 
	takeSamples();
 }
}
