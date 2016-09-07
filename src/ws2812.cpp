#include "ws2812.h"
#include "spi.h"
#include <stdio.h>
#include <stdlib.h>

unsigned ledColour[NUM_PIXELS];

uint8_t DMABuffer[NUM_PIXELS*9]; // This will be used as a DMA Buffer for the SPI bus


//////////////---------------- WS2812 ----------------------/////////////


void Ws2812::latch() {
	delay(300); // This is about 80us at this clock speed; enough to
			    // be considered as a latch signal by the WS2812B's
}

void Ws2812::delay(int dly)
{
  while( dly--);
}

void Ws2812::setup() {
	initSPI();
}

void Ws2812::writeDMABuffer(int DeviceNumber, unsigned long Value) {
    // have to expand each bit to 3 bits
    // Can then output 110 for WS2812B logic '1'
    // and 100 for WS2812B logic '0'
    uint32_t Encoding=0;
    uint8_t SPI_Data[9];
    int Index;

    // Process the GREEN byte
    Index=0;
    Encoding=0;
    while (Index < 8)
    {
        Encoding = Encoding << 3;
        if (Value & (1 << 23))
        {
            Encoding |= 0b110;
        }
        else
        {
            Encoding |= 0b100;
        }
        Value = Value << 1;
        Index++;

    }
    SPI_Data[0] = ((Encoding >> 16) & 0xff);
    SPI_Data[1] = ((Encoding >> 8) & 0xff);
    SPI_Data[2] = (Encoding & 0xff);

    // Process the RED byte
    Index=0;
    Encoding=0;
    while (Index < 8)
    {
        Encoding = Encoding << 3;
        if (Value & (1 << 23))
        {
            Encoding |= 0b110;
        }
        else
        {
            Encoding |= 0b100;
        }
        Value = Value << 1;
        Index++;

    }
    SPI_Data[3] = ((Encoding >> 16) & 0xff);
    SPI_Data[4] = ((Encoding >> 8) & 0xff);
    SPI_Data[5] = (Encoding & 0xff);

    // Process the BLUE byte
    Index=0;
    Encoding=0;
    while (Index < 8)
    {
        Encoding = Encoding << 3;
        if (Value & (1 << 23))
        {
            Encoding |= 0b110;
        }
        else
        {
            Encoding |= 0b100;
        }
        Value = Value << 1;
        Index++;

    }
    SPI_Data[6] = ((Encoding >> 16) & 0xff);
    SPI_Data[7] = ((Encoding >> 8) & 0xff);
    SPI_Data[8] = (Encoding & 0xff);

    Index=0;
	while(Index < 9)
    {
		DMABuffer[Index+DeviceNumber*9]=SPI_Data[Index];
		Index++;
	}
}

void Ws2812::setPixelColor(int led, int red, int green, int blue) {
	unsigned color = blue + (red<<8) + (green<<16);
	ledColour[led] = color;
}

void Ws2812::setPixelColor(int led, unsigned color) {
	ledColour[led] = color;
}

void Ws2812::nscale8(int led, int brightness) {
	uint8_t r, g, b;
	uint16_t scale_fixed = brightness + 1;

	r = ledColour[led] >> 8;
	g = ledColour[led] >> 16;
	b = ledColour[led] >> 0;

	r = (((uint16_t)r) * scale_fixed) >> 8;
	g = (((uint16_t)g) * scale_fixed) >> 8;
	b = (((uint16_t)b) * scale_fixed) >> 8;

	setPixelColor(led,r,g,b);
}

void Ws2812::show() {
	for (int i=0;i<NUM_PIXELS;i++)
		writeDMABuffer(i,ledColour[i]); // Output a colour Format: GGRRBB
	// Now send out the bits to the SPI bus
	writeSPI(DMABuffer,sizeof(DMABuffer));
	latch(); // latch the values to the LED's
}

int Ws2812::numPixels() {
	return NUM_PIXELS;
}


unsigned long getRainbow()
{   // Cycle through the colours of the rainbow (non-uniform brightness however)
	// Inspired by : http://academe.co.uk/2012/04/arduino-cycling-through-colours-of-the-rainbow/
	static unsigned Red = 255;
	static unsigned Green = 0;
	static unsigned Blue = 0;
	static int State = 0;
	switch (State)
	{
		case 0:{
			Green++;
			if (Green == 255)
				State = 1;
			break;
		}
		case 1:{
			Red--;
			if (Red == 0)
				State = 2;
			break;
		}
		case 2:{
			Blue++;
			if (Blue == 255)
				State = 3;
			break;
		}
		case 3:{
			Green--;
			if (Green == 0)
				State = 4;
			break;
		}
		case 4:{
			Red++;
			if (Red == 255)
				State = 5;
			break;
		}
		case 5:{
			Blue --;
			if (Blue == 0)
				State = 0;
			break;
		}
	}
	return (Green << 16) + (Red << 8) + Blue;
}

uint32_t wheel(uint8_t wheelPos) {
	uint8_t red, green, blue;

	wheelPos = 255 - wheelPos;
	if(wheelPos < 85)
	{
		red = 255 - wheelPos * 3;
		blue = wheelPos * 3;
		green = 0;
	}
	else if(wheelPos < 170)
	{
		wheelPos -= 85;
		red = 0;
		green = wheelPos * 3;
		blue = 255 - wheelPos * 3;

	}
	else
	{
		wheelPos -= 170;
		red = wheelPos * 3;
		green = 255 - wheelPos * 3;
		blue = 0;
	}

	return blue + (red<<8) + (green<<16);
}


