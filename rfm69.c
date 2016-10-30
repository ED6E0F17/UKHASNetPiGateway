// RFM69.c
//
// Ported to Arduino 2014 James Coxon
//
// Copyright (C) 2014 Phil Crump
//
// Based on RF22 Copyright (C) 2011 Mike McCauley ported to mbed by Karl Zweimueller
// Based on RFM69 LowPowerLabs (https://github.com/LowPowerLab/RFM69/)

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>

#include <wiringPi.h>
#include <wiringPiSPI.h>

#include "rfm69.h"
#include "rfm69config.h"

#define RFM69_DIO0_PIN 0   // RasPi GPIO0 Pin
#define RFM69_DIO4_PIN 4   // RasPi GPIO4 Pin
#define RFM69_RESET_PIN 7  // RasPi GPIO7 Pin

    volatile uint8_t    _mode;

    uint8_t             _sleepMode;
    uint8_t             _idleMode;
    uint8_t             _afterTxMode;
    uint8_t             _channel;
    //SPI                 _spi;
    //InterruptIn         _interrupt;
    uint8_t             _deviceType;

    // These volatile members may get changed in the interrupt service routine
    volatile uint8_t    _bufLen;
    uint8_t             _buf[RFM69_MAX_MESSAGE_LEN];

    volatile boolean    _rxBufValid;

    volatile boolean    _txPacketSent;
    volatile uint8_t    _txBufSentIndex;
  
    volatile uint16_t   _rxBad;
    volatile uint16_t   _rxGood;
    volatile uint16_t   _txGood;

    volatile int        _lastRssi;
    volatile int        _floorRssi;
    volatile uint8_t    _threshold_val;

void spiWrite(uint8_t reg, uint8_t val)
{
	unsigned char data[2];
    
	data[0] = reg | RFM69_SPI_WRITE_MASK;
	data[1] = val;
	wiringPiSPIDataRW(_channel, data, 2);
}

uint8_t spiRead(uint8_t reg)
{
	unsigned char data[2];
	uint8_t val;
	
	data[0] = reg & ~RFM69_SPI_WRITE_MASK;
	data[1] = 0;
	wiringPiSPIDataRW(_channel, data, 2);
	val = data[1];
	
        return val;
}

void spiBurstRead(uint8_t reg, uint8_t* dest, uint8_t len)
{
	unsigned char data[128];
	int i;
	
	data[0] = reg & ~RFM69_SPI_WRITE_MASK;
	wiringPiSPIDataRW(_channel, data, len+1);

	for (i=0; i<len; i++)
	{
		dest[i] = data[i+1];
	}
}

void setMode(uint8_t newMode)
{
    spiWrite(RFM69_REG_01_OPMODE, (spiRead(RFM69_REG_01_OPMODE) & 0xE3) | newMode);
    while((spiRead(RFM69_REG_27_IRQ_FLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0)
    {
        usleep(1000);
    }
    _mode = newMode;
    printf ("Mode = %d\n", spiRead(RFM69_REG_01_OPMODE));
}

boolean rfm69_init(int chan)
{
    int i;

    _idleMode = RFM69_MODE_SLEEP; // Default idle state is SLEEP, our lowest power mode
    _mode = RFM69_MODE_RX; // We start up in RX mode
    _rxGood = 0;
    _rxBad = 0;
    _txGood = 0;
    _channel = chan;
    _afterTxMode = RFM69_MODE_RX;

    if (wiringPiSetup() < 0) {
        fprintf (stderr, "Unable to setup wiringPi: %s\n", strerror (errno));
        return false;
    }
    if (wiringPiSPISetup(_channel, 500000) < 0)
    {
        fprintf(stderr, "Failed to open SPI port.  Try loading spi device tree param.");
        return false;
    }

    // Reset device
    // first drive pin high
    pinMode(RFM69_RESET_PIN, OUTPUT);
    digitalWrite(RFM69_RESET_PIN, HIGH);
    // pause for 100 microseconds
    usleep(100);
    // release pin
    pullUpDnControl(RFM69_RESET_PIN, PUD_OFF);
    pinMode(RFM69_RESET_PIN, INPUT);
    // pause for 5 ms
    usleep(5000);

    // Check for device presence
    if (spiRead(RFM69_REG_10_VERSION) != 0x24)
    {
        return false;
    }

    // Set up device
    for (i=0; CONFIG[i][0] != 255; i++)
    {
        spiWrite(CONFIG[i][0], CONFIG[i][1]);
    }

    _threshold_val = spiRead(RFM69_REG_29_RSSI_THRESHOLD);
    setMode(_mode);

    return true;
}

boolean rfm69_available()
{
    return _rxBufValid;
}

int rssiRead()
{
    return -spiRead(RFM69_REG_24_RSSI_VALUE)/2;
}

int rssiMeasure()
{
    int count = 0;
    uint8_t rssi_val;

    spiWrite(RFM69_REG_29_RSSI_THRESHOLD, 0xff);
    spiWrite(RFM69_REG_23_RSSI_CONFIG, RF_RSSI_START);
    while((spiRead(RFM69_REG_23_RSSI_CONFIG) & RF_RSSI_DONE) == 0)
    {
        usleep(1000);
        if(count++ > 100) {
            return 0;
        }
    }
    rssi_val = spiRead(RFM69_REG_24_RSSI_VALUE);
    if(rssi_val - 6 < _threshold_val)
    {
        _threshold_val--;
    }
    else
    {
        _threshold_val++;
    }
    //printf("RSSI %d dBm, threshold %d dBm\n", -rssi_val/2, -_threshold_val/2);
    spiWrite(RFM69_REG_29_RSSI_THRESHOLD, _threshold_val);
    spiWrite(RFM69_REG_3D_PACKET_CONFIG2, spiRead(RFM69_REG_3D_PACKET_CONFIG2) | RF_PACKET2_RXRESTART);
    return -rssi_val/2;
}

uint8_t rfmM69_recv(uint8_t* buf, uint8_t len)
{
    if (_bufLen == 0)
    {
        len = 0;
    }
    else
    {
        if (len > _bufLen)
        {
            len = _bufLen;
        }
        memcpy(buf, _buf, len);
    }
    buf[len] = '\0';

    return len;
}

void RFM69_tx(char *Message) {
	char buff[66];
	uint8_t inpos, outpos;
	uint8_t len;

	setMode( RFM69_MODE_TX );
	usleep(1e4);

	len = 63 & strlen(Message);
	inpos = outpos = 0;
	buff[outpos++] = RFM69_REG_00_FIFO | RFM69_SPI_WRITE_MASK;
	buff[outpos++] = len;
	while (len--)
		buff[outpos++] = Message[inpos++];
        wiringPiSPIDataRW(1, (uint8_t *)&buff[0], outpos);

	while (!( (spiRead(RFM69_REG_28_IRQ_FLAGS2) & RF_IRQFLAGS2_PACKETSENT) ))
		usleep(1e4);
	setMode( RFM69_MODE_SLEEP );	
}

int RFM69_lastRssi()
{
	return _lastRssi;
}

