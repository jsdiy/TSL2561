//	照度センサー TSL2561 (モジュール名 MH-2561/GY-2561) ライブラリ
//	『昼夜逆転』工作室	http://jsdiy.starfree.jp
//	2025/01	@jsdiy

#include	<Arduino.h>
#include	<Wire.h>
#include	"TSL2561.hpp"

//Commandレジスタのビットフィールド
enum	ECmdReg	: uint8_t
{
	CMD		= 7,	//Select command register. Must write as 1.
	CLEAR	= 6,	//Interrupt clear. Clears any pending interrupt. This bit is a write-one-to-clear bit. It is self clearing.
	WORD	= 5,	//SMB Write/Read Word Protocol. 1 indicates that this SMB transaction is using either the SMB Write Word or Read Word protocol.
	BLOCK	= 4,	//Block Write/Read Protocol. 1 indicates that this transaction is using either the Block Write or the Block Read protocol. See Note below.
	ADDRESS	= 0		//[3:0] Register Address. This field selects the specific control or status register for following write and read commands according to Table 2.
};

//Controlレジスタのビットフィールド
enum	EControlReg	: uint8_t
{
//	Resv2	= 2,	//[7:2] Reserved. Write as 0.
	POWER	= 0		//[1:0] Power up/power down. By writing a 03h to this register, the device is powered up. By writing a 00h to this register, the device is powered down.
					//NOTE: If a value of 03h is written, the value returned during a read cycle will be 03h. This feature can be used to verify that the device is communicating properly.
};

//PowerUp/Down（Controlレジスタで設定する）
enum	EPower : uint8_t
{
	Up		= 0x03,
	Down	= 0x00
};

//Timingレジスタのビットフィールド
enum	ETimingReg	: uint8_t
{
//	Resv5	= 5,	//[7:5] Reserved. Write as 0.
	GAIN	= 4,	//Switches gain between low gain and high gain modes. Writing a 0 selects low gain (1×); writing a 1 selects high gain (16×).
	Manual	= 3,	//Manual timing control. Writing a 1 begins an integration cycle. Writing a 0 stops an integration cycle.
					//NOTE: This field only has meaning when INTEG = 11. It is ignored at all other times.
//	Resv2	= 2,	//Reserved. Write as 0.
	INTEG	= 0		//[1:0] Integrate time. This field selects the integration time for each conversion.
					/*	INTEG FIELD VALUE : SCALE : NOMINAL INTEGRATION TIME
						00	:	0.034	:	13.7 ms
						01	:	0.252	:	101 ms
						10	:	1		:	402 ms
						11	:	−−		:	N/A
					*/
};

//Lux計算用の定義
#pragma region
/*
//****************************************************************************
//
// Copyright (c) 2004−2005 TAOS, Inc.
//
// THIS CODE AND INFORMATION IS PROVIDED ”AS IS” WITHOUT WARRANTY OF ANY
// KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR
// PURPOSE.
//
// Module Name:
// lux.cpp
//
//****************************************************************************
*/
#define LUX_SCALE 14 // scale by 2^14
#define RATIO_SCALE 9 // scale ratio by 2^9
//−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−
// Integration time scaling factors
//−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−
#define CH_SCALE 10 // scale channel values by 2^10
#define CHSCALE_TINT0 0x7517 // 322/11 * 2^CH_SCALE
#define CHSCALE_TINT1 0x0fe7 // 322/81 * 2^CH_SCALE
//−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−
// T, FN, and CL Package coefficients
//−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−
/*
// For Ch1/Ch0=0.00 to 0.50
// Lux/Ch0=0.0304−0.062*((Ch1/Ch0)^1.4)
// piecewise approximation
// For Ch1/Ch0=0.00 to 0.125:
// Lux/Ch0=0.0304−0.0272*(Ch1/Ch0)
//
// For Ch1/Ch0=0.125 to 0.250:
// Lux/Ch0=0.0325−0.0440*(Ch1/Ch0)
//
// For Ch1/Ch0=0.250 to 0.375:
// Lux/Ch0=0.0351−0.0544*(Ch1/Ch0)
//
// For Ch1/Ch0=0.375 to 0.50:
// Lux/Ch0=0.0381−0.0624*(Ch1/Ch0)
//
// For Ch1/Ch0=0.50 to 0.61:
// Lux/Ch0=0.0224−0.031*(Ch1/Ch0)
//
// For Ch1/Ch0=0.61 to 0.80:
// Lux/Ch0=0.0128−0.0153*(Ch1/Ch0)
//
// For Ch1/Ch0=0.80 to 1.30:
// Lux/Ch0=0.00146−0.00112*(Ch1/Ch0)
//
// For Ch1/Ch0>1.3:
// Lux/Ch0=0
//−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−
*/
#define K1T 0x0040 // 0.125 * 2^RATIO_SCALE
#define B1T 0x01f2 // 0.0304 * 2^LUX_SCALE
#define M1T 0x01be // 0.0272 * 2^LUX_SCALE
#define K2T 0x0080 // 0.250 * 2^RATIO_SCALE
#define B2T 0x0214 // 0.0325 * 2^LUX_SCALE
#define M2T 0x02d1 // 0.0440 * 2^LUX_SCALE
#define K3T 0x00c0 // 0.375 * 2^RATIO_SCALE
#define B3T 0x023f // 0.0351 * 2^LUX_SCALE
#define M3T 0x037b // 0.0544 * 2^LUX_SCALE
#define K4T 0x0100 // 0.50 * 2^RATIO_SCALE
#define B4T 0x0270 // 0.0381 * 2^LUX_SCALE
#define M4T 0x03fe // 0.0624 * 2^LUX_SCALE
#define K5T 0x0138 // 0.61 * 2^RATIO_SCALE
#define B5T 0x016f // 0.0224 * 2^LUX_SCALE
#define M5T 0x01fc // 0.0310 * 2^LUX_SCALE
#define K6T 0x019a // 0.80 * 2^RATIO_SCALE
#define B6T 0x00d2 // 0.0128 * 2^LUX_SCALE
#define M6T 0x00fb // 0.0153 * 2^LUX_SCALE
#define K7T 0x029a // 1.3 * 2^RATIO_SCALE
#define B7T 0x0018 // 0.00146 * 2^LUX_SCALE
#define M7T 0x0012 // 0.00112 * 2^LUX_SCALE
#define K8T 0x029a // 1.3 * 2^RATIO_SCALE
#define B8T 0x0000 // 0.000 * 2^LUX_SCALE
#define M8T 0x0000 // 0.000 * 2^LUX_SCALE

//−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−
// CS package coefficients
//−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−
/*
// For 0 <= Ch1/Ch0 <= 0.52
// Lux/Ch0 = 0.0315−0.0593*((Ch1/Ch0)^1.4)
// piecewise approximation
// For 0 <= Ch1/Ch0 <= 0.13
// Lux/Ch0 = 0.0315−0.0262*(Ch1/Ch0)
// For 0.13 <= Ch1/Ch0 <= 0.26
// Lux/Ch0 = 0.0337−0.0430*(Ch1/Ch0)
// For 0.26 <= Ch1/Ch0 <= 0.39
// Lux/Ch0 = 0.0363−0.0529*(Ch1/Ch0)
// For 0.39 <= Ch1/Ch0 <= 0.52
// Lux/Ch0 = 0.0392−0.0605*(Ch1/Ch0)
// For 0.52 < Ch1/Ch0 <= 0.65
// Lux/Ch0 = 0.0229−0.0291*(Ch1/Ch0)
// For 0.65 < Ch1/Ch0 <= 0.80
// Lux/Ch0 = 0.00157−0.00180*(Ch1/Ch0)
// For 0.80 < Ch1/Ch0 <= 1.30
// Lux/Ch0 = 0.00338−0.00260*(Ch1/Ch0)
// For Ch1/Ch0 > 1.30
// Lux = 0
//−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−
*/
#define K1C 0x0043 // 0.130 * 2^RATIO_SCALE
#define B1C 0x0204 // 0.0315 * 2^LUX_SCALE
#define M1C 0x01ad // 0.0262 * 2^LUX_SCALE
#define K2C 0x0085 // 0.260 * 2^RATIO_SCALE
#define B2C 0x0228 // 0.0337 * 2^LUX_SCALE
#define M2C 0x02c1 // 0.0430 * 2^LUX_SCALE
#define K3C 0x00c8 // 0.390 * 2^RATIO_SCALE
#define B3C 0x0253 // 0.0363 * 2^LUX_SCALE
#define M3C 0x0363 // 0.0529 * 2^LUX_SCALE
#define K4C 0x010a // 0.520 * 2^RATIO_SCALE
#define B4C 0x0282 // 0.0392 * 2^LUX_SCALE
#define M4C 0x03df // 0.0605 * 2^LUX_SCALE
#define K5C 0x014d // 0.65 * 2^RATIO_SCALE
#define B5C 0x0177 // 0.0229 * 2^LUX_SCALE
#define M5C 0x01dd // 0.0291 * 2^LUX_SCALE
#define K6C 0x019a // 0.80 * 2^RATIO_SCALE
#define B6C 0x0101 // 0.0157 * 2^LUX_SCALE
#define M6C 0x0127 // 0.0180 * 2^LUX_SCALE
#define K7C 0x029a // 1.3 * 2^RATIO_SCALE
#define B7C 0x0037 // 0.00338 * 2^LUX_SCALE
#define M7C 0x002b // 0.00260 * 2^LUX_SCALE
#define K8C 0x029a // 1.3 * 2^RATIO_SCALE
#define B8C 0x0000 // 0.000 * 2^LUX_SCALE
#define M8C 0x0000 // 0.000 * 2^LUX_SCALE
#pragma endregion

//レジスタに値を書き込む（コマンド送信＋データ1byte書込み）
void	TSL2561::WriteByte(uint8_t cmd, uint8_t data)
{
	Wire.beginTransmission(slaveAddress);
	Wire.write(cmd);
	Wire.write(data);
	Wire.endTransmission();
}

//レジスタの値を読み込む
uint8_t	TSL2561::ReadByte(uint8_t regAddr)
{
	regAddr = (1 << ECmdReg::CMD) | regAddr;

	Wire.beginTransmission(slaveAddress);
	Wire.write(regAddr);
	Wire.endTransmission();

	Wire.requestFrom(slaveAddress, (size_t)1);
	uint8_t	data = Wire.read();
	return data;
}

//レジスタの値を読み込む
//参考：TSL2561は指定アドレスからLoByte-HiByteの順に値が格納されている。
uint16_t	TSL2561::ReadWord(uint8_t regAddr)
{
	regAddr = (1 << ECmdReg::CMD) | (1 << ECmdReg::WORD) | regAddr;

	Wire.beginTransmission(slaveAddress);
	Wire.write(regAddr);
	Wire.endTransmission();

	Wire.requestFrom(slaveAddress, (size_t)2);
	uint8_t	loByte = Wire.read();
	uint8_t	hiByte = Wire.read();
	uint16_t	data = (hiByte << 8) | loByte;
	return data;
}

//コマンドを送信する
void	TSL2561::SendCommand(ECmdAddr cmd, uint8_t regBits)
{
	uint8_t	cmdReg = (1 << ECmdReg::CMD) | (cmd << ECmdReg::ADDRESS) |
		(0 << ECmdReg::CLEAR) | (0 << ECmdReg::WORD) | (0 << ECmdReg::BLOCK);
	WriteByte(cmdReg, regBits);
}

//初期化
bool	TSL2561::Initialize(EGain gain, EIntegTime integTime)
{
	PowerOn();

	//TSL2561と通信できているか
	//・PowerUp()の後、CONTROLレジスタの下位2bitがEPower::Upと同値なら通信できている
	uint8_t	data = ReadByte(ECmdAddr::CONTROL);
	if ((data & 0x03) != EPower::Up) { return false; }

	SetTiming(gain, integTime);
	deviceId = GetDeviceID();
	return true;
}

/*	Control Register (0h)
The CONTROL register contains two bits and is primarily used to power the TSL256x device up and down as
shown in Table 4.
*/
//センサー動作開始
uint16_t	TSL2561::PowerOn(bool waitIntegTime)
{
	SendCommand(ECmdAddr::CONTROL, EPower::Up);

	uint16_t	waitMSec = 0;
	if (!waitIntegTime) { return waitMSec; }
	switch (configuredIntegTime)
	{
	case EIntegTime::IntegT13ms:	waitMSec = 20;	break;
	case EIntegTime::IntegT101ms:	waitMSec = 110;	break;
	case EIntegTime::IntegT402ms:	waitMSec = 410;	break;
	default:	break;
	}
	delay(waitMSec);
	return waitMSec;
}

//センサー動作停止
void	TSL2561::PowerOff(void)
{
	SendCommand(ECmdAddr::CONTROL, EPower::Down);
}

/*	Timing Register (1h)
The TIMING register controls both the integration time and the gain of the ADC channels. A common set of
control bits is provided that controls both ADC channels. The TIMING register defaults to 02h at power on.
*/
void	TSL2561::SetTiming(EGain gain, EIntegTime integTime)
{
	uint8_t regBits = (gain << ETimingReg::GAIN) | (integTime << ETimingReg::INTEG) |
		(0 << ETimingReg::Manual);
	SendCommand(ECmdAddr::TIMING, regBits);

	//指定値を保存する
	configuredGain = gain;
	configuredIntegTime = integTime;
}

/*	Interrupt Threshold Register (2h − 5h)
	省略
*/

/*	Interrupt Control Register (6h)
	省略
*/

/*	ID Register (Ah)
The ID register provides the value for both the part number and silicon revision number for that part number.
It is a read-only register, whose value never changes.
*/
EPartNumId	TSL2561::GetDeviceID(uint8_t* retRevisionNumberId)
{
	uint8_t	data = ReadByte(ECmdAddr::ID);
	if (retRevisionNumberId != nullptr) { *retRevisionNumberId = data & 0x0F; }
	uint8_t	partNumberId = (data >> 4) & 0x0F;

	switch (partNumberId)
	{
	case 0b0000:	return EPartNumId::TSL2560CS;
	case 0b0001:	return EPartNumId::TSL2561CS;
	case 0b0100:	return EPartNumId::TSL2560T;
	case 0b0101:	return EPartNumId::TSL2561T;
	default:	break;
	}
	return EPartNumId::IdUnknown;
}

//デバイス名を取得する
String	TSL2561::DeviceName(void)
{
	switch (deviceId)
	{
	case EPartNumId::TSL2560CS:	return String("TSL2560CS");
	case EPartNumId::TSL2561CS:	return String("TSL2561CS");
	case EPartNumId::TSL2560T:	return String("TSL2560T/FN/CL");
	case EPartNumId::TSL2561T:	return String("TSL2561T/FN/CL");
	default:	break;
	}
	return String("Unknown");
}

/*	ADC Channel Data Registers (Ch − Fh)
The ADC channel data are expressed as 16-bit values spread across two registers. The ADC channel 0 data
registers, DATA0LOW and DATA0HIGH provide the lower and upper bytes, respectively, of the ADC value of
channel 0. Registers DATA1LOW and DATA1HIGH provide the lower and upper bytes, respectively, of the ADC
value of channel 1. All channel data registers are read-only and default to 00h on power up.
参考：
[Calculating Lux]
One of the photodiodes (channel 0) is sensitive to both visible and infrared light, while the second photodiode (channel 1) is sensitive primarily to infrared light. 
→フォトダイオードの 1 つ (チャネル 0) は可視光と赤外光の両方に敏感ですが、2 番目のフォトダイオード (チャネル 1) は主に赤外光に敏感です。
*/
void	TSL2561::GetAdcChannelData(uint16_t* ch0Data, uint16_t* ch1Data)
{
	*ch0Data = ReadWord(ECmdAddr::DATA0LOW);
	*ch1Data = ReadWord(ECmdAddr::DATA1LOW);
}

//Calculating Lux
uint16_t	TSL2561::Lux(void)
{
	uint16_t	ch0Data, ch1Data;
	GetAdcChannelData(&ch0Data, &ch1Data);
Serial.printf("ch0=%d, ch1=%d\n", ch0Data, ch1Data);
	unsigned int ch0 = static_cast<unsigned int>(ch0Data);
	unsigned int ch1 = static_cast<unsigned int>(ch1Data);
	unsigned int iGain = static_cast<unsigned int>(configuredGain);
	unsigned int tInt = static_cast<unsigned int>(configuredIntegTime);
	int iType = (deviceId == EPartNumId::TSL2560T || deviceId == EPartNumId::TSL2561T) ? 0 : 1;
	unsigned int calcLux = CalculateLux(iGain, tInt, ch0, ch1, iType);
	uint16_t	lux = static_cast<uint16_t>(calcLux);
	return lux;
}

// lux equation approximation without floating point calculations
//////////////////////////////////////////////////////////////////////////////
// Routine: unsigned int CalculateLux(unsigned int ch0, unsigned int ch0, int iType)
//
// Description: Calculate the approximate illuminance (lux) given the raw
// channel values of the TSL2560. The equation if implemented
// as a piece−wise linear approximation.
//
// Arguments: unsigned int iGain − gain, where 0:1X, 1:16X
// unsigned int tInt − integration time, where 0:13.7mS, 1:100mS, 2:402mS,
// 3:Manual
// unsigned int ch0 − raw channel value from channel 0 of TSL2560
// unsigned int ch1 − raw channel value from channel 1 of TSL2560
// unsigned int iType − package type (T or CS)
//
// Return: unsigned int − the approximate illuminance (lux)
//
//////////////////////////////////////////////////////////////////////////////
unsigned int TSL2561::CalculateLux(unsigned int iGain, unsigned int tInt,
	unsigned int ch0, unsigned int ch1, int iType)
{
	//−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−
	// first, scale the channel values depending on the gain and integration time
	// 16X, 402mS is nominal.

	// scale if integration time is NOT 402 msec
	unsigned long chScale;
	unsigned long channel1;
	unsigned long channel0;
	switch (tInt)
	{
	case 0: // 13.7 msec
		chScale = CHSCALE_TINT0;
		break;
	case 1: // 101 msec
		chScale = CHSCALE_TINT1;
		break;
	default: // assume no scaling
		chScale = (1 << CH_SCALE);
		break;
	}

	// scale if gain is NOT 16X
	if (!iGain) chScale = (chScale << 4); // scale 1X to 16X

	// scale the channel values
	channel0 = ((ch0 * chScale) >> CH_SCALE);
	channel1 = ((ch1 * chScale) >> CH_SCALE);
	//−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−

	// find the ratio of the channel values (Channel1/Channel0)
	// protect against divide by zero
	unsigned long ratio1 = 0;
	if (channel0 != 0) ratio1 = (channel1 << (RATIO_SCALE+1)) / channel0;

	// round the ratio value
	unsigned long ratio = ((ratio1 + 1) >> 1);

	// is ratio <= eachBreak ?
	unsigned int b, m;
	switch (iType)
	{
	case 0: // T, FN and CL package
		if ((ratio >= 0) && (ratio <= K1T)) {b=B1T; m=M1T;}
		else if (ratio <= K2T) {b=B2T; m=M2T;}
		else if (ratio <= K3T) {b=B3T; m=M3T;}
		else if (ratio <= K4T) {b=B4T; m=M4T;}
		else if (ratio <= K5T) {b=B5T; m=M5T;}
		else if (ratio <= K6T) {b=B6T; m=M6T;}
		else if (ratio <= K7T) {b=B7T; m=M7T;}
		else if (ratio > K8T) {b=B8T; m=M8T;}
		break;

	case 1:// CS package
		if ((ratio >= 0) && (ratio <= K1C)) {b=B1C; m=M1C;}
		else if (ratio <= K2C) {b=B2C; m=M2C;}
		else if (ratio <= K3C) {b=B3C; m=M3C;}
		else if (ratio <= K4C) {b=B4C; m=M4C;}
		else if (ratio <= K5C) {b=B5C; m=M5C;}
		else if (ratio <= K6C) {b=B6C; m=M6C;}
		else if (ratio <= K7C) {b=B7C; m=M7C;}
		else if (ratio > K8C) {b=B8C; m=M8C;}
		break;
	}

	//unsigned long temp;	//↓
	signed long temp;		//↓2行下で負数判定している。ということはunsignedは誤り。
	temp = ((channel0 * b) - (channel1 * m));

	// do not allow negative lux value
	if (temp < 0) temp = 0;

	// round lsb (2^(LUX_SCALE−1))
	temp += (1 << (LUX_SCALE-1));

	// strip off fractional portion
	unsigned long lux = (temp >> LUX_SCALE);
	return(lux);
}
