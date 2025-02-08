//	照度センサー TSL2561 (モジュール名 MH-2561/GY-2561) ライブラリ
//	『昼夜逆転』工作室	http://jsdiy.starfree.jp
//	2025/01	@jsdiy

#pragma	once

#include	<Arduino.h>

//スレーブアドレス(7bit)	※どれか一つを有効、他はコメント化する
//・TSL2561のADDR_SELピンをGND/VCCに接続、または、どことも未接続にすることで、3つのスレーブアドレスを設定できる。
//・モジュール表面の3点パッドの中央がADDR_SELピン。LはGND、HはVCC。デフォルトでは未接続(浮いた状態)。
//#define	TSL2561_SLAVE_ADDRESS	0x29	//モジュール表面のパッド中央をLとショート
#define		TSL2561_SLAVE_ADDRESS	0x39	//モジュールのデフォルト状態
//#define	TSL2561_SLAVE_ADDRESS	0x49	//モジュール表面のパッド中央をHとショート

//Command Address
//・interrupt関係はこのアプリでは非対応とする。
enum	ECmdAddr	: uint8_t
{
	CONTROL			= 0x0,	//Control of basic functions
	TIMING			= 0x1,	//Integration time/gain control
//	THRESHLOWLOW	= 0x2,	//Low byte of low interrupt threshold
//	THRESHLOWHIGH	= 0x3,	//High byte of low interrupt threshold
//	THRESHHIGHLOW	= 0x4,	//Low byte of high interrupt threshold
//	THRESHHIGHHIGH	= 0x5,	//High byte of high interrupt threshold
//	INTERRUPT		= 0x6,	//Interrupt control
//	Reserved0x7		= 0x7,	//Reserved
//	CRC				= 0x8,	//Factory test - not a user register
//	Reserved0x9		= 0x9,	//Reserved
	ID				= 0xA,	//Part number/ Rev ID
//	Reserved0xB		= 0xB,	//Reserved
	DATA0LOW		= 0xC,	//Low  byte of ADC channel 0
	DATA0HIGH		= 0xD,	//High byte of ADC channel 0
	DATA1LOW		= 0xE,	//Low  byte of ADC channel 1
	DATA1HIGH		= 0xF	//High byte of ADC channel 1
};

//ADCのゲイン（Timingレジスタで設定する）
enum	EGain : uint8_t
{
	Gain1x	= 0,	//デフォルト
	Gain16x	= 1
};

//ADCの積分時間（Timingレジスタで設定する）
enum	EIntegTime : uint8_t
{
	IntegT13ms	= 0,	//13.7ms (Scale:0.034)
	IntegT101ms	= 1,	//101ms (Scale:0.252)
	IntegT402ms	= 2		//402ms (Scale:1)	デフォルト
};

//IDレジスタ関連の値
enum	EPartNumId : uint8_t
{
	TSL2560CS	= 0b0000,	//TSL2560CS
	TSL2561CS	= 0b0001,	//TSL2561CS
	TSL2560T	= 0b0100,	//TSL2560T/FN/CL
	TSL2561T	= 0b0101,	//TSL2561T/FN/CL
	IdUnknown	= 0b1111
};

class	TSL2561
{
private:
	const	uint8_t	slaveAddress = TSL2561_SLAVE_ADDRESS;
	EPartNumId	deviceId;
	EGain	configuredGain;
	EIntegTime	configuredIntegTime;
	void	WriteByte(uint8_t cmd, uint8_t data);
	uint8_t	ReadByte(uint8_t regAddr);
	uint16_t	ReadWord(uint8_t regAddr);
	void	SendCommand(ECmdAddr cmd, uint8_t regBits);
	void	SetTiming(EGain gain, EIntegTime integTime);
	EPartNumId	GetDeviceID(uint8_t* retRevisionNumberId = nullptr);
	void	GetAdcChannelData(uint16_t* ch0Data, uint16_t* ch1Data);
	unsigned int CalculateLux(unsigned int iGain, unsigned int tInt, unsigned int ch0, unsigned int ch1, int iType);

public:
	TSL2561(void) {}
	bool	Initialize(EGain gain = EGain::Gain1x, EIntegTime integTime = EIntegTime::IntegT402ms);
	uint16_t	PowerOn(bool waitIntegTime = true);
	void	PowerOff(void);
	String	DeviceName(void);
	uint16_t	Lux(void);
};
