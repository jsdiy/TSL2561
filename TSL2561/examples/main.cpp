//	照度センサー TSL2561 (モジュール名 MH-2561/GY-2561) テスト
//	『昼夜逆転』工作室	http://jsdiy.starfree.jp
//	2025/01	@jsdiy

#include	<Arduino.h>
#include	<Wire.h>
#include	"TSL2561.hpp"

TSL2561	sensor;

void	setup(void)
{
	Serial.begin(115200);
	Wire.begin();
	sensor.Initialize(EGain::Gain1x, EIntegTime::IntegT101ms);
}

void	loop(void)
{
	uint16_t waitMSec = sensor.PowerOn();
	uint16_t lux = sensor.Lux();
	sensor.PowerOff();
	Serial.printf("illuminance = %d(Lux)\n", lux);
	delay(1000 - waitMSec);
}
