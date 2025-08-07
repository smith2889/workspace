/*
 * ADC_sensing.h
 *
 *  Created on: 20-May-2019
 *      Author: Pradeep Kumar
 */
#include "driverlib/adc.h"
#include "../Configurations/Configuration.h"

#ifndef ADC_SENSING_H_
#define ADC_SENSING_H_

#define ADC0SS0_Samplecount   10000//40
#define ADC1SS0_Samplecount   100//40
#define ADC0SS3_Samplecount   100//40
#define ShuntSampleMultipleCount	2
#define VoltSampleMultipleCount		1
#define OtherSampleMultipleCount	8
#define ADCCurrentGainFactor			1.69117//1.69122
#define ADCVoltageGainFactor			0.05510//0.05518
#define TemperatureGainFactor			0.39091

/**************************SEQUENCER DEFINITION*****************/
#define SEQ0 0
#define SEQ1 1
#define SEQ2 2
#define SEQ3 3
/**************************PORT DEFINITION FOR ADC*****************/
#define ADC_PORT_INIT1   SYSCTL_PERIPH_GPIOE
#define ADC_PORT_INIT2   SYSCTL_PERIPH_GPIOD
#define ADC_PORT_INIT3   SYSCTL_PERIPH_GPIOB
#if TM4C1231H6PZI
#define ADC_PORT_INIT4   SYSCTL_PERIPH_GPIOH
#elif TM4C1294NCPDT
#define ADC_PORT_INIT4   SYSCTL_PERIPH_GPIOK
#endif

/**************************CURRENT CHANNELS*****************/
#define SHUNT_1        ADC_CTL_CH2
#define SHUNT_2        ADC_CTL_CH16
#define SHUNT_3        ADC_CTL_CH1
#define SHUNT_4        ADC_CTL_CH3
#define SHUNT_5        ADC_CTL_CH18
#define SHUNT_6        ADC_CTL_CH17
#define CH_00          ADC_CTL_CH0
#define CH_01          ADC_CTL_CH1
#define CH_02          ADC_CTL_CH2
#define CH_04          ADC_CTL_CH4
#define CH_05          ADC_CTL_CH5
#define CH_06          ADC_CTL_CH6
#define CH_07          ADC_CTL_CH7

/**************************VOLTAGE CHANNELS*****************/
#define BATT1_VOLT     ADC_CTL_CH6
#define BATT2_VOLT     ADC_CTL_CH11
#define BATT3_VOLT     ADC_CTL_CH7
#define BATT4_VOLT     ADC_CTL_CH10
#define BATT5_VOLT     ADC_CTL_CH4
#define BATT6_VOLT     ADC_CTL_CH5

/**************************TEMPERATURE CHANNELS*****************/
#define BAT_TEMP       ADC_CTL_CH0
#define CABINET_TEMP   ADC_CTL_CH12
#define SHEL_TEMP      ADC_CTL_CH13

/**************************FUSE SENSING*************************/
#define BATT_FUSE1     ADC_CTL_CH14
#define BATT_FUSE2     ADC_CTL_CH15

typedef enum {
    Shunt_1=0,
	Shunt_2,
	Shunt_3,
	Shunt_4,
	Shunt_5,
	Shunt_6,
	Volt_1,
	Volt_2,
	Volt_3,
	Volt_4,
	Volt_5,
	Volt_6,
	Volt_7,
	Volt_8,
	BattTemp,
	CabinetTemp,
	BattFuse,
	Solv
} Sensing_index;

#pragma pack(push,1)
typedef union {
    float float_val;
    struct
    {
        uint8_t BYTE_ARRAY_FIELD[4];
    }byte;

}BYTE_FLOAT_CONVERSION;

typedef union
{
		uint32_t BYTE_ARRAY_FIELD[31];
		struct
		{
			unsigned BATI                   	: 32;
			unsigned SYSI                   	: 32;
			unsigned IN1                   		: 32;
			unsigned IN2                   		: 32;
			unsigned IN3               			: 32;
			unsigned IN4               			: 32;
			unsigned BATV1                  	: 32;
			unsigned BATV2                  	: 32;
			unsigned INV1                  		: 32;
			unsigned INV2                  		: 32;
			unsigned INV3                  		: 32;
			unsigned INV4            			: 32;
			unsigned Fusefail_1                	: 32;
			unsigned Fusefail_2                 : 32;
			unsigned Battery_Temp               : 32;
			unsigned Cabinet_Temp	            : 32;
			unsigned UNUSED              		: 32;
			unsigned HUM          				: 32;
			unsigned SOLV              			: 32;
			unsigned INV5          				: 32;
			unsigned UNUSED4              		: 32;
			unsigned UNUSED5          			: 32;
		} Channel;
		struct
		{
			uint32_t UNUSED[21];
			unsigned SetChanel					: 1;
			unsigned StartSmapling				: 1;
			unsigned ADCConversionComplete		: 1;
			unsigned ChannelToggleDelay			: 1;
			unsigned ADCSampleAfterDelay		: 1;
			unsigned UNUSEDBITS					: 27;
		}Flags;
		struct
		{
			uint32_t UNUSED[22];
			union
			{
				uint16_t ValueBuff[20];
				struct
				{
					int16_t BattCurrent;
					uint16_t SysCurrent;
					uint16_t Load1Current;
					uint16_t Load2Current;
					int16_t Load3Current;
					uint16_t Load4Current;
					uint16_t Channel1Volt;
					uint16_t Channel2Volt;
					uint16_t Channel3Volt;
					uint16_t Channel4Volt;
					uint16_t Channel5Volt;
					uint16_t Channel6Volt;
					uint16_t Channel7Volt;
					uint16_t Channel8Volt;
					uint16_t SolarVoltage;
					uint16_t Cabinet_Temp;
					uint16_t Battery_Temp;
					uint16_t Reserved1;
					uint16_t Reserved2;
					uint16_t Reserved3;
				};
			};
		}ADCValues;
} Reading_Struct;

extern uint16_t ShuntCurrentFactor[6];

#pragma pack(pop)
extern void ADC_Configure(void);
extern void StartSamplingADC(void);
extern void ProcessAnalogData(void);



#endif /* ADC_SENSING_H_ */
