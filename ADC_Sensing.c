/*
 * ADC_Sensing.c
 *
 *  Created on:  08-may-2023
 *      Author: Pradeep Kumar----Ankit
 */
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "inc/hw_types.h"
#include "inc/tm4c123be6pm.h"
//#include "inc/tm4c1231h6pz.h" //this need to be change in actual Board@@@
//Interrupt file also need to change through selecting actual controller in Properties@@@
#include "driverlib/rom.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/adc.h"
#include "ADC_sensing.h"

//#define TARGET_IS_BLIZZARD_RB1
uint32_t ADC0SS0Buff[3]; //STORE ADC0SS0 DATA
uint32_t ADC0SS0Valuesum[20];
uint32_t ADC1SS0Buff[3]; //STORE ADC0SS0 DATA
uint32_t ADC1SS0Valuesum[3];
uint32_t ADC0SS3Buff[1]; //STORE ADC0SS3 DATA
uint32_t ADC0SS3Valuesum[1];
uint32_t ADC0SS0_count=0;
uint32_t ADC1SS0_count=0;
uint32_t ADC0SS3_count=0;
uint32_t ADCSampleBuff[20];
static uint32_t ADCSampleCounter;
Reading_Struct ADC_reading;
extern uint8_t  channel_MUX;
uint16_t ShuntCurrentFactor[6];
uint16_t ShuntMiliVoltFactor[6];
uint16_t ShuntCalibrationFactor[6];
uint16_t ShuntCalibrationAdjuster[6];
uint16_t VoltageCalibrationFactor[8];
uint16_t VoltageCalibrationAdjuster[8];
uint16_t OtherCalibrationFactor[5];
uint16_t OtherCalibrationAdjuster[5];
uint16_t TemperatureFactor[3];
uint16_t TemperatureAdjuster[3];
uint16_t NTCTemperatureBetaValue;
uint16_t NTCNominalTemperature;
uint16_t NTCNominalResistance;
uint16_t NTCTemperatureResistorValue;

void ADC_Configure(void)
{
    ///////////////////////////ADC0 READ CONFIGURE///////////////////////////////////
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);   //Enable ADC0 Peripheral
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);   //Enable ADC1 Peripheral
    ROM_SysCtlPeripheralReset(SYSCTL_PERIPH_ADC0);    //RESET ADC0
    ROM_SysCtlPeripheralReset(SYSCTL_PERIPH_ADC1);    //RESET ADC1
    ROM_ADCSequenceDisable(ADC0_BASE, SEQ0);
    //ROM_ADCSequenceDisable(ADC0_BASE, SEQ3);
    ROM_ADCSequenceDisable(ADC1_BASE, SEQ0);
    ROM_ADCSequenceConfigure(ADC0_BASE, SEQ0, ADC_TRIGGER_PROCESSOR, 0);  //CONFIGURE ADC0 SEQUENCE0 //get 8 channels
    //ROM_ADCSequenceConfigure(ADC0_BASE, SEQ3, ADC_TRIGGER_PROCESSOR, 0);  //CONFIGURE ADC0 SEQUENCE3 //get 1 channel
    ROM_ADCSequenceConfigure(ADC1_BASE, SEQ0, ADC_TRIGGER_PROCESSOR, 0);  //CONFIGURE ADC1 SEQUENCE0 //get 8 channels
    // SysCtlPeripheralEnable(ADC_PORT_INIT1);   //ENABLE GPOI E //Alreay initialized in gpio.c
    // SysCtlPeripheralEnable(ADC_PORT_INIT2);   //ENABLE GPOI D //Alreay initialized in gpio.c
    // SysCtlPeripheralEnable(ADC_PORT_INIT3);   //ENABLE GPOI B //Alreay initialized in gpio.c
    // SysCtlPeripheralEnable(ADC_PORT_INIT4);   //ENABLE GPOI H/k -->Depends upon Controller
    ROM_GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_2);
    ROM_GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_2|GPIO_PIN_0);
    //AN0-->PE3, AN1-->PE2, AN2-->PE1, AN4-->PD3, AN5-->PD2, AN6-->PD1, AN7-->PD0,
    ROM_ADCHardwareOversampleConfigure(ADC0_BASE, 2);
    ROM_ADCHardwareOversampleConfigure(ADC1_BASE, 2);
    ROM_ADCSequenceStepConfigure(ADC0_BASE, SEQ0, 0,   CH_00);  //SINGLE ENDED MODE BAT_TEMP
    ROM_ADCSequenceStepConfigure(ADC0_BASE, SEQ0, 1,   CH_01);  //SINGLE ENDED MODE
    ROM_ADCSequenceStepConfigure(ADC0_BASE, SEQ0, 2,   CH_02|ADC_CTL_IE | ADC_CTL_END);  //SINGLE ENDED MODE

    ROM_ADCSequenceStepConfigure(ADC1_BASE, SEQ0, 0,   CH_04);//SOLV
//	ROM_ADCSequenceStepConfigure(ADC1_BASE, SEQ0, 1,   CH_05);
	ROM_ADCSequenceStepConfigure(ADC1_BASE, SEQ0, 1,   CH_06);//Cabinate_Temp
	ROM_ADCSequenceStepConfigure(ADC1_BASE, SEQ0, 2,   CH_07|ADC_CTL_IE | ADC_CTL_END);//Battery_Temp

    //Get 1 channels from AN18 SS3
    /////////////////TRIGGER ADC0 SS0 INTERRUPT//////////////////
    ROM_IntEnable(INT_ADC0SS0);                 //ENABLE INTERRUPT FOR ADC0 SEQUENCER 0
    ROM_ADCIntEnable(ADC0_BASE, SEQ0);          //ENABLE ADC0 SS0 INTERRUPT
    ROM_ADCSequenceEnable(ADC0_BASE, SEQ0);     //ENABLE ADC0 SEQUENCER0
    /////////////////TRIGGER ADC0 SS3 INTERRUPT//////////////////
    //ROM_IntEnable(INT_ADC0SS3);               //ENABLE INTERRUPT FOR ADC0 SEQUENCER 3
    //ROM_ADCIntEnable(ADC0_BASE, SEQ3);        //ENABLE ADC0 SS1 INTERRUPT
    //ROM_ADCSequenceEnable(ADC0_BASE, SEQ3);   //ENABLE ADC0 SEQUENCER1
    /////////////////TRIGGER ADC1 SS0 INTERRUPT//////////////////
    ROM_IntEnable(INT_ADC1SS0);                 //ENABLE INTERRUPT FOR ADC1 SEQUENCER 0
    ROM_ADCIntEnable(ADC1_BASE, SEQ0);          //ENABLE ADC1 SS0 INTERRUPT
    ROM_ADCSequenceEnable(ADC1_BASE, SEQ0);     //ENABLE ADC1 SEQUENCER0

    ROM_IntMasterEnable(); // ENABLE PROCESSOR INTERRUPTS
    ////////////////////////////////////////////////////////////////
    //ROM_ADCProcessorTrigger(ADC0_BASE, SEQ0);  //TRIGGER PROCESSOR ADC0 SEQUENCER0
  //  ROM_ADCProcessorTrigger(ADC0_BASE, SEQ3);  //TRIGGER PROCESSOR ADC0 SEQUENCER3
    //ROM_ADCProcessorTrigger(ADC1_BASE, SEQ0);  //TRIGGER PROCESSOR ADC1 SEQUENCER0
    ///////////////////////////////////////////////////////////////////
}
void ADC0SS0ISRHandler(void)
{
    while (!ROM_ADCIntStatus(ADC0_BASE, SEQ0, false)){}; //WAIT WHILE CONVERSION IS IN PROGRESS
    ROM_ADCIntClear(ADC0_BASE, SEQ0);   //CLEAR ADC0 SS0 INTERRUPT//
    ROM_ADCSequenceDataGet(ADC0_BASE, SEQ0, ADC0SS0Buff);
 //   ROM_ADCProcessorTrigger(ADC0_BASE, SEQ0);  //TRIGGER PROCESSOR
    /*************************Calculation of Gain for Final Value********************************/

    /*************************Update Values in Structure*****************************************/

    switch(channel_MUX)
	{
	case 0:
		//ADC_reading.Channel.BATV1 = ADC0SS0Buff[0];
		//ADC_reading.Channel.BATI = ADC0SS0Buff[1];  d
		//ADC_reading.Channel.IN3 = ADC0SS0Buff[2];
		ADCSampleBuff[Volt_1] += ADC0SS0Buff[0];
		ADCSampleBuff[Shunt_1] += ADC0SS0Buff[1];
		ADCSampleBuff[Shunt_5] += ADC0SS0Buff[2];

		break;
	case 1:
		//ADC_reading.Channel.BATV2 = ADC0SS0Buff[0];
		//ADC_reading.Channel.SYSI = ADC0SS0Buff[1];
		//ADC_reading.Channel.IN4 = ADC0SS0Buff[2];
		ADCSampleBuff[Volt_2] += ADC0SS0Buff[0];
		ADCSampleBuff[Shunt_2] += ADC0SS0Buff[1];
		ADCSampleBuff[Shunt_6] += ADC0SS0Buff[2];
		break;
	case 2:
		//ADC_reading.Channel.INV1 = ADC0SS0Buff[0];
		//ADC_reading.Channel.IN1 = ADC0SS0Buff[1];
		ADCSampleBuff[Volt_3] += ADC0SS0Buff[0];
		ADCSampleBuff[Shunt_3] += ADC0SS0Buff[1];
		break;
	case 3:
		//ADC_reading.Channel.INV2 = ADC0SS0Buff[0];
		//ADC_reading.Channel.IN2 = ADC0SS0Buff[1];
		ADCSampleBuff[Volt_4] += ADC0SS0Buff[0];
		ADCSampleBuff[Shunt_4] += ADC0SS0Buff[1];
		break;
	case 4:
		//ADC_reading.Channel.INV3 = ADC0SS0Buff[0];
		//ADC_reading.Channel.BATI = ADC0SS0Buff[1];
		//ADC_reading.Channel.IN3 = ADC0SS0Buff[2];
		ADCSampleBuff[Volt_5] += ADC0SS0Buff[0];
		ADCSampleBuff[Shunt_1] += ADC0SS0Buff[1];
		ADCSampleBuff[Shunt_5] += ADC0SS0Buff[2];
		break;
	case 5:
		//ADC_reading.Channel.Fusefail_1 = ADC0SS0Buff[0];
		//ADC_reading.Channel.SYSI = ADC0SS0Buff[1];
		//ADC_reading.Channel.IN4 = ADC0SS0Buff[2];
		ADCSampleBuff[Volt_6] += ADC0SS0Buff[0];
		ADCSampleBuff[Shunt_2] += ADC0SS0Buff[1];
		ADCSampleBuff[Shunt_6] += ADC0SS0Buff[2];
		break;
	case 6:
		//ADC_reading.Channel.Fusefail_2 = ADC0SS0Buff[0];
		//ADC_reading.Channel.IN1 = ADC0SS0Buff[1];
		ADCSampleBuff[Volt_7] += ADC0SS0Buff[0];
		ADCSampleBuff[Shunt_3] += ADC0SS0Buff[1];
		break;
	case 7:
		//ADC_reading.Channel.HUM = ADC0SS0Buff[0];
		//ADC_reading.Channel.IN2 = ADC0SS0Buff[1];
		ADCSampleBuff[Volt_8] += ADC0SS0Buff[0];
		ADCSampleBuff[Shunt_4] += ADC0SS0Buff[1];
		break;
	}

    ADCSampleCounter++;
    if(ADCSampleCounter == 10000)
    {
    	ADCSampleCounter = 0;
    	ADC_reading.Flags.ADCSampleAfterDelay = 1;
    }
    else
    {
    	ROM_ADCProcessorTrigger(ADC0_BASE, SEQ0);
		ROM_ADCProcessorTrigger(ADC1_BASE, SEQ0);
    }



    /*ui32ADC0SS0Valuesum[0] += ui32ADC0SS0Value[0];
    ui32ADC0SS0Valuesum[1] += ui32ADC0SS0Value[1];
    ui32ADC0SS0Valuesum[2] += ui32ADC0SS0Value[2];
    ui32ADC0SS0Valuesum[3] += ui32ADC0SS0Value[3];
    ui32ADC0SS0Valuesum[4] += ui32ADC0SS0Value[4];
    ui32ADC0SS0Valuesum[5] += ui32ADC0SS0Value[5];
    ui32ADC0SS0Valuesum[6] += ui32ADC0SS0Value[6];
    ui32ADC0SS0Valuesum[7] += ui32ADC0SS0Value[7];
    ADC0SS0_count++;
    if(ADC0SS0_count>=ADC0SS0_Samplecount)
    {
        ADC_reading.BIT_FIELD.Shunt_1 = (ui32ADC0SS0Valuesum[0]/ADC0SS0_count);
        ADC_reading.BIT_FIELD.Shunt_2 =  (ui32ADC0SS0Valuesum[1]/ADC0SS0_count);
        ADC_reading.BIT_FIELD.Shunt_3 =  (ui32ADC0SS0Valuesum[2]/ADC0SS0_count);
        ADC_reading.BIT_FIELD.Shunt_4 =  (ui32ADC0SS0Valuesum[3]/ADC0SS0_count);
        ADC_reading.BIT_FIELD.Shunt_5 =  (ui32ADC0SS0Valuesum[4]/ADC0SS0_count);
        ADC_reading.BIT_FIELD.Shunt_6 =  (ui32ADC0SS0Valuesum[5]/ADC0SS0_count);
        ADC_reading.BIT_FIELD.Volt_1 =  (ui32ADC0SS0Valuesum[6]/ADC0SS0_count);
        ADC_reading.BIT_FIELD.Volt_2 =  (ui32ADC0SS0Valuesum[7]/ADC0SS0_count);
        ui32ADC0SS0Valuesum[0]=0;
        ui32ADC0SS0Valuesum[1]=0;
        ui32ADC0SS0Valuesum[2]=0;
        ui32ADC0SS0Valuesum[3]=0;
        ui32ADC0SS0Valuesum[4]=0;
        ui32ADC0SS0Valuesum[5]=0;
        ui32ADC0SS0Valuesum[6]=0;
        ui32ADC0SS0Valuesum[7]=0;
        ADC0SS0_count=0;
    }*/
    //ROM_ADCProcessorTrigger(ADC0_BASE, SEQ0);  //TRIGGER PROCESSOR
}
void ADC1SS0ISRHandler(void)
{
    while (!ROM_ADCIntStatus(ADC1_BASE, SEQ0, false)){}; //WAIT WHILE CONVERSION IS IN PROGRESS
    ROM_ADCIntClear(ADC1_BASE, SEQ0);   //CLEAR ADC0 SS3 INTERRUPT//
    ROM_ADCSequenceDataGet(ADC1_BASE, SEQ0, ADC1SS0Buff);
  //  ROM_ADCProcessorTrigger(ADC1_BASE, SEQ0);  //TRIGGER PROCESSOR
    /*************************Calculation of Gain for Final Value********************************/

    /*************************Update Values in Structure*****************************************/

    /*ADC_reading.Channel.SOLV = ADC1SS0Buff[0];
    ADC_reading.Channel.Cabinet_Temp = ADC1SS0Buff[1];
    ADC_reading.Channel.Battery_Temp = ADC1SS0Buff[2];*/
    ADCSampleBuff[Solv] += ADC1SS0Buff[0];
    ADCSampleBuff[CabinetTemp] += ADC1SS0Buff[1];
    ADCSampleBuff[BattTemp] += ADC1SS0Buff[2];

    /*ui32ADC1SS0Valuesum[0] += ui32ADC1SS0Value[0];
    ui32ADC1SS0Valuesum[1] += ui32ADC1SS0Value[1];
    ui32ADC1SS0Valuesum[2] += ui32ADC1SS0Value[2];
    ui32ADC1SS0Valuesum[3] += ui32ADC1SS0Value[3];
    ui32ADC1SS0Valuesum[4] += ui32ADC1SS0Value[4];
    ui32ADC1SS0Valuesum[5] += ui32ADC1SS0Value[5];
    ui32ADC1SS0Valuesum[6] += ui32ADC1SS0Value[6];
    ui32ADC1SS0Valuesum[7] += ui32ADC1SS0Value[7];
    ADC1SS0_count++;
    if(ADC1SS0_count>=ADC1SS0_Samplecount)
    {
        ADC_reading.BIT_FIELD.Volt_3 = (ui32ADC1SS0Valuesum[0]/ADC1SS0_count);
        ADC_reading.BIT_FIELD.Volt_4 =  (ui32ADC1SS0Valuesum[1]/ADC1SS0_count);
        ADC_reading.BIT_FIELD.Volt_5 =  (ui32ADC1SS0Valuesum[2]/ADC1SS0_count);
        ADC_reading.BIT_FIELD.Volt_6 =  (ui32ADC1SS0Valuesum[3]/ADC1SS0_count);
        ADC_reading.BIT_FIELD.Batt_fuse1 =  (ui32ADC1SS0Valuesum[4]/ADC1SS0_count);
        ADC_reading.BIT_FIELD.Batt_fuse2 =  (ui32ADC1SS0Valuesum[5]/ADC1SS0_count);
        ADC_reading.BIT_FIELD.BAT_temp =  (ui32ADC1SS0Valuesum[6]/ADC1SS0_count);
        ADC_reading.BIT_FIELD.CABINET_temp =  (ui32ADC1SS0Valuesum[7]/ADC1SS0_count);
        ui32ADC1SS0Valuesum[0]=0;
        ui32ADC1SS0Valuesum[1]=0;
        ui32ADC1SS0Valuesum[2]=0;
        ui32ADC1SS0Valuesum[3]=0;
        ui32ADC1SS0Valuesum[4]=0;
        ui32ADC1SS0Valuesum[5]=0;
        ui32ADC1SS0Valuesum[6]=0;
        ui32ADC1SS0Valuesum[7]=0;
        ADC1SS0_count=0;
    }*/
   // ROM_ADCProcessorTrigger(ADC1_BASE, SEQ0);  //TRIGGER PROCESSOR
}
void ADC0SS3ISRHandler(void)
{
    while (!ROM_ADCIntStatus(ADC0_BASE, SEQ3, false)){}; //WAIT WHILE CONVERSION IS IN PROGRESS
    ROM_ADCIntClear(ADC0_BASE, SEQ3);   //CLEAR ADC0 SS3 INTERRUPT//
    ROM_ADCSequenceDataGet(ADC0_BASE, SEQ3, ADC0SS3Buff);
  //  ROM_ADCProcessorTrigger(ADC0_BASE, SEQ3);  //TRIGGER PROCESSOR
    /*************************Calculation of Gain for Final Value********************************/

    /*************************Update Values in Structure*****************************************/
    ADC0SS3Valuesum[0] += ADC0SS3Buff[0];
    ADC0SS3_count++;
    if(ADC0SS3_count>=ADC0SS3_Samplecount)
    {
        //ADC_reading.BIT_FIELD.SHEL_temp = (ADC0SS3Valuesum[0]/ADC0SS3_count);
        //ADC0SS3Valuesum[0]=0;
        //ADC0SS3_count=0;
    }
   // ROM_ADCProcessorTrigger(ADC0_BASE, SEQ3);  //TRIGGER PROCESSOR
}

void StartSamplingADC(void)
{
	ROM_ADCProcessorTrigger(ADC0_BASE, SEQ0);
	ROM_ADCProcessorTrigger(ADC1_BASE, SEQ0);
}

void ProcessAnalogData(void)
{
	float Value = 0;
	uint8_t x = 0;
	float Temp=0;
	float Temprature=0;

	ADC_reading.Channel.BATV1 = ADCSampleBuff[Volt_1]/(ADC0SS0_Samplecount*VoltSampleMultipleCount);
	ADC_reading.Channel.BATV2 = ADCSampleBuff[Volt_2]/(ADC0SS0_Samplecount*VoltSampleMultipleCount);
	ADC_reading.Channel.INV1 = ADCSampleBuff[Volt_3]/(ADC0SS0_Samplecount*VoltSampleMultipleCount);
	ADC_reading.Channel.INV2 = ADCSampleBuff[Volt_4]/(ADC0SS0_Samplecount*VoltSampleMultipleCount);
	ADC_reading.Channel.INV3 = ADCSampleBuff[Volt_5]/(ADC0SS0_Samplecount*VoltSampleMultipleCount);
	ADC_reading.Channel.Fusefail_1 = ADCSampleBuff[Volt_6]/(ADC0SS0_Samplecount*VoltSampleMultipleCount);
	ADC_reading.Channel.Fusefail_2 = ADCSampleBuff[Volt_7]/(ADC0SS0_Samplecount*VoltSampleMultipleCount);
	ADC_reading.Channel.HUM = ADCSampleBuff[Volt_8]/(ADC0SS0_Samplecount*VoltSampleMultipleCount);

	ADC_reading.Channel.BATI = ADCSampleBuff[Shunt_1]/(ADC0SS0_Samplecount*ShuntSampleMultipleCount);
	ADC_reading.Channel.SYSI = ADCSampleBuff[Shunt_2]/(ADC0SS0_Samplecount*ShuntSampleMultipleCount);
	ADC_reading.Channel.IN1 = ADCSampleBuff[Shunt_3]/(ADC0SS0_Samplecount*ShuntSampleMultipleCount);
	ADC_reading.Channel.IN2 = ADCSampleBuff[Shunt_4]/(ADC0SS0_Samplecount*ShuntSampleMultipleCount);
	ADC_reading.Channel.IN3 = ADCSampleBuff[Shunt_5]/(ADC0SS0_Samplecount*ShuntSampleMultipleCount);
	ADC_reading.Channel.IN4 = ADCSampleBuff[Shunt_6]/(ADC0SS0_Samplecount*ShuntSampleMultipleCount);
	ADC_reading.Channel.SOLV = ADCSampleBuff[Solv]/(ADC0SS0_Samplecount*OtherSampleMultipleCount);
	ADC_reading.Channel.Cabinet_Temp = ADCSampleBuff[CabinetTemp]/(ADC0SS0_Samplecount*OtherSampleMultipleCount);
	ADC_reading.Channel.Battery_Temp = ADCSampleBuff[BattTemp]/(ADC0SS0_Samplecount*OtherSampleMultipleCount);

	if(ADC_reading.Channel.BATI < 2094)
	{
		Value = (2094 - ADC_reading.Channel.BATI)*1000;
		Value = (((Value / (ShuntCurrentFactor[0]/ShuntMiliVoltFactor[0]))*ADCCurrentGainFactor)/100)*(-1);
		ADC_reading.ADCValues.BattCurrent = Value-(ShuntCalibrationAdjuster[0]);
	}
	else
	{
		Value = (ADC_reading.Channel.BATI - 2094)*1000;
		Value = ((Value / (ShuntCurrentFactor[0]/ShuntMiliVoltFactor[0]))*ADCCurrentGainFactor)/100;
		ADC_reading.ADCValues.BattCurrent = Value-(ShuntCalibrationAdjuster[0]);
	}
    if(ADC_reading.Channel.IN3 < 2094)
    {
        Value = (2094 - ADC_reading.Channel.IN3)*1000;
        Value = (((Value / (ShuntCurrentFactor[0]/ShuntMiliVoltFactor[0]))*ADCCurrentGainFactor)/100)*(-1);
        ADC_reading.ADCValues.Load3Current = Value-(ShuntCalibrationAdjuster[0]);
    }
    else
    {
        Value = (ADC_reading.Channel.IN3 - 2094)*1000;
        Value = ((Value / (ShuntCurrentFactor[0]/ShuntMiliVoltFactor[0]))*ADCCurrentGainFactor)/100;
        ADC_reading.ADCValues.Load3Current = Value-(ShuntCalibrationAdjuster[0]);
    }
	for(x=0;x<5;x++)
	{
		if(ADC_reading.BYTE_ARRAY_FIELD[Shunt_2+x] < 2094)
			ADC_reading.BYTE_ARRAY_FIELD[Shunt_2+x] = 2094;
		Value = (ADC_reading.BYTE_ARRAY_FIELD[Shunt_2+x] - 2094)*1000;
		Value = ((Value / (ShuntCurrentFactor[x+1]/ShuntMiliVoltFactor[x+1]))*ADCCurrentGainFactor)/100;
		if((Value-ShuntCalibrationAdjuster[x+1]) < 0)
			ADC_reading.ADCValues.ValueBuff[Shunt_2+x] = 0;
		else
			ADC_reading.ADCValues.ValueBuff[Shunt_2+x] = Value-ShuntCalibrationAdjuster[x+1];
	}

	for(x=0;x<8;x++)
	{
		if(ADC_reading.BYTE_ARRAY_FIELD[Volt_1+x] < 1637)
			ADC_reading.BYTE_ARRAY_FIELD[Volt_1+x] = 1637;
		Value = (ADC_reading.BYTE_ARRAY_FIELD[Volt_1+x] - 1637)*1000;
		Value = (Value*ADCVoltageGainFactor)/10;
		ADC_reading.ADCValues.ValueBuff[Volt_1+x] = Value + VoltageCalibrationAdjuster[x];
	}

	Temp= 0;
	Temprature=0;
	Temp = (float)(4095 - ADC_reading.Channel.Cabinet_Temp);
	Temp = 4095 / Temp - 1;
	Temp = NTCTemperatureResistorValue / Temp;
	Temprature = Temp / NTCNominalResistance;
	Temprature = log(Temprature);
	Temprature /= NTCTemperatureBetaValue;
	Temprature += 1.0 / (NTCNominalTemperature + 273.15);
	Temprature = 1.0 / Temprature;
	Temprature -= 273.15;
	ADC_reading.ADCValues.Cabinet_Temp = Temprature*100;

	Temp= 0;
	Temprature=0;
	Temp = (float)(4095 - ADC_reading.Channel.Battery_Temp);
	Temp = 4095 / Temp - 1;
	Temp = NTCTemperatureResistorValue / Temp;
	Temprature = Temp / NTCNominalResistance;
	Temprature = log(Temprature);
	Temprature /= NTCTemperatureBetaValue;
	Temprature += 1.0 / (NTCNominalTemperature + 273.15);
	Temprature = 1.0 / Temprature;
	Temprature -= 273.15;
	ADC_reading.ADCValues.Battery_Temp = Temprature*100;

	for(x=0;x<20;x++)
	{
		ADCSampleBuff[x] = 0;
	}
	ADC_reading.Flags.ADCConversionComplete = 0;
}



