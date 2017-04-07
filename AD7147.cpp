#include "AD7147.h"
#include <SPI.h>

AD7147::AD7147(){
    
}

bool AD7147::begin(int CS_PIN, int INT_PIN){
    this->CS_PIN = CS_PIN;
    this->INT_PIN = INT_PIN;
    pinMode(CS_PIN, OUTPUT);
    pinMode(INT_PIN, INPUT_PULLUP);
    digitalWrite(CS_PIN, HIGH);
    
    SPI.begin();
	
	//make sure we can get device id
	if(getDeviceID() == 0){
		return false;
	}
	
	//create all connections by default
	for(byte i=0; i<NUMBER_OF_SENSORS; i++){
		setConnection(i, i);
		SliderBins[i] = VALS_PER_PAD * i;
	}
   
	//--------------------------------------------------------------------------//
	//-------------------------Bank 1 Registers---------------------------------//
	//--------------------------------------------------------------------------//
	//Initialisation of the first register bank but not the AMBCOMPCTL_REG0
	AD7147Registers[PWR_CONTROL]=0x02B2;	//Register 0x00
	this->write(PWR_CONTROL, 1, AD7147Registers, PWR_CONTROL);
	
	this->read(STAGE_LOW_LIMIT_INT, 3, AD7147Registers, STAGE_LOW_LIMIT_INT);
	AD7147Registers[AMB_COMP_CTRL0]=0x3233;	//Register 0x02
	AD7147Registers[AMB_COMP_CTRL1]=0x0819;	//Register 0x03
	AD7147Registers[AMB_COMP_CTRL2]=0x0832;	//Register 0x04
	this->write(AMB_COMP_CTRL0, 3, AD7147Registers, AMB_COMP_CTRL0);
	
	setIntTypeConversion();
	
	//Enable data path for all sequences
	AD7147Registers[STAGE_CAL_EN]=0x07FF;	//Register 0x01
	this->write(STAGE_CAL_EN, 1, AD7147Registers, STAGE_CAL_EN);

	this->read(STAGE_LOW_LIMIT_INT, 3, AD7147Registers, STAGE_LOW_LIMIT_INT);
	
	return true;
}

void AD7147::setIntTypeThreshold(){
		AD7147Registers[STAGE_LOW_INT_EN]= 0xFFFF & ~(1 << 15);//Register 0x05
		AD7147Registers[STAGE_HIGH_INT_EN]= 0xFFFF & ~(1 << 15);	//Register 0x06
		AD7147Registers[STAGE_COMPLETE_INT_EN]=0x0000;	//Register 0x07
		this->write(STAGE_LOW_INT_EN, 3, AD7147Registers, STAGE_LOW_INT_EN);
}

void AD7147::setIntTypeConversion(){
	AD7147Registers[STAGE_LOW_INT_EN]=0x0000;//Register 0x05
	AD7147Registers[STAGE_HIGH_INT_EN]=0x0000;	//Register 0x06
	AD7147Registers[STAGE_COMPLETE_INT_EN]= 0x0000 | 1 << 11;	//Register 0x07
	this->write(STAGE_LOW_INT_EN, 3, AD7147Registers, STAGE_LOW_INT_EN);
}

void AD7147::readAllSensors(void){
	
	byte i;
	uint16_t  AmbientValueAddress;
		
	//If any slider stages is activated then we read data from the AD7147
	this->read(ADCRESULT_S0, NUMBER_OF_SENSORS, AD7147Registers, ADCRESULT_S0);
		
	AmbientValueAddress=STAGE0_AMBIENT;
	for(i=0;i<NUMBER_OF_SENSORS;i++)
	{
		this->read(AmbientValueAddress, 1, AmbientValues, i);
		AmbientValueAddress=AmbientValueAddress+NB_OF_REGS_PER_STAGE;
	}
	
	//Calculate the sensor responses
	for(i=0;i<NUMBER_OF_SENSORS;i++)
	{
		if(AD7147Registers[ADCRESULT_S0+i]>AmbientValues[i])
		SensorValues[i]=abs(AD7147Registers[ADCRESULT_S0+i]-AmbientValues[i]);
		else
		SensorValues[i]=0;
	}
}

void AD7147::setAmbientValues(void)
{
	byte i;
	uint16_t AmbientValueAddress;
		
	AmbientValueAddress = STAGE0_AMBIENT;
	for (i=0; i<NUMBER_OF_SENSORS; i++)
	{	
		//Initialise ambient values
		this->read(AmbientValueAddress, 1, AmbientValues, i);
		AmbientValueAddress=AmbientValueAddress+NB_OF_REGS_PER_STAGE;
		SensorValues[i] = 0;
	}
    Serial.println("ambient values set!");
}

bool AD7147::update(void)
{	
    if(digitalRead(INT_PIN)) return false;
    
	//Read thresholds registers
	this->read(STAGE_LOW_LIMIT_INT, 3, AD7147Registers, STAGE_LOW_LIMIT_INT);
	
	if(InterruptCounterForInitialisation < (NB_OF_INT+2))
	{		
		if (InterruptCounterForInitialisation==(NB_OF_INT-1))
		{
			setAmbientValues();
			
			setIntTypeThreshold();
		}
		InterruptCounterForInitialisation++;
		
		this->read(ADCRESULT_S0, NUMBER_OF_SENSORS, AD7147Registers, ADCRESULT_S0);
		return false;
	}
	
	if(InterruptCounterForInitialisation>=NB_OF_INT)
	{
		//============================
		//= Recalibrate if required. =
		//============================
		if ((AD7147Registers[STAGE_LOW_LIMIT_INT] & POWER_UP_INTERRUPT) != 0x0000)
		{
			forceCalibration();
			InterruptCounterForInitialisation = 0;
		}
		else
		{
			readAllSensors();
			//setIntTypeConversion();
			return true;
		}
	}
}

void AD7147::setConnection(byte stage, byte pos){
	//this only supports single ended positive currently
	uint16_t StageBuffer[8];
	
	if(pos > 6){
		StageBuffer[0]=0xFFFF;
		StageBuffer[1]=CS_SETUP_SINGLE_POS(CS_SINGLE_ENDED_POS(pos - 7));
	}
	else{
		StageBuffer[0]= CS_SINGLE_ENDED_POS(pos);
		StageBuffer[1] = CS_SETUP_SINGLE_POS(0xFFFF);
	}
	
	StageBuffer[2]=0x0000;
	StageBuffer[3]=0x2424;
	StageBuffer[4]=1600;
	StageBuffer[5]=1600;
	StageBuffer[6]=1600;
	StageBuffer[7]=1600;
	this->write(STAGE0_CONNECTION + (stage * 8), 8, StageBuffer, 0);
}

uint16_t AD7147::getDeviceID(){
	AD7147Registers[DEVID] = 0x00;
	this->read(DEVID, 1, AD7147Registers, DEVID);
	return AD7147Registers[DEVID];
}

uint16_t AD7147::calculateSiderPosition(){
	//this assumes the pads are set up in a line as a slider
	uint16_t sum = 0;
	float total = 0;
	bool threshReached = false;
	
	for (byte i=0; i<NUMBER_OF_SENSORS; i++) {
		sum += SensorValues[i];
		if(SensorValues[i] > ACTIVATION_THRESHOLD) threshReached = true;
	}
	if(!threshReached) return 0;
	
	for (byte i=0; i<NUMBER_OF_SENSORS; i++) {
		 total += (float)((float)SensorValues[i] / (float)sum) * SliderBins[i];
	}
	return (uint16_t)total;
}

void AD7147::forceCalibration(void)
{
	this->read(AMB_COMP_CTRL0, 1, AD7147Registers, AMB_COMP_CTRL0);
	AD7147Registers[AMB_COMP_CTRL0] |= 0x4000;//Set "forced cal" bool
	this->write(AMB_COMP_CTRL0, 1, AD7147Registers, AMB_COMP_CTRL0);
}

void AD7147::write(const uint16_t RegisterAddress, const byte NumberOfRegisters, uint16_t *DataBuffer, const byte OffsetInBuffer)
{
	uint16_t ControlValue;
	uint16_t ValueToWrite;
	uint16_t RegisterIndex;

	//Create the 16-bool header
	ControlValue = 0xE000 | (RegisterAddress & POWER_UP_INTERRUPT);
	
        digitalWrite(CS_PIN, LOW);
        SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
        
        SPI.transfer16(ControlValue);
        
        for (RegisterIndex=0; RegisterIndex<NumberOfRegisters; RegisterIndex++)
	{
		ValueToWrite= *(DataBuffer+RegisterIndex+OffsetInBuffer);
		SPI.transfer16(ValueToWrite);
	}
        
        SPI.endTransaction();
        digitalWrite(CS_PIN, HIGH);
}


void AD7147::read(const uint16_t RegisterStartAddress, const byte NumberOfRegisters, uint16_t *DataBuffer, const uint16_t OffsetInBuffer)
{
	uint16_t ControlValue;
	byte RegisterIndex;

	//Create the 16-bool header
	ControlValue = 0xE400 | (RegisterStartAddress & 0x03FF);
	
        digitalWrite(CS_PIN, LOW);
        SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
        
        SPI.transfer16(ControlValue);
        
        for (RegisterIndex=0; RegisterIndex<NumberOfRegisters; RegisterIndex++)
        {
            *(DataBuffer+OffsetInBuffer+RegisterIndex) = SPI.transfer16(0x00);
        }
        
        SPI.endTransaction();
        digitalWrite(CS_PIN, HIGH);
}
