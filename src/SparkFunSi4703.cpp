#include "Arduino.h"
#include "SparkFunSi4703.h"
#include "Wire.h"

Si4703_Breakout::Si4703_Breakout(int resetPin, int sdioPin, int sclkPin)
{
  _resetPin = resetPin;
  _sdioPin = sdioPin;
  _sclkPin = sclkPin;

  _rdsLastPoll = 0;
  _rdsTimePoll = 0;
  _rdsMinutes = 0;
  _rdsModifiedJulianDayCode = 0;
}

void Si4703_Breakout::powerOn()
{
    si4703_init();
}

void Si4703_Breakout::setChannel(int channel)
{
  //Freq(MHz) = 0.200(in USA) * Channel + 87.5MHz
  //97.3 = 0.2 * Chan + 87.5
  //9.8 / 0.2 = 49
  int newChannel = channel * 10; //973 * 10 = 9730
  newChannel -= 8750; //9730 - 8750 = 980
  newChannel /= 10; //980 / 10 = 98

  // Reset cached RDS infos on channel change
  memset(_rdsName, 0, 9);
  memset(_rdsText, 0, 65);

  //These steps come from AN230 page 20 rev 0.5
  readRegisters();
  si4703_registers[CHANNEL] &= 0xFE00; //Clear out the channel bits
  si4703_registers[CHANNEL] |= newChannel; //Mask in the new channel
  si4703_registers[CHANNEL] |= (1<<TUNE); //Set the TUNE bit to start
  updateRegisters();

  //delay(60); //Wait 60ms - you can use or skip this delay

  //Poll to see if STC is set
  while(1) {
    readRegisters();
    if( (si4703_registers[STATUSRSSI] & (1<<STC)) != 0) break; //Tuning complete!
  }

  readRegisters();
  si4703_registers[CHANNEL] &= ~(1<<TUNE); //Clear the tune after a tune has completed
  updateRegisters();

  //Wait for the si4703 to clear the STC as well
  while(1) {
    readRegisters();
    if( (si4703_registers[STATUSRSSI] & (1<<STC)) == 0) break; //Tuning complete!
  }
}

int Si4703_Breakout::seekUp()
{
	return seek(SEEK_UP);
}

int Si4703_Breakout::seekDown()
{
	return seek(SEEK_DOWN);
}

void Si4703_Breakout::setVolume(int volume)
{
  readRegisters(); //Read the current register set
  if(volume < 0) volume = 0;
  if (volume > 15) volume = 15;
  si4703_registers[SYSCONFIG2] &= 0xFFF0; //Clear volume bits
  si4703_registers[SYSCONFIG2] |= volume; //Set new volume
  updateRegisters(); //Update
}

void Si4703_Breakout::readRDS(char* buffer, long timeout)
{ 
	long endTime = millis() + timeout;
    while(strlen(getRDSName()) < 8 && millis() < endTime) {
        pollRDS();
    }
    memcpy(buffer, getRDSName(), 9);
    buffer[8] = '\0';
}

void Si4703_Breakout::pollRDS()
{
    //From AN230, using the polling method 40ms should be sufficient amount of time between checks
    if (millis() - _rdsLastPoll < 40)
        return;
    _rdsLastPoll = millis();

    readRegisters();
    const byte blockerrors = (si4703_registers[STATUSRSSI] & 0x0600) >> 9;
    if (blockerrors != 0)
         return;
    if (si4703_registers[STATUSRSSI] & (1 << RDSR)) {

        const uint8_t groupType = (si4703_registers[RDSB] & 0xF000) >> 12;
        const bool BType = (si4703_registers[RDSB] & 0x0800) >> 11;

        switch (groupType) {
        case 0: // Station Name (8 chars)
        {
            const uint8_t CS = constrain(si4703_registers[RDSB] & 0x0003, 0, 7);
            const uint8_t C0 = (si4703_registers[RDSD] & 0xFF00) >> 8;
            const uint8_t C1 = (si4703_registers[RDSD] & 0x00FF);

            // Reset text if some changes are detected.
            if ((_rdsName[2 * CS + 0] != '\0' && _rdsName[2 * CS + 0] != C0) ||
                (_rdsName[2 * CS + 1] != '\0' && _rdsName[2 * CS + 1] != C1)) {
                memset(_rdsName, 0, 9);
            }
            _rdsName[2 * CS + 0] = C0;
            _rdsName[2 * CS + 1] = C1;
        } break;
        case 2: // Radio Text (64 chars)
        {
            const bool clear = (si4703_registers[RDSB] & 0x0010) >> 4;
            const uint8_t CS = constrain(si4703_registers[RDSB] & 0x000F, 0, 63);
            const uint8_t C0 = (si4703_registers[RDSC] & 0xFF00) >> 8;
            const uint8_t C1 = (si4703_registers[RDSC] & 0x00FF);
            const uint8_t C2 = (si4703_registers[RDSD] & 0xFF00) >> 8;
            const uint8_t C3 = (si4703_registers[RDSD] & 0x00FF);

            if (clear) {
                memset(_rdsText, 0, 65);
            }
            if (!BType) {
                _rdsText[4 * CS + 0] = C0;
                _rdsText[4 * CS + 1] = C1;
                _rdsText[4 * CS + 2] = C2;
                _rdsText[4 * CS + 3] = C3;
            } else {
                _rdsText[2 * CS + 0] = C2;
                _rdsText[2 * CS + 1] = C3;
            }
        } break;
        case 4: // Clock Time and Date (CT)
        {
            const uint8_t localTimeOffset = (si4703_registers[RDSD] & 0x001F);
            const int8_t localOffsetSign = (si4703_registers[RDSD] & 0x0020) >> 5 ? -1 : 1;
            const uint8_t UTCMinutes = (si4703_registers[RDSD] & 0x0FC0) >> 6;
            const uint8_t UTCHours = ((si4703_registers[RDSC] & 0x0001) << 4) | ((si4703_registers[RDSD] & 0xF000) >> 12);

            _rdsTimePoll = millis();
            _rdsMinutes = 60 * UTCHours + UTCMinutes + localOffsetSign * 30 * localTimeOffset;
            _rdsModifiedJulianDayCode = ((si4703_registers[RDSB] & 0x0003) << 15) | (si4703_registers[RDSC] >> 1);
        } break;
        default:
            break;
        }
     }
}

const char* Si4703_Breakout::getRDSName() const
{
    return strlen(_rdsName) > 0 ? _rdsName : NULL;
}

const char* Si4703_Breakout::getRDSText() const
{
    return strlen(_rdsText) > 0 ? _rdsText : NULL;
}

bool Si4703_Breakout::getRDSTime(uint8_t& hours, uint8_t& minutes) const
{
    if (_rdsMinutes == 0)
        return false;
    const unsigned long elapsed = (millis() - _rdsTimePoll) / 60000;
    hours = (_rdsMinutes + elapsed) / 60;
    minutes = (_rdsMinutes + elapsed) % 60;

    return true;
}

bool Si4703_Breakout::getRDSDate(uint8_t& day, uint8_t& month, uint16_t& year) const
{
    if (_rdsModifiedJulianDayCode == 0)
        return false;

    uint32_t J = _rdsModifiedJulianDayCode + 2400001 + 68569;
    uint32_t C = 4 * J / 146097;
    J = J - (146097 * C + 3) / 4;
    uint32_t Y = 4000 * (J + 1) / 1461001;
    J = J - 1461 * Y / 4 + 31;
    uint32_t M = 80 * J / 2447;
    day = J - 2447 * M / 80;
    J = M / 11;
    month = M + 2 - (12 * J);
    year = 100 * (C - 49) + Y + J;

    return true;
}
 



//To get the Si4703 inito 2-wire mode, SEN needs to be high and SDIO needs to be low after a reset
//The breakout board has SEN pulled high, but also has SDIO pulled high. Therefore, after a normal power up
//The Si4703 will be in an unknown state. RST must be controlled
void Si4703_Breakout::si4703_init() 
{
  pinMode(_resetPin, OUTPUT);
  pinMode(_sdioPin, OUTPUT); //SDIO is connected to A4 for I2C
  digitalWrite(_sdioPin, LOW); //A low SDIO indicates a 2-wire interface
  digitalWrite(_resetPin, LOW); //Put Si4703 into reset
  delay(1); //Some delays while we allow pins to settle
  digitalWrite(_resetPin, HIGH); //Bring Si4703 out of reset with SDIO set to low and SEN pulled high with on-board resistor
  delay(1); //Allow Si4703 to come out of reset

  Wire.begin(); //Now that the unit is reset and I2C inteface mode, we need to begin I2C

  readRegisters(); //Read the current register set
  //si4703_registers[0x07] = 0xBC04; //Enable the oscillator, from AN230 page 9, rev 0.5 (DOES NOT WORK, wtf Silicon Labs datasheet?)
  si4703_registers[0x07] = 0x8100; //Enable the oscillator, from AN230 page 9, rev 0.61 (works)
  updateRegisters(); //Update

  delay(500); //Wait for clock to settle - from AN230 page 9

  readRegisters(); //Read the current register set
  si4703_registers[POWERCFG] = 0x4001; //Enable the IC
  //  si4703_registers[POWERCFG] |= (1<<SMUTE) | (1<<DMUTE); //Disable Mute, disable softmute
  si4703_registers[SYSCONFIG1] |= (1<<RDS); //Enable RDS

  si4703_registers[SYSCONFIG1] |= (1<<DE); //50kHz Europe setup
  si4703_registers[SYSCONFIG2] |= (1<<SPACE0); //100kHz channel spacing for Europe

  si4703_registers[SYSCONFIG2] &= 0xFFF0; //Clear volume bits
  si4703_registers[SYSCONFIG2] |= 0x0001; //Set volume to lowest
  updateRegisters(); //Update

  delay(110); //Max powerup time, from datasheet page 13
}

//Read the entire register control set from 0x00 to 0x0F
void Si4703_Breakout::readRegisters(){

  //Si4703 begins reading from register upper register of 0x0A and reads to 0x0F, then loops to 0x00.
  Wire.requestFrom(SI4703, 32); //We want to read the entire register set from 0x0A to 0x09 = 32 bytes.

  while(Wire.available() < 32) ; //Wait for 16 words/32 bytes to come back from slave I2C device
  //We may want some time-out error here

  //Remember, register 0x0A comes in first so we have to shuffle the array around a bit
  for(int x = 0x0A ; ; x++) { //Read in these 32 bytes
    if(x == 0x10) x = 0; //Loop back to zero
    si4703_registers[x] = Wire.read() << 8;
    si4703_registers[x] |= Wire.read();
    if(x == 0x09) break; //We're done!
  }
}

//Write the current 9 control registers (0x02 to 0x07) to the Si4703
//It's a little weird, you don't write an I2C addres
//The Si4703 assumes you are writing to 0x02 first, then increments
byte Si4703_Breakout::updateRegisters() {

  Wire.beginTransmission(SI4703);
  //A write command automatically begins with register 0x02 so no need to send a write-to address
  //First we send the 0x02 to 0x07 control registers
  //In general, we should not write to registers 0x08 and 0x09
  for(int regSpot = 0x02 ; regSpot < 0x08 ; regSpot++) {
    byte high_byte = si4703_registers[regSpot] >> 8;
    byte low_byte = si4703_registers[regSpot] & 0x00FF;

    Wire.write(high_byte); //Upper 8 bits
    Wire.write(low_byte); //Lower 8 bits
  }

  //End this transmission
  byte ack = Wire.endTransmission();
  if(ack != 0) { //We have a problem! 
    return(FAIL);
  }

  return(SUCCESS);
}

//Seeks out the next available station
//Returns the freq if it made it
//Returns zero if failed
int Si4703_Breakout::seek(byte seekDirection){
  // Reset cached RDS infos on channel change
  memset(_rdsName, 0, 9);
  memset(_rdsText, 0, 65);

  readRegisters();
  //Set seek mode wrap bit
  si4703_registers[POWERCFG] |= (1<<SKMODE); //Allow wrap
  //si4703_registers[POWERCFG] &= ~(1<<SKMODE); //Disallow wrap - if you disallow wrap, you may want to tune to 87.5 first
  if(seekDirection == SEEK_DOWN) si4703_registers[POWERCFG] &= ~(1<<SEEKUP); //Seek down is the default upon reset
  else si4703_registers[POWERCFG] |= 1<<SEEKUP; //Set the bit to seek up

  si4703_registers[POWERCFG] |= (1<<SEEK); //Start seek
  updateRegisters(); //Seeking will now start

  //Poll to see if STC is set
  while(1) {
    readRegisters();
    if((si4703_registers[STATUSRSSI] & (1<<STC)) != 0) break; //Tuning complete!
  }

  readRegisters();
  int valueSFBL = si4703_registers[STATUSRSSI] & (1<<SFBL); //Store the value of SFBL
  si4703_registers[POWERCFG] &= ~(1<<SEEK); //Clear the seek bit after seek has completed
  updateRegisters();

  //Wait for the si4703 to clear the STC as well
  while(1) {
    readRegisters();
    if( (si4703_registers[STATUSRSSI] & (1<<STC)) == 0) break; //Tuning complete!
  }

  if(valueSFBL) { //The bit was set indicating we hit a band limit or failed to find a station
    return(0);
  }
return getChannel();
}

//Reads the current channel from READCHAN
//Returns a number like 973 for 97.3MHz
int Si4703_Breakout::getChannel() {
  readRegisters();
  int channel = si4703_registers[READCHAN] & 0x03FF; //Mask out everything but the lower 10 bits
  //Freq(MHz) = 0.100(in Europe) * Channel + 87.5MHz
  //X = 0.1 * Chan + 87.5
  channel += 875; //98 + 875 = 973
  return(channel);
}
