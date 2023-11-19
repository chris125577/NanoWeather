/*  This is a challenging hardware project to combine measurements for temp,humidity,pressure,skytemp,ambient(MLX),SQM and rain rate
* It also displays these values on a 7-segment display, so it can be used as a hand held meter
* Version 1.0  Chris Woodhouse FRAS Oct 20 2020
* note: short delay required after FTDi Serial transmission, it seems to interfere with timings for MLX read
* uncalibrated - requires magnitude offset
* cloud cover is not explicitly calculated - left for another app to compare temperatures (choice of ambients)
* V1.1 - displays SQM on power up
* V1.2 - LED readout is continuous flow through on button press
* V1.3 - warm up rain sensor at beginning for 2 minutes - then controlled by parameters
* V1.4 - fixed display bug - quickened light level response by not waiting up to 2x pulse length and pulseInLong
*       interrupt driven flag to show all values on LED display
* V1.5 - change in power supply to heater - now PP3 - more sensitive heating controls updated
* V1.6 - 3.3V ref was O/C - fixed and removed unecessary delays before MLX 
* V1.7 - changed over to serial from serial1 for operation and power (FTDI had partial power state confusing board)
* V1.8 - updates - potentially have to go back to using Serial1 and a 12V power supply
* V1.9  - added relay control on D11 and blinking display for rain
* V2.0 - moved to 12V control and added piezo
* V2.1 - dual Serial port output - more flexible and allows for both ASCOM drivers to work on different physical interfaces
* V2.2 - decided to fall back on adjusted sky temp on LED display using default parameters
* V2.3 - slight tweaks to rain heater
* V2.4 - added standby time parameter
* V2.5 - modified rain sensor heater arrangements
* V2.6 - added external command to turn buzzer on and off
* V2.7 - refined rain heater and error control on sky temp output
* V2.8 - added reset external command to update sensors (like rain)
* V2.9 - trying to make MLX stable - do NOT use V2.x of Softwire, averaged readings
* V3.0 - better stability now - putting in EEPROM controls to store trimvalues
* V3.1 - change to sky temp and cloud determination using ambient and sky difference temperature, also UL and LL thresholds
* V3.2 - fixed eeprom save issue
* V3.3 - cloud cover percent issue
* V3.4 - trying to fix occasional crash with disabled heater
* 
*/

#include <SoftwareSerial.h>
#define version "V3.4"   // change with version number

#include <AsyncDelay.h>  // for mlx communication
#include <SoftWire.h>  // for mlx communication emulating I2C
#include <Arduino.h>
#include <TM1637TinyDisplay.h>  // for 7 seg LED
#include <Wire.h>          // for standard I2C communication
#include <math.h>          // for ln() function
#include <BlueDot_BME280.h>  // for reading Bosch BME sensor and calibration
#include <string.h>
#include <EEPROM.h>

enum MessageIndex { mag, RH, hPa, temp, skygain, skyoffset, skyhighT, cloud100, cloud0, rainlimit, reset, buzzer } ;
const int wordlength = 12;  // perameters in message

// trim values and defaults
double magOffset = 0;  // added to log2.5 light level to create sky magnitude;
double rhtrim = 0;  // to modify output humidity %
double hpatrim = 0; // to modify output pressure
double temptrim = 0;  // to modifiy output ambient temp
double gain = 33;  // sky gain coefficient
double offset = 0;   // sky offset coefficient
double highT = 4;   // sky coefficient (high t )
double SkyUL = 10; // 100% cloud cover (initial value)
double SkyLL = -10; // 0% cloud cover (iniitial value)
const double skyTmax = 25;
const double skyTmin = -25;
double rainalarm = 2.0;  // threshold for rain alarm
String restart, buzzenable;  // strings for commands from ascom
const char delim = ','; // command string delimeter

// Cloud Monitor MLX90614 parameters and objects
const uint8_t IRsdaPin = 4;  // soft SDA line
const uint8_t IRsclPin = 5;  // soft SCL line
SoftWire i2c(IRsdaPin, IRsclPin);  // i2c object is soft version
AsyncDelay samplingInterval; // precision delays for software i2c timing
uint16_t rawamb[5]; // holds 5 readings for median
uint16_t rawsky[5]; // holds 5 readings for median

// weather sensor parameters and objects
BlueDot_BME280 bme1;      //bme1 is sensor object
uint8_t bme1Detected = 0;
const uint8_t cmdAmbient = 6;
const uint8_t cmdObject1 = 7;
const uint8_t cmdFlags = 0xf0;
const uint8_t cmdSleep = 0xff;


// ambient/sky/skydiff come from MLX, tempC/humidity/pressure come from BME, skymag from TSL and rainRatio from cap sensor.
// populated with defaults
double ambient = 10.0; // MLX ambient reading
double skytemp = 10.0;  // MLX sky temp
double tempC = 10.0;   // ambient temp (BME)
double humidity = 50.0;  // humidity (BME)
double pressure = 1000.0; // pressure (BME)
double skymag = 18.0;  // sky quality (TLS)
double rainRatio = 1.0;  // raining ratio 1= dry
double adjtemp = 10; // adjusted sky temp
double cloudcover = 100; // % cloud

// Define display seven seg TM1637 Pins
const uint8_t CLK=  3;
const uint8_t DIO = 2;
TM1637TinyDisplay display(CLK, DIO);  // define LED display object
boolean displaymode = false; // used to change what is displayed
boolean rainblink = false;

//TSL light sensor magnitude conversion global constants
const uint8_t TSLpin = 6; // pin number for light sensor input
constexpr auto logmag = 0.92103;  // for log conversion from base e to base 2.5;
unsigned long lightTimeout = 40000000; // light sensor mode timeout period (35 seconds ~ M22)

// rain detector parameters
const uint8_t rainSensorInPin = 9;  // rain sensor sense input (high Z, no pullup)
const uint8_t rainSensorOutPin = 10; // rain sensor charge/discharge pin (through 2.2M ohm)
const uint8_t NTCpin = 0; // temp sensor on rain sensor
const uint8_t Heatpin = 8; // output for turning on rain sensor heater
const uint8_t Readoutpin = 7; // pullup input for reading momentary action switch
const uint16_t timeOut_us = 10000; // rain sensor timout period  (us) equiv to about 2n2F, 20x dry capacitance
const float RainHeatThresh = 1.05; // rain heater damp threshold

const float HeaterAmbThreshold = 10.0;  // ambient temp heater threshold
const float humiditythreshold = 75.0;  // humidity threshold for heater
const float NTCthreshold = 470;  // threshold for NTC heater
double drytime = 10.0; // initial value for RC reference time when lowest capacitance (dry)
double rainratio = 1.0;  // initial ratio
const uint8_t RelayPin = 11;  // potential for relay output/buzzer output
bool buzz = true;  // flag for buzzer enable
String cmd;  // for received string commands from serial port via ASCOM
const unsigned long baud = 115200; // serial baud rate
String NanoStatus [wordlength];  //  " mag, RH, hPa, temp, skygain, offset, skyhighT, cloud100, cloud0, rainlimit, reset,buzzer#"

// support functions
// Switch from PWM mode (if it was enabled) for MLX90614
void exitPWM(void)
{
    // Make SMBus request to force SMBus output instead of PWM
    SoftWire::setSclLow(&i2c);
    delay(3); // Must be > 1.44ms
    SoftWire::setSclHigh(&i2c);
    delay(2);
}

// routine to read data from MLX90614 IR sensor
uint16_t readIRsensor(uint8_t command, uint8_t& crc)
{
    heaterOff();  // to avoid interference
    uint8_t address = 0x5A;
    uint8_t dataLow = 0;
    uint8_t dataHigh = 0;
    uint8_t pec = 0;
    uint8_t errors = 0;
    //digitalWrite(LED_BUILTIN, HIGH); for diagnostics
    delayMicroseconds(50);
    // Send command=pe
    errors += i2c.startWait(address, SoftWire::writeMode);
    errors += i2c.write(command);

    // Read results
    errors += i2c.repeatedStart(address, SoftWire::readMode);
    errors += i2c.readThenAck(dataLow);  // Read 1 byte and then send ack
    errors += i2c.readThenAck(dataHigh); // Read 1 byte and then send ack
    errors += i2c.readThenNack(pec);
    i2c.stop();
    //digitalWrite(LED_BUILTIN, LOW); for diagnostics

    crc = 0;
    crc = SoftWire::crc8_update(crc, address << 1); // Write address
    crc = SoftWire::crc8_update(crc, command);
    crc = SoftWire::crc8_update(crc, (address << 1) + 1); // Read address
    crc = SoftWire::crc8_update(crc, dataLow);
    crc = SoftWire::crc8_update(crc, dataHigh);
    crc = SoftWire::crc8_update(pec, pec);

    if (errors) {
        crc = 0xFF;
        return 0xFFFF;
    }
    return (uint16_t(dataHigh) << 8) | dataLow;
}

// take median of 5 sky samples, adjust sky temp and calculate cloud cover
void ReadIR(void) // 
{
    uint8_t crcAmbient, crcObject1;
    uint16_t amb_reading, sky_reading;
    double tempamb = 0;
    double tempsky = 0;
    for (int j = 0; j < 5; j++) // make 5 readings
    {
        amb_reading = readIRsensor(cmdAmbient, crcAmbient);
        sky_reading = readIRsensor(cmdObject1, crcObject1);
        if (!(crcObject1 | crcAmbient) && !((amb_reading | sky_reading) & 0x8000)) // only if good reading
        {
            rawamb[j] = amb_reading & (uint16_t)0x7FFF;  // add new samples;
            rawsky[j] = sky_reading & (uint16_t)0x7FFF;  // add new samples;
        }
        else
        {
            j--; // wait for good readings
            display.showString(".");
            delay(100);
            display.clear();
        }
        delay(100);
    }

    for (int i = 0; i < 5; ++i)  // sort array and then find median
    {
        tempamb += rawamb[i];
        tempsky += rawsky[i];
    }

    ambient = tempamb/5;  // average value
    skytemp = tempsky/5;  // average value
    // convert to centigrade
    ambient = (ambient / 50) - 273.15;  // conversion to centigrade
    skytemp = (skytemp / 50) - 273.15;  // conversion to centigrade
    
    // make adjustments to sky temp
    skytemp -= ((ambient-(offset/10)) * gain / 100);  // main offset and gain
    skytemp -= ((ambient * highT / 10.0) * (ambient * highT / 10.0) / 100.0);  //compensate for higher temperatures
    adjtemp = skytemp; // decided that to broadcast skytemp, without clipping
    // automatic trim of limits SkyUL and SkyLL to sky temp extremes - but less than error state limits
    if (adjtemp < skyTmin)
    {
        SkyLL = skyTmin;// very clear, less than limit
        adjtemp = skyTmin;
    }
    if (adjtemp > skyTmax)
    {
        SkyUL = skyTmax; // very cloudy
        adjtemp = skyTmax;
    }
    // otherwise modifiy SkyLL and SkyUL between outer limts
    if (adjtemp < SkyLL && adjtemp > skyTmin)  SkyLL = adjtemp;// very clear, less than limit
    if (adjtemp > SkyUL && adjtemp < skyTmax) SkyUL = adjtemp; // very cloudy
    
   // cloud cover calculation - ratio of sky temp between limits
    cloudcover = 100 * (adjtemp - SkyLL) / (SkyUL - SkyLL);
}

// output data string - floats have one DP - through FTDi cable or USB
// all strings representing doubles, two decimal points
// "$tempC,humidity,pressure,skytemp,ambient(MLX),skymag,rain ratio,#"
void FTDi(void)
{
    Serial1.print('$');  // start character
    Serial1.print(tempC,2); // from BME
    Serial1.print(',');
    Serial1.print(humidity,2); // from BME
    Serial1.print(',');
    Serial1.print(pressure,2); // from BME
    Serial1.print(',');
    Serial1.print(skytemp,2); // from MLX
    Serial1.print(',');
    Serial1.print(ambient, 2); // from MLX
    Serial1.print(',');
    Serial1.print(skymag,2);
    Serial1.print(',');
    Serial1.print(rainRatio,2);  // ratio - not boolean
    Serial.print(',');
    Serial.print(cloudcover, 2);  // percentage
    Serial1.print('#');  // terminal character
}

void USBserial(void)   // same as FTDi using alternative port
{
    Serial.print('$');  // start character
    Serial.print(tempC, 2); // from BME
    Serial.print(',');
    Serial.print(humidity, 2); // from BME
    Serial.print(',');
    Serial.print(pressure, 2); // from BME
    Serial.print(',');
    Serial.print(skytemp, 2); // from MLX
    Serial.print(',');
    Serial.print(ambient, 2); // from MLX
    Serial.print(',');
    Serial.print(skymag, 2);
    Serial.print(',');
    Serial.print(rainRatio, 2);  // ratio - not boolean
    Serial.print(',');
    Serial.print(cloudcover, 2);  // percentage
    Serial.print('#');  // terminal character
}

// diagnostic string only - for debug purposes
void monitor(void)
{
    Serial.print("amb:");
    Serial.print(tempC, 2); // from BME
    Serial.print("  humidity:");
    Serial.print(humidity, 2);
    Serial.print("  pressure:");
    Serial.print(pressure, 2);
    Serial.print("  sky temp:");
    Serial.print(skytemp, 2);
    Serial.print("  object temp:");
    Serial.print(ambient, 2);
    Serial.print("  skymag:");
    Serial.print(skymag, 2);
    Serial.print("  rain ratio:");
    Serial.print(rainRatio, 2);  // ratio - not boolean
    Serial.print("  raintime:");
    Serial.print(raintime(),2);
    Serial.print("  NTC:");
    Serial.println(analogRead(NTCpin));
}

//  routine to measure light level - to quicken response, detect level first and look for first change
// sensor is light to frequency chip - just measure length of pulse (longer with darker conditions)
void SkyQuality(void)
{
    heaterOff();
    unsigned long duration;
    if (digitalRead(TSLpin)) duration = pulseInLong(TSLpin,LOW,lightTimeout);
    else    duration = pulseInLong(TSLpin, HIGH, lightTimeout);
    if (duration < 1) skymag = 99.9;  // error value (very bright)
    else    skymag = (log((double)duration) / logmag) + magOffset;  // ASCOM driver allows for further trimming
}

// try to establish contact with BME sensor - sets OK flag
void detectBME(void)
{
    if (bme1.init() != 0x60)
    {
        //Serial.println(F("Ops! First BME280 Sensor not found!"));
        bme1Detected = 0;
    }
    else
    {
        //Serial.println(F("First BME280 Sensor detected!"));
        bme1Detected = 1;
    }
}

// read environmental values from BME sensor and apply trim values, if set
void ReadBME(void)
{
    if (bme1Detected)
    {
    tempC = bme1.readTempC() + temptrim;
    humidity = bme1.readHumidity() + rhtrim;
    pressure = bme1.readPressure() + hpatrim;
    }
}

// rain RC measurement (averaged over 20 samples)
double raintime(void)
{
    double q =0;
    for (int i = 0; i < 20; i++)
        q += getRCtime();
    q /= 20.0;  // average 20 samples
    return(q);
}

// rain detection calculation - sets capacitive ratio - self adjusting limits
void raining(void)
{
    rainRatio = (raintime() / drytime);
    if (rainRatio < 0.98) calibrateRain();  // update calibration if it gets dryer slightly less than 1 to accommodate noise)
}

// rain sensor is a capacitor, this measures charge/discharge time into schmidt trigger to determine value
// measures rise/fall time of capacitive rains sensor in microsecs
unsigned long getRCtime(void)
{
    heaterOff();
    digitalWrite(rainSensorOutPin, 0);
    delay(10); // 6ms discharge
    unsigned long t1 = micros();
    digitalWrite(rainSensorOutPin, 1); // charge
    while (!digitalRead(rainSensorInPin) && (micros() - t1) < timeOut_us); // while < 2.5V
    unsigned long us = micros() - t1;
    delay(10); // 6ms charge to 5V
    unsigned long t0 = micros();
    digitalWrite(rainSensorOutPin, 0); // discharge
    while (digitalRead(rainSensorInPin) && (micros() - t0) < timeOut_us); // while > 2.5V 
    us += micros() - t0;
    return us;
}

// takes a rain sample and uses it as a baseline for ratio
void calibrateRain(void)
{
    drytime = raintime();
}

// heaterOn checks for valid reasons to put heater on (wet, cold or high humidity) board LED used to indicate state
// NTCpin is analog input representing rain sensor temperature
void heaterOn(void)
{
    if ((rainRatio > RainHeatThresh) || (tempC < HeaterAmbThreshold) || (humidity > humiditythreshold)) // conditional heating (raining, cold and humid)   
    {
        if(analogRead(NTCpin) > NTCthreshold) digitalWrite(Heatpin, HIGH);  // prevents over heating (about 20C)
        digitalWrite(LED_BUILTIN, HIGH);  // for diagnostics
    }   
    else // turn heater off
    {
        digitalWrite(Heatpin, LOW);
        digitalWrite(LED_BUILTIN, LOW); // for diagnostics
    }    
   // periodic and initial heating of rain sensor, to clear condensation build-up
    if (( millis() < 60000) || ((millis() % 300000) < 60000))  // 60 seconds of heating at outset or every 5th minute for a minute
    {
        if (analogRead(NTCpin) > NTCthreshold)  // prevents over heating (about 28C)
        {
            digitalWrite(Heatpin, HIGH); // heat for the first minute after turn on (override)
            digitalWrite(LED_BUILTIN, HIGH);
        }
    }
}

// simple routine to turn heater element off
void heaterOff(void)  
{
    digitalWrite(Heatpin, LOW); 
    digitalWrite(LED_BUILTIN, LOW); // for diagnostics
}

// interrupt routine to set LEDreadout enable flag
void displayLED(void)
{
    displaymode = true;
    display.showString("...");
}

// displaymode checks switch status and briefly turns on display, cycling around different output values
void LEDreadout(void)
{
    if (displaymode)  // if switch pressed
        {
            display.showString("mag");
            delay(500);
            display.clear();
            delay(250);
            display.showNumber(skymag);
            delay(1000);
            display.clear();
            delay(250);
            display.showString("hPA");
            delay(500);
            display.clear();
            delay(250);
            display.showNumber(pressure);
            delay(1000);
            display.clear();
            delay(250);            
            display.showString("ir C");
            delay(500);
            display.clear();
            delay(250);
            display.showNumber(skytemp);
            delay(1000);
            display.clear();
            delay(250);
            display.showString("ClCo");
            delay(500);
            display.clear();
            delay(250);
            display.showNumber(cloudcover);
            delay(1000);
            display.clear();     
            delay(250);
            display.showString("t C");
            delay(500);
            display.clear();
            delay(250);
            display.showNumber(tempC);
            delay(1000);
            display.clear();
            delay(250);
            display.showString("rh");
            delay(500);
            display.clear();
            delay(250);
            display.showNumber(humidity);
            delay(1000);
            display.clear();
            delay(250);
            display.showString("SAFE");
            delay(500);
            display.clear();
            delay(250);
            if (rainRatio > 3.0) display.showString("wet");
            else if (rainRatio > 1.5) display.showString("dAmP");  // drytime is more useful than ratio 
            else display.showString("dry");
            delay(1000);
            display.clear();
            displaymode = false;  // clear display flag
    }    
}

void populate()  // update more convenient global variables from incoming stream
{
    magOffset = NanoStatus[mag].toDouble();
    rhtrim = NanoStatus[RH].toDouble();
    hpatrim = NanoStatus[hPa].toDouble();
    temptrim = NanoStatus[temp].toDouble();
    gain = NanoStatus[skygain].toDouble();
    offset = NanoStatus[skyoffset].toDouble();
    highT = NanoStatus[skyhighT].toDouble();
    SkyUL = NanoStatus[cloud100].toDouble();
    SkyLL = NanoStatus[cloud0].toDouble();
    rainalarm = NanoStatus[rainlimit].toDouble();
    restart = NanoStatus[reset];
    buzzenable = NanoStatus[buzzer];
    if (SkyUL == SkyLL) // trap error condition if EEPROM is messed up
    {
        SkyUL = 10;
        SkyLL = -10;
    }
}

void splitStrings(String str, char dl)
{
    String word = "";
    str = str + dl + '#'; // delimiters at end
    int i = 0; // clear character counter
    int param = 0; // clear word counter
    word = "";
    while (str[i] != '#' && i < str.length() && param < (wordlength+1)) 
    {
        if (str[i] != dl) word += str[i];  // build up word
        else // conclude word when you get delimiter
        {
            NanoStatus[param] = word;
            param++;
            word = "";
        }
        i++;
    }
}

// update EEPROM with calibration string cmd
void updateEEPROM()
{
    int i;
    EEPROM.write(0, '$');
    for (i = 0; (i < cmd.length() && cmd.length()<(6*wordlength)); i++)
        EEPROM.write(i+1, cmd[i]);
    EEPROM.write(cmd.length()+1, '#'); // write framing terminator
    //display.showNumber((int)cmd.length()); // for diagnostics
}

// read string up to '#' and put in cmd string, if nothing in eeprom, return false
bool readfromEEPROM()
{
    EEPROM.read(0);
    if (EEPROM.read(0) != '$')
    {
        display.showString("No Coefficients");
        return false;
    }
    char c;
    cmd = ""; // initialize command string
    int index = 1;
    c = EEPROM.read(index);
    while (c != '#' && index <(6*wordlength)) // until terminator character or max chars reached
    {
        cmd = cmd + c;  // build command word
        index++;
        c = EEPROM.read(index); // read next character      
    }
    //display.showNumber(index);  // diagnostics
    return true;
}

void setup(void)
{
    
    // pin I/O setups
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(TSLpin, INPUT); // light sensor input
    pinMode(rainSensorOutPin, OUTPUT); // used for sending pulses
    pinMode(rainSensorInPin, INPUT);  // used for reading pulse decay
    pinMode(Heatpin, OUTPUT);  // to transistor to turn on heater on rain sensor
    pinMode(Readoutpin, INPUT_PULLUP);  //input pin from momentary action switch
    attachInterrupt(digitalPinToInterrupt(Readoutpin), displayLED, LOW);
    pinMode(RelayPin, OUTPUT);  // buzzer output
    analogReference(EXTERNAL);  // uses AREF pin
    // setups
    digitalWrite(RelayPin, LOW); // turn buzzer off
    heaterOff(); // turn off heater at beginning
    display.clear();
    display.setBrightness(BRIGHT_LOW);  // LED brightness
    display.showString(version);
    delay(1000);
    display.clear();

    // diagnostic only for e2 - initialize
    //for (int i = 0; i < 100; i++) EEPROM.write(i, 0xff);
    
    Serial.begin(baud);  // used for development and duplicate output channel
    //Serial1.begin(baud);  // used for FTDi serial option
    i2c.setDelay_us(5); // mlx sensor
    i2c.begin();    // mlx sensor
    delay(300); // Data is available 0.25s after wakeup
    exitPWM(); // ensure MLX is not in PWM mode

    // environment sensor setup
    bme1.parameter.I2CAddress = 0x76;                    //I2C Address for Sensor 1 (bme1)
    bme1.parameter.sensorMode = 0b11;     // normal mode 
    bme1.parameter.IIRfilter = 0b100;      // filter actor 16             //IIR Filter for Sensor 1
    bme1.parameter.humidOversampling = 0b101;   // filter factor 16 (default value)
    bme1.parameter.tempOversampling = 0b101;  // filter factor 16 (default value)
    bme1.parameter.pressOversampling = 0b101;  // filter factor 16 (default value)
    bme1.parameter.standby = 0b011;    
    detectBME();

    // read calibration coefficients
    if (readfromEEPROM())
    {
        splitStrings(cmd, delim);
        populate(); // read values from eeprom and put into cmd string (if valid)
    }
    // initial sensor sampling
    calibrateRain();
    ReadIR();
    ReadBME();
    SkyQuality(); // initial measurement
}

void loop(void)
{   
    SkyQuality(); // read sky magnitude every 2 minutes
    for (int pace=1; pace < 31; pace++)
    {
        heaterOff();
        if (Serial.available() > 0)  // check to see if ASCOM driver is disabling buzzer
        {
            cmd = Serial.readStringUntil('#');
            splitStrings(cmd, delim); // parameterize the string
            populate(); // store parameters into variables
            if (buzzenable == "buzz off") 
            {
                buzz = false;
                digitalWrite(RelayPin, LOW);   // buzzer or relay off
            }
            if (buzzenable == "buzz on")
            {
                buzz = true;
                digitalWrite(RelayPin, HIGH);   // buzzer or relay on
                delay(1000);
                digitalWrite(RelayPin, LOW);   // buzzer or relay off
            }

            if (restart == "reset")
            {
                detectBME();
                calibrateRain();
                ReadIR();
                ReadBME();
                displayLED();
                LEDreadout();
            }
            updateEEPROM();
        }
        raining(); // measure every 4 seconds
        if (rainRatio > rainalarm)
        {
            rainblink = !rainblink; // toggle display
            if (rainblink) display.showString("rAIn");
            else display.clear();
            if (buzz) digitalWrite(RelayPin, HIGH);   // buzzer or relay on
            else digitalWrite(RelayPin, LOW); //buzzer off
            heaterOn();
        }
        else
        {
            digitalWrite(RelayPin, LOW); // buzzer or relay off
            display.clear();
        }
        
        heaterOn();
        if ((pace % 10) == 0)  // every 40 seconds
        {
            heaterOff();
            delay(100);
            ReadIR();
            ReadBME(); // read tempC, humidity and pressure every 15 seconds
            heaterOn();    
            //Serial.println(analogRead(NTCpin)); // diagnostic to report rain sensor temp
        }
        // diagnostics
        //monitor();  // diagnostics on Serial
        //FTDi();    // broadcast on Serial1 (optional)
         
        USBserial(); // same broadcast, but on Arduino boot cable
        delay(4000); // to let serial broadcast and avoid interference
        LEDreadout(); // check if display is required
    }
}
