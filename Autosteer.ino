/*
   UDP Autosteer code for Teensy 4.1
   For AgOpenGPS
   01 Feb 2022
   Like all Arduino code - copied from somewhere else :)
   So don't claim it as your own
*/

////////////////// User Settings /////////////////////////

//How many degrees before decreasing Max PWM
#define LOW_HIGH_DEGREES 1.5

/*  PWM Frequency ->
	 490hz (default) = 0
	 122hz = 1
	 3921hz = 2
*/
#define PWM_Frequency 1
#include "SteeringSafety.h"
#include "AogPacket.h"
bool wasHealthy = true;
SteeringSafety steeringSafety;
uint8_t stableSteerPin = HIGH, lastRawSteerPin = HIGH;
uint32_t steerPinChangedAt = 0;

//WAS Calabration
float inputWAS[] = { -50.00, -45.0, -40.0, -35.0, -30.0, -25.0, -20.0, -15.0, -10.0, -5.0, 0, 5.0, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0, 50.0 };  //Input WAS do not adjust
float outputWAS[] = { -50.00, -45.0, -40.0, -35.0, -30.0, -25.0, -20.0, -15.0, -10.0, -5.0, 0, 5.0, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0, 50.0 };

/////////////////////////////////////////////

// if not in eeprom, overwrite
#define EEP_Ident 2500 // Keya-specific for twin-track

//--------------------------- Switch Input Pins ------------------------
#define STEERSW_PIN 4
#define WORKSW_PIN 34
#define REMOTE_PIN 37

#define CONST_180_DIVIDED_BY_PI 57.2957795130823

#include <Wire.h>
#include <EEPROM.h>

#include <IPAddress.h>
// ethernet
#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>

//uint8_t Ethernet::buffer[200]; // udp send and receive buffer
uint8_t autoSteerUdpData[serial_buffer_size];  // Buffer For Receiving UDP Data

//loop time variables in microseconds
const uint16_t LOOP_TIME = 25;  //40Hz
uint32_t autsteerLastTime = LOOP_TIME;
uint32_t currentTime = LOOP_TIME;

//Heart beat hello AgIO
uint8_t helloFromIMU[] = { 128, 129, 121, 121, 5, 0, 0, 0, 0, 0, 71 };
uint8_t helloFromAutoSteer[] = { 0x80, 0x81, 126, 126, 5, 0, 0, 0, 0, 0, 71 };
int16_t helloSteerPosition = 0;

//fromAutoSteerData FD 253 - ActualSteerAngle*100 -5,6, SwitchByte-7, pwmDisplay-8
uint8_t PGN_253[] = { 0x80,0x81, 126, 0xFD, 8, 0, 0, 0, 0, 0,0,0,0, 0xCC };
int8_t PGN_253_Size = sizeof(PGN_253) - 1;

//fromAutoSteerData FD 250 - sensor values etc
uint8_t PGN_250[] = { 0x80,0x81, 126, 0xFA, 8, 0, 0, 0, 0, 0,0,0,0, 0xCC };
int8_t PGN_250_Size = sizeof(PGN_250) - 1;
uint8_t aog2Count = 0;
float sensorReading;
float sensorSample;

elapsedMillis gpsSpeedUpdateTimer = 0;

//EEPROM
int16_t EEread = 0;

//Relays
bool isRelayActiveHigh = true;
uint8_t relay = 0, relayHi = 0, uTurn = 0;
uint8_t tram = 0;

//Switches
uint8_t remoteSwitch = 0, workSwitch = 0, steerSwitch = 1, switchByte = 0;

//On Off
uint8_t guidanceStatus = 0;



//speed sent as *10
float gpsSpeed = 0;

//steering variables
float steerAngleActual = 0;
float steerAngleSetPoint = 0; //the desired angle from AgOpen


//Steer switch button  ***********************************************************************************************************
uint8_t currentState = 1, reading, previous = 0;
uint8_t pulseCount = 0; // Steering Wheel Encoder
bool encEnable = false; //debounce flag
uint8_t thisEnc = 0, lastEnc = 0;

//Variables for settings
struct Storage {
	uint8_t Kp = 40;              // proportional gain
	uint8_t lowPWM = 10;          // band of no action
	int16_t wasOffset = 0;
	uint8_t minPWM = 9;
	uint8_t highPWM = 60;         // max PWM value
	float steerSensorCounts = 30;
	float AckermanFix = 1;        // sent as percent
};  Storage steerSettings;      // 11 bytes

//Variables for settings - 0 is false
struct Setup {
	uint8_t InvertWAS = 0;
	uint8_t IsRelayActiveHigh = 0;    // if zero, active low (default)
	uint8_t MotorDriveDirection = 0;
	uint8_t SingleInputWAS = 1;
	uint8_t CytronDriver = 1;
	uint8_t SteerSwitch = 0;          // 1 if switch selected
	uint8_t SteerButton = 0;          // 1 if button selected
	uint8_t ShaftEncoder = 0;
	uint8_t PressureSensor = 0;
	uint8_t CurrentSensor = 0;
	uint8_t PulseCountMax = 5;
	uint8_t IsDanfoss = 0;
	uint8_t IsUseY_Axis = 0;     //Set to 0 to use X Axis, 1 to use Y avis
}; Setup steerConfig;               // 9 bytes


void autosteerSetup()
{


	//keep pulled high and drag low to activate, noise free safe
	pinMode(WORKSW_PIN, INPUT_PULLUP);
	pinMode(STEERSW_PIN, INPUT_PULLUP);
	pinMode(REMOTE_PIN, INPUT_PULLUP);
	stableSteerPin = lastRawSteerPin = digitalRead(STEERSW_PIN);
	steerPinChangedAt = millis();

	//set up communication
	Wire1.end();
	Wire1.begin();

	EEPROM.get(0, EEread);              // read identifier

	if (EEread != EEP_Ident)            // check on first start and write EEPROM
	{
		EEPROM.put(0, EEP_Ident);
		EEPROM.put(10, steerSettings);
		EEPROM.put(40, steerConfig);
		EEPROM.put(60, networkAddress);
	}
	else
	{
		EEPROM.get(10, steerSettings);     // read the Settings
		EEPROM.get(40, steerConfig);
		EEPROM.get(60, networkAddress);
	}

	if (Autosteer_running)
	{
		Serial.println("Autosteer running, waiting for AgOpenGPS");
		// Autosteer Led goes Red if ADS1115 is found
		digitalWrite(AUTOSTEER_ACTIVE_LED, 0);
		digitalWrite(AUTOSTEER_STANDBY_LED, 1);
	}
	else
	{
		Autosteer_running = false;  //Turn off auto steer if no ethernet (Maybe running T4.0)
		//    if(!Ethernet_running)Serial.println("Ethernet not available");
		Serial.println("Autosteer disabled, GPS only mode");
		return;
	}
}// End of Setup

void autosteerLoop()
{

	ReceiveUdp();
	KeyaBus_Receive();
    SteeringSafety::Action action = SteeringSafety::None;

	// Input sampling and steering feedback update at 40 Hz; permission is checked every pass.
	currentTime = systick_millis_count;

	if (currentTime - autsteerLastTime >= LOOP_TIME)
	{
		autsteerLastTime = currentTime;

		//reset debounce
		encEnable = true;

        // Debounce both switch types; a held input at boot is not a new request.
        workSwitch = digitalRead(WORKSW_PIN);
        reading = digitalRead(STEERSW_PIN);
        if (reading != lastRawSteerPin) {
            lastRawSteerPin = reading;
            steerPinChangedAt = millis();
        }
        if (reading != stableSteerPin && uint32_t(millis() - steerPinChangedAt) >= 30) {
            stableSteerPin = reading;
            if (steerConfig.SteerSwitch) {
                action = reading == LOW ? SteeringSafety::Engage : SteeringSafety::Disengage;
            } else if (steerConfig.SteerButton && reading == LOW) {
                action = steeringSafety.requested ? SteeringSafety::Disengage : SteeringSafety::Engage;
            }
        }
        // Maintained switch OFF always overrides tablet engagement.
        if (steerConfig.SteerSwitch && stableSteerPin == HIGH)
            action = SteeringSafety::Disengage;
		// use Keya encoder
		if (keyaDetected && !steerConfig.SingleInputWAS)
		{
			static float keya_GPS_offset = 0;
			float wasDiff;

			if (gpsSpeed > 1.2 && !steerConfig.InvertWAS)
			{
				wasDiff = (keyaCurrentSteerAngleScaled + keya_GPS_offset) - wheelAngleGPS;

				keya_GPS_offset -= (wasDiff * (0.001));
			}

			steerAngleActual = keyaCurrentSteerAngleScaled + keya_GPS_offset;
			/*
			Serial.print(wasDiff);
			Serial.print(",\t");
			Serial.print(wheelAngleGPS);
			Serial.print(",\t");
			Serial.print(keyaSteerAngleActual);
			Serial.print(",\t");
			Serial.print(keya_GPS_offset);
			Serial.print(",\t");
			Serial.print(steerAngleActual);
			Serial.println(",\t");
			*/
		}
		// use ADS1115
		else
		{
			//DETERMINE ACTUAL STEERING POSITION
			helloSteerPosition = keyaCurrentSteerAngleScaled;
		}

        wasHealthy = steerAngleActual >= inputWAS[0] && steerAngleActual <= inputWAS[20];
        float mappedWAS = multiMap<float>(steerAngleActual, inputWAS, outputWAS, 21);
        steerAngleActual = mappedWAS;
	} //end of timed loop

    // Revalidate on EVERY pass before servicing any pending motor enable.
    const bool takeover = (steerConfig.ShaftEncoder && pulseCount >= steerConfig.PulseCountMax) ||
        (steerConfig.CurrentSensor && keyaDetected && sensorReading >= steerConfig.PulseCountMax);
    const bool motorHealthy = keyaDetected && lastKeyaHeatbeat < 1000 && !keyaMotorFault && !keyaTxFault;
    steeringSafety.update(millis(), motorHealthy && wasHealthy && !takeover &&
        steerSettings.steerSensorCounts > 0 && steerSettings.AckermanFix > 0 && !updateRawPositionOffset, action);
    keyaIntendToSteer = steeringSafety.motorAllowed;
    steerSwitch = steeringSafety.requested ? 0 : 1;
    currentState = steerSwitch;
    if (!steeringSafety.requested) pulseCount = 0;
    remoteSwitch = digitalRead(REMOTE_PIN);
    switchByte = (remoteSwitch << 2) | (steerSwitch << 1) | workSwitch;
    digitalWrite(AUTOSTEER_ACTIVE_LED, keyaIntendToSteer ? 1 : 0);
    digitalWrite(AUTOSTEER_STANDBY_LED, keyaIntendToSteer ? 0 : 1);

    // Nonblocking motor sequencer; disable cancels a pending enable.
    SteerKeya(keyaIntendToSteer);

	//This runs continuously, outside of the timed loop, keeps checking for new udpData, turn sense
	//delay(1);

	// Speed pulse
	if (gpsSpeedUpdateTimer < 1000)
	{
		if (speedPulseUpdateTimer > 200) // 100 (10hz) seems to cause tone lock ups occasionally
		{
			speedPulseUpdateTimer = 0;

			//130 pp meter, 3.6 kmh = 1 m/sec = 130hz or gpsSpeed * 130/3.6 or gpsSpeed * 36.1111
			//gpsSpeed = ((float)(autoSteerUdpData[5] | autoSteerUdpData[6] << 8)) * 0.1;
			float speedPulse = gpsSpeed * 36.1111;

			//Serial.print(gpsSpeed); Serial.print(" -> "); Serial.println(speedPulse);

			if (gpsSpeed > 0.11) { // 0.10 wasn't high enough
				tone(velocityPWM_Pin, uint16_t(speedPulse));
			}
			else {
				noTone(velocityPWM_Pin);
			}
		}
	}
	else  // if gpsSpeedUpdateTimer hasn't update for 1000 ms, turn off speed pulse
	{
		noTone(velocityPWM_Pin);
	}

	if (encEnable)
	{
		thisEnc = digitalRead(REMOTE_PIN);
		if (thisEnc != lastEnc)
		{
			lastEnc = thisEnc;
			if (lastEnc) EncoderFunc();
		}
	}

} // end of main loop


// UDP Receive
void ReceiveUdp()
{
	uint16_t len = Eth_udpAutoSteer.parsePacket();

	// if (len > 0)
	// {
	//  Serial.print("ReceiveUdp: ");
	//  Serial.println(len);
	// }

	// Check for len > 4, because we check byte 0, 1, 3 and 3
	if (len > 4)
	{
		const int received = Eth_udpAutoSteer.read(autoSteerUdpData, sizeof(autoSteerUdpData));
        if (received != len || len > sizeof(autoSteerUdpData) || !validAogPacket(autoSteerUdpData, len)) return;

		if (autoSteerUdpData[0] == 0x80 && autoSteerUdpData[1] == 0x81 && autoSteerUdpData[2] == 0x7F) //Data
		{
			if (autoSteerUdpData[3] == 0xFE && Autosteer_running)  //254
			{
				gpsSpeed = ((float)(autoSteerUdpData[5] | autoSteerUdpData[6] << 8)) * 0.1;
				gpsSpeedUpdateTimer = 0;

				guidanceStatus = autoSteerUdpData[7];
				steeringSafety.command(millis(), gpsSpeed, bitRead(guidanceStatus, 0) != 0);

				//Bit 8,9    set point steer angle * 100 is sent
				steerAngleSetPoint = decodeSteerDegrees(autoSteerUdpData[8], autoSteerUdpData[9]); //high low bytes

				XTE = autoSteerUdpData[9];
				//Serial.println(gpsSpeed);

				//Bit 10 Tram
				tram = autoSteerUdpData[10];

				//Bit 11
				relay = autoSteerUdpData[11];

				//Bit 12
				relayHi = autoSteerUdpData[12];

				//----------------------------------------------------------------------------
				//Serial Send to agopenGPS



				// ======================================================================
				int16_t sa = (int16_t)(keyaCurrentSteeringPositionScaled * 100);

				// ======================================================================

				PGN_253[5] = (uint8_t)sa;
				PGN_253[6] = sa >> 8;

				// heading
				PGN_253[7] = (uint8_t)9999;
				PGN_253[8] = 9999 >> 8;

				// roll
				PGN_253[9] = (uint8_t)8888;
				PGN_253[10] = 8888 >> 8;

				PGN_253[11] = switchByte;
				PGN_253[12] = 100;

				//checksum
				int16_t CK_A = 0;
				for (uint8_t i = 2; i < PGN_253_Size; i++)
					CK_A = (CK_A + PGN_253[i]);

				PGN_253[PGN_253_Size] = CK_A;

				//off to AOG
				SendUdp(PGN_253, sizeof(PGN_253), Eth_ipDestination, portDestination);

				//Steer Data 2 -------------------------------------------------
				if (steerConfig.PressureSensor || steerConfig.CurrentSensor)
				{
					if (aog2Count++ > 2)
					{
						//Send fromAutosteer2
						PGN_250[5] = (byte)sensorReading;

						//add the checksum for AOG2
						CK_A = 0;

						for (uint8_t i = 2; i < PGN_250_Size; i++)
						{
							CK_A = (CK_A + PGN_250[i]);
						}

						PGN_250[PGN_250_Size] = CK_A;

						//off to AOG
						SendUdp(PGN_250, sizeof(PGN_250), Eth_ipDestination, portDestination);
						aog2Count = 0;
					}
				}

				//Serial.println(steerAngleActual);
				//--------------------------------------------------------------------------
			}

			//steer settings
			else if (autoSteerUdpData[3] == 0xFC && Autosteer_running)  //252
			{
				//PID values
				steerSettings.Kp = ((float)autoSteerUdpData[5]);   // read Kp from AgOpenGPS

				steerSettings.highPWM = autoSteerUdpData[6]; // read high pwm

				steerSettings.lowPWM = (float)autoSteerUdpData[7];   // read lowPWM from AgOpenGPS

				steerSettings.minPWM = autoSteerUdpData[8]; //read the minimum amount of PWM for instant on

				float temp = (float)steerSettings.minPWM * 1.2;
				steerSettings.lowPWM = (byte)temp;

				steerSettings.steerSensorCounts = autoSteerUdpData[9]; //sent as setting displayed in AOG

				steerSettings.wasOffset = (autoSteerUdpData[10]);  //read was zero offset Lo

				steerSettings.wasOffset |= (autoSteerUdpData[11] << 8);  //read was zero offset Hi
				Serial.println("was offset: " + String(steerSettings.wasOffset) + " last: " + String(lastWASOffset));	
				if (steerSettings.wasOffset != lastWASOffset)
				{
					lastWASOffset = steerSettings.wasOffset;
					updateRawPositionOffset = true;
				}
				steerSettings.AckermanFix = (float)autoSteerUdpData[12];

				//crc
				//autoSteerUdpData[13];

				//store in EEPROM
				EEPROM.put(10, steerSettings);

			}

			else if (autoSteerUdpData[3] == 0xFB)  //251 FB - SteerConfig
			{
				uint8_t sett = autoSteerUdpData[5]; //setting0

				if (bitRead(sett, 0)) steerConfig.InvertWAS = 1; else steerConfig.InvertWAS = 0;
				if (bitRead(sett, 1)) steerConfig.IsRelayActiveHigh = 1; else steerConfig.IsRelayActiveHigh = 0;
				if (bitRead(sett, 2)) steerConfig.MotorDriveDirection = 1; else steerConfig.MotorDriveDirection = 0;
				if (bitRead(sett, 3)) steerConfig.SingleInputWAS = 1; else steerConfig.SingleInputWAS = 0;
				if (bitRead(sett, 4)) steerConfig.CytronDriver = 1; else steerConfig.CytronDriver = 0;
				if (bitRead(sett, 5)) steerConfig.SteerSwitch = 1; else steerConfig.SteerSwitch = 0;
				if (bitRead(sett, 6)) steerConfig.SteerButton = 1; else steerConfig.SteerButton = 0;
				if (bitRead(sett, 7)) steerConfig.ShaftEncoder = 1; else steerConfig.ShaftEncoder = 0;

				steerConfig.PulseCountMax = autoSteerUdpData[6];

				//was speed
				//autoSteerUdpData[7];

				sett = autoSteerUdpData[8]; //setting1 - Danfoss valve etc

				if (bitRead(sett, 0)) steerConfig.IsDanfoss = 1; else steerConfig.IsDanfoss = 0;
				if (bitRead(sett, 1)) steerConfig.PressureSensor = 1; else steerConfig.PressureSensor = 0;
				if (bitRead(sett, 2)) steerConfig.CurrentSensor = 1; else steerConfig.CurrentSensor = 0;
				if (bitRead(sett, 3)) steerConfig.IsUseY_Axis = 1; else steerConfig.IsUseY_Axis = 0;

				//crc
				//autoSteerUdpData[13];

				EEPROM.put(40, steerConfig);

			}//end FB
			else if (autoSteerUdpData[3] == 233) // XTE
			{
				XTE = ((float)(autoSteerUdpData[9] | ((int8_t)autoSteerUdpData[8]) << 8)); //high low bytes
				Serial.println("!");
			}
			else if (autoSteerUdpData[3] == 200) // Hello from AgIO
			{
				if (Autosteer_running)
				{
					int16_t sa = (int16_t)(steerAngleActual * 100);

					helloFromAutoSteer[5] = (uint8_t)sa;
					helloFromAutoSteer[6] = sa >> 8;

					helloFromAutoSteer[7] = (uint8_t)helloSteerPosition;
					helloFromAutoSteer[8] = helloSteerPosition >> 8;
					helloFromAutoSteer[9] = switchByte;

					SendUdp(helloFromAutoSteer, sizeof(helloFromAutoSteer), Eth_ipDestination, portDestination);
				}
				if (useBNO08xI2C)
				{
					SendUdp(helloFromIMU, sizeof(helloFromIMU), Eth_ipDestination, portDestination);
				}
			}

			else if (autoSteerUdpData[3] == 201)
			{
				//make really sure this is the subnet pgn
				if (autoSteerUdpData[4] == 5 && autoSteerUdpData[5] == 201 && autoSteerUdpData[6] == 201)
				{
					networkAddress.ipOne = autoSteerUdpData[7];
					networkAddress.ipTwo = autoSteerUdpData[8];
					networkAddress.ipThree = autoSteerUdpData[9];

					//save in EEPROM and restart
					EEPROM.put(60, networkAddress);
					SCB_AIRCR = 0x05FA0004; //Teensy Reset
				}
			}//end 201

			//whoami
			else if (autoSteerUdpData[3] == 202)
			{
				//make really sure this is the reply pgn
				if (autoSteerUdpData[4] == 3 && autoSteerUdpData[5] == 202 && autoSteerUdpData[6] == 202)
				{
					IPAddress rem_ip = Eth_udpAutoSteer.remoteIP();

					//hello from AgIO
					uint8_t scanReply[] = { 128, 129, Eth_myip[3], 203, 7,
						Eth_myip[0], Eth_myip[1], Eth_myip[2], Eth_myip[3],
						rem_ip[0],rem_ip[1],rem_ip[2], 23 };

					//checksum
					int16_t CK_A = 0;
					for (uint8_t i = 2; i < sizeof(scanReply) - 1; i++)
					{
						CK_A = (CK_A + scanReply[i]);
					}
					scanReply[sizeof(scanReply) - 1] = CK_A;

					static uint8_t ipDest[] = { 255,255,255,255 };
					uint16_t portDest = 9999; //AOG port that listens

					//off to AOG
					SendUdp(scanReply, sizeof(scanReply), ipDest, portDest);
				}
			}
		} //end if 80 81 7F
	}
}

#ifdef ARDUINO_TEENSY41
void SendUdp(uint8_t* data, uint8_t datalen, IPAddress dip, uint16_t dport)
{
	Eth_udpAutoSteer.beginPacket(dip, dport);
	Eth_udpAutoSteer.write(data, datalen);
	Eth_udpAutoSteer.endPacket();
}
#endif

//ISR Steering Wheel Encoder
void EncoderFunc()
{
	if (encEnable)
	{
		pulseCount++;
		encEnable = false;
	}
}

//Rob Tillaart, https://github.com/RobTillaart/MultiMap
template<typename T>
T multiMap(T value, T* _in, T* _out, uint8_t size)
{
	// take care the value is within range
	// value = constrain(value, _in[0], _in[size-1]);
	if (value <= _in[0]) return _out[0];
	if (value >= _in[size - 1]) return _out[size - 1];

	// search right interval
	uint8_t pos = 1;  // _in[0] already tested
	while (value > _in[pos]) pos++;

	// this will handle all exact "points" in the _in array
	if (value == _in[pos]) return _out[pos];

	// interpolate in the right segment for the rest
	return (value - _in[pos - 1]) * (_out[pos] - _out[pos - 1]) / (_in[pos] - _in[pos - 1]) + _out[pos - 1];
}
