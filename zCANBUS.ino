#ifndef zCANBUS_H
#define zCANBUS_H
uint8_t keyaAbsolutePositionMode[] = { 0x03, 0x0D, 0x20, 0x31, 0x00, 0x00, 0x00, 0x00 };
uint8_t keyaResetPosition[] = { 0x23, 0x0c, 0x20, 0x09, 0x00, 0x00, 0x00, 0x00 };

uint8_t keyaDisableCommand[] = { 0x23, 0x0C, 0x20, 0x01, 0x00, 0x00, 0x00, 0x00 };
uint8_t keyaDisableResponse[] = { 0x60, 0x0C, 0x20, 0x00 };

//uint8_t keyaEnableCommand[] = { 0x23, 0x0D, 0x20, 0x01, 0x01, 0x90, 0x00, 0x00 };
uint8_t keyaEnableCommand[] = { 0x23, 0x0D, 0x20, 0x01, 0x00, 0x00, 0x00, 0x00 };
uint8_t keyaEnableResponse[] = { 0x60, 0x0D, 0x20, 0x00 };

uint8_t keyaSpeedCommand[] = { 0x23, 0x00, 0x20, 0x01 };
uint8_t keyaSpeedResponse[] = { 0x60, 0x00, 0x20, 0x00 };

uint8_t keyaPositionCommand[] = { 0x23, 0x02, 0x20, 0x01 };
uint8_t keyaPositionResponse[] = { 0x60, 0x00, 0x20, 0x00 };

uint8_t keyaCurrentQuery[] = { 0x40, 0x00, 0x21, 0x01 };
uint8_t keyaCurrentResponse[] = { 0x60, 0x00, 0x21, 0x01 };

uint8_t keyaFaultQuery[] = { 0x40, 0x12, 0x21, 0x01 };
uint8_t keyaFaultResponse[] = { 0x60, 0x12, 0x21, 0x01 };

uint8_t keyaVoltageQuery[] = { 0x40, 0x0D, 0x21, 0x02 };
uint8_t keyaVoltageResponse[] = { 0x60, 0x0D, 0x21, 0x02 };

uint8_t keyaTemperatureQuery[] = { 0x40, 0x0F, 0x21, 0x01 };
uint8_t keyaTemperatureResponse[] = { 0x60, 0x0F, 0x21, 0x01 };

uint8_t keyaVersionQuery[] = { 0x40, 0x01, 0x11, 0x11 };
uint8_t keyaVersionResponse[] = { 0x60, 0x01, 0x11, 0x11 };

uint8_t keyaEncoderResponse[] = { 0x60, 0x04, 0x21, 0x01 };

uint8_t keyaEncoderSpeedResponse[] = { 0x60, 0x03, 0x21, 0x01 };

uint64_t KeyaID = 0x06000001; // 0x01 is default ID
CAN_message_t KeyaBusSendData;
int16_t intendedAngleConverted;

bool debugKeya = true;

float majorScale = 10.0f;

int16_t degreesToPosition(float degrees) {
	const float UNITS_PER_REV = 10000.0f;   // 0x2710
	const float DEGREES_PER_REV = 360.0f;

	// steerSensorCounts acts as a scale factor, default 100 → 1:1 mapping for 360° = 10000 units
	float units = (degrees / DEGREES_PER_REV) * (UNITS_PER_REV * (steerSettings.steerSensorCounts / majorScale));
	// Round to nearest int16_t
	if (units >= 0)
		return (int16_t)(units + 0.5f);
	else
		return (int16_t)(units - 0.5f);
}

float positionToDegrees(float position) {
	const float UNITS_PER_REV = 10000.0f;
	const float DEGREES_PER_REV = 360.0f;

	float degrees = (position * DEGREES_PER_REV) / (UNITS_PER_REV * ( steerSettings.steerSensorCounts / majorScale));

	return degrees;
}


void CAN_Setup()
{
	Serial.println("Setting up CANBUS");
	Keya_Bus.begin();
	Keya_Bus.setBaudRate(250000);
	KeyaBusSendData.id = KeyaID;
	KeyaBusSendData.flags.extended = true;
	KeyaBusSendData.len = 8;
    KeyaBusSendData.seq = true; // Preserve speed/target/enable order.
	memcpy(KeyaBusSendData.buf, keyaAbsolutePositionMode, 8);
	Keya_Bus.write(KeyaBusSendData);
	Serial.println("Setting position mode (RAM)");
	Serial.println("Keya CANBUS setup complete");
}

static inline void packKeya32(int32_t value, uint8_t* buf)
{
	buf[4] = (uint8_t)((value >> 8) & 0xFF);   // DATA_L(H)
	buf[5] = (uint8_t)(value & 0xFF);          // DATA_L(L)
	buf[6] = (uint8_t)((value >> 24) & 0xFF);  // DATA_H(H)
	buf[7] = (uint8_t)((value >> 16) & 0xFF);  // DATA_H(L)
}

void KeyaBus_Receive()
{
	CAN_message_t KeyaBusReceiveData;
	if (Keya_Bus.read(KeyaBusReceiveData))
	{
		// Heartbeat

		if (KeyaBusReceiveData.id == 0x07000001 && KeyaBusReceiveData.flags.extended && KeyaBusReceiveData.len == 8)
		{
			lastKeyaHeatbeat = 0;

			if (!keyaDetected)
			{
				Serial.println("Keya heartbeat detected! Enabling Keya CANBUS");
				keyaDetected = true;
			}
			keyaCurrentSteeringPositionScaled = ((int16_t)((int16_t)KeyaBusReceiveData.buf[0] << 8 | (int16_t)KeyaBusReceiveData.buf[1]) * -1 * (majorScale / steerSettings.steerSensorCounts));
			keyaCurrentSteerAngleScaled = positionToDegrees(keyaCurrentSteeringPositionScaled);
			if (debugKeya) {
				//Serial.println();
				//Serial.print(" keyaCurrentSteeringPositionUnScaled: ");
				//Serial.print(keyaCurrentSteeringPositionUnScaled);
				//Serial.print(" keyaCurrentSteerAngleScaled: ");
				//Serial.print(keyaCurrentSteerAngleScaled);
				//Serial.print(" ");
			}

			keyaCurrentActualSpeed = (int16_t)((int16_t)KeyaBusReceiveData.buf[2] << 8 | (int16_t)KeyaBusReceiveData.buf[3]);
			// Current bytes are not used by the upstream speed-mismatch takeover heuristic.

			// do this with an "error margin" or checking the error flag instead?

			int16_t error = abs(keyaCurrentActualSpeed - keyaCurrentSetSpeed);
			static int16_t counter = 0;

			if (error > abs(keyaCurrentSetSpeed) + 10)
			{
				if (counter++ < 8)
				{
					//Serial.print("Counter\t");
				}
				else
				{
					//Serial.print("Stop\t");
					sensorReading = abs(abs(keyaCurrentSetSpeed) - error);
				}
			}
			else
			{
				//Serial.print("Run\t");
				sensorReading = 0;
				counter = 0;
			}

			//Serial.print("\tCurrentActualSpeed: ");
			//Serial.print(keyaCurrentActualSpeed);
			//Serial.print("\tCurrent: ");
			//Serial.print(current);
			//Serial.print("\tCurrentSetSpeed: ");
			//
			//
			//Serial.print(keyaCurrentSetSpeed);
			//Serial.print("\tCurrentActualSpeed: ");
			//Serial.print(keyaCurrentActualSpeed);

			//            Serial.print(" Error:");
						// Serial.print(error);
						// Serial.print("\t");

						// if (bitRead(KeyaBusReceiveData.buf[7], 0)) Serial.println("Disabled\t");
						// else Serial.println("Enabled \t");

						// check if there's any motor diag/error data and parse it
            keyaMotorFault = (KeyaBusReceiveData.buf[7] & 0xFE) != 0 || KeyaBusReceiveData.buf[6] != 0;
			if (KeyaBusReceiveData.buf[7] > 1 || KeyaBusReceiveData.buf[6] > 0)
			{
				if (bitRead(KeyaBusReceiveData.buf[7], 1)) Serial.print("Over voltage\t");
				if (bitRead(KeyaBusReceiveData.buf[7], 2)) Serial.print("Hardware protection\t");
				if (bitRead(KeyaBusReceiveData.buf[7], 3)) Serial.print("E2PROM\t");
				if (bitRead(KeyaBusReceiveData.buf[7], 4)) Serial.print("Under voltage\t");
				if (bitRead(KeyaBusReceiveData.buf[7], 5)) Serial.print("N/A\t");
				if (bitRead(KeyaBusReceiveData.buf[7], 6)) Serial.print("Over current\t");
				if (bitRead(KeyaBusReceiveData.buf[7], 7)) Serial.print("Mode failure\t");

				if (bitRead(KeyaBusReceiveData.buf[6], 0)) Serial.print("Less phase\t");
				if (bitRead(KeyaBusReceiveData.buf[6], 1)) Serial.print("Motor stall\t");
				if (bitRead(KeyaBusReceiveData.buf[6], 2)) Serial.print("Reserved\t");
				if (bitRead(KeyaBusReceiveData.buf[6], 3)) Serial.print("Hall failure\t");
				if (bitRead(KeyaBusReceiveData.buf[6], 4)) Serial.print("Current sensing\t");
				if (bitRead(KeyaBusReceiveData.buf[6], 5)) Serial.print("No RS232 Steer Command\t");
				if (bitRead(KeyaBusReceiveData.buf[6], 6)) Serial.print("No CAN Steer Command\t");
				if (bitRead(KeyaBusReceiveData.buf[6], 7)) Serial.print("Motor stalled\t");

				Serial.println("Kill Autosteer");
				steerSwitch = 1;
				currentState = 1;
				previous = 0;
			}

			//Serial.println();
		}
	}
}

// Keep the upstream 100 ms speed-to-position settling interval, but never
// block processing of stop, OFF, timeout or takeover events during that interval.
void SteerKeya(bool intendToSteer)
{
    static bool enabled = false, waiting = false, disabledSent = false;
    static uint32_t speedSentAt = 0, lastTargetAt = 0, lastDisableAt = 0;
    static float lastTarget = 0, lastScale = 0;
    static uint16_t lastSpeed = 0;
    const uint32_t now = millis();

    // Disable is unconditional: heartbeat loss must not suppress a release
    // attempt. Repeat at 10 Hz, including after a motor reconnects.
    if (!intendToSteer) {
        waiting = false;
        if (!disabledSent || enabled || uint32_t(now - lastDisableAt) >= 100) {
            memcpy(KeyaBusSendData.buf, keyaDisableCommand, 8);
            if (!Keya_Bus.write(KeyaBusSendData)) return; // Retry next loop.
            lastDisableAt = now;
            disabledSent = true;
            enabled = false;
            keyaTxFault = false; // Gate has already latched off before this release attempt.
        }
        // A reset must never replace the disable frame. Only service a
        // calibration request while disabled; reset itself does not enable.
        if (updateRawPositionOffset && keyaDetected && !keyaMotorFault) {
            memcpy(KeyaBusSendData.buf, keyaResetPosition, 8);
            if (Keya_Bus.write(KeyaBusSendData)) {
                updateRawPositionOffset = false;
                sendHardwareMessage("Keya zero reset; re-engage above 1.5 km/h", 3);
            }
        }
        return;
    }
    disabledSent = false;
    if (!keyaDetected || keyaMotorFault || updateRawPositionOffset ||
        steerSettings.steerSensorCounts <= 0 || steerSettings.AckermanFix <= 0) return;

    const uint16_t speedToSet = (uint16_t)(((uint32_t)steerSettings.AckermanFix * 10000u + 127u) / 200u);
    if ((!enabled && !waiting) || speedToSet != lastSpeed) {
        memcpy(KeyaBusSendData.buf, keyaSpeedCommand, 4);
        packKeya32((int32_t)speedToSet, KeyaBusSendData.buf);
        if (!Keya_Bus.write(KeyaBusSendData)) { keyaTxFault = true; return; }
        lastSpeed = speedToSet;
        speedSentAt = now;
        waiting = true;
    }
    if (waiting && uint32_t(now - speedSentAt) < 100) return;
    if (!enabled || waiting || steerAngleSetPoint != lastTarget ||
        steerSettings.steerSensorCounts != lastScale || uint32_t(now - lastTargetAt) >= 100) {
        // Preserve the upstream steering mapping; this change addresses motor
        // permission and sequencing, not the experimental steering calibration.
        int16_t slack = ((2 / steerSettings.AckermanFix) * 100);
        if (intendedSteerAngle < 0) slack *= -1;
        intendedSteerAngle = (steerAngleSetPoint * -1) + slack;
        intendedAngleConverted = degreesToPosition(intendedSteerAngle + KeyaCenterOffset);
        memcpy(KeyaBusSendData.buf, keyaPositionCommand, 4);
        packKeya32((int32_t)intendedAngleConverted, KeyaBusSendData.buf);
        if (!Keya_Bus.write(KeyaBusSendData)) { keyaTxFault = true; return; }
        memcpy(KeyaBusSendData.buf, keyaEnableCommand, 8);
        if (!Keya_Bus.write(KeyaBusSendData)) { keyaTxFault = true; return; }
        waiting = false;
        enabled = true;
        lastTarget = steerAngleSetPoint;
        lastScale = steerSettings.steerSensorCounts;
        lastTargetAt = now;
    }
}

void sendKeyaCommand() {
    Keya_Bus.write(KeyaBusSendData);
}

#endif
