#pragma once
#include <stdint.h>

// No Arduino dependencies: the exact firmware state machine is host-tested.
class SteeringSafety {
public:
    static constexpr float StopKph = 1.0f;
    static constexpr float EngageKph = 1.5f;
    static constexpr float MaxKph = 25.0f;
    static constexpr uint32_t CommandTimeoutMs = 500;
    enum Action { None, Engage, Disengage };

    void command(uint32_t now, float speed, bool engaged) {
        haveCommand = true;
        lastCommand = now;
        speedKph = speed;
        aogEngaged = engaged;
        if (!engaged) {
            offSeen = true;
            pendingOff = true;
        } else {
            if (offSeen) pendingOn = true;
            offSeen = false;
        }
    }

    void update(uint32_t now, bool hardwareHealthy, Action action = None) {
        const bool fresh = haveCommand && uint32_t(now - lastCommand) < CommandTimeoutMs;
        // Comparisons also reject NaN, negative speeds and infinity.
        const bool safe = fresh && hardwareHealthy && speedKph >= StopKph && speedKph <= MaxKph;
        if (!safe || action == Disengage) {
            trip();
        } else {
            // AOG OFF must immediately remove torque after engagement. A local
            // engage may first need AOG to see the switch reply and acknowledge ON.
            if (requested && seenEngaged && pendingOff) trip();
            if ((pendingOn || action == Engage) && speedKph >= EngageKph) {
                requested = true;
                requestedAt = now;
                seenEngaged = aogEngaged;
            }
            if (requested && aogEngaged) seenEngaged = true;
            if (requested && !aogEngaged &&
                (seenEngaged || uint32_t(now - requestedAt) >= CommandTimeoutMs)) trip();
        }
        pendingOn = pendingOff = false; // Events are consumed once, including rejected requests.
        motorAllowed = requested && safe && aogEngaged;
    }

    bool requested = false; // Reported to AOG as the local steering switch.
    bool motorAllowed = false;
    float speedKph = 0;
private:
    bool haveCommand = false, aogEngaged = false, offSeen = false;
    bool pendingOn = false, pendingOff = false, seenEngaged = false;
    uint32_t lastCommand = 0, requestedAt = 0;
    void trip() {
        requested = motorAllowed = seenEngaged = false;
        // Do not manufacture an OFF->ON edge when AOG stays ON after a trip.
    }
};
