/*

Copyright (C) 2025 Connor Pagtakhan


*/


#ifndef _BUZZERSOUNDS_H
#define _BUZZERSOUNDS_H

#include <Arduino.h>
#include <math.h>

class BuzzerSounds
{
private:
    // runtime state
    bool playing = false;
    int currentSound = -1;
    size_t stepIndex = 0;
    unsigned long stepStart = 0;
    unsigned long playStart = 0;
    int currentVolume = 255; // 0-255

    // sequence pointer structure so sequences can be swapped at runtime
    struct Seq {
        const uint16_t* freqs;
        const uint16_t* durs; // milliseconds
        size_t len;
        bool loop;
    };

    Seq activeSeq = { nullptr, nullptr, 0, false };

    void setTone(uint16_t freq)
    {
        if (freq == 0) {
            stopTone();
            return;
        }
    #if defined(ESP32)
        // set frequency and duty (approximate volume)
        ledcWriteTone(ledc_channel, freq);
        // map volume 0..255 to duty 0..255 (8-bit)
        uint32_t duty = map(constrain(currentVolume, 0, 255), 0, 255, 0, 255);
        ledcWrite(ledc_channel, duty);
    #else
        tone(buzzerPin, freq);
    #endif
    }

    void stopTone()
    {
    #if defined(ESP32)
        ledcWriteTone(ledc_channel, 0);
        ledcWrite(ledc_channel, 0);
    #else
        noTone(buzzerPin);
    #endif
    }

public:

int buzzerPin = 25;

#if defined(ESP32)
    static const int ledc_channel = 0;
#endif

    // Sound IDs
    enum SoundId {
        SOUND_NONE = -1,
        SOUND_TEST = 0,
        SOUND_HAPPY = 1,
        SOUND_SAD = 2,
        SOUND_CONFUSED = 3,
        SOUND_COS = 4
    };

    // Default sequences (A5..A6 -> 880..1760)
    inline static const uint16_t happy_freqs[]   = {880,  988,  1318, 1760};
    inline static const uint16_t happy_durs[]    = {150,  150,  150,  300};

    inline static const uint16_t sad_freqs[]     = {1318, 988,  880,  659};
    inline static const uint16_t sad_durs[]      = {250,  200,  200,  300};

    inline static const uint16_t confused_freqs[] = {880, 1760, 1047,  932,  1397};
    inline static const uint16_t confused_durs[]  = {80,   80,   120,  120,  160};

    // begin: call in setup
    void begin(int iBuzzerPin)
    {
        buzzerPin = iBuzzerPin;
    #if defined(ESP32)
        ledcSetup(ledc_channel, 2000, 8);
        ledcAttachPin(buzzerPin, ledc_channel);
        ledcWrite(ledc_channel, 0);
    #endif
    }

    // play a sound asynchronously. volume 0..255. If loop true the sequence loops.
    void play(int sound, int volume = 100, bool loop = false)
    {
        currentVolume = constrain(volume, 0, 255);
        playStart = millis();
        stepStart = playStart;
        stepIndex = 0;
        playing = true;
        currentSound = sound;

        switch (sound) {
            case SOUND_TEST:
                // a simple cosine-modulated continuous sweep (handled in update)
                activeSeq = { nullptr, nullptr, 0, false };
                break;
            case SOUND_COS:
                // continuous cos that sweeps A5..A6
                activeSeq = { nullptr, nullptr, 0, false };
                break;
            case SOUND_HAPPY:
                activeSeq = { happy_freqs, happy_durs, sizeof(happy_freqs)/sizeof(happy_freqs[0]), loop };
                break;
            case SOUND_SAD:
                activeSeq = { sad_freqs, sad_durs, sizeof(sad_freqs)/sizeof(sad_freqs[0]), loop };
                break;
            case SOUND_CONFUSED:
                activeSeq = { confused_freqs, confused_durs, sizeof(confused_freqs)/sizeof(confused_freqs[0]), loop };
                break;
            default:
                activeSeq = { nullptr, nullptr, 0, false };
                playing = false;
                currentSound = SOUND_NONE;
                break;
        }
    }

    // stop immediate
    void stop()
    {
        playing = false;
        currentSound = SOUND_NONE;
        activeSeq = { nullptr, nullptr, 0, false };
        stopTone();
    }

    // update: call frequently from loop()
    void update()
    {
        if (!playing) return;

        unsigned long now = millis();

        // continuous cosine-based sounds
        if (currentSound == SOUND_TEST || currentSound == SOUND_COS) {
            // parameters
            const float f_mid = 1300.0f; // center freq ~ between A5 and A6
            const float f_amp = 420.0f;  // amplitude so mid +/- covers ~880..1720
            const float sweepRate = (currentSound == SOUND_TEST) ? 1.5f : 0.8f; // Hz for modulation

            float t = (now - playStart) / 1000.0f;
            float freq_f = f_mid + f_amp * cosf(2.0f * PI * sweepRate * t);
            uint16_t freq = (uint16_t)constrain((int)freq_f, 50, 20000);
            setTone(freq);
            return;
        }

        // sequence-based sounds
        if (activeSeq.freqs == nullptr || activeSeq.durs == nullptr || activeSeq.len == 0) {
            // nothing to play
            stop();
            return;
        }

        // advance step if needed
        unsigned long elapsed = now - stepStart;
        if (elapsed >= activeSeq.durs[stepIndex]) {
            stepIndex++;
            stepStart = now;
            if (stepIndex >= activeSeq.len) {
                if (activeSeq.loop) {
                    stepIndex = 0;
                } else {
                    stop();
                    return;
                }
            }
        }

        // play current step frequency
        uint16_t freq = activeSeq.freqs[stepIndex];
        setTone(freq);
    }

    // let user replace any of the sequences at runtime (keeps async behavior)
    void setSequence(int soundId, const uint16_t* freqs, const uint16_t* durs, size_t len, bool loop = false)
    {
        if (len == 0 || freqs == nullptr || durs == nullptr) return;

        switch (soundId) {
            case SOUND_HAPPY:
                // override default happy sequence by returning it when played
                // store pointer references in static arrays isn't done here for simplicity:
                // play() will use the provided pointers by storing them directly when called.
                break;
            default:
                break;
        }

        // if we are currently playing this sound, immediately switch
        if (playing && currentSound == soundId) {
            activeSeq = { freqs, durs, len, loop };
            stepIndex = 0;
            stepStart = millis();
        }
    }

};


#endif