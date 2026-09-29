#include "targets.h"
#include "common.h"
#include "config.h"
#include "logging.h"

#include <functional>
#include <Wire.h>

static const int maxDeferredFunctions = 3;

struct deferred_t {
    unsigned long started;
    unsigned long timeout;
    std::function<void()> function;
};

static deferred_t deferred[maxDeferredFunctions] = {
    {0, 0, nullptr},
    {0, 0, nullptr},
    {0, 0, nullptr},
};

boolean i2c_enabled = false;
int i2c_gpio_sda = UNDEF_PIN;
int i2c_gpio_scl = UNDEF_PIN;
static unsigned long rebootTime_Ms = 0;

static void setupWire()
{
    int gpio_scl = GPIO_PIN_SCL;
    int gpio_sda = GPIO_PIN_SDA;

#if defined(TARGET_RX)
    for (int ch = 0 ; ch < GPIO_PIN_PWM_OUTPUTS_COUNT ; ++ch)
    {
        auto pin = GPIO_PIN_PWM_OUTPUTS[ch];
        auto pwm = config.GetPwmChannel(ch);
        // if the PWM pin is nominated as SDA or SCL, and it's not configured for I2C then undef the pins
        if ((pin == GPIO_PIN_SCL && pwm->val.mode != somSCL) || (pin == GPIO_PIN_SDA && pwm->val.mode != somSDA))
        {
            gpio_scl = UNDEF_PIN;
            gpio_sda = UNDEF_PIN;
            break;
        }
        // If I2C pins are not defined in the hardware, then look for configured I2C
        if (GPIO_PIN_SCL == UNDEF_PIN && pwm->val.mode == somSCL)
        {
            gpio_scl = pin;
        }
        if (GPIO_PIN_SCL == UNDEF_PIN && pwm->val.mode == somSDA)
        {
            gpio_sda = pin;
        }
    }
#endif
    i2c_gpio_sda = gpio_sda;
    i2c_gpio_scl = gpio_scl;

    if(gpio_sda != UNDEF_PIN && gpio_scl != UNDEF_PIN)
    {
#if defined(PLATFORM_ESP8266)
        // CRITICAL FOR ESP8266 OPEN-DRAIN SIMULATION:
        // On ESP8266, software I2C (core_esp8266_si2c.cpp) drives LOW by setting GPES (output enable).
        // It requires the GPIO output data registers to be 0 (LOW)!
        // If output data registers were set to 1 by digitalWrite(HIGH), GPES will drive 3.3V instead of 0V.
        pinMode(gpio_sda, INPUT_PULLUP);
        pinMode(gpio_scl, INPUT_PULLUP);
        digitalWrite(gpio_sda, LOW);
        digitalWrite(gpio_scl, LOW);
        GPOC = (1 << gpio_sda) | (1 << gpio_scl);
#endif

        DBGLN("Starting wire on SCL %d, SDA %d", gpio_scl, gpio_sda);
        Wire.begin(gpio_sda, gpio_scl);
        Wire.setClock(400000);
        i2c_enabled = true;
    }
}

void setupTargetCommon()
{
    setupWire();
}

void deferExecutionMicros(unsigned long us, std::function<void()> f)
{
    for (int i=0 ; i<maxDeferredFunctions ; i++)
    {
        if (deferred[i].function == nullptr)
        {
            deferred[i].started = micros();
            deferred[i].timeout = us;
            deferred[i].function = f;
            return;
        }
    }

    // Bail out, there are no slots available!
    DBGLN("No more deferred function slots available!");
}

void executeDeferredFunction(unsigned long now)
{
    // execute deferred function if its time has elapsed
    for (int i=0 ; i<maxDeferredFunctions ; i++)
    {
        if (deferred[i].function != nullptr && (now - deferred[i].started) > deferred[i].timeout)
        {
            deferred[i].function();
            deferred[i].function = nullptr;
        }
    }
}

/***
 * @brief Set a time in milliseconds to reboot the MCU from the main loop thread
 * */
void scheduleRebootTime(unsigned long inMs)
{
    rebootTime_Ms = millis() + inMs;
}

/**
 * @brief Call from the main thread to check if it is time to reboot. May not return.
 */
void checkRebootTime(unsigned long now)
{
    // If the reboot time is set and the current time is past the reboot time then reboot.
    // Wait for any pending config change to be committed first
    if (rebootTime_Ms != 0 && !config.IsModified() && now > rebootTime_Ms ) {
        ESP.restart();
    }
}