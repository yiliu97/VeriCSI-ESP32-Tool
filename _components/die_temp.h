#ifndef ESP32_CSI_DIE_TEMP_H
#define ESP32_CSI_DIE_TEMP_H

/*
 * Die temperature, sampled alongside CSI records.
 *
 * WHY THIS EXISTS
 * ---------------
 * The device-clock rate calibration this toolkit reports is only transferable
 * at thermal equilibrium: a single board's rate moves by several ppm within
 * minutes of the radio becoming busy. Reporting that as a caveat is weaker
 * than reporting it as a covariate, and a covariate needs the temperature to
 * travel with the record. Every SoC in the matrix except the original ESP32
 * exposes an on-die sensor, so this is nearly free.
 *
 * SEMANTICS
 * ---------
 * die_temp_read_c10() returns tenths of a degree Celsius as an int, so the
 * CSV stays integer-only, and CSI_FIELD_NA (-1000, distinct from a real
 * -0.1 C reading) when the part has no sensor or the driver is not up. The
 * sensor is sampled on a slow timer, not per record: an on-die sensor settles
 * far slower than a 100 Hz record stream, and reading it per record would put
 * a driver call inside the CSI callback path.
 */

#include "sdkconfig.h"
#include "esp_err.h"
#include "esp_log.h"
#include "soc/soc_caps.h"

#define DIE_TEMP_NA (-1000)

/*
 * Two gates, and they mean different things.
 *
 * SOC_TEMP_SENSOR_SUPPORTED is a hardware fact: the original ESP32 has no
 * on-die sensor. ESP32_CSI_TOOL_RECORD_DIE_TEMP is a build choice: the sensor
 * read costs a driver call on the diagnostic cadence, and a deployment that
 * does not want it should not pay for it.
 *
 * Neither gate removes the column. The schema is a fixed superset across every
 * SoC and every build, so die_temp_c10 is always emitted and reports
 * DIE_TEMP_NA when the part has no sensor or the build did not ask for it. A
 * reader can then tell "not measured" from a measured value, which is the
 * whole point of the NULL semantics.
 */
#if SOC_TEMP_SENSOR_SUPPORTED && defined(CONFIG_ESP32_CSI_TOOL_RECORD_DIE_TEMP)
#include "driver/temperature_sensor.h"

static temperature_sensor_handle_t s_die_temp_handle = NULL;
static volatile int s_die_temp_c10 = DIE_TEMP_NA;

static inline void die_temp_init(void)
{
    temperature_sensor_config_t cfg = TEMPERATURE_SENSOR_CONFIG_DEFAULT(-10, 80);
    if (temperature_sensor_install(&cfg, &s_die_temp_handle) != ESP_OK) {
        s_die_temp_handle = NULL;
        return;
    }
    if (temperature_sensor_enable(s_die_temp_handle) != ESP_OK) {
        temperature_sensor_uninstall(s_die_temp_handle);
        s_die_temp_handle = NULL;
    }
}

/* Called from the consumer task, not the CSI callback. */
static inline void die_temp_sample(void)
{
    float c = 0.0f;
    if (s_die_temp_handle == NULL) return;
    if (temperature_sensor_get_celsius(s_die_temp_handle, &c) == ESP_OK) {
        s_die_temp_c10 = (int)(c * 10.0f);
    }
}

static inline int die_temp_read_c10(void) { return s_die_temp_c10; }

#else  /* no on-die sensor, or the build did not ask for it */

static inline void die_temp_init(void) {}
static inline void die_temp_sample(void) {}
static inline int  die_temp_read_c10(void) { return DIE_TEMP_NA; }

#endif /* SOC_TEMP_SENSOR_SUPPORTED && CONFIG_ESP32_CSI_TOOL_RECORD_DIE_TEMP */

#endif /* ESP32_CSI_DIE_TEMP_H */
