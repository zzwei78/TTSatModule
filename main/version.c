/*
 * version.c - Firmware Version Information Implementation
 *
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include "version.h"

const char* get_firmware_version(void)
{
    return FIRMWARE_VERSION_STRING;
}

const char* get_software_version(void)
{
    return SOFTWARE_VERSION_STRING;
}

const char* get_manufacturer_name(void)
{
    return PRODUCT_MANUFACTURER;
}

const char* get_model_number(void)
{
    return PRODUCT_MODEL_NUMBER;
}

const char* get_hardware_revision(void)
{
    return PRODUCT_HARDWARE_REVISION;
}

const char* get_build_datetime(void)
{
    /* Combine date and time */
    static char build_datetime[32] = {0};
    if (build_datetime[0] == '\0') {
        snprintf(build_datetime, sizeof(build_datetime), "%s %s", FIRMWARE_BUILD_DATE, FIRMWARE_BUILD_TIME);
    }
    return build_datetime;
}
