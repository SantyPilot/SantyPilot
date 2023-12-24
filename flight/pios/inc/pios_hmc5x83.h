/**
 ******************************************************************************
 * @addtogroup PIOS PIOS Core hardware abstraction layer
 * @{
 * @addtogroup PIOS_HMC5x83 HMC5x83 Functions
 * @brief Deals with the hardware interface to the magnetometers
 * @{
 *
 * @file       pios_hmc5x83.h
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2012.
 * @brief      HMC5x83 functions header.
 * @see        The GNU Public License (GPL) Version 3
 *
 *****************************************************************************/
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#ifndef PIOS_HMC5x83_H
#define PIOS_HMC5x83_H
#include <stdint.h>
#include <pios_sensors.h>
/* HMC5x83 Addresses */
#define PIOS_HMC5x83_I2C_ADDR           0x1E
#define PIOS_HMC5x83_I2C_READ_ADDR      0x3D
#define PIOS_HMC5x83_I2C_WRITE_ADDR     0x3C

#define PIOS_HMC5x83_SPI_READ_FLAG      0x80
#define PIOS_HMC5x83_SPI_AUTOINCR_FLAG  0x40
#define PIOS_HMC5x83_CONFIG_REG_A       (uint8_t)0x00
#define PIOS_HMC5x83_CONFIG_REG_B       (uint8_t)0x01
#define PIOS_HMC5x83_MODE_REG           (uint8_t)0x02
#define PIOS_HMC5x83_DATAOUT_XMSB_REG   0x03
#define PIOS_HMC5x83_DATAOUT_XLSB_REG   0x04
#define PIOS_HMC5x83_DATAOUT_ZMSB_REG   0x05
#define PIOS_HMC5x83_DATAOUT_ZLSB_REG   0x06
#define PIOS_HMC5x83_DATAOUT_YMSB_REG   0x07
#define PIOS_HMC5x83_DATAOUT_YLSB_REG   0x08
#define PIOS_HMC5x83_DATAOUT_STATUS_REG 0x09
#define PIOS_HMC5x83_DATAOUT_IDA_REG    0x0A
#define PIOS_HMC5x83_DATAOUT_IDB_REG    0x0B
#define PIOS_HMC5x83_DATAOUT_IDC_REG    0x0C

/* Output Data Rate */
#define PIOS_HMC5x83_ODR_0_75           0x00
#define PIOS_HMC5x83_ODR_1_5            0x04
#define PIOS_HMC5x83_ODR_3              0x08
#define PIOS_HMC5x83_ODR_7_5            0x0C
#define PIOS_HMC5x83_ODR_15             0x10
#define PIOS_HMC5x83_ODR_30             0x14
#define PIOS_HMC5x83_ODR_75             0x18

/* Measure configuration */
#define PIOS_HMC5x83_MEASCONF_NORMAL    0x00
#define PIOS_HMC5x83_MEASCONF_BIAS_POS  0x01
#define PIOS_HMC5x83_MEASCONF_BIAS_NEG  0x02

/* Gain settings */
#define PIOS_HMC5x83_GAIN_0_88          0x00
#define PIOS_HMC5x83_GAIN_1_3           0x20
#define PIOS_HMC5x83_GAIN_1_9           0x40
#define PIOS_HMC5x83_GAIN_2_5           0x60
#define PIOS_HMC5x83_GAIN_4_0           0x80
#define PIOS_HMC5x83_GAIN_4_7           0xA0
#define PIOS_HMC5x83_GAIN_5_6           0xC0
#define PIOS_HMC5x83_GAIN_8_1           0xE0

#define PIOS_HMC5x83_CTRLA_TEMP         0x40

/* Modes */
#define PIOS_HMC5x83_MODE_CONTINUOUS    0x00
#define PIOS_HMC5x83_MODE_SINGLE        0x01
#define PIOS_HMC5x83_MODE_IDLE          0x02
#define PIOS_HMC5x83_MODE_SLEEP         0x03

/* Sensitivity Conversion Values */
#define PIOS_HMC5x83_Sensitivity_0_88Ga 1370 // LSB/Ga
#define PIOS_HMC5x83_Sensitivity_1_3Ga  1090    // LSB/Ga
#define PIOS_HMC5x83_Sensitivity_1_9Ga  820     // LSB/Ga
#define PIOS_HMC5x83_Sensitivity_2_5Ga  660     // LSB/Ga
#define PIOS_HMC5x83_Sensitivity_4_0Ga  440     // LSB/Ga
#define PIOS_HMC5x83_Sensitivity_4_7Ga  390     // LSB/Ga
#define PIOS_HMC5x83_Sensitivity_5_6Ga  330     // LSB/Ga
#define PIOS_HMC5x83_Sensitivity_8_1Ga  230     // LSB/Ga  --> NOT RECOMMENDED

/* Status Register */
#define PIOS_HMC5x83_DATAOUT_STATUS_RDY 0x01

typedef uintptr_t pios_hmc5x83_dev_t;

struct pios_hmc5x83_io_driver {
    int32_t (*Write)(pios_hmc5x83_dev_t handler, uint8_t address, uint8_t buffer);
    int32_t (*Read)(pios_hmc5x83_dev_t handler, uint8_t address, uint8_t *buffer, uint8_t len);
};

#ifdef PIOS_INCLUDE_SPI
extern const struct pios_hmc5x83_io_driver PIOS_HMC5x83_SPI_DRIVER;
#endif

#ifdef PIOS_INCLUDE_I2C
extern const struct pios_hmc5x83_io_driver PIOS_HMC5x83_I2C_DRIVER;
#endif
// xyz axis orientation
enum PIOS_HMC5X83_ORIENTATION {
    PIOS_HMC5X83_ORIENTATION_EAST_NORTH_UP,
    PIOS_HMC5X83_ORIENTATION_SOUTH_EAST_UP,
    PIOS_HMC5X83_ORIENTATION_WEST_SOUTH_UP,
    PIOS_HMC5X83_ORIENTATION_NORTH_WEST_UP,
    PIOS_HMC5X83_ORIENTATION_EAST_SOUTH_DOWN,
    PIOS_HMC5X83_ORIENTATION_SOUTH_WEST_DOWN,
    PIOS_HMC5X83_ORIENTATION_WEST_NORTH_DOWN,
    PIOS_HMC5X83_ORIENTATION_NORTH_EAST_DOWN,
};


struct pios_hmc5x83_cfg {
#ifdef PIOS_HMC5X83_HAS_GPIOS
    const struct pios_exti_cfg *exti_cfg; /* Pointer to the EXTI configuration */
#endif
    uint8_t M_ODR; // OUTPUT DATA RATE --> here below the relative define (See datasheet page 11 for more details) */
    uint8_t Meas_Conf; // Measurement Configuration,: Normal, positive bias, or negative bias --> here below the relative define */
    uint8_t Gain; // Gain Configuration, select the full scale --> here below the relative define (See datasheet page 11 for more details) */
    uint8_t Mode;
    bool    TempCompensation; // enable temperature sensor on HMC5983 for temperature gain compensation
    enum PIOS_HMC5X83_ORIENTATION Orientation;
    const struct pios_hmc5x83_io_driver *Driver;
};

/* Public Functions */
extern pios_hmc5x83_dev_t PIOS_HMC5x83_Init(const struct pios_hmc5x83_cfg *cfg, uint32_t port_id, uint8_t device_num);
extern void PIOS_HMC5x83_Register(pios_hmc5x83_dev_t handler, PIOS_SENSORS_TYPE sensortype);

extern bool PIOS_HMC5x83_NewDataAvailable(pios_hmc5x83_dev_t handler);
extern int32_t PIOS_HMC5x83_ReadMag(pios_hmc5x83_dev_t handler, int16_t out[3]);
extern uint8_t PIOS_HMC5x83_ReadID(pios_hmc5x83_dev_t handler, uint8_t out[4]);
extern int32_t PIOS_HMC5x83_Test(pios_hmc5x83_dev_t handler);
extern bool PIOS_HMC5x83_IRQHandler(pios_hmc5x83_dev_t handler);

extern const PIOS_SENSORS_Driver PIOS_HMC5x83_Driver;

#endif /* PIOS_HMC5x83_H */

/**
 * @}
 * @}
 */
