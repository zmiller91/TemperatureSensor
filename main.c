/**
 * A few things to note here: 
 * 
 * 1. This file must contain the config bits from MCC
 * 2. Otherwise this file delegates the configuration to the BaseBoardCore package
 * 
 * In order to link the two, you'll need to:
 * 
 * 1. Right Click "Libraries" and add the BaseBoardCore.x library
 * 2. Go to Properties -> XC8 Compiler -> Include Directories and add the 
 *    BaseBoardCore.x library
 * 
 */


#include "run.h"
#include "log.h"
#include "lora.h"
#include "i2c_manager.h"
#include "string_utils.h"
#include <stdbool.h>
#include "mcc_generated_files/system/config_bits.h"


#ifndef BME280_ADDR
#define BME280_ADDR 0x76u   // use 0x77 if SDO=VDD
#endif

/**
 * The BME280 calibration data provides some signed 12 bit integers. This method
 * takes those signed 12 bit ints and turns them into signed 16 bit ints. 
 */
static inline int16_t signext12(uint16_t x) {
    return (x & 0x0800) ? (int16_t)(x | 0xF000) : (int16_t)(x & 0x0FFF);
}

/**
 * Normally the BME280 driver for Bosch would be used, but its too heavy for this pic
 * and consumes a lot of program space and stack space. When integrating it with
 * BaseBoardCore, I ran into stack issues where the chip would reset due to a
 * stack overflow. 
 * 
 * The code in this method extracts the primary functions of the BME280 without
 * using the driver. All of the compensation formulas were take from the BME280
 * data sheet. 
 */
void task(void) {
    
    LED_EN_SetHigh();
    I2C_EN_SetHigh();
    __delay_ms(1000);
        
    // Device Chip ID must be 96. If it's not, then return early because the
    // chip isn't working. 
    uint8_t id;
    uint8_t device_reg = 0xD0;
    i2c_mgr_write_read(BME280_ADDR, &device_reg, 1, &id, 1);
    if(id != 0x60) {
        log_debug("BME280 not initialized");
        return;
    }
    
    // Before doing anything we need to set the oversampling rates for humidity
    // and temperature (optionally pressure). The humidity oversampling config
    // is stored in a separate register, so set that here. 
    uint8_t ctrl_hum_reg = 0xF2;
    uint8_t ctrl_hum_val = 0x01;
    uint8_t ctrl_hum_write[2] = {ctrl_hum_reg, ctrl_hum_val}; 
    i2c_mgr_write(BME280_ADDR, ctrl_hum_write, 2);
    
    // Temperature, pressure, and the sampling mode are all in the same register
    // so they can all be set at once. 
    uint8_t ctrl_meas_reg = 0xF4;
    uint8_t ctrl_meas_val = 0x21; // 001 oversampling for temp, 000 oversampling for pressure, 01 mode (forced)
    uint8_t ctrl_meas_write[2] = {ctrl_meas_reg, ctrl_meas_val};
    i2c_mgr_write(BME280_ADDR, ctrl_meas_write, 2);
    
    // Continuously check the status bit to clear, indicating the measurement is
    // finished. Performing a bitwise and on 0x08 will return bit 3. 
    uint8_t status;
    uint8_t status_reg = 0xF3;
    do {
        i2c_mgr_write_read(BME280_ADDR, &status_reg, 1, &status, 1);
    } while(status & 0x08); // bitwise and with 0x08 returns true when the measuring bit (3) is 1
    
    // Collect the data for pressure, temperature, and humidity. 
    uint8_t data[8];
    uint8_t data_reg = 0xF7;
    i2c_mgr_write_read(BME280_ADDR, &data_reg, 1, data, 8);
    
    // Parse the data and store it into it's own variable for calibration. 
    uint32_t temp_raw = ((uint32_t)data[3] << 12) | ((uint32_t)data[4] << 4) | ((uint32_t)data[5] >> 4);
    uint32_t hum_raw = ((uint32_t)data[6] << 8) | data[7];
    
    /**
     *
     * TEMPERATURE COMPENSATION SECTION
     * 
     */
    
    // Perform the temperature compensation using calibration data
    uint8_t dig_t_reg = 0x88;
    uint8_t dig_t_data[6];
    i2c_mgr_write_read(BME280_ADDR, &dig_t_reg, 1, dig_t_data, 6);
    
    uint16_t dig_T1 = (uint16_t)((uint16_t)dig_t_data[1] << 8 | dig_t_data[0]);
    int16_t  dig_T2 = (int16_t) ((uint16_t)dig_t_data[3] << 8 | dig_t_data[2]);
    int16_t  dig_T3 = (int16_t) ((uint16_t)dig_t_data[5] << 8 | dig_t_data[4]);
    
    // Calibration formula can be found in the data sheet 
    int32_t var1 = ((((int32_t)(temp_raw >> 3) - ((int32_t)dig_T1 << 1))) * (int32_t)dig_T2) >> 11;
    int32_t var2 = (((((int32_t)(temp_raw >> 4) - (int32_t)dig_T1) * (((int32_t)temp_raw >> 4) - (int32_t)dig_T1)) >> 12) * (int32_t)dig_T3) >> 14;
    int32_t t_fine = var1 + var2;
    
    // Temperature in °C * 100
    int32_t temp = (t_fine * 5 + 128) >> 8;
    
    /**
     *
     * HUMIDITY COMPENSATION SECTION
     * 
     */
    
    // Perform the humidity compensation using calibration data. dig_H1 is
    // stored in it's own register, so it needs to be pulled out individually.
    uint8_t dig_h1_reg = 0xA1;
    uint8_t dig_H1;
    i2c_mgr_write_read(BME280_ADDR, &dig_h1_reg, 1, &dig_H1, 1);
    
    uint8_t dig_h_reg = 0xE1;
    uint8_t dig_h_data[7];
    i2c_mgr_write_read(BME280_ADDR, &dig_h_reg, 1, dig_h_data, 7);
    
    int16_t dig_H2 = (int16_t)(((uint16_t)dig_h_data[1] << 8) | dig_h_data[0]);
    uint8_t dig_H3 = dig_h_data[2];
    int16_t dig_H4 = signext12( (uint16_t)((dig_h_data[3] << 4) | (dig_h_data[4] & 0x0F)) );
    int16_t dig_H5 = signext12( (uint16_t)((dig_h_data[5] << 4) | (dig_h_data[4] >> 4)) );
    int8_t dig_H6 = (int8_t) dig_h_data[6];
    
    // Compensation formulas can be found in the data sheet
   int32_t v_x1 = t_fine - 76800;
   int32_t tmp  = (((((int32_t)hum_raw << 14) - ((int32_t)dig_H4 << 20) - ((int32_t)dig_H5 * v_x1)) + 16384) >> 15);
   int32_t tmp2 = (((((v_x1 * (int32_t)dig_H6) >> 10) * (((v_x1 * (int32_t)dig_H3) >> 11) + 32768)) >> 10) + 2097152);
   tmp2 = (tmp2 * (int32_t)dig_H2 + 8192) >> 14;
    v_x1 = tmp * tmp2;
    v_x1 = v_x1 - (((((v_x1 >> 15) * (v_x1 >> 15)) >> 7) * (int32_t)dig_H1) >> 4);
    if (v_x1 < 0) v_x1 = 0;
    if (v_x1 > 419430400) v_x1 = 419430400;

    // %RH * 1024
    uint32_t humidity = (uint32_t)(v_x1 >> 12);            
    
   
    // Transmit the data
    lora_enable();
    int32_t result[] = {temp, (int32_t) humidity};
    lora_send(32, result, 2);
    
    I2C_EN_SetLow();
    LED_EN_SetLow();
    set_sleep_period(THIRTY_MINUTES);
}

int main(void) {
    return run(task);   
}