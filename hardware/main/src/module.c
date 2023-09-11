/**
 ******************************************************************************
 * @file    module.c
 * @author  GEEKROS,  site:www.geekros.com
 ******************************************************************************
 */

#include "module.h"

Serial_Data_Struct Serial_Read_Data;

void Module_Handle(char *json)
{
    cJSON *serial_data;
    serial_data = cJSON_Parse(json);
    if (serial_data)
    {
        Serial_Read_Data.type = cJSON_GetObjectItem(serial_data, "type")->valuestring;
        if (strcmp(Serial_Read_Data.type, "version") == 0)
        {
            Get_Version();
        }
        if (strcmp(Serial_Read_Data.type, "board-voltage") == 0 || strcmp(Serial_Read_Data.type, "pwm-voltage") == 0 || strcmp(Serial_Read_Data.type, "low-voltage") == 0)
        {
            Adc_Serial_Callback(serial_data);
        }
        if (strcmp(Serial_Read_Data.type, "cpu-temperature") == 0)
        {
            Cpu_Serial_Callback(serial_data);
        }
        if (strcmp(Serial_Read_Data.type, "hmi-init") == 0)
        {
            Hmi_Serial_Callback(serial_data);
        }
        if (strcmp(Serial_Read_Data.type, "io-mode") == 0 || strcmp(Serial_Read_Data.type, "io-status") == 0)
        {
            Io_Serial_Callback(serial_data);
        }
        if (strcmp(Serial_Read_Data.type, "key-status") == 0)
        {
            Key_Serial_Callback(serial_data);
        }
        if (strcmp(Serial_Read_Data.type, "led-status") == 0)
        {
            Led_Serial_Callback(serial_data);
        }
        if (strcmp(Serial_Read_Data.type, "mpu") == 0 || strcmp(Serial_Read_Data.type, "mpu-attitude") == 0)
        {
            Mpu_Serial_Callback(serial_data);
        }
        if (strcmp(Serial_Read_Data.type, "power-status") == 0)
        {
            Power_Serial_Callback(serial_data);
        }
        if (strcmp(Serial_Read_Data.type, "pwm") == 0)
        {
            Pwm_Serial_Callback(serial_data);
        }
        if (strcmp(Serial_Read_Data.type, "rocker") == 0)
        {
            Rocker_Serial_Callback(serial_data);
        }
        if (strcmp(Serial_Read_Data.type, "wifi-init") == 0)
        {
            Wifi_Serial_Callback(serial_data);
        }
    }

    cJSON_Delete(serial_data);
}

void Get_Version(void)
{
    Usb_Write_Data("{\"type\":\"version\",\"version\":%s}\r\n", "1.0.0");
}