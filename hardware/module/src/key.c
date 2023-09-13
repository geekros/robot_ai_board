/**
 ******************************************************************************
 * @file    key.c
 * @author  GEEKROS,  site:www.geekros.com
 ******************************************************************************
 */

#include "key.h"

void Key_Init(void)
{
    
}

void Key_Serial_Callback(cJSON *serial_data)
{
    cJSON *type = cJSON_GetObjectItem(serial_data, "type");
    if (type && cJSON_IsString(type))
    {
        if(strcmp(type->valuestring, "key-status") == 0)
        {
            
        }
    }
}
