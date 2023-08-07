#include "MPU6050_tockn.h"
#include <SimpleFOC.h>

#include "AS5600.h"

#include "DengFOC.h"
#include "esp_flash.h"
#include <Preferences.h>

int Sensor_DIR=1;    
int Motor_PP=7;    
int pin = 0;
TaskHandle_t Task_Log;

#define LOG_BUFF_SIZE 1024
static const char *const g_pcHex = "0123456789abcdef";
struct log_buffer
{
  uint8_t data[LOG_BUFF_SIZE];
  uint16_t write_pos;
  uint16_t read_pos;
}log_buff;

void UART_putc(uint8_t data)
{
#ifdef LOG_PRINT_SYN
	Serial.write((uint8_t *)&data,(size_t )sizeof(data));
#else
    log_buff.data[log_buff.write_pos++] = data;
    if(log_buff.write_pos >= LOG_BUFF_SIZE)
    {
        log_buff.write_pos= 0;
    }
#endif    
}

void log_buffer_flush()
{
    uint8_t data = 0;
    while(log_buff.read_pos != log_buff.write_pos)
    {
        data = log_buff.data[log_buff.read_pos++];
        Serial.write((uint8_t *)&data,(size_t )sizeof(data));

        if(log_buff.read_pos>=LOG_BUFF_SIZE)
        {
            log_buff.read_pos= 0;
        }
    }
}

int32_t UART_dataWrite(const char *pcBuf, uint32_t ulLen)
{
    uint32_t uIdx;

    /* Send the characters */
    for (uIdx = 0; uIdx < ulLen; uIdx++)
    {
        /* If the character to the UART is \n, then add a \r before it so that
         * \n is translated to \n\r in the output. */
        if (pcBuf[uIdx] == (char) '\n')
        {
            UART_putc(((uint8_t)('\r')));
        }

        /* Send the character to the UART output. */
        UART_putc((uint8_t)pcBuf[uIdx]);
    }

    /* Return the number of characters written. */
    return ((int32_t)uIdx);
}

static int32_t UART_convertVal(uint32_t ulValue, uint32_t ulPos, uint32_t ulBase, uint32_t ulNeg, uint32_t ulCount, char cFill, char *pcBuf)
{
    uint32_t ulIdx;
    uint32_t count = ulCount;
    uint32_t neg = ulNeg;
    uint32_t pos = ulPos;

    for (ulIdx = 1u;
         (((ulIdx * ulBase) <= ulValue) &&
          (((ulIdx * ulBase) / ulBase) == ulIdx));
         (ulIdx = (ulIdx * ulBase)))
    {
        count--;
    }

    /* If the value is negative, reduce the count of padding
     * characters needed. */
    if (neg != 0U)
    {
        count--;
    }

    /* If the value is negative and the value is padded with
     * zeros, then place the minus sign before the padding. */
    if ((neg != 0U) && ((int8_t)cFill == (int8_t) '0'))
    {
        /* Place the minus sign in the output buffer. */
        pcBuf[pos] = (char) '-';
        pos++;

        /* The minus sign has been placed, so turn off the
         * negative flag. */
        neg = 0;
    }

    /* Provide additional padding at the beginning of the
     * string conversion if needed. */
    if ((count > 1u) && (count < 16u))
    {
        for (count--; count != 0U; count--)
        {
            pcBuf[pos] = cFill;
            pos++;
        }
    }

    /* If the value is negative, then place the minus sign
     * before the number. */
    if (neg != 0U)
    {
        /* Place the minus sign in the output buffer. */
        pcBuf[pos] = (char) '-';
        pos++;
    }

    /* Convert the value into a string. */
    for (; ulIdx != 0U; ulIdx /= ulBase)
    {
        pcBuf[pos] = g_pcHex[(ulValue / ulIdx) % ulBase];
        pos++;
    }

    /* Write the string. */
    (void)UART_dataWrite(pcBuf, pos);

    return 0;
}

void UART_printf(const char *pcString, ...)
{
    uint32_t ulIdx, ulValue, ulPos, ulCount, ulBase, ulNeg;
    char    *pcStr, pcBuf[16], cFill;
    va_list  vaArgP;
    int32_t temp_var = 0;
    const char *pStr = pcString;

    /* Start the varargs processing. */
    (void)va_start(vaArgP, pcString);

    /* Loop while there are more characters in the string. */
    while (*pStr != (char)0U)
    {
        /* Find the first non-% character, or the end of the string. */
        for (ulIdx = 0;
             (pStr[ulIdx] != (char) '%') &&
             (pStr[ulIdx] != (char) '\0');
             ulIdx++)
        {}

        /* Write this portion of the string. */
        (void)UART_dataWrite(pStr, ulIdx);

        /* Skip the portion of the string that was written. */
        pStr += ulIdx;

        /* See if the next character is a %. */
        if (*pStr == (char) '%')
        {
            /* Skip the %. */
            pStr++;

            /* Set the digit count to zero, and the fill character to space
             * (i.e. to the defaults). */
            ulCount = 0;
            cFill   = (char) ' ';

            /* Determine how to handle the next character. */ 
            while((*pStr >= (char)'0') && (*pStr <= (char)'9'))
            {
                /* If this is a zero, and it is the first digit, then the
                 * fill character is a zero instead of a space. */
                if ((pStr[-1] == (char) '0') && (ulCount == 0U))
                {
                    cFill = (char) '0';
                }

                /* Update the digit count. */
                ulCount *= 10u;
                ulCount += ((uint32_t)(*pStr)) - (uint32_t) '0';

                /* Get the next character. */
                pStr++;
            }
            switch (*pStr)
            {
                /* Handle the %c command. */
                case (char) 'c':
                {
                    /* Get the value from the varargs. */
                    ulValue = (uint32_t)va_arg(vaArgP, uint32_t);

                    /* Print out the character. */
                    (void)UART_dataWrite((const char *) &ulValue, 1u);

                    /* This command has been handled. */
                    break;
                }

                /* Handle the %d command. */
                case (char) 'd':
                {
                    /* Get the value from the varargs. */
                    ulValue = (uint32_t)va_arg(vaArgP, uint32_t);

                    /* Reset the buffer position. */
                    ulPos = 0;

                    /* If the value is negative, make it positive and indicate
                     * that a minus sign is needed. */
                    if ((int32_t) ulValue < 0)
                    {
                        /* Make the value positive. */
                        temp_var = (0 - ((int32_t)ulValue));
                        ulValue = (uint32_t)temp_var;

                        /* Indicate that the value is negative. */
                        ulNeg = 1u;
                    }
                    else
                    {
                        /* Indicate that the value is positive so that a minus
                         * sign isn't inserted. */
                        ulNeg = 0;
                    }

                    /* Set the base to 10. */
                    ulBase = 10u;

                    /* Convert the value to ASCII. */
                    (void)UART_convertVal(ulValue, ulPos, ulBase, ulNeg, ulCount, cFill, pcBuf);
                    
                    break;
                }

                /* Handle the %s command. */
                case (char) 's':
                {
                    /* Get the string pointer from the varargs. */
                    pcStr = va_arg(vaArgP, char *);

                    /* Determine the length of the string. */
                    for (ulIdx = 0; pcStr[ulIdx] != (char) '\0'; ulIdx++)
                    {}

                    /* Write the string. */
                    (void)UART_dataWrite(pcStr, ulIdx);

                    /* Write any required padding spaces */
                    if (ulCount > ulIdx)
                    {
                        ulCount -= ulIdx;
                        while ((ulCount--)  != 0U)
                        {
                            (void)UART_dataWrite(" ", 1u);
                        }
                    }
                    /* This command has been handled. */
                    break;
                }

                /* Handle the %u command. */
                case (char) 'u':
                {
                    /* Get the value from the varargs. */
                    ulValue = (uint32_t)va_arg(vaArgP, uint32_t);

                    /* Reset the buffer position. */
                    ulPos = 0;

                    /* Set the base to 10. */
                    ulBase = 10u;

                    /* Indicate that the value is positive so that a minus sign
                     * isn't inserted. */
                    ulNeg = 0;

                    /* Convert the value to ASCII. */
                    (void)UART_convertVal(ulValue, ulPos, ulBase, ulNeg, ulCount, cFill, pcBuf);
                    
                    break;
                }

                /* Handle the %x and %X commands.  Note that they are treated
                 * identically; i.e. %X will use lower case letters for a-f
                 * instead of the upper case letters is should use.  We also
                 * alias %p to %x. */
                case (char) 'x':
                case (char) 'X':
                case (char) 'p':
                {
                    /* Get the value from the varargs. */
                    ulValue = (uint32_t)va_arg(vaArgP, uint32_t);

                    /* Reset the buffer position. */
                    ulPos = 0;

                    /* Set the base to 16. */
                    ulBase = 16u;

                    /* Indicate that the value is positive so that a minus sign
                     * isn't inserted. */
                    ulNeg = 0;

                    /* Determine the number of digits in the string version of
                     * the value. */
                    (void)UART_convertVal(ulValue, ulPos, ulBase, ulNeg, ulCount, cFill, pcBuf);
                    
                    break;
                }

                /* Handle the %% command. */
                case (char) '%':
                {
                    /* Simply write a single %. */
                    (void)UART_dataWrite(pStr - 1u, 1u);

                    /* This command has been handled. */
                    break;
                }

                /* Handle all other commands. */
                default:
                {
                    /* Indicate an error. */
                    (void)UART_dataWrite("ERROR", 5u);

                    /* This command has been handled. */
                    break;
                }
            } 
            pStr++;
        }
    }

    /* End the varargs processing. */
    va_end(vaArgP);

}

void log_task(void *pvParameters);
volatile uint32_t timer_count = 0;
void tim1Interrupt()
{
  timer_count++;
//   UART_printf("loop 1S %d\r\n",timer_count);
}

hw_timer_t *tim1;
Preferences preferences;
unsigned int counter;
float Sensor_Vel;
float target_speed=4.0f;
PIDController pid_log = PIDController{.P = 0.5, .I = 0.01, .D = 0.001, .ramp = 100000, .limit = 100};

void setup() {
 
  Serial.begin(115200);

  preferences.begin("my-app", false);

  pid_log.P = preferences.getFloat("PID_P", 0);
  pid_log.D = preferences.getFloat("PID_D", 0);
  pid_log.I = preferences.getFloat("PID_I", 0);
  target_speed = preferences.getFloat("target_speed",0);
  preferences.end();

  tim1 = timerBegin(0, 80, true);
  timerAttachInterrupt(tim1, tim1Interrupt, true);
  timerAlarmWrite(tim1, 1000ul, true);
  timerAlarmEnable(tim1);

  xTaskCreatePinnedToCore(log_task,"log_task",10000,NULL,1,&Task_Log,1);
  DFOC_Vbus(7.2);   
  DFOC_alignSensor(Motor_PP,Sensor_DIR);
  pinMode(LED_BUILTIN, OUTPUT);
}


void loop() 
{
    if(timer_count%1000==0)
    {
        UART_printf("Current counter value: %u\n", counter);
        UART_printf("loop 1S %d\r\n",timer_count);
    }
    if(timer_count%5==0)
    {
        timer_count++;
        digitalWrite(LED_BUILTIN,pin);
        pin=!pin;
        DFOC_M0_SET_ANGLE_PID(0.5,0,0,0);
        DFOC_M0_SET_VEL_PID(pid_log.P,pid_log.I,pid_log.D,1000);
        Sensor_Vel=DFOC_M0_Velocity();
        setTorque(DFOC_M0_VEL_PID((target_speed-Sensor_Vel)),_electricalAngle());   //速度闭环
        // DFOC_M0_setVelocity(target_speed);
    }

}
void set_pid();
void log_task(void *pvParameters)
{
  for(;;)
  {
    static uint8_t count=0;
    //Serial.printf("speed %f read speed%f\r\n",target_speed,Sensor_Vel);
    set_pid();
    if(count++ %10 ==0)
    {
        log_printf("target %f now %f\r\n",target_speed,Sensor_Vel);
    }
    log_buffer_flush();
    delay(100);
  }
}
void set_pid()
{
    String data = "";
    uint8_t read_char = 0;

    if(Serial.available())
    {
        while(Serial.available())
        {
            data += char(Serial.read());
        }

        log_printf("recv cmd %s",data);
        String set_cmd = data.substring(0,2);
        String value = data.substring(2,data.length());
        log_printf("set cmd %s",set_cmd);
        log_printf("set value %f",value.toFloat());

        if(set_cmd=="P=")
        {
            pid_log.P=value.toFloat();
        }
        if(set_cmd=="I=")
        {
            pid_log.I=value.toFloat();
        }
        if(set_cmd=="D=")
        {
            pid_log.D=value.toFloat();
        }
        if(set_cmd=="S=")
        {
            target_speed=value.toFloat();
        }
        if(set_cmd=="RE")
        {  
            // Restart ESP
            ESP.restart();
        }
        // Store the counter to the Preferences
        // pid_log.P = preferences.getFloat("PID_P", 0);
        // pid_log.P = preferences.getFloat("PID_P", 0);
        // pid_log.P = preferences.getFloat("PID_P", 0);
        // target_speed = preferences.getFloat("target_speed",0);
        preferences.begin("my-app", false);
        preferences.putFloat("PID_P", pid_log.P);
        preferences.putFloat("PID_D", pid_log.D);
        preferences.putFloat("PID_I", pid_log.I);
        preferences.putFloat("target_speed", target_speed);
        preferences.end();
        
        log_printf("PID values P I D %f %f %f \r\ntarget speede\r\n",pid_log.P,pid_log.I,pid_log.D,target_speed);
    }
}