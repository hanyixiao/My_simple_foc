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
TaskHandle_t Task_Ctrl;

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

void LOG_print(const char *pcString, ...)
{
    char buf[128] = {};
    va_list  vaArgP;
    int len = 0;

    (void)va_start(vaArgP, pcString);
    len = vsnprintf(buf,sizeof(buf),pcString,vaArgP);

    va_end(vaArgP);
    UART_dataWrite(buf,len);
}

void log_task(void *pvParameters);
void control_task(void *pvParameters);

volatile uint32_t timer_count = 0;
volatile uint32_t timer_50us = 0;

hw_timer_t *tim1;
hw_timer_t *tim2;
Preferences preferences;
unsigned int counter;
float Sensor_Vel;
float target_speed=4.0f;
PIDController pid_log = PIDController{.P = 0.5, .I = 0.01, .D = 0.001, .ramp = 100000, .limit = 100};
SemaphoreHandle_t xSemaphore;
static BaseType_t xHigherPriorityTaskWoken;
 
void tim1Interrupt()
{
    timer_count++;

    switch(timer_count%2)
    {
        case 0:
        {
            xHigherPriorityTaskWoken = pdFALSE;
            xSemaphoreGiveFromISR( xSemaphore, &xHigherPriorityTaskWoken);
            break;
        }
        default:
        break;
    }
}
void tim2Interrupt()
{
    timer_50us++;
}

void setup() {
 
  preferences.begin("my-app", false);

  pid_log.P = preferences.getFloat("PID_P", 0);
  pid_log.D = preferences.getFloat("PID_D", 0);
  pid_log.I = preferences.getFloat("PID_I", 0);
  target_speed = preferences.getFloat("target_speed",0);
  preferences.end();

  tim1 = timerBegin(0, 80, true);
  tim2 = timerBegin(1, 80, true);//40M

  timerAttachInterrupt(tim1, tim1Interrupt, true);
  timerAlarmWrite(tim1, 1000ul, true);
  timerAlarmEnable(tim1);

  timerAttachInterrupt(tim2, tim2Interrupt, true);
  timerAlarmWrite(tim2, 50ul, true);
  timerAlarmEnable(tim2);
  xSemaphore = xSemaphoreCreateBinary();


  DFOC_Vbus(11.2);   
  DFOC_alignSensor(Motor_PP,Sensor_DIR);
  pinMode(LED_BUILTIN, OUTPUT);
  LOG_print("aduino run at core %d frequence %d",xPortGetCoreID(),ESP.getCpuFreqMHz());

  xTaskCreatePinnedToCore(control_task,"control_task",8192,NULL,3,&Task_Ctrl,1);
  xTaskCreatePinnedToCore(log_task,"log_task",8192,NULL,2,&Task_Log,0);
}


void loop() 
{
    sleep(1000);
}

void set_pid();
extern float Ua,Ub,Uc;
void control_task(void *pvParameters)
{ 
    LOG_print("control_task task run at %d\r\n",xPortGetCoreID());
    long timer_pre = 0;
    long timer_now = 0;
    long start_timer = 0;
    uint32_t timer_pre_50us = 0;

    BaseType_t rec;
    BaseType_t msToWait = 1000; // 信号量接收等待时间
    TickType_t ticks;

    uint8_t count = 0;
    float t = 0;
    for(;;)
    {
        rec = xSemaphoreTake(xSemaphore, pdMS_TO_TICKS(msToWait));//pdMS_TO_TICKS将ms转成对应的滴答数
        if (rec==pdTRUE) {
                // timer_now = micros();
                // LOG_print("timer 50us counter%d\r\n",timer_50us-timer_pre_50us);
                // LOG_print("timer  micros() %d\r\n",timer_now-timer_pre);
                // LOG_print("timer1ms %d\r\n",timer_count);
                // timer_pre = timer_now;
                // timer_pre_50us = timer_50us;
                digitalWrite(LED_BUILTIN,pin);
                pin=!pin;
                runFOC();
                DFOC_M0_SET_CURRENT_PID(50,200,0,100);
                DFOC_M0_setTorque(serial_motor_target());
                count++;
                if(count>30)
                {
                    // count=0;
                    // Serial.printf("%f %f\n", DFOC_M0_Current(), serial_motor_target());
                    // t += 0.1;
                    // 发送数据
                    float ch[4];  
                    ch[0] = DFOC_M0_Current();
                    ch[1] = Uc;
                    ch[2] = Ua;
                    ch[3] = Ub;
                    Serial.write((char *)ch, sizeof(float) * 4); 
                    // 发送帧尾
                    char tail[4] = {0x00, 0x00, 0x80, 0x7f};
                    Serial.write(tail, 4);
                }
                // Sensor_Vel=DFOC_M0_Velocity();
                // setTorque(DFOC_M0_VEL_PID((target_speed-Sensor_Vel)),_electricalAngle());   //速度闭环
                //接收串口
                serialReceiveUserCommand();
        }
        else {
        ticks = pdMS_TO_TICKS(msToWait);
        Serial.printf("Fialed to receive semaphore after waiting %d ticks!\n", ticks);
        }

    }
}

void log_task(void *pvParameters)
{
  Serial.begin(115200);
  LOG_print("log task run at %d\r\n",xPortGetCoreID());
  uint8_t count=0;
//   long timer_pre = 0;
//   long timer = 0;
  for(;;)
  {
  
    //Serial.printf("speed %f read speed%f\r\n",target_speed,Sensor_Vel);
    // set_pid();
    // if(count++ %5 ==0)
    // {
    //     LOG_print("target %f now %f\r\n",target_speed,Sensor_Vel);
    // }
    // // timer = timer_count;
    // // LOG_print("test time %d\r\n",timer - timer_pre);
    // log_buffer_flush();
    // timer_pre =  timer;
    delay(50);
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