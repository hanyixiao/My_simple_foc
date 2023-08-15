#include "lowpass_filter.h"

LowPassFilter::LowPassFilter(float time_constant)
    : Tf(time_constant)
    , y_prev(0.0f)
{
    timestamp_prev = micros();
}

extern void UART_printf(const char *pcString, ...);
float LowPassFilter::operator() (float x)
{
    static float x_pre;
    unsigned long timestamp = micros();

    float dt = (timestamp - timestamp_prev)*1e-6f;
    
    // log_printf("dt time value: %d \n", timestamp-timestamp_prev);
    // log_printf("dv value %f\r\n",x);

    /*speed limit */
    if(abs(x)>=60)
    {
        x=x_pre;
    }
    x_pre = x;

    if (dt < 0.0f ) dt = 1e-3f;
    else if(dt > 0.3f) {
        y_prev = x;
        timestamp_prev = timestamp;
        return x;
    }

    float alpha = Tf/(Tf + dt);
    float y = alpha*y_prev + (1.0f - alpha)*x;
    y_prev = y;
    timestamp_prev = timestamp;
    return y;
}
