#include "gd32f30x.h"
#include <string.h>
#include "main.h"
#include "peripheral.h"
#include "gd32f30x_it.h"
#include "SEGGER_RTT.h"
#include "PID.h"
#include <stdlib.h>
#include "NSA2860x.h"
#include "AD7792.h"

extern double adcvalV[2];
extern fpga_txData_t fpgaTXData;       // данные на запись в ПЛИС
extern fpga_rxData_t fpgaRXData;       // данные на чтение из ПЛИС
extern gpio_input_state_t gpioInput;
extern gpio_output_state_t gpioOutput;

double exponentialFilter(double Tf, double new_value, double seampleRatems);

// Инициализация PID-регулятора
void PID_Init(fpga_rxData_t *rxStruct, double Kp, double Ti, double Td, double Tf)
{
    rxStruct->Kp = Kp;
    rxStruct->Ti = Ti;
    rxStruct->Td = Td;
    rxStruct->Tf = Tf;
}

// вычисление периода прерывания таймера в секундах
double getPeriodTimeinSeconds(uint32_t _timer)
{
    // вычисляем период прерывания таймера в ms
    uint32_t perTime = SystemCoreClock/(TIMER_PSC(_timer)+1)*(TIMER_CAR(_timer)+1)/1000000;
    
    // возвращаем период прерывания таймера в секундах
    return (double)perTime / 1000.0;
}

// Функция для расчета выходного сигнала PID-регулятора
double Kp=0.35;
double Ti=0.02;
double Td=0.01;

double PID_Update(fpga_txData_t *txData, fpga_rxData_t *rxData, gpio_output_state_t *gpioOutput, double error)
{
    static double previous_error_1=1.0;
    static double previous_error_2=1.0;
    static double prevOut=1.0;

    uint8_t err=0;
    (void)err;
    double dt=getPeriodTimeinSeconds(TIMER2);

    // Раскомментировать, если требуется получать значения из головного контроллера
    #if PID_COEF_FROM_BUS==1
        _Kp=(double)rxData->Kp;
        _Ti=(double)rxData->Ti;
        _Td=(double)rxData->Td;
        _Tf=(double)rxData->Tf;
    #endif

    // Для коэфф d требуется фильтровать ошибку. Поэтому пропускаем её через фильтр с коэфф Tf
    double expFilter_TdErr = exponentialFilter(_Tf, previous_error_2, dt);

    // расчёт PID регулятора в соответствии с формулой:
    // https://en.wikipedia.org/wiki/Proportional-integral-derivative_controller
    double output = prevOut + (_Kp*((1.0+dt/_Ti +_Td/dt)*error -(1.0+((2.0*_Td)/dt))*previous_error_1+((_Td/dt)*expFilter_TdErr)));

    // проверка и корректировка выходного значения регулятора,
    // если переменная double имеет крайние или неопределённые состояния NaN, -Inf и +Inf
    if (isnan(output))  					output=0.0; 				// детектирование NaN
    if (isinf(output)&& signbit(output))  	output=-PID_OUTPUT_OFFSET; 	// детектирование -Inf      
    else if (isinf(output))					output=PIDOUT_MAX_LIMIT;    // детектирование +Inf
     
    previous_error_2=previous_error_1; previous_error_1=error;

    output=clampSignal(output, -PID_OUTPUT_OFFSET, PIDOUT_MAX_LIMIT, &err);

    // если включен режим слежения
    if (_gpioOutput->DO3==HIGHT) {
        // если Trip_E высокое, то на выход PID регулятора, в зависимости от состояния флага Safe_Pos, выдаём 0% или 100%
        // если Trip_E низкое, то на выход PID регулятора просто дублируем Track_V
        if (txData->Trip_E){
            output=(rxData->Safe_Pos)?PIDOUT_MAX_LIMIT:-PID_OUTPUT_OFFSET;
        }else{
            output=rxData->Track_V;
		}	
    } else {
        output+=PID_OUTPUT_OFFSET;
    }

    output=clampSignal(output, PIDOUT_MIN_LIMIT, PIDOUT_MAX_LIMIT, NULL);
    prevOut=output;
    return output;
}

double exponentialFilter(double Tf, double newValue, double seampleRatems)
{
    static double filteredValue=0.0; // Переменная для хранения предыдущего значения
    Tf=(Tf==0.0)?0.000001:Tf;
    double alpha=1.0-exp(((double)seampleRatems)/Tf*(-1.0));

    // если функция вызывается впервые, то требуется инициализировать фильтр текущим значением
    // далее просто используем формулу фильтра
    filteredValue=(filteredValue==0.0f)?newValue:alpha*newValue+(1.0-alpha)*filteredValue;
    return filteredValue;
}

void processPID(void)
{
    uint8_t _err=0;
    processingGPIO();
    adcval[0]=AD7792_ContinuousReadABS();
    double error=getErrorPID(&_err);

    // считаем выходное воздействие
    fpgaTXData.Out_V=PID_Update(&fpgaTXData, &fpgaRXData, &gpioOutput, error);
    DACsetABS(SPI_GPIO_CS_DAC0, fpgaTXData.Out_V);
}


