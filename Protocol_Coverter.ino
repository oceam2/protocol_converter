////////////////////////////////////////////////////////////////
// Protocol Converter
// Conversor parte i2c-j1939
// Ver. 4.2
////////////////////////////////////////////////////////////////

#if defined ARDUINO_SAM_DUE
#include <FreeRTOS_ARM.h>
#else
#include <FreeRTOS_AVR.h>
#endif

#include "mcp_can_j1939_v2.h"
#include "can_ext_j1939_v2.h"
#include <Wire.h>
#include "LedControl.h"
#include "medianFilterLib.h"
#include <EEPROM.h>

LedControl lc = LedControl(31, 29, 30, 1); // (dts,clk,cs,1) MAX72197

#define pin_GLP 4
#define PWM 3
#define Int_Can 2
#define mem_tramas_can 150  // normalmente 200
#define LED_Rojo 16 //*
#define LED_Verde 23 //*
#define LED_Azul 25 //*
#define led_verde_ON digitalWrite(LED_Verde, HIGH)
#define led_verde_OFF digitalWrite(LED_Verde, LOW)
#define led_rojo_ON digitalWrite(LED_Rojo, HIGH)
#define led_rojo_OFF digitalWrite(LED_Rojo, LOW)
#define led_azul_ON  digitalWrite(LED_Azul, HIGH)
#define led_azul_OFF digitalWrite(LED_Azul, LOW)
#define bomba_GLP_ON digitalWrite(pin_GLP, LOW)
#define bomba_GLP_OFF digitalWrite(pin_GLP, HIGH)
#define valor_max_byte 0xFA
#define valor_max_word 0xFAFF
#define valor_max_dword 0xFAFFFFFF
#define numero_mensajes 10
#define led_derecho 14

SemaphoreHandle_t xSemaphore = NULL;

struct mystruct
{
        uint16_t PGN;
        uint16_t periodo;
        uint8_t origen;
        uint8_t destino;
        uint8_t priority;
        uint8_t msg[8];
} mensaje_can[numero_mensajes];

struct AMessage {
        byte nPriority;
        byte nSrcAddr;
        byte nDestAddr;
        byte nData[8];
        int nDataLen;
        long lPGN;
        long time;
        uint32_t cuenta = 0;
} xMessage, *pxMessage, *pxRxedMessage;

struct trama_can {
        //TRF1
        uint16_t temperatura_transmision;
        //EEC2
        uint16_t Accelerator_Pedal_Position_1;
        uint16_t Engine_Percent_Load_At_Current_Speed;
        float Actual_Maximum_Available_Engine_torque;
        //IC1
        uint16_t Engine_Intake_Manifold_1_Pressure;
        uint16_t Engine_Intake_Manifold_1_Temperature;
        //EEC1
        uint16_t pressure_measurement_air_intake_manifold;
        uint16_t Drivers_Demand_Engine_requested;
        uint16_t actual_engine_calculated_torque;
        uint16_t engine_demand_requested_torque;
        float engine_speed_rpm;
        float engine_speed_rpm2;
        //VEP1
        float Vehicle_Electrical_Power;
        //ET1
        uint16_t Engine_Coolant_Temperature;
        //EFL_P2
        uint16_t Fan_Drive_State;
        uint16_t Estimated_Percent_Fan_Speed;
        //AMB
        uint16_t Barometric_Pressure_Absolute;
        //DM01
        //EFL_P1
        uint16_t Engine_Fuel_Delivery_Pressure;
        uint16_t Engine_Oil_Level;
        uint16_t Engine_Oil_Pressure;
        uint16_t Engine_Coolant_Level;
        //CCSS
        uint16_t Maximum_Vehicle_SpeedLimit;
        uint16_t Cruise_Control_LowSet_LimitSpeed;
        uint16_t Cruise_Control_HighSet_LimitSpeed;
        //DM01
        bool ProtectWarninglamp;
        bool AmberWarninglamp;
        bool RedWarninglamp;
        bool MalfunctionWarninglamp;
        bool FlashProtectWarninglamp;
        bool FlashAmberWarninglamp;
        bool FlashRedWarninglamp;
        bool FlashMalfunctionWarninglamp;
        //Fuel
        uint16_t Fuel_Level;
} can_mega2560;

//Asignaciones
QueueHandle_t xQueue1;
MedianFilter<int> medianFilter_1(20);

//Variables
TaskHandle_t xHandle_1, xHandle_2, xHandle_4, xHandle[numero_mensajes];  // colas CAN
static int dato1, dato2, dato3, A, B; // recibido I2C
int eeAddress;
int inicio = 27;
int match;
unsigned long tiempo_acumulado, tiempo_total;

void a_can16(uint16_t trama, byte n_byte, byte mensaje)

{
        mensaje_can[mensaje].msg[n_byte - 1] = (uint8_t)(trama & 0x00FF);
        mensaje_can[mensaje].msg[n_byte] = (uint8_t)(trama >> 8);
}

void a_can8(uint16_t trama, byte n_byte, byte mensaje)

{
        mensaje_can[mensaje].msg[n_byte - 1] = trama;
}

void receiveEvent() {

        static bool largo;
        digitalWrite(led_derecho, HIGH);

        if (Wire.available() == 3)
        {
                largo = true;
                dato1 = Wire.read();
        }

        if (Wire.available() == 2)
        {
                if (largo) dato2 = Wire.read(); else dato1 = Wire.read();
        }

        if (largo) dato3 = Wire.read(); else { dato2 = Wire.read(); dato3 = 0xFF; }
        largo = false;
}

void mostrar_LCD_derecho(int i, int p)
{
        lc.setDigit(0, 0, i % 10, p == 0);
        lc.setDigit(0, 1, (i / 10) % 10, p == 1);
        lc.setDigit(0, 2, (i / 100) % 10, p == 2);
        lc.setDigit(0, 3, (i / 1000) % 10, p == 3);
}

void mostrar_LCD_izquierdo(int i, int p)
{
        lc.setDigit(0, 4, i % 10, p == 0);
        lc.setDigit(0, 5, (i / 10) % 10, p == 1);
        lc.setDigit(0, 6, (i / 100) % 10, p == 2);
        lc.setDigit(0, 7, (i / 1000) % 10, p == 3);
}

/*********************************************************************************
**  setup()
**
**  Initialize the Arduino and setup serial communication.
**
**  Input:  None
**  Output: None
*********************************************************************************/
void setup()
{
        analogReference(INTERNAL2V56);

        //EEPROM
        EEPROM.get(eeAddress, match);

        if (match != inicio) // primera vez
        {
                EEPROM.put(eeAddress, inicio);
                eeAddress += sizeof(int);
                EEPROM.put(eeAddress, (unsigned long)0);
        }
        else
        {
                eeAddress += sizeof(int);
                EEPROM.get(eeAddress, tiempo_acumulado);
        }

        //LCD
        lc.shutdown(0, false);
        lc.setIntensity(0, 8);
        lc.clearDisplay(0);

        // Unimos este dispositivo al bus I2C con dirección 1
        Wire.begin(1);
        // Registramos el evento al recibir datos
        Wire.onReceive(receiveEvent);

        // Tramas autogeneradas motor GLP
        mensaje_can[0] = { 0XF003,50,0,0xff,3,{ 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF } }; //EEC2 61443 spn 91 92 3357
        /*61443    Electronic Engine Controller 2        8       50 ms        EEC2 2       8       91     Accelerator Pedal Position 1
        "The ratio of actual position of the analog engine speed/torque request input device (such as an accelerator pedal or throttle lever) to the maximum position of the input device.  This parameter is intended for the primary accelerator control in an application.  If an application has only one accelerator control, use SPN 91.
        For on-highway vehicles, this will typically be the operator’s accelerator pedal.  Although it is used as an input to determine powertrain demand, it also provides anticipatory information to transmission and ASR algorithms about driver actions.
        In marine applications, this will typically be the operator’s throttle lever. If a low idle validation switch is used in conjunction with accelerator pedal position 1, use Accelerator Pedal Low Idle Switch 1, SPN 558."
        0 to 100 %         0.4 %/bit   0       %     01-oct-98   Published        14/11/2002        Published   J1939-71   J1939-71

        61443        Electronic Engine Controller 2        8       50 ms        EEC2 3       8       92     Engine Percent Load At Current Speed The ratio of actual engine percent torque (indicated) to maximum indicated torque available at the current engine speed, clipped to zero torque during engine braking.
        0 to 250 % 0 to 125% 1 %/bit      0       %     01-oct-98        Published   30/11/2003        Published   J1939-71   J1939-71

        61443        Electronic Engine Controller 2        8       50 ms        EEC2 7       8       3357 Actual Maximum Available Engine - Percent Torque
        This is the maximum amount of torque that the engine can immediately deliver as a percentage of the reference engine torque (SPN 544).
        The Actual Maximum Available Engine - Percent Torque shall take into consideration all engine torque derates (e.g. air fuel ratio control (AFC),
        noise control, etc.) that could potentially be active in the system.  This parameter differentiates itself from the engine percent torque points
        1 through 5 of the engine configuration map because it takes into account all dynamic internal inputs such as  AFC and that it is updated on a 50ms
        basis. 0 to 100 %         0.4 %/bit   0       %     11-nov-04        Published   19/08/2004        Published   J1939-71   J1939-71
        */
        mensaje_can[1] = { 0XFEF6,500,0,0xff,6,{ 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF } }; //IC1 65270 spn 102 105

        /*65270    Inlet/Exhaust Conditions 1    8       0.5 s IC1   2        8       102   Engine Intake Manifold #1 Pressure       The gage pressure measurement of the air intake manifold.
        If there are multiple air pressure sensors in the intake stream, this is the last one in flow direction before entering the combustion chamber.
        This should be the pressure used to drive gauges and displays. See also SPNs 1127-1130 and SPN 3562  for alternate range and resolution.
        If there is only one pressure measurement of the air intake manifold to report and this range and resolution is adequate, this parameter should be used.
        0 to 500 kPa               2 kPa/bit    0       kPa   19-may-05        Published   19/05/2005        Published   J1939-71   J1939-71

        65270        Inlet/Exhaust Conditions 1    8       0.5 s IC1   3        8       105   Engine Intake Manifold 1 Temperature        Temperature of pre-combustion air found in intake manifold of engine air supply system.
        -40 to 210 deg C         1 deg C/bit -40   C      01-oct-98        Published   30/11/2003        Published   J1939-71   J1939-71*/

        mensaje_can[2] = { 0XF004,50,0,0xff,3,{ 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF } }; //EEC1 61444 spn 102 512 513 2432 190
        /*65270    Inlet/Exhaust Conditions  1    8       0.5 s IC1   2        8       102   Engine Intake Manifold #1 Pressure       The gage pressure measurement of the air intake
        manifold. If there are multiple air pressure sensors in the intake stream, this is the last one in flow direction before entering the combustion
        chamber.  This should be the pressure used to drive gauges and displays. See also SPNs 1127-1130 and SPN 3562  for alternate range and resolution.
        If there is only one pressure measurement of the air intake manifold to report and this range and resolution is adequate, this parameter should be
        used. 0 to 500 kPa               2 kPa/bit    0       kPa   19-may-05        Published   19/05/2005        Published   J1939-71   J1939-71

        61444        Electronic Engine Controller 1        8       engine speed dependent EEC1 2       8       512   Driver's Demand Engine - Percent Torque   The requested
        torque output of the engine by the driver. It is based on input from the following requestors external to the powertrain: operator (via the
        accelerator pedal), cruise control and/or road speed limit governor. Dynamic commands from internal powertrain functions such as smoke control,
        low- and high-speed engine governing; ASR and shift control are excluded from this calculation. The data is transmitted in indicated torque as a
        percent of the reference engine torque. See PGN 65251 for the engine configuration message. Several status bits are defined separately to indicate
        the request which is currently being honored. This parameter may be used for shift scheduling.
        -125 to 125 %    0 to 125% 1 %/bit      -125  %     01-oct-98        Published            Published   J1939-71   J1939-71

        61444        Electronic Engine Controller 1        8       engine speed dependent EEC1 3       8       513   Actual Engine - Percent Torque       The calculated output
        torque of the engine.  The data is transmitted in indicated torque as a percent of reference engine torque (see the engine configuration message,
        PGN 65251).  The engine percent torque value will not be less than zero and it includes the torque developed in the cylinders required to overcome
        friction.      -125 to 125 %    0 to 125% 1 %/bit      -125  %        01-oct-98   Published            Published   J1939-71   J1939-71

        61444        Electronic Engine Controller 1        8       engine speed dependent EEC1 8       8       2432 Engine Demand – Percent Torque       The requested torque
        output of the engine by all dynamic internal inputs, including smoke control, noise control and low and high speed governing.
        -125 to 125 %    -125% to +125%        1 %/bit      -125  %        09-nov-00  Published   09/11/2000        Published   J1939-71        J1939-71

        61444        Electronic Engine Controller 1        8       engine speed dependent EEC1 4-5   16     190   Engine Speed     Actual engine speed which is calculated
        over a minimum crankshaft angle of 720 degrees divided by the number of cylinders.
        0 to 8,031.875 rpm              0.125 rpm/bit     0       rpm   01-oct-98        Published            Published   J1939-71   J1939-71*/

        mensaje_can[3] = { 0XFEF7,1000,0,0xff,6,{ 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF } }; //VEP1 65271  168
        /*65271    Vehicle Electrical Power 1     8       1 s    VEP1 5-6        16     168   Battery Potential / Power Input 1
        This parameter measures the first source of battery potential as measured at the input of the ECM/actuator etc. coming from one or more batteries,
        irrespective of the distance between the component and the battery.  This SPN is also used when ECM's are interconnected in a series configuration,
        where the source of power is coming directly or indirectly from the same battery/batteries.
        0 to 3212.75 V            0.05 V/bit  0       Volts 19-may-05        Published   19/05/2005        Published   J1939-71   J1939-71*/

        mensaje_can[4] = { 0XFEEE,1000,0,0xff,6,{ 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF } }; //ET1 65262 spn 110
        /*65262    Engine Temperature 1  8       1 s    ET1   1       8        110   Engine Coolant Temperature Temperature of liquid found in engine cooling system.
        -40 to 210 deg C         1 deg C/bit -40   C      01-oct-98        Published   30/11/2003        Published   J1939-71   J1939-71*/

        mensaje_can[5] = { 0XFEBD,100,0,0xff,6,{ 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF } }; //FD1
        /*65213    Fan Drive   8       1 s    FD     2.1    4       977        Fan Drive State   "This parameter is used to indicate the current state or mode of operation by the fan drive.

        0000 Fan off
        0001 Engine system–General
        0010 Excessive engine air temperature
        0011 Excessive engine oil temperature
        0100 Excessive engine coolant temperature
        0101 Excessive transmission oil temperature
        0110 Excessive hydraulic oil temperature
        0111 Default Operation
        1000 Not defined
        1001 Manual control
        1010 Transmission retarder
        1011 A/C system
        1100 Timer
        1101 Engine brake
        1110 Other
        1111 Not available

        Fan off  0000b —Used to indicate that the fan clutch is disengaged and the fan is inactive
        Engine system–General  0001b —Used to indicate that the fan is active due to an engine system not otherwise defined.
        Excessive engine air temperature  0010b —Used to indicate that the fan is active due to high air temperature.
        Excessive engine oil temperature  0011b —Used to indicate that the fan is active due to high oil temperature.
        Excessive engine coolant temperature  0100b —Used to indicate that the fan is active due to high coolant temperature.
        Manual control  1001b —Used to indicate that the fan is active as requested by the operator.
        Transmission retarder  1010b —Used to indicate that the fan is active as required by the transmission retarder.
        A/C system  1011b —Used to indicate that the fan is active as required by the air conditioning system.
        Timer  1100b —Used to indicate that the fan is active as required by a timing function.
        Engine brake  1101b —Used to indicate that the fan is active as required to assist engine braking.
        Excessive transmission oil temperature - 0101b  - Used to indicate fan is active due to excessive transmission oil temperature.
        Excessive hydraulic oil temperature - 0110b - Used to indicate fan is active due to excessive hydraulic oil temperature.
        Default Operation - 0111b - Used to indicate fan is active due to a error condition resulting in default operation"
        0 to 15               16 states/4 bit    0       bit     01-oct-98        Published   08/02/1999        Published   J1939-71   J1939-71

        65213        Fan Drive   8       1 s    FD     1       8       975        Estimated Percent Fan Speed        "Estimated fan speed as a ratio of the fan drive (current speed) to the fully engaged fan drive (maximum fan speed).
        A two state fan (off/on) will use 0% and 100% respectively.  A three state fan (off/intermediate/on) will use 0%, 50% and 100% respectively.  A variable speed fan will use 0% to 100%.
        Multiple fan systems will use 0 to 100% to indicate the percent cooling capacity being provided.
        Note that the intermediate fan speed of a three state fan will vary with different fan drives, therefore 50% is being used to indicate that the intermediate speed is required from the
        fan drive."  0 to 100 %         0.4 %/bit   0       %     01-oct-98        Published   29/07/1999        Published   J1939-71        J1939-71*/

        mensaje_can[6] = { 0XFEF5,100,0,0xff,6,{ 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF } }; //AMB 65269 spn 108

        /*65269    Ambient Conditions      8       1 s    AMB  1       8        108   Barometric Pressure     Absolute air pressure of the atmosphere.  See Figures SPN16_A & SPN16_B.
        0 to 125 kPa               0.5 kPa/bit 0       kPa   01-oct-98        Published            Published   J1939-71   J1939-71*/

        //mensaje_can[7] = { 0XFECA,100,0,0xff,6,{ 0x13, 0xFF, 0xB8, 0x04, 0x03, 0x01, 0xFF, 0xFF } }; //DM01 65226 spn

        mensaje_can[7] = { 0XFECA,100,0,0xff,6,{ 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF } };   // DM01 65226 spn

        mensaje_can[8] = { 0XFEEF,100,0,0xff,6,{ 0xff, 0xff, 0xff, 0xFF, 0xff, 0xff, 0xff, 0xff } }; //EFL_P1 65263 spn 94 98 100 111

        /*65263    Engine Fluid Level/Pressure 1        8       0.5 s        EFL/P1       1       8       94     Engine Fuel Delivery Pressure        Gage pressure of fuel in system as delivered from supply pump to the injection pump.
        See Figures SPN16_A & SPN16_B. 0 to 1000 kPa  4 kPa/bit       0       kPa

        65263        Engine Fluid Level 1        8       0.5 s        EFL/P1       3       8       98     Engine Oil Level  Ratio of current volume of engine sump oil to maximum required volume.
        0 to 100 %         0.4 %/bit   0       %     01-oct-98   Published           Published   J1939-71   J1939-71

        65263        Engine Fluid Pressure 1        8       0.5 s        EFL/P1       4       8       100   Engine Oil Pressure  Gage pressure of oil in engine lubrication system as provided by oil pump.
        0 to 1000 kPa              4 kPa/bit    0       kPa

        65263        Engine Coolant Level 1        8       0.5 s        EFL/P1       8       8       111   Engine Coolant Level   Ratio of volume of liquid found in engine cooling system to total cooling system volume.
        Typical monitoring location is in the coolant expansion tank.        0 to 100 %         0.4 %/bit   0       %     09-nov-00  Published        30/11/2003        Published   J1939-71   J1939-71 */

        mensaje_can[9] = { 0XFEED,100,0,0xff,6,{ 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF } }; //CCSS 65261

        /*MaximumVehicleSpeedLimit 74
        CruiseControlLowSetLimitSpeed 88
        CruiseControlHighSetLimitSpeed 87

        65261      Maximum Vehicle Speed Limit         8       On request      CCSS 1       8       74            Maximum vehicle velocity allowed.
        0 to 250 km/h             1 km/h per bit    0       kph   01-oct-98        Published            Published   J1939-71   J1939-71

        65261        Cruise Control/Vehicle Speed Setup        8       On request      CCSS 3       8       88     Cruise Control Low Set Limit Speed        Minimum vehicle velocity at which cruise can be set or minimum
        vehicle velocity for cruise operation before it will exit cruise control operation.
        0 to 250 km/h             1 km/h per bit    0       kph   01-oct-98        Published            Published   J1939-71   J1939-71

        65261        Cruise Control/Vehicle Speed Setup        8       On request      CCSS 2       8       87     Cruise Control High Set Limit Speed        Maximum vehicle velocity at which cruise can be set.
        0 to 250 km/h             1 km/h per bit    0       kph   01-oct-98        Published            Published   J1939-71   J1939-71   */

        //            mensaje_can[10] = { 0XFEFC,1000,0x12,0xff,6,{ 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF } }; //Can Fuel

        pinMode(LED_Rojo, OUTPUT);
        pinMode(LED_Verde, OUTPUT);
        pinMode(LED_Azul, OUTPUT);
        pinMode(led_derecho, OUTPUT);
        pinMode(pin_GLP, OUTPUT);
        pinMode(PWM, OUTPUT);
        bomba_GLP_ON;

        for (byte i = 0; i < numero_mensajes; i++)
                if (xTaskCreate(vPeriodicTask3, "Tramas_CAN", mem_tramas_can, &mensaje_can[i], tskIDLE_PRIORITY + 3, &xHandle[i]) == pdPASS)
                {
                        Serial.print(" ");
                        Serial.print(i);
                }

        Serial.begin(115200);

        BaseType_t xReturned;
        pinMode(Int_Can, INPUT);

        // Creamos la cola
        xQueue1 = xQueueCreate(10, sizeof(struct AMessage *));
        if (xQueue1 == NULL) Serial.println("Queue Fail !"); else Serial.println("Queue Created OK.");

        canInitialize(10); // 11=250k @8MHz 10=250K @16MHz

        // Start the scheduler so our tasks start executing
        xSemaphore = xSemaphoreCreateMutex();

        if (xTaskCreate(vPeriodicTask1, "CAN_RX", 300, NULL, tskIDLE_PRIORITY + 1, &xHandle_1) == pdPASS) { NULL; }
        if (xTaskCreate(vPeriodicTask2, "Alive", 300, NULL, tskIDLE_PRIORITY + 1, &xHandle_2) == pdPASS) { NULL; }
        if (xTaskCreate(vPeriodicTask4, "LCD", 300, NULL, tskIDLE_PRIORITY + 1, &xHandle_4) == pdPASS) { NULL; }

        // Place your custom setup code here
        attachInterrupt(digitalPinToInterrupt(Int_Can), recibir_can, FALLING);   //Can

        //tiempo de motor
       // tiempo_motor();

        vTaskStartScheduler();
        for (;;);
}

uint16_t data(uint8_t byte_inicio, uint8_t bit_inicio, uint8_t n_bits = 8)
{
        uint16_t i;

        if (n_bits == 16)
        {
                i = pxRxedMessage->nData[byte_inicio] + 256 * pxRxedMessage->nData[byte_inicio + 1];
        }
        else
                if (n_bits == 8)
                {
                        i = pxRxedMessage->nData[byte_inicio];
                }
        return i;
}

// Tarea 1
void vPeriodicTask1(void *pvParameters)
{
        static char sString[80];

        TickType_t xLastWakeTime = xTaskGetTickCount();

        for (;;)
        {
                // Lectura cola CAN RX
                if (xQueue1 != 0)
                {
                        // Receive a message on the created queue.  Block for 10 ticks if a message is not immediately available
                        if (xQueueReceive(xQueue1, &(pxRxedMessage), (TickType_t)10))
                        {
                                long lMessageID2 = ((long)pxRxedMessage->nPriority << 26) + ((long)pxRxedMessage->lPGN << 8) + (long)pxRxedMessage->nSrcAddr;
                                switch (lMessageID2)
                                {

                                case 0x18FEF803: {

                                       can_mega2560.temperatura_transmision = (uint16_t)(0.03125* (pxRxedMessage->nData[4] + 256 * pxRxedMessage->nData[5]) - 273);

                                       break;       }

                                case 0x0CF00203: {
                                       /*
                                       can_mega2560.engine_speed_rpm = .125*(pxRxedMessage->nData[5] + 256 * pxRxedMessage->nData[6]);

                                       if (can_mega2560.engine_speed_rpm > 10) {   // led rojo panel encendido si RPM>10
                                       led_rojo_ON;
                                       can_mega2560.engine_speed_rpm2 = can_mega2560.engine_speed_rpm;
                                       }
                                       else {
                                       led_rojo_OFF;
                                       can_mega2560.engine_speed_rpm2 = 8* (100*map(analogRead(A14), 0, 1023, 0, 5000) / 100 - 400); // 0.125rpm/bit 0CF00400 A0
                                       }
                                       */
                                       break;       }

                                default:
                                {
                                       break;
                                }

                                }
                                /*
                                ltoa(lMessageID2, sString, 16);
                                strupr(sString);
                                Serial.print("ID:");
                                Serial.print(sString);
                                Serial.print(" ");
                                sprintf(sString, "PGN:%05u P:%X S:%X D:%X ", (int)pxRxedMessage->lPGN, pxRxedMessage->nPriority, pxRxedMessage->nSrcAddr, pxRxedMessage->nDestAddr);
                                Serial.print(sString);
                                Serial.print("Data:");

                                for (int nIndex = 0; nIndex < pxRxedMessage->nDataLen; nIndex++)
                                {
                                sprintf(sString, "%X ", pxRxedMessage->nData[nIndex]);
                                Serial.print(sString);
                                } Serial.println("");

                                Serial.print(dato1, HEX);
                                Serial.print(" ");
                                Serial.print(dato2, HEX);
                                Serial.print(" ");
                                Serial.println(dato3, HEX);
                                */
                                vTaskDelayUntil(&xLastWakeTime, (50 / portTICK_PERIOD_MS));
                        }
                }//if xQueue1
        }
}

// Tarea 2 Alive
void vPeriodicTask2(void *pvParameters)
{
        TickType_t xLastWakeTime = xTaskGetTickCount();
        static uint16_t dato_largo;
        int psi;
        static enum posibles_estados_FSM {
                inicio,
                parado,
                en_marcha
        }
        estado = inicio;

        can_mega2560.Maximum_Vehicle_SpeedLimit = 69; // fija 1 km/h per bit
        can_mega2560.Engine_Coolant_Level = 250; //  0 to 100 %  0.4 % / bit 250=lleno
        can_mega2560.Engine_Oil_Level = 250;    //   0 to 100 %  0.4 %/bit   250=lleno
        can_mega2560.Fuel_Level = 250 / 4;      //   0 to 100 %  0.4 %/bit   250=lleno

        for (;;)
        {
                psi = 32.106*map(analogRead(A15), 0, 1023, 0, 5000) / 1000 - 16.569;
                //psi = 72; // fake !!!
                if (psi >= 0) can_mega2560.Engine_Oil_Pressure = 6.89475728 * psi / 4; else can_mega2560.Engine_Oil_Pressure = 0xff; // 1 PSI = 6,89475728 kPa

                A = dato2;
                B = dato3;

                if (dato3 != 0xFF) // es largo, 16bits
                {
                        switch (dato1)
                        {
                        case 0x0C: {

                                can_mega2560.engine_speed_rpm = 2 * (256 * A + B); // 0.125rpm/bit 0CF00400 A0

                                if (can_mega2560.engine_speed_rpm > 10) {
                                       led_rojo_ON;
                                       can_mega2560.engine_speed_rpm2 = can_mega2560.engine_speed_rpm;
                                }
                                else {
                                       led_rojo_OFF; // led rojo panel encendido si RPM>10
                                       can_mega2560.engine_speed_rpm2 = 8 * (map(medianFilter_1.GetFiltered(), 0, 1023, 0, 2560) - 400); // 0.125rpm/bit 0CF00400 A0

                                }
                                break; }
                        case 0x42: {
                                can_mega2560.Vehicle_Electrical_Power = (256 * A + B) / 50; // 0.05 volts/bit 18FEF700 A1  (1/1000)*(1/0.05)
                                break; }
                        default:
                                break;
                        }
                }
                else  //dato corto
                {
                        switch (dato1)
                        {
                        case 0x05: {
                                can_mega2560.Engine_Coolant_Temperature = A; //18feee00 A2
                                break; }
                        case 0x49: {
                                can_mega2560.Accelerator_Pedal_Position_1 = 1.572327*(A - 56); //A3 0CF00300  (1/.4)*(A-56)*100/159;
                                break; }
                        case 0x04: {
                                can_mega2560.Engine_Percent_Load_At_Current_Speed = A / 2.55; //A4v
                                break; }
                        case 0x0E: {
                                can_mega2560.Engine_Intake_Manifold_1_Temperature = A - 40 - 30; //  1 deg C/bit -40C 0XFEF6   A - 40 - 30;
                                break; }
                        case 0x0B: {
                                can_mega2560.Engine_Intake_Manifold_1_Pressure = 2 * A; //  2 kPa/bit 0XFEF6
                                break; }
                        case 0x33: {
                                can_mega2560.Barometric_Pressure_Absolute = A / 2; // 0.5 kPa/bit  0XFEF5
                                break; }
                        default:
                                break;
                        }
                }

                digitalWrite(led_derecho, LOW);

                // envío can
                a_can16(can_mega2560.engine_speed_rpm2, 4, 2); //dato,byte,mensaje_can[x]
                a_can16(can_mega2560.Vehicle_Electrical_Power, 5, 3);
                //EEC2
                a_can8(can_mega2560.Accelerator_Pedal_Position_1, 2, 0);
                a_can8(can_mega2560.Engine_Percent_Load_At_Current_Speed, 3, 0);
                a_can16(can_mega2560.Actual_Maximum_Available_Engine_torque, 7, 0);
                //IC2
                a_can8(can_mega2560.Engine_Intake_Manifold_1_Pressure, 2, 1);
                a_can8(can_mega2560.Engine_Intake_Manifold_1_Temperature, 3, 1);
                //ET1
                a_can8(can_mega2560.Engine_Coolant_Temperature, 1, 4);
                //AMB
                a_can8(can_mega2560.Barometric_Pressure_Absolute, 1, 6);
                //EFL_P1
                a_can8(can_mega2560.Engine_Fuel_Delivery_Pressure, 1, 8);
                a_can8(can_mega2560.Engine_Oil_Level, 3, 8);
                a_can8(can_mega2560.Engine_Oil_Pressure, 4, 8);
                a_can8(can_mega2560.Engine_Coolant_Level, 8, 8);
                //CCSS
                a_can8(can_mega2560.Maximum_Vehicle_SpeedLimit, 1, 9);
                a_can8(can_mega2560.Cruise_Control_LowSet_LimitSpeed, 3, 9);
                a_can8(can_mega2560.Cruise_Control_HighSet_LimitSpeed, 2, 9);
                //EEC1
                a_can8(can_mega2560.Drivers_Demand_Engine_requested, 2, 2);
                a_can8(can_mega2560.actual_engine_calculated_torque, 3, 2);
                a_can8(can_mega2560.engine_demand_requested_torque, 8, 2);
                //EFL_P2
                a_can8(can_mega2560.Fan_Drive_State, 2, 5);
                a_can8(can_mega2560.Estimated_Percent_Fan_Speed, 1, 5);
                //Fuel level
                a_can8(can_mega2560.Fuel_Level, 2, 10);

                //5to ventilador
                if (can_mega2560.temperatura_transmision >= 70) analogWrite(PWM, constrain((can_mega2560.temperatura_transmision*1.2 - 44) * 2.55, 1, 255)); // valor en %
                else analogWrite(PWM, 1);

                //analogWrite(PWM, 250);

                // FSM control bomba GLP
                switch (estado)
                {
                case inicio:
                        if (.125*can_mega2560.engine_speed_rpm <= 200) estado = parado; else estado = en_marcha;
                        break;
                case parado:
                        if ((can_mega2560.Engine_Coolant_Temperature - 40) > 50)  bomba_GLP_OFF;
                        estado = inicio;
                        break;
                case en_marcha:
                        bomba_GLP_ON;
                        estado = inicio;
                        break;

                default:

                        break;
                }

                medianFilter_1.AddValue(analogRead(A14));

                vTaskDelayUntil(&xLastWakeTime, (20 / portTICK_PERIOD_MS));
        }
}

void vPeriodicTask3(struct mystruct* can_tx) // Creación tramas CAN
{
        TickType_t xLastWakeTime = xTaskGetTickCount();

        /* As per most tasks, this task is implemented in an infinite loop. */
        for (;; )
        {
                // Ponderación del periodo
                if (can_tx->periodo == 50) can_tx->periodo = 49;
                else if (can_tx->periodo == 100) can_tx->periodo = 98;
                else if (can_tx->periodo == 200) can_tx->periodo = 196;
                else if (can_tx->periodo == 250) can_tx->periodo = 244;
                else if (can_tx->periodo == 500) can_tx->periodo = 489;
                else if (can_tx->periodo == 1000) can_tx->periodo = 977;
                else if (can_tx->periodo == 5000) can_tx->periodo = 4883;
                else if (can_tx->periodo == 10000) can_tx->periodo = 9766;

                if (xSemaphoreTake(xSemaphore, (TickType_t)10) == pdTRUE)
                {
                        noInterrupts();
                        j1939Transmit(can_tx->PGN, can_tx->priority, can_tx->origen, can_tx->destino, can_tx->msg, 8);  // Transmit the message. 0xFF -> destino
                        interrupts();
                        xSemaphoreGive(xSemaphore);
                }

                /* We want this task to execute exactly every period milliseconds. */
                vTaskDelayUntil(&xLastWakeTime, (can_tx->periodo/ portTICK_PERIOD_MS));
        }
}

//LCD
void vPeriodicTask4(void *pvParameters)
{
        TickType_t xLastWakeTime = xTaskGetTickCount();
        static unsigned long time;
        static  int i,key=0;

        for (;;)
        {
                //can_mega2560.engine_speed_rpm = 100;

                if ((can_mega2560.engine_speed_rpm)>10) { // con revoluciones

                        mostrar_LCD_izquierdo(.125*can_mega2560.engine_speed_rpm, 0);
                        mostrar_LCD_derecho(can_mega2560.Accelerator_Pedal_Position_1, 0);
                        delay(3000);

                        mostrar_LCD_izquierdo(can_mega2560.Engine_Coolant_Temperature - 40, 0);
                        mostrar_LCD_derecho(can_mega2560.temperatura_transmision, 0); // temperatura cambio
                        delay(3000);

                        // Horometro
                        while (i++ < 6) {

                                time = millis() - tiempo_total;
                                mostrar_LCD_derecho((int)(((time / 1000) % 60) + 100 * ((time / 60000) % 60)), 2);
                                mostrar_LCD_izquierdo((int)((time / 3600000) % 24) + 100 * (time / 86400000), 2);
                                delay(1000);

                        } i = 0;

                        EEPROM.put(eeAddress, (unsigned long)time + tiempo_acumulado);

                }
                else tiempo_total = millis();

                if (key) mostrar_LCD_izquierdo(.5*can_mega2560.Vehicle_Electrical_Power, 1); else mostrar_LCD_izquierdo(10*(can_mega2560.Engine_Coolant_Temperature - 40), 1);
                key = !key;
                //mostrar_LCD_derecho(.1*(map(medianFilter_1.GetFiltered(), 0, 1023, 0, 2560) - 400), 1); // sensor presión 100ohms // 1 Decimal
                mostrar_LCD_derecho((map(medianFilter_1.GetFiltered(), 0, 1023, 0, 2560) - 400), 2); // sensor presión 100ohms
                delay((can_mega2560.engine_speed_rpm > 10) * 2500 + 500);

                vTaskDelayUntil(&xLastWakeTime, (500 / portTICK_PERIOD_MS));
        }
}


void tiempo_motor() {
        int i;
        while (i++ < 20) {
                mostrar_LCD_derecho((int)(((tiempo_acumulado / 1000) % 60) + 100 * ((tiempo_acumulado / 60000) % 60)), 2);
                mostrar_LCD_izquierdo((int)((tiempo_acumulado / 3600000) % 24) + 100 * (tiempo_acumulado / 86400000), 2);
                delay(500);
        }
}

void recibir_can() {

        if (xQueue1 != NULL)
        {
                j1939Receive(&xMessage.lPGN, &xMessage.nPriority, &xMessage.nSrcAddr, &xMessage.nDestAddr, xMessage.nData, &xMessage.nDataLen);
                pxMessage = &xMessage;
                xQueueSend(xQueue1, (void *)&pxMessage, (TickType_t)0);
        }
}

void loop() { }


/*
const int analogInPin = A15;  // Analog input pin that the potentiometer is attached to

int psi;        // value read from the pot
int kPa;        // value output to the PWM (analog out)

void setup() {
// initialize serial communications at 9600 bps:
Serial.begin(9600);
}

void loop() {

psi = 32.106*map(analogRead(A15), 0, 1023, 0, 5000)/1000-16.569;

kPa =  6.89475728 *psi;

// print the results to the serial monitor:
Serial.print("sensor (kPa) = ");
Serial.println(kPa);

delay(250);
}*/



/*
//***************************************************
// Sirocco J1979
// v0.3
//
//***************************************************
#include <SPI.h>
#include "mcp_can.h"
#include "OLedI2C.h"
OLedI2C LCD;

typedef struct
{
byte rpm[5] = {0x04, 0x41, 0x0C, 0xff, 0xff}; //usado
byte electrical_power[5] = {0x04, 0x41, 0x42, 0xff, 0xff};//usado
byte Engine_Coolant_Temperature[4] = {0x03, 0x41, 0x05, 0xff}; //usado
byte pedalD[4] = {0x03, 0x41, 0x49, 0xFF};
byte Engine_Percent_Load_At_Current_Speed[4] = {0x03, 0x41, 0x04, 0xff};
byte Engine_Intake_Manifold_1_Temperature[4] = {0x03, 0x41, 0x0E, 0xff};
byte Engine_Intake_Manifold_1_Pressure[4] = {0x03, 0x41, 0x0B, 0xff};
byte Barometric_Pressure_Absolute[4] = {0x03, 0x41, 0x33, 0xff};
} pidstruct;

//Inicializa en CAN
char data[8];
MCP_CAN CAN(10);

//j1979
uint16_t id = 0x7e8;
INT8U ext;
INT8U len;
pidstruct pids;

void setup()
{
Serial.begin(115200);
Wire.begin();
LCD.init();
//Begin Can bus
while (CAN_OK != CAN.begin(CAN_500KBPS))              // init can bus : baudrate = 500k/250k/...
{
Serial.println("CAN BUS Shield init fail");
}
Serial.println("CAN BUS Shield init ok!");
}

void sendValues(char pid)
{
Serial.print("Mandando pid:");Serial.println(pid);
LCD.sendString("RPM: ", 1, 0); // columna, fila

switch (pid){
case 0x0C:
{
uint16_t rpm  = map(analogRead(A0), 0, 1023, 0, 65535);
pids.rpm[3] = highByte(rpm);
pids.rpm[4] =lowByte(rpm);
CAN.sendMsgBuf(id, ext, 5, pids.rpm);
Serial.print("RPM:");Serial.println(rpm);
LCD.sendFloat(rpm, 5, 0, 6, 0);
break;
}

case 0x42:
{    uint16_t electrical_power  = map(analogRead(A1), 0, 1023, 0, 65535);
pids.electrical_power[3] = highByte(electrical_power);
pids.electrical_power[4] =lowByte(electrical_power);
CAN.sendMsgBuf(id, ext, 5, pids.electrical_power);
Serial.print("Electrical_power");Serial.println(pids.electrical_power[3]);
LCD.sendString("VOL: ", 1, 1); // columna, fila
LCD.sendFloat(electrical_power, 5, 0, 6, 1);
break;
}
case 0x05:
{
pids.Engine_Coolant_Temperature[3] = map(analogRead(A2), 0, 1023, 0, 255);
CAN.sendMsgBuf(id, ext, 4, pids.Engine_Coolant_Temperature);
Serial.print(" Engine Coolant Temperature:"); Serial.println(pids.Engine_Coolant_Temperature[3] - 40);
LCD.sendFloat(pids.Engine_Coolant_Temperature[3], 5, 0, 10, 1);
break;
}
case 0x49:
{
pids.pedalD[3] = map(analogRead(A3), 0, 1023, 0, 255);
CAN.sendMsgBuf(id, ext, 4, pids.pedalD);
Serial.print("PEDAL:");Serial.println(pids.pedalD[3]/2.55);
break;
}

case 0x04:
{
pids.Engine_Percent_Load_At_Current_Speed[3] = map(analogRead(A4), 0, 1023, 0, 255);
CAN.sendMsgBuf(id, ext, 4, pids.Engine_Percent_Load_At_Current_Speed);
//       Serial.print("Engine_Percent_Load_At_Current_Speed:");Serial.println( pids.Engine_Percent_Load_At_Current_Speed);
break;
case 0x0E: //Engine_Intake_Manifold_1_Temperature
{
pids.pedalD[3] = map(analogRead(A5), 0, 1023, 0, 255);
CAN.sendMsgBuf(id, ext, 4, pids.Engine_Intake_Manifold_1_Temperature);
break;
}
}
case 0x0B: //Engine_Intake_Manifold_1_Pressure
{
pids.pedalD[3] = map(analogRead(A5), 0, 1023, 0, 255);
CAN.sendMsgBuf(id, ext, 4, pids.Engine_Intake_Manifold_1_Pressure);
break;
}
case 0x33: //Barometric_Pressure_Absolute
{
pids.pedalD[3] = map(analogRead(A5), 0, 1023, 0, 255);
CAN.sendMsgBuf(id, ext, 4, pids.Barometric_Pressure_Absolute);
break;
}
default:
{
break;
}
}
}

void loop()
{
char buf[8];
if(CAN_MSGAVAIL == CAN.checkReceive())            // check if data coming
{
CAN.readMsgBuf(&len, buf);
sendValues(buf [2]);
}
}

*/

/*//***************************************************
// Conversor parte Motor-J1979-i2c
// v0.6
// MASTER
//***************************************************

#include <mcp_can.h>
#include <SPI.h>
#include <rExcel.h>
#include <Wire.h>
rExcel myExcel;
#define xcel_si 0

MCP_CAN CAN(10);

unsigned long t_ON;
unsigned long t_OFF;
uint16_t delta;


char stmp[] = {0x02, 0x01, 0x0c};
byte pids[] = {0x0C, 0x42, 0x05, 0x49, 0x04, 0x0E, 0x0B,0x33};
int canId;
char len;
char buf[8];
#define led_izq 14
#define led_rojo 16

void setup() {
Serial.begin(115200);
CAN.begin(CAN_500KBPS);
Wire.begin();
//rExcel
myExcel.clearInput();
pinMode(led_izq, OUTPUT);
}

void loop() {

for (int i=0;i<sizeof(pids);i++)
{
stmp[2]=pids[i];
CAN.sendMsgBuf(0x7df, 0, 3, stmp);
delay(10);
recibir();
}
}
void recibir (void) {

if(CAN_MSGAVAIL == CAN.checkReceive())            // check if data coming
{
CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf
canId = CAN.getCanId();

if (canId==0x7E8)
{
Wire.beginTransmission(1);
digitalWrite(led_izq, HIGH);   // turn the LED on (HIGH is the voltage level)
//resultado

Serial.print(buf[2],HEX);
Wire.write(buf[2]);
Serial.print(" ");
Serial.print(buf[3],HEX);
Wire.write(buf[3]);
Serial.print(" ");

#if xcel_si
myExcel.writeIndexed("kevin",3+buf[2] ,3, buf[3]);
#endif

if (buf[0]==4) {
Serial.println(buf[4],HEX);
Wire.write(buf[4]);

#if xcel_si
myExcel.writeIndexed("kevin",3+buf[2] ,3, buf[3]);myExcel.writeIndexed("kevin",3+buf[2] ,4, buf[4]);
#endif
} else  Serial.println("");

// Paramos la transmisión
Wire.endTransmission();
digitalWrite(led_izq, LOW);    // turn the LED off by making the voltage LOW
}//if
}
}
*/
