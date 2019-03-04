/***************************************************************************
 *   Filename       : Avionics.ino
 *   Target machine : Arduino Uno with GY91 sensor module
 *
 *   Maximum stable thruput   : sample / 10 ms
 *   Maximum unstable thruput : sample / 5  ms
 *
 *   Contributor :
 *      rev.1 11/27/2017
 *            Beom Park
 *
 *      rev.2 12/01/20
 *            Jaerin Lee
 *            Beom Park
 *
 *      rev.3 02/01/2018
 *            Jaerin Lee
 *
 *      rev.4 02/10/2018
 *            Jaerin Lee
 *
 *      rev.5 06/30/2018
 *            Jaerin Lee
 *
 *      rev.6 07/31/2018
 *            Mingyu Park
 *
 *      rev.7 10/29/2018
 *            Jaerin Lee
 *      
 *      rev.8 02/19/2019
 *            Inbum Park
 *
 *      Hanaro, SNU
 ***************************************************************************/

#include <Wire.h>
#include <SPI.h>

/* External libraries for peripherals */
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <FaBo9Axis_MPU9250.h>


/* Threshold altitude for deployment of parachutes */
#define A_THRESHOLD 1.2
#define DROGUE_PARA_THRESHOLD 2.5
#define REL_MAIN_PARA_THRESHOLD 50
#define ABS_MAIN_PARA_THRESHOLD 320

/* Mode bit for main parachute deploy mode */
#define REL_PARA_MODE ((byte)0)
#define ABS_PARA_MODE ((byte)1)

/* Calibration for stable boot */
#define INIT_ALT_DUMP_COUNT 10
#define INIT_ALT_COUNT 20

/* Parachute deploy signal */
#define MAIN 6
#define DROGUE 7

/* For use in Arduino Mega */
// #define MAIN 25
// #define DROGUE 23

/* Data save cycle */
#define PERIOD_DATA_RETRIEVAL 5 /* ms */

/* Structure of the data packet. You may customize the contents in the
 * packet, though the change should be reflected in the decoder simul-
 * taneously. */
typedef struct data_packet_t {
    uint32_t current_time; /* 4          */
    float a[3];            /* 4 * 3 = 12 */
    float g[3];            /* 4 * 3 = 12 */
    float m[3];            /* 4 * 3 = 12 */ 
    float temp;            /* 4          */
    //uint32_t press;      /* 4          */
    float alt_press;       /* 4          */
    float alt_max;         /* 4          */
    byte state;            /* 1          */
} data_packet_t;

typedef union data_t {
    data_packet_t packet;
    char charstring[sizeof(struct data_packet_t)];
    byte bytestring[sizeof(struct data_packet_t)];
} data_t;
data_t data;               /* Cache memory for acquired data */

FaBo9Axis fabo_9axis;      /* I2C */
Adafruit_BMP280 bmp;       /* I2C */

uint32_t curr_time;
uint32_t prev_time = 0;
float ini_altitude;
float a_threshold = A_THRESHOLD;
float last_altitude;
float max_altitude;
byte para_mode = REL_PARA_MODE;
uint32_t trigger_apogee = 0;
uint32_t rocket_on = 0;
uint32_t para_drogue = 0;
uint32_t para_main = 0;


void setup() {
    /* Serial communication for debug/communication begin */
    Serial.begin(115200);
    delay(3000);        /* For Leonardo/Micro */
    Serial.println("[  WORK] INIT CONF");
    Wire.begin();

    /* Enable if you want to use ss pin(10) in Arduino Uno as cs pin */
    // pinMode(53, OUTPUT);
    // digitalWrite(53, LOW);

    /* Reset parachute deploy signal */
    pinMode(MAIN, OUTPUT);
    digitalWrite(MAIN, LOW);
    pinMode(DROGUE, OUTPUT);
    digitalWrite(DROGUE, LOW);

    /* Configure MPU9250 sensor */
    if (fabo_9axis.begin()) {
        fabo_9axis.configMPU9250(MPU9250_GFS_1000, MPU9250_AFS_16G);
        Serial.println("[STATUS] CONF: FaBo 9Axis I2C Brick");
    } else {
        Serial.println("[ ERROR] NO FaBo 9Axis I2C Brick");
        while (1)
            ; // Die here
    }

    /* Configure BMP280 sensor */
    if (bmp.begin()) {
        Serial.println("[STATUS] CONF: BMP 280");
    } else {
        Serial.println("[ ERROR] NO BMP280 (Check wiring!)");
        while (1)
            ; // Die here
    }

    /* Read altitude out for calibration */
    float tmp1;
    Serial.print("[  WORK] DUMP INIT ALT: ");
    for (uint32_t i = 0; i < INIT_ALT_DUMP_COUNT; ++i) {
        delay(100);
        tmp1 = bmp.readAltitude();
        Serial.print(tmp1);
        Serial.print(" ");
    }
    Serial.print("\n");

    /* Evaluate initial altitude offset for calibration */
    float tmp2;
    float calc1 = 0;
    float calc2 = 0;
    float ax, ay, az;
    Serial.print("[  WORK] EVAL INIT ALT: ");
    for (uint32_t i = 0; i < INIT_ALT_COUNT; ++i) {
        delay(100);
        fabo_9axis.readAccelXYZ(&ax, &ay, &az);
        calc1 += (tmp1 = bmp.readAltitude());
        calc2 += (tmp2 = sqrt(ax * ax + ay * ay + az * az));
        Serial.print(tmp1);
        Serial.print(" ");
        Serial.print(tmp2);
        Serial.print("\t");
    }
    Serial.print("\n");

    calc1 /= INIT_ALT_COUNT;
    max_altitude = last_altitude = ini_altitude = calc1;

    calc2 /= INIT_ALT_COUNT;
    a_threshold = calc2 + A_THRESHOLD - 1;

    Serial.print("[  DATA] INIT ALT: ");
    Serial.println(ini_altitude);
    Serial.print("[  DATA] INIT ACC: ");
    Serial.println(calc2);
    Serial.print("[  DATA] MIN ACC FOR PARA EJECT: ");
    Serial.println(a_threshold);

    /* Main loop
     * To optimize performance in the Arduino chip, main loop does not reside
     * on the loop() function. It is better to have a loop in the setup
     * function in order to avoid nasty problems involving function calls. */
    while (1) {
        curr_time = millis();
        if (curr_time - prev_time >= PERIOD_DATA_RETRIEVAL) {
            prev_time = curr_time;

            /* Calculate A first (optimization)
             * We need to minimize the use of long-living local variables. */
            float ax, ay, az;
            float A = 0;
            fabo_9axis.readAccelXYZ(&ax, &ay, &az);
            A += ax * ax; A += ay * ay; A += az * az;

            /* Retrieve rest of the data */
            float gx, gy, gz;
            float mx, my, mz;
            float temp;
            fabo_9axis.readGyroXYZ(&gx, &gy, &gz);
            fabo_9axis.readMagnetXYZ(&mx, &my, &mz);

            /* Data acquisition in the in-body DAQ */
            //uint32_t press;
            //press = (uint32_t)analogRead(A0);

            /* Temporal data in stack (optimization) */
            float pres_altitude = 0;
            pres_altitude = bmp.readAltitude();
            temp = bmp.readTemperature();

            /* Determine initial state transition of the rocket. Parachute
             * logic is only valid if the rocket is set still at the initial
             * state. */
            A = sqrt(A);
            if (!rocket_on && A >= a_threshold) {
                rocket_on = 1;
            }

            /* Parachute deployment code */
            if (rocket_on) {
                switch (para_mode) {
                    case REL_PARA_MODE:
                        /* Mode change */
                        if (pres_altitude - ini_altitude > ABS_MAIN_PARA_THRESHOLD + REL_MAIN_PARA_THRESHOLD) {
                            para_mode = ABS_PARA_MODE;
                        }

                        /* Update maximum altitude */
                        if (pres_altitude >= max_altitude) {
                            max_altitude = pres_altitude;
                            trigger_apogee = 0;
                        /* Main parachute ejection */
                        } else if (max_altitude - pres_altitude >= REL_MAIN_PARA_THRESHOLD) {
                            digitalWrite(MAIN, HIGH);
                            para_main = 1;
                            trigger_apogee = 1;
                        /* Drogue parachute ejection */
                        } else if (max_altitude - pres_altitude >= DROGUE_PARA_THRESHOLD) {
                            if (trigger_apogee && pres_altitude != last_altitude) {
                                digitalWrite(DROGUE, HIGH);
                                para_drogue = 1;
                                trigger_apogee = 1;
                            } else {
                                trigger_apogee = 1;
                            }
                        } else {
                            trigger_apogee = 0;
                        }
                        break;
                    case ABS_PARA_MODE:
                        /* Update maximum altitude */
                        if (pres_altitude >= max_altitude) {
                            max_altitude = pres_altitude;
                            trigger_apogee = 0;
                        /* Drogue parachute ejection */
                        } else if (max_altitude - pres_altitude >= DROGUE_PARA_THRESHOLD) {
                            if (trigger_apogee && pres_altitude != last_altitude) {
                                digitalWrite(DROGUE, HIGH);
                                para_drogue = 1;
                                trigger_apogee = 1;
                            } else {
                                trigger_apogee = 1;
                            }
                        } else {
                            trigger_apogee = 0;
                        }
                        /* Main parachute ejection */
                        if (para_drogue && pres_altitude - ini_altitude < ABS_MAIN_PARA_THRESHOLD) {
                            digitalWrite(MAIN, HIGH);
                            para_main = 1;

                            trigger_apogee = 1;
                        }
                        break;
                    default:
                        para_mode = REL_PARA_MODE;
                        break;
                }
            }

            /* Code for state */
            byte state = (byte)((rocket_on << 2) + (para_drogue << 1) + para_main);

            /* Package remaining data */
            data.packet.a[0] = ax;
            data.packet.a[1] = ay;
            data.packet.a[2] = az;
            data.packet.g[0] = gx;
            data.packet.g[1] = gy;
            data.packet.g[2] = gz;
            data.packet.m[0] = mx;
            data.packet.m[1] = my;
            data.packet.m[2] = mz;
            data.packet.temp = temp;
            //data.packet.press = press;
            data.packet.current_time = curr_time;
            data.packet.alt_press = pres_altitude;
            data.packet.alt_max = max_altitude;
            data.packet.state = state;


            /* Communication mode 1: write to the serial directly */

            /* Send data to the USB serial */
            
            Serial.write('$'); // Start character
            Serial.write('$'); // Start character
            Serial.write(data.bytestring, sizeof(data_packet_t));
            Serial.write('#'); // End character
            Serial.write('#'); // End character
        }
    }
}

/* For optimization of performance, loop function is not used */
void loop() {
}
