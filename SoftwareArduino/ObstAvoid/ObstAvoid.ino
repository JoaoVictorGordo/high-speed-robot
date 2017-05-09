#include <SPI.h>
#include <Time.h>
#include <PWM.h>

#include "string_handling.h"

#define trigPin 2
#define echoPin0 8
#define echoPin1 7
#define echoPin2 A0 
#define echoPin3 A1 
#define echoPin4 A2

#define buffer_size 150
// --------- deadlines (in millisseconds)----------------------------
short int outOfRange;
// -------- communication related variables -----------------
short int buffer[buffer_size][5];
unsigned long t;
bool go = HIGH;
//--------- UltraSonic Sensor type --------------------------
struct sensor_t
{
    uint8_t Bit;
    uint8_t port;
    bool data_available; // HIGH => distance is up to date, LOW => reading data || no obstacle detected during last reading operation.
    unsigned long duration; // pulse duration in echoPin.
    unsigned short int distance[5]; // current obstacle distance calculated.
    unsigned short int farthest; // farthest obstacle distance calculated (10 measures iteration).
    unsigned short int closest; // closest obstacle distance calculated (10 measures iteration).
    unsigned short int mean; // mean obstacle distance calculated (10 measures iteration).
    unsigned short int pointer; // points to most recent data in circular vector distance.
} USS[5]; 


//--------- obstacle avoidance related functions ---
void SensorSettings();
int myPulseIn(bool, short int, short int);
void getExtreamValues();
void testing();
void test(bool, short int, bool, short int);
/*=======================================================================================================*/

void setup()
{

	pinMode(trigPin, OUTPUT);
	pinMode(echoPin0, INPUT);
	pinMode(echoPin1, INPUT);
	pinMode(echoPin2, INPUT);
	pinMode(echoPin3, INPUT);
	pinMode(echoPin4, INPUT);
	Serial.begin(115200);
    SensorSettings();
    InitTimersSafe();
}

void loop()
{
	while(go)
	{
		if(Serial.available())
		{
			go = !go;
			while(Serial.available() > 0)
				Serial.read();
		}
	}
	go = !go;
	if(go)
	{
    	testing();
	}
	Serial.println("-");
}

/*---------------------------------  OBSTACLE AVOIDANCE STUFF    --------------------------------------------------------------------*/ 

// incluir tratativas de erro?
void SensorSettings()
{
    USS[0].Bit = digitalPinToBitMask(echoPin0);
    USS[0].port = digitalPinToPort(echoPin0);
    USS[0].pointer = 4;

    USS[1].Bit = digitalPinToBitMask(echoPin1);
    USS[1].port = digitalPinToPort(echoPin1);
    USS[1].pointer = 4;

    USS[2].Bit = digitalPinToBitMask(echoPin2);
    USS[2].port = digitalPinToPort(echoPin2);
    USS[2].pointer = 4;

    USS[3].Bit = digitalPinToBitMask(echoPin3);
    USS[3].port = digitalPinToPort(echoPin3);
    USS[3].pointer = 4;

    USS[4].Bit = digitalPinToBitMask(echoPin4);
    USS[4].port = digitalPinToPort(echoPin4);
    USS[4].pointer = 4;
}


/*
       Cache the port and bit of the pins in order to speed up the
       pulse width measuring loop and achieve finer resolution.
       Calling digitalRead() instead yields much coarser resolution.
*/
int myPulseIn(bool dynamical_reading, short int time_out, short int outOfRange)
{
/*TODO:
 * 1) check if it's possible to analyze all sensors at once:  *portInputRegister(USS[i].port)
 * 2) guarantee that micros() won't overflow during execution!!!
 */
    unsigned long initTime = millis();

    boolean finished = LOW; // finished reading all sensors data flag
    boolean previous_pulse[5]; // auxiliar variable that states if previous pulse has already ended or not.
    boolean reading[5];  // auxiliar variable that states if data is being read or not.

    for(int i = 0; i < 5; i++)
    {
        reading[i] = LOW;
        previous_pulse[i] = LOW;
        //	processing distance, so state is set to LOW and duration initialized 0.
        USS[i].data_available = LOW;
        USS[i].duration = 0;
        // adjusting pointer
        if(USS[i].pointer == 4)
            USS[i].pointer = 0;
        else
            USS[i].pointer = USS[i].pointer + 1;
    }
    // trigger
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // wait for echo
    while (!finished)
    {
        for(int i = 0; i < 5; i++)
        {
            if (!(USS[i].data_available)) // if data hasn't already been read
            {
                if (previous_pulse[i]) // if previous pulse has ended
                {
                    if (reading[i]) // if data is being read
                    {
                        // if data has just finished being read
                        if ((*portInputRegister(USS[i].port) & USS[i].Bit) != USS[i].Bit)
                        {
                            USS[i].duration = micros() - USS[i].duration;
                            USS[i].distance[USS[i].pointer] = USS[i].duration / 58.8;
                            USS[i].data_available = HIGH;
                        }
                    }
                    else // if data hasn't started to be read
                    {
                        // check if it has just started.
                        if ((*portInputRegister(USS[i].port) & USS[i].Bit) == USS[i].Bit)
                        {
                            USS[i].duration = micros();
                            reading[i] = HIGH;
                        }
                    }
                }
                else
                {
                    // check if previous pulse has just ended and set previous_pulse if it has.
                    if ((*portInputRegister(USS[i].port) & USS[i].Bit) != USS[i].Bit)
                        previous_pulse[i] = HIGH;
                }
            }
        }


        if (millis() > initTime + time_out)
        {
            // if there is no echo, we assume there is nothing in front of us
            for(int i = 0; i < 5; i++)
            {
                if( !(USS[i].data_available) )
                    USS[i].distance[USS[i].pointer] = outOfRange;
            }
            return 0;
        }

        finished =  USS[0].data_available &  USS[1].data_available &  USS[2].data_available &  USS[3].data_available & USS[4].data_available;
    }

    if(!dynamical_reading)
        while(millis() < initTime + time_out) ; // TODO: wasted time, find out something useful to do here.

    return 1; // TODO: return 0 instead of 1!!!
}

void getExtreamValues()
{
    for(int i = 0; i< 5; i++)
    {
        USS[i].farthest = USS[i].distance[0];
        USS[i].closest = USS[i].distance[0];
        USS[i].mean = USS[i].distance[0];
    }

    for(int j = 1; j< 5; j++)
        for(int i = 0; i< 5; i++)
        {
            if( USS[i].farthest < USS[i].distance[j])
                 USS[i].farthest = USS[i].distance[j];
            if(USS[i].closest > USS[i].distance[j])
                USS[i].closest = USS[i].distance[j];
            USS[i].mean += USS[i].distance[j];
        }

    for(int i = 0; i< 5; i++)
        USS[i].mean = ( USS[i].mean - USS[i].farthest - USS[i].closest ) / 3;
}

void testing()
{
    test(LOW/*dyn_reading*/, 30/*ms*/, LOW/*xtrem_values*/, 1000/*Range*/);
    test(HIGH/*dyn_reading*/, 30/*ms*/, LOW/*xtrem_values*/, 1000/*Range*/);
    test(LOW/*dyn_reading*/, 30/*ms*/, HIGH/*xtrem_values*/, 400/*Range*/);
    test(HIGH/*dyn_reading*/, 30/*ms*/, HIGH/*xtrem_values*/, 400/*Range*/);

    test(LOW/*dyn_reading*/, 45/*ms*/, LOW/*xtrem_values*/, 1000/*Range*/);
    test(HIGH/*dyn_reading*/, 45/*ms*/, LOW/*xtrem_values*/, 1000/*Range*/);
    test(LOW/*dyn_reading*/, 45/*ms*/, HIGH/*xtrem_values*/, 400/*Range*/);
    test(HIGH/*dyn_reading*/, 45/*ms*/, HIGH/*xtrem_values*/, 400/*Range*/);

    test(LOW/*dyn_reading*/, 60/*ms*/, LOW/*xtrem_values*/, 1000/*Range*/);
    test(HIGH/*dyn_reading*/, 60/*ms*/, LOW/*xtrem_values*/, 1000/*Range*/);
    test(LOW/*dyn_reading*/, 60/*ms*/, HIGH/*xtrem_values*/, 400/*Range*/);
    test(HIGH/*dyn_reading*/, 60/*ms*/, HIGH/*xtrem_values*/, 400/*Range*/);
}

void test(bool dyn, short int time_out, bool get_extream_values, short int outOfRange)
{
    for (short j = 0; j < buffer_size; j++)
    {
        myPulseIn(dyn, time_out, outOfRange);
        if(get_extream_values)
            getExtreamValues();
        for (short i = 0; i < 5; i++)
            buffer[j][i] = USS[i].distance[USS[i].pointer];
    }

    Serial.print(dyn);
    Serial.print(",");
    Serial.print(time_out);
    Serial.print(",");
    Serial.print(get_extream_values);
    Serial.print(",");
    Serial.println(outOfRange);
    for (short j = 0; j < buffer_size; j++)
    {
        for (short i = 0; i < 5; i++)
        {
            Serial.print(buffer[j][i]);
            Serial.print(",");
        }
        Serial.println();
    }
    Serial.println();
}
