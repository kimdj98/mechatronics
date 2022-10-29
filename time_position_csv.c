#include <stdio.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <softPwm.h>
#include <math.h>

#define ENCODER_A 17
#define ENCODER_B 27
#define ENC2REDGEAR 216
#define MOTOR1 19
#define MOTOR2 26
#define PULSE 21

FILE* fpt; // file pointer to save gains and itae

int encA;
int encB;
int encoderPosition = 0; // encoder position out of 216
float redGearPosition = 0; // motor position in rounds

//###################### configuration ######################
// configuration for tuning code

float Pgain = 650;
float Igain = 2;
float Dgain = 4.5;
char file_name[] = "P650_I2_D45e-1"; // name for time_position.csv
//##################### configuration ends ####################

void funENCODER_A()
{
	encA = digitalRead(ENCODER_A);
	encB = digitalRead(ENCODER_B);
	if (encA == HIGH)    // A is rising
	{
		if (encB == LOW)    // If B is low, turn right
			encoderPosition++;
		else                // if B is High, turn left
			encoderPosition--;
	}
	else                // A is falling
	{
		if (encB == LOW)
			encoderPosition--;  // if B is low, turn left
		else
			encoderPosition++;  // if B is High, turn right
	}

	redGearPosition = ((float)encoderPosition) / ENC2REDGEAR; // calculate Gear position
//	printf("funcEncoder_A Result: encPos:%d gearPos:%f\n", encoderPosition, redGearPosition); // uncomment this to see motor position
}


void funENCODER_B()
{
	encA = digitalRead(ENCODER_A);
	encB = digitalRead(ENCODER_B);
	if (encB == HIGH)   // B is rising
	{
		if (encA == LOW)    //if A is low, turn left
			encoderPosition--;
		else                //If A is high, turn right
			encoderPosition++;
	}
	else                // B is falling
	{
		if (encA == LOW)    //If A is Low, turn right
			encoderPosition++;

		else                //If A is High, turn left
			encoderPosition--;
	}
	redGearPosition = ((float)encoderPosition) / ENC2REDGEAR;//Calculate the Gear Position
//	printf("funcEncoder_A Result: encPos:%d gearPos:%f\n", encoderPosition, redGearPosition); // uncomment this to see motor position in realtime

}

void set_amplifer(float m, float error);
double PID_Control(int reference, float Pgain, float Igain, float Dgain, FILE* fpt);
int main()
{
	wiringPiSetupGpio();
	pinMode(ENCODER_A, INPUT);
	pinMode(ENCODER_B, INPUT);

	int test_iteration = 0;
	int test_reference = 0;
    float itae = 0.0;

	softPwmCreate(MOTOR1, 0, 100);
	softPwmCreate(MOTOR2, 0, 100);

	wiringPiISR(ENCODER_A, INT_EDGE_BOTH, funENCODER_A); // interrupt
	wiringPiISR(ENCODER_B, INT_EDGE_BOTH, funENCODER_B);

    int pulse = 0; // set pulse = 0

//#################     time_position csv code     ########################
    fpt = fopen(file_name, "w+");
    fprintf(fpt, "time, position\n");

//    printf("Enter number of iterations: ");
//    scanf("%d", &test_iteration);
//
//    int references[10] = {0,}; // array for saving reference value
//
//    for (int i = 0; i < test_iteration; i++) {
//        printf("Enter reference number: %d", i+1);
//        scanf("%d", &references[i]);
//    }


    printf("iteration: 1\n reference: 5\n");
    test_iteration = 1;
    test_reference = 5;

    for (int i = 0; i < test_iteration; i++) {
        while(1)
        {
//            pulse = digitalRead(PULSE);
            pulse = 1; // no need to put pulse
            if (pulse == HIGH) { //If program gets a pulse, start PID control
                itae += PID_Control(test_reference, Pgain, Igain, Dgain, fpt); // cumulate itae for each iteration
                printf("itae(iteration: %d): %f\n", i, itae);
                break;
            }
        }
    }
    printf("total performance(itae): %f\n", itae);

//################     time_position csv code ends     #####################

    softPwmWrite(MOTOR1, 0);
    softPwmWrite(MOTOR2, 0);
    return 0;
//################    stop motor 1,2     #########################
}

double PID_Control(int reference, float Pgain, float Igain, float Dgain, FILE* fpt)
{
	float m = 0;
	float m1 = 0;
	float error = 0;
	float curr_pos = 0;
	float error_1 = 0, error_2 = 0;
	float T = 0.01;
    double itae = 0;
	float G1 = (Pgain + Igain * T + Dgain / T); // Descretize the equation
	float G2 = -(Pgain + 2 * Dgain / T);
	float G3 = Dgain / T;

	unsigned int startTime = millis(); //start recording time to calculate 'T' term inside ITAE integrand
	while (1)
	{
		unsigned int inner_startTime = millis(); //start recording to measure the sampling time

		curr_pos = redGearPosition;
		error = (float)reference - curr_pos;

		m = m1 + G1 * error + G2 * error_1 + G3 * error_2; // using the difference equation, get new actuating signal

		//printf("Error Value: %f\n", error);
		//printf("Actuating signal Value: %f\n", m);

		set_amplifer(m, error); // rotate the motor with actuating signal

		error_2 = error_1; // update previous error
		error_1 = error; // update previous error
		m1 = m;

		unsigned int inner_endTime = millis();
		while (inner_endTime - inner_startTime < T * 1000) // delay until sampling time passes
		{
			inner_endTime = millis();
		}
		itae += (inner_endTime - startTime) / 1000 * fabs(error) * T; // accumulate the ITAE value

        fprintf(fpt, "%f, %f\n", (inner_endTime - startTime), redGearPosition); // change millisecond to second if needed by replacing (startTime - inner_endTime)/1000

        unsigned int endtime = millis();
        if (endtime - startTime >= 5000) // perform pid control for 4 seconds
		{
            softPwmWrite(MOTOR1, 0);
            softPwmWrite(MOTOR2, 0);
//			printf("PID CONTROL FINISH\n");
            printf("itae: %lf\n", itae);
            break;
        }
    }
    return itae;
}

void set_amplifer(float m, float error) // motor control code
{
	if (error > 0)
	{
		softPwmWrite(MOTOR1, m);
		softPwmWrite(MOTOR2, 0);
	}
	else
	{
		softPwmWrite(MOTOR1, 0);
		softPwmWrite(MOTOR2, -m);
	}
}