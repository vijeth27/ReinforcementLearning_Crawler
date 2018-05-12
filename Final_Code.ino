//including libraries for servo and the ultrasonic sensor
#include <hcsr04.h>
#include <Servo.h>


//Defining the attachment points for the sensor and actuator
int servo_base = 9;
int servo_arm = 10;
#define TRIG_PIN 12
#define ECHO_PIN 13

//Initializing the servos and the sensor
Servo baseServo;
Servo armServo;
HCSR04 hcsr04(TRIG_PIN, ECHO_PIN, 20, 4000);

//Defining the initial states and range of each servo angle
const int baseServoMin=70;
const int baseServoMax=110;
const int armServoMin=70;
const int armServoMax=110;
const int deltaArm=10;         //We move the arm servo in steps of 4
const int deltaBase=10;       //We move the base servo in steps of 10 
const int NBase=5; //(baseServoMax-baseServoMin)/deltaBase+1
const int NArm=5; //(armServoMax-armServoMin)/deltaBase+1

//Delays to be added in various locations of the code
const int servoMotionTime=500; //This delay is introduced to give time for the servo to complete its motion and settle down.

// NBase, NArmhave to be found deciding the range of the sweep we need as well as the sweep step size.
int R[NBase][NArm][4]; // Initialize to zero for all I care. Doesn't matter. Reward stores the time averaged reqard for each action from each state.
float Q[NBase][NArm][4]; // Initiallize all states to 10 randomly. We might wanna go lower than that TBH. recheck this.  Q is in a manner of speaking the learned reward.
int state[2]={baseServoMin,armServoMin};  //This vector array always holds the correct state of the system in terms of angles of tboth motors.
int index[2]={0,0};             //This stores the states {baseServo angle, } in consecutive integers with 0 for min and n for highest state.
int indexPrime[2];          //This stores the next state we will reach if we were to perfome the chosen action. It is used to find the look ahead term

//Distance measurement. See if we can switch to float
float prevDist=0; //The range finder reading in the previous step.
float r; //This stores the distance change after performing the action (in mm). 
int Scaling_factor=1; //A constant to make the reward matrix.

//Action state
int a; // a=0,1,2,3 for base++, Arm++, Arm--, Base-- (Where base ++ means adding a deltaBase angle to the base servo position)

//Takes average of the measured ultrasonic reading over a 50ms interval.
float getAvgR(){
  float AvgR=0;
  for (int i=1;i<6;i++){
    delay(10);
    AvgR=AvgR+(float(hcsr04.distanceInMillimeters())-AvgR)/i;
  }
  return AvgR;
}

//Suppose we were to perform an action we need to update the states, index as well as move the servos itself. So the below function takes the given state and updates both the state array and index array. It also instructs the arm to move by a certain angle.

void doAction(int action)  //Take an action among the 4 listed above
{
  if(action==0 && state[0]<baseServoMax)
  {
  baseServo.write(baseServo.read()+deltaBase);
  delay(servoMotionTime);             //This delay is introduced to give time for the servo to complete its motion and settle down.
  state[0]=state[0]+deltaBase;
  index[0]++;
  }

  else if(action==1 && state[1]<armServoMax)
  {
  armServo.write(armServo.read()+deltaArm);
  delay(servoMotionTime);             //This delay is introduced to give time for the servo to complete its motion and settle down.
  state[1]=state[1]+deltaArm;
  index[1]++;
  }
  
  else if(action==2 && state[1]>armServoMin)
  {
  armServo.write(armServo.read()-deltaArm);
  delay(servoMotionTime);             //This delay is introduced to give time for the servo to complete its motion and settle down.
  state[1]=state[1]-deltaArm;
  index[1]--;
  }

  else if(action==3 && state[0]>baseServoMin)
  {
  baseServo.write(baseServo.read()-deltaBase);
  delay(servoMotionTime);             //This delay is introduced to give time for the servo to complete its motion and settle down.
  state[0]=state[0]-deltaBase;
  index[0]--;
  }
  else
  {
  Q[index[0]][index[1]][action]=-1;  //This will only get implemented if the servo is at the limits and action being performed drives it outside the limits.
                  //We set the Q for these limit actions -1 so that they don't come again in Qmax at any point.
  }
}

//Creating the probability of performing a random action.
float epsilon=100; //This is the value which decreses with time and reduces the possibility of a random action being performed and insteads chooses the argmaxQ action.
float tau=150; //This gives the number of loop cycles we have been through so far.

//This function gives use the action=argmaxQ with a probability of (1-epsilon) and a random action with probability epsilon.
int getAction()
{
  int action;         //Variable that is finally return to the function.
  float val_max=-10000;   //This is used to store the Qmax we obtain over the various actions. It is initialized to a large negative number.
  int a_max;        //This is the argmax(Q) over all actions.

  float randVal = float(random(0,101));

  //With a probability of (1-epsilon) we will have this if condition to be true. and epsilon decays with time from 1.
  if (randVal>epsilon)  //epsilon is 100 at t=0 and decays exponentially. Likelihood of this case happening increases with time.
  {     
    for(int i=0;i<4;i++)
    {
      if(Q[index[0]][index[1]][i]>val_max)
      {
        val_max=Q[index[0]][index[1]][i];
        a_max = i;
      }
    }
    action=a_max;
  }
  //With a probability of (epsilon) we will have this else condition to be true.
  else 
  {
    action = int(random(0,4)); 
  }
  return action;
}

//This function gets the maximum Q of the next possible steps 
float getQPrime(int action)
{
  float QPrimeMax=-2000;        //Variable that is finally return to the function.
  
  int a_max;        //This is the argmax(Q) over all actions.
  indexPrime[0]=index[0]; 
  indexPrime[1]=index[1];
  
  if(action==0 && state[0]<baseServoMax){
  indexPrime[0]=index[0]+1;
  }

  if(action==1 && state[1]<armServoMax){
  indexPrime[1]=index[1]+1;
  }

  if(action==2 && state[1]>armServoMin){
  indexPrime[1]=index[1]-1;
  }

  if(action==3 && state[0]>baseServoMin){
  indexPrime[0]=index[0]-1;
  }
  

  if ((indexPrime[0]==index[0]&&indexPrime[1]==index[1])==0)
  {
  for(int i=0;i<4;i++)
    {
      if(Q[indexPrime[0]][indexPrime[1]][i]>QPrimeMax)
      {
        QPrimeMax=Q[indexPrime[0]][indexPrime[1]][i];
        a_max = i;
      }
    }
  }
  else
  {
    QPrimeMax= -2500;
  }
  return QPrimeMax;
}

void setup() {
  Serial.begin(9600);
  baseServo.attach(servo_base);
  armServo.attach(servo_arm);

  delay(2000);//Remove later
  //Start by placing the arm furthest from the ground or some thing like that
  baseServo.write(baseServoMin);
  delay(500);
  armServo.write(armServoMin);

  delay(3000); //This gives some time for the ultrasonic readings to stabilize.

   //Initialising Q and R
    for (int i=0;i<NBase;i++){
    for (int j=0;j<NArm;j++){
      for (int k=0;k<4;k++){
         Q[i][j][k]=10;    // Optimistic initialization of Q.
         R[i][j][k]=0;
      }
    }
  }
}

//Keeps a count of how many cycles have gone past.
float cycle = 0;
const float alphaR=0.8;
const float alphaQ=0.2;
const float gamma=0.75;
int prevIndex[2];
 
void loop() {
cycle++;
epsilon = 100*(exp(-cycle/tau));     //epsilon=100(e^(-t/tau)) is 100 at t=0 and decreases with time to zero at t=inf
a=getAction();
Serial.print("Action: "); Serial.println(a);
Serial.print(" State1: ");Serial.print(index[0]);Serial.print(" State2: ");Serial.println(index[1]);
float QPrime=getQPrime(a);
Serial.print("QPrime: "); Serial.println(QPrime);
prevDist=getAvgR();
prevIndex[0]=index[0];
prevIndex[1]=index[1];
doAction(a);
delay(200);
//Calculation of reward and update of reward matrix
r=Scaling_factor*(prevDist-getAvgR());      //Since our sensor is forward facing the difference will be negative for moving forward.
R[prevIndex[0]][prevIndex[1]][a]=(1-alphaR)*R[prevIndex[0]][prevIndex[1]][a]+alphaR*r;
Serial.print("R(k) "); Serial.println(R[prevIndex[0]][prevIndex[1]][a]);

//Calculation of q and update of Q matrix
float q=R[prevIndex[0]][prevIndex[1]][a]+gamma*QPrime;
Q[prevIndex[0]][prevIndex[1]][a]=(1-alphaQ)*Q[prevIndex[0]][prevIndex[1]][a]+alphaQ*q;
Serial.print("Q(k) "); Serial.println(Q[prevIndex[0]][prevIndex[1]][a]);
delay(50);
}
