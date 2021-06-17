#include <Arduino.h>
#include <SPI.h>
#include <MergCBUS.h>
#include <Message.h>
#include <EEPROM.h>
#include <DCCStepper.h>

#define DEBUGNODE 0
#define DEBUGSTEPPER 0
#define DEBUGMOVE 0
/**
   The following block of #defines configures the pins that are
   used for various special functions:
     CHIPSELECT  is the select pin for the CANBUS interface
     INTPIN      is the "interrupt" pin used by the CANBUS interface
                 to signal that a CANBUS packet is ready to be read
*/
#define GREEN_LED     A4     //merg green led port
#define YELLOW_LED    A3     //merg yellow led port
#define PUSH_BUTTON   A2     //std merg push button
#define CANPORT       10
#define INTPIN         8
#define RELAY1         2
#define RELAY2        19

/**
   Node Variables:
      1         Stepper 1 Number of steps MSB
      2         Stepper 1 Number of steps LSB
      3         Stepper 1 Speed
      4         Stepper 1 mode (bit 0 controls normal direction, bit 1 == 0 BiPolar else UniPolar)

      5         Stepper 2 Number of steps MSB
      6         Stepper 2 Number of steps LSB
      7         Stepper 2 Speed
      8         Stepper 2 mode (bit 0 controls normal direction, bit 1 == 0 BiPolar else UniPolar')

      9         Configuration
                          Bit  Meaning
                          0    If set then move on startup to preset day pisition
                          1    If set move stepper 1 to "normal" at end/start of day
                          2    If set move stepper 2 to "normal" at ebd/start of day
*/
#define NUM_NODE_VARS  9    //the node variables
#define NUM_EVENTS     32   //supported events
#define NUM_EVENT_VARS 1    //no need for supported event variables
#define NUM_DEVICES    1    //one device number
#define MODULE_ID      55   //module id
#define MANUFACTURER_ID 165 //manufacturer id
#define MIN_CODE       0    //min code version
#define MAX_CODE       1    //max code version

/**
   Internal variables used to track the current position
   and the last direction of travel
*/
#define INTERNAL_VAR_POSLSB   1
#define INTERNAL_VAR_POSMSB   2
#define INTERNAL_VAR_DIRECT   3

// Stored in internal variable 2
#define DIRECTION_CLOCK 0x01  // Moving clockwise
#define DIRECTION_ANTI  0x02  // Moving anti-cockwise



MergCBUS cbus = MergCBUS(NUM_NODE_VARS, NUM_EVENTS, NUM_EVENT_VARS, NUM_DEVICES);

DCCStepper *step1 = NULL;
DCCStepper *step2 = NULL;

boolean     s1Normal, s2Normal;
boolean     s1EOD, s2EOD;
unsigned long millis();

void myUserFunc(Message * msg, MergCBUS * mcbus) {
  /* getting standard on/off events */
  boolean onEvent;

  if (mcbus->eventMatch()) {
    onEvent = mcbus->isAccOn();
    int motor = mcbus->getEventVar(msg, 1); // Get first event variable

    if (onEvent)
    {
      // An On event, we move anti-clockwise
      if (motor == 0)
      {
#if DEBUGMOVE
        Serial.println("Motor 0 On");
#endif
        step1->setSpeed(100, s1Normal);
      }
      if (motor == 1)
      {
#if DEBUGMOVE
        Serial.println("Motor 1 On");
#endif
        step2->setSpeed(100, s2Normal);
      }
      if (motor == 255) // End of day event
      {
        step1->setSpeed(100, s1EOD);
        step1->setSpeed(100, s2EOD);
      }
#if RECORD_POSITION
      cbus.setInternalNodeVariable(INTERNAL_VAR_DIRECT, DIRECTION_ANTI);
#endif
    }
    else
    {
      // An Off event, we move clockwise
      if (motor == 0)
      {
#if DEBUGMOVE
        Serial.println("Motor 0 Off");
#endif
        step1->setSpeed(100, s1Normal ? false : true);
      }
      if (motor == 1)
      {
#if DEBUGMOVE
        Serial.println("Motor 1 Off");
#endif
        step2->setSpeed(100, s2Normal ? false : true);
      }
#if RECORD_POSITION
      cbus.setInternalNodeVariable(INTERNAL_VAR_DIRECT, DIRECTION_CLOCK);
#endif
    }
  }

  // Message directed to us
  if (msg->getNodeNumber() == cbus.getNodeId()->getNodeNumber()) {
    if (msg->getOpc() == OPC_NVSET)
    {
      int ind = msg->getNodeVariableIndex(); //the CBUS index start with 1
      int val = msg->getNodeVariable();
      switch (ind)
      {
        case 1:
          step1->setMaxStepsLSB(val);
          break;
        case 2:
          step1->setMaxStepsMSB(val);
          break;
        case 3:
          {
            byte rpm = val;
            step1->setRPM(rpm);
            break;
          }
        case 4:
          {
            byte mode = val;
            step1->setMode(mode);
            break;
          }
        case 5:
          step2->setMaxStepsLSB(val);
          break;
        case 6:
          step2->setMaxStepsMSB(val);
          break;
        case 7:
          {
            byte rpm = val;
            step2->setRPM(rpm);
            break;
          }
        case 8:
          {
            byte mode = val;
            step2->setMode(mode);
            break;
          }
        case 9:
          {
            s1EOD = val & 0x02;
            s2EOD = val & 0x04;
            break;
          }
      }
    }
  }
}

void myUserFuncDCC(Message * msg, MergCBUS * mcbus)
{
}

/*
   Callback from the stepper library used to record the stepper position.
   Also trigger the frog relay
*/
void notifyStepperPosition(DCCStepper *motor, unsigned int position)
{
  if (motor == step1)
  {
#if RECORD_POSITION
    cbus.setInternalNodeVariable(INTERNAL_VAR_POSLSB, position & 0xff);
    cbus.setInternalNodeVariable(INTERNAL_VAR_POSMSB, (position >> 8) & 0xff);
#endif
    if (position < step1->getMaxSteps() / 2)
      digitalWrite(RELAY1, LOW);
    else
      digitalWrite(RELAY1, HIGH);
  }

  if (motor == step2)
  {
#if RECORD_POSITION
    cbus.setInternalNodeVariable(INTERNAL_VAR_POSLSB, position & 0xff);
    cbus.setInternalNodeVariable(INTERNAL_VAR_POSMSB, (position >> 8) & 0xff);
#endif
    if (position < step2->getMaxSteps() / 2)
      digitalWrite(RELAY2, LOW);
    else
      digitalWrite(RELAY2, HIGH);
  }
}

void setup() {
  Serial.begin(9600);
  //Configuration data for the node
  cbus.getNodeId()->setNodeName("STEP", 4);        //node name
  cbus.getNodeId()->setModuleId(MODULE_ID);            //module number
  cbus.getNodeId()->setManufacturerId(MANUFACTURER_ID);//merg code
  cbus.getNodeId()->setMinCodeVersion(MIN_CODE);       //Version 1
  cbus.getNodeId()->setMaxCodeVersion(MAX_CODE);
  cbus.getNodeId()->setProducerNode(true);
  cbus.getNodeId()->setConsumerNode(true);
  cbus.setPushButton(PUSH_BUTTON);//set the push button ports
  cbus.setStdNN(999); //standard node number

  //used to manually reset the node. while turning on keep the button pressed
  //this forces the node for slim mode with an empty memory for learned events and devices
  if (digitalRead(PUSH_BUTTON) == LOW) {
    Serial.println("Setup new memory");
    cbus.setUpNewMemory();
    cbus.saveNodeFlags();
    cbus.setNodeVariable(1, 0);
    cbus.setNodeVariable(2, 100);
    cbus.setNodeVariable(3, 50);
    cbus.setNodeVariable(4, 0);
    cbus.setNodeVariable(5, 0);
    cbus.setNodeVariable(6, 100);
    cbus.setNodeVariable(7, 50);
    cbus.setNodeVariable(8, 0);
    cbus.setNodeVariable(9, 0);
    cbus.setInternalNodeVariable(INTERNAL_VAR_POSLSB, 0);
    cbus.setInternalNodeVariable(INTERNAL_VAR_POSMSB, 0);
    cbus.setInternalNodeVariable(INTERNAL_VAR_DIRECT, 0);
  }

  cbus.setLeds(GREEN_LED, YELLOW_LED); //set the led ports

  cbus.setUserHandlerFunction(&myUserFunc);//function that implements the node logic
  cbus.setDCCHandlerFunction(&myUserFuncDCC);
  cbus.initCanBus(CANPORT, CAN_125KBPS, MCP_16MHz, 20, 30);  //initiate the transport layer
  cbus.setFlimMode();

  pinMode(INTPIN, INPUT);

  pinMode(RELAY1, OUTPUT);
  pinMode(RELAY2, OUTPUT);

  int maxsteps = cbus.getNodeVar(1) * 256 + cbus.getNodeVar(2);
  byte rpm = cbus.getNodeVar(3);
  byte mode = cbus.getNodeVar(4);

  bool startupMove = cbus.getNodeVar(9) & 0x01;
  s1EOD = cbus.getNodeVar(9) & 0x02;
  s2EOD = cbus.getNodeVar(9) & 0x04;

#if DEBUGSTEPPER
  Serial.print("Stepper 0: ");
  Serial.print(maxsteps);
  Serial.print(" max steps, ");
  Serial.print(rpm);
  Serial.print(" RPM ");
  Serial.print(mode);
  Serial.println(" Mode");
#endif

  step1 = new DCCStepper(((mode & 0x02 == 0) ? STEPPER_BIPOLAR : 0) | STEPPER_MODE_CONSTRAINED, maxsteps, 64, rpm, 4, 5, 3, 7);

#if RECORD_POSITION
  int position = cbus.getInternalNodeVar(INTERNAL_VAR_POSLSB) | (cbus.getInternalNodeVar(INTERNAL_VAR_POSMSB) << 8);

  step1->setCurrentPosition(position);
  step1->setActive(true);
  if (cbus.getInternalNodeVar(INTERNAL_VAR_DIRECT) == DIRECTION_CLOCK)
    step1->setSpeed(100, true);
  else if (cbus.getInternalNodeVar(INTERNAL_VAR_DIRECT) == DIRECTION_ANTI)
    step1->setSpeed(100, false);
#else
  step1->setCurrentPosition(0);
  step1->setActive(true);
  s1Normal = ((mode & 0x01) == 0);
  if (startupMove)
    step1->setSpeed(100, s1Normal);
#endif

  maxsteps = cbus.getNodeVar(5) * 256 + cbus.getNodeVar(6);
  rpm = cbus.getNodeVar(7);
  mode = cbus.getNodeVar(8);

#if DEBUGSTEPPER
  Serial.print("Stepper 1: ");
  Serial.print(maxsteps);
  Serial.print(" max steps, ");
  Serial.print(rpm);
  Serial.print(" RPM ");
  Serial.print(mode);
  Serial.println(" Mode");
#endif
  step2 = new DCCStepper(((mode & 0x02 == 0) ? STEPPER_BIPOLAR : 0) | STEPPER_MODE_CONSTRAINED, maxsteps, 64, rpm, 6, 9, 14, 15);

#if RECORD_POSITION
  int position = cbus.getInternalNodeVar(INTERNAL_VAR_POSLSB) | (cbus.getInternalNodeVar(INTERNAL_VAR_POSMSB) << 8);

  step2->setCurrentPosition(position);
  step2->setActive(true);
  if (cbus.getInternalNodeVar(INTERNAL_VAR_DIRECT) == DIRECTION_CLOCK)
    step2->setSpeed(100, true);
  else if (cbus.getInternalNodeVar(INTERNAL_VAR_DIRECT) == DIRECTION_ANTI)
    step2->setSpeed(100, false);
#else
  step2->setCurrentPosition(0);
  step2->setActive(true);
  s2Normal = ((mode & 0x01) == 0);
  if (startupMove)
    step2->setSpeed(100, s2Normal);
#endif
#ifdef DEBUGNODE
  Serial.println("Setup finished");
  Serial.print("NNN: ");
  Serial.print(cbus.getNN());
  Serial.print("\t");
  Serial.println(cbus.getPromNN());
  Serial.print("CANID: ");
  Serial.println(cbus.getNodeId()->getCanID());

  Serial.print("Flim: ");
  Serial.println(cbus.getNodeId()->isFlimMode());
#endif
}

void loop() {

  cbus.cbusRead();
  cbus.run();//do all logic
  step1->loop();
  step2->loop();
  //debug memory
  if (digitalRead(PUSH_BUTTON) == LOW) {
    cbus.dumpMemory();
  }
}
