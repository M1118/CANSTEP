#include <Arduino.h>
#include <SPI.h>
#include <MergCBUS.h>
#include <Message.h>
#include <EEPROM.h>
#include <DCCStepper.h>

#define DEBUGNODE 1
/**
   The following block of #defines configures the pins that are
   used for various special functions:
     CHIPSELECT  is the select pin for the CANBUS interface
     INTPIN      is the "interrupt" pin used by the CANBUS interface
                 to signal that a CANBUS packet is ready to be read
*/
#define GREEN_LED     A3     //merg green led port
#define YELLOW_LED    A4     //merg yellow led port
#define PUSH_BUTTON   A2     //std merg push button
#define CANPORT       10
#define INTPIN         8
#define RELAY          2


#define NUM_NODE_VARS  4    //the node variables
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

void myUserFunc(Message * msg, MergCBUS * mcbus) {
  /* getting standard on/off events */
  boolean onEvent;

  if (mcbus->eventMatch()) {
    onEvent = mcbus->isAccOn();

    if (onEvent)
    {
      // An On event, we move anti-clockwise
      step1->setSpeed(100, false);
      cbus.setInternalNodeVariable(INTERNAL_VAR_DIRECT, DIRECTION_ANTI);
    }
    else
    {
      // An Off event, we move clockwise
      step1->setSpeed(100, true);
      cbus.setInternalNodeVariable(INTERNAL_VAR_DIRECT, DIRECTION_CLOCK);
    }
  }
}

void myUserFuncDCC(Message * msg, MergCBUS * mcbus) {
  //  Serial.print("DCC Code: ");
  //  Serial.println(msg->getOpc());
}

/*
   Callback from the stepper library used to record the stepper position.
   Also trigger the frog relay
*/
void notifyStepperPosition(DCCStepper *motor, unsigned int position)
{
  if (motor == step1)
  {
    cbus.setInternalNodeVariable(INTERNAL_VAR_POSLSB, position & 0xff);
    cbus.setInternalNodeVariable(INTERNAL_VAR_POSMSB, (position >> 8) & 0xff);
    if (position < step1->getMaxSteps() / 2)
      digitalWrite(RELAY, LOW);
    else
      digitalWrite(RELAY, HIGH);
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
    cbus.setNodeVariable(2, 200);
    cbus.setNodeVariable(3, 50);
    cbus.setInternalNodeVariable(INTERNAL_VAR_POSLSB, 0);
    cbus.setInternalNodeVariable(INTERNAL_VAR_POSMSB, 0);
    cbus.setInternalNodeVariable(INTERNAL_VAR_DIRECT, 0);
  }

  cbus.setLeds(GREEN_LED, YELLOW_LED); //set the led ports

  cbus.setUserHandlerFunction(&myUserFunc);//function that implements the node logic
  cbus.setDCCHandlerFunction(&myUserFuncDCC);
  cbus.initCanBus(CANPORT, CAN_125KBPS, MCP_8MHz, 20, 30);  //initiate the transport layer
  cbus.setFlimMode();

  pinMode(INTPIN, INPUT);

  int maxsteps = cbus.getNodeVar(1) * 256 + cbus.getNodeVar(2);
  byte rpm = cbus.getNodeVar(3);

#if DEBUGSTEPPER
  Serial.print("Stepper: ");
  Serial.print(maxsteps);
  Serial.print(" max steps, ");
  Serial.print(rpm);
  Serial.println(" RPM");
#endif
  step1 = new DCCStepper(STEPPER_BIPOLAR | STEPPER_MODE_CONSTRAINED, maxsteps, 64, rpm, 4, 3, 5, 7);

  int position = cbus.getInternalNodeVar(INTERNAL_VAR_POSLSB) | (cbus.getInternalNodeVar(INTERNAL_VAR_POSMSB) << 8);

  step1->setCurrentPosition(position);
  step1->setActive(true);
  if (cbus.getInternalNodeVar(INTERNAL_VAR_DIRECT) == DIRECTION_CLOCK)
    step1->setSpeed(100, true);
  else if (cbus.getInternalNodeVar(INTERNAL_VAR_DIRECT) == DIRECTION_ANTI)
    step1->setSpeed(100, false);
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
  //debug memory
  if (digitalRead(PUSH_BUTTON) == LOW) {
    cbus.dumpMemory();
  }
}
