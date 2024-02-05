#include "mlc.h"

//Interrupts.
volatile int mems_event = 0;

// MLC
ucf_line_t *ProgramPointer;
int32_t LineCounter;
int32_t TotalNumberOfLine;

// Gyro
LSM6DSOXSensor AccGyr(&DEV_I2C, LSM6DSOX_I2C_ADD_L);

/**
 * @brief Callback function for INT1 event.
 */
void INT1Event_cb() {
  mems_event = 1;
}

/**
 * @brief Initializes the MLC (Machine Learning Controller).
 * This function is called during the setup phase of the program.
 * It sets up the necessary configurations and resources for the MLC.
 */
void setupMLC() {
  uint8_t mlc_out[8];

  // Force INT1 of LSM6DSOX low in order to enable I2C
  pinMode(INT_1, OUTPUT);

  digitalWrite(INT_1, LOW);

  delay(200);
  
  // Initialize I2C bus.
  DEV_I2C.begin();

  AccGyr.begin();
  AccGyr.Enable_X();
  AccGyr.Enable_G();

  /* Feed the program to Machine Learning Core */
  /* Activity Recognition Default program */  
  ProgramPointer = (ucf_line_t *)lsm6dsox_activity_recognition_for_mobile;
  TotalNumberOfLine = sizeof(lsm6dsox_activity_recognition_for_mobile) / sizeof(ucf_line_t);
  SerialPort.println("Activity Recognition for LSM6DSOX MLC");
  SerialPort.print("UCF Number Line=");
  SerialPort.println(TotalNumberOfLine);

  for (LineCounter=0; LineCounter<TotalNumberOfLine; LineCounter++) {
    if(AccGyr.Write_Reg(ProgramPointer[LineCounter].address, ProgramPointer[LineCounter].data)) {
      SerialPort.print("Error loading the Program to LSM6DSOX at line: ");
      SerialPort.println(LineCounter);
      while(1) {
        // Led blinking.
        digitalWrite(LED_BUILTIN, HIGH);
        delay(250);
        digitalWrite(LED_BUILTIN, LOW);
        delay(250);
      }
    }
  }

  SerialPort.println("Program loaded inside the LSM6DSOX MLC");

  //Interrupts.
  pinMode(INT_1, INPUT);
  attachInterrupt(INT_1, INT1Event_cb, RISING);

  /* We need to wait for a time window before having the first MLC status */
  delay(3000);

  AccGyr.Get_MLC_Output(mlc_out);
  printMLCStatus(mlc_out[0]);
}

/**
 * @brief Prints the status of the MLC (Multi-Level Cell) device.
 * 
 * @param status The status of the MLC device.
 */
void printMLCStatus(uint8_t status) {
  switch(status) {
    case 0:
      SerialPort.println("Activity: Stationary");
      addToLogBuffer("Stationary");
      break;
    case 1:
      SerialPort.println("Activity: Walking");
      addToLogBuffer("Walking");
      break;
    case 4:
      SerialPort.println("Activity: Jogging");
      addToLogBuffer("Jogging");
      break;
    case 8:
      SerialPort.println("Activity: Biking");
      addToLogBuffer("Biking");
      break;
    case 12:
      SerialPort.println("Activity: Driving");
      addToLogBuffer("Driving");
      break;
    default:
      SerialPort.println("Activity: Unknown");
      addToLogBuffer("Unknown");
      break;
  }	  
}

/**
 * @brief This function represents the main loop for the MLC (Machine Learning Controller).
 *        It is responsible for executing the machine learning algorithm and controlling the system behavior.
 *        This function is called repeatedly in the main program loop.
 */
void loopMLC() {
  if (mems_event) {
    mems_event=0;
    LSM6DSOX_MLC_Status_t status;
    AccGyr.Get_MLC_Status(&status);
    if (status.is_mlc1) {
      uint8_t mlc_out[8];
      AccGyr.Get_MLC_Output(mlc_out);
      printMLCStatus(mlc_out[0]);
    }
  }
}