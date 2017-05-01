#ifndef ARM_H_
#define ARM_H_
/******************************************************************************/
/** \file       arm.h
 *******************************************************************************
 *
 *  \brief      Functions to move roboter
 *
 *  \author     ingmacmech
 *
 ******************************************************************************/
/*
 *  function
 *
 ******************************************************************************/

//----- Header-Files -----------------------------------------------------------


//----- Macros -----------------------------------------------------------------
// Left
#define ROBOT_L_STATUS_REQUEST_ID   0x150
#define ROBOT_L_STATUS_RETURN_ID    0x151
#define ROBOT_L_COMAND_REQUEST_ID   0x152
#define ROBOT_L_COMAND_RETURN_ID    0x153
#define ROBOT_L_RESET_ID            0x15F
#define COMAND_DLC                  0x006
#define STATUS_REQEST_DLC           0x002


// Right
#define ROBOT_R_STATUS_REQUEST_ID   0x160
#define ROBOT_R_STATUS_RETURN_ID    0x161
#define ROBOT_R_COMAND_REQUEST_ID   0x162
#define ROBOT_R_COMAND_RETURN_ID    0x163
#define ROBOT_R_RESET_ID            0x16F

#define MASK_SWITCH_0               0x01
#define MASK_SWITCH_1               0x02
#define MASK_SWITCH_2               0x04
#define MASK_SWITCH_3               0x08
#define MASK_SWITCH_4               0x10


// Min Max Values for roboter position
#define BASIS_MIN       -75
#define BASIS_MAX        75
#define SHOULDER_MIN    -25
#define SHOULDER_MAX     25
#define ELBOW_MIN       -55
#define ELBOW_MAX        70
#define HAND_MIN        -70
#define HAND_MAX         70
#define GRIPPER_MIN       0
#define GRIPPER_MAX       1

// Position for every joint to reach a position


//
#define BLOCK_TIME_MIDDLE_POS 200000 // block time for mutex midle position
#define MSG_QUEUE_SIZE 1
#define ARM_TASK_PRIORITY 2
#define ARM_TASK_STACKSIZE 256
#define TASK_DELAY 100

//----- Data types -------------------------------------------------------------


uint32_t *pcMsgBuffer;
//----- Function prototypes ----------------------------------------------------
extern  void  vMoveRoboter(void *pvData);
extern  void  vManualArmMovment(void *pvData);
extern  void  waitUntilPos(uint8_t *pos, int side);
extern  void  init_arm();

//----- Data -------------------------------------------------------------------


#endif /* ARM_H_ */
