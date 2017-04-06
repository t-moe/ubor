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
#define ROBOT_L_RESET_ID  			0x15F
#define COMAND_DLC					0x060
#define STATUS_REQEST_DLC			0x020
#define STAUS_ARM					0x0201

// Right
#define ROBOT_R_STATUS_REQUEST_ID 	0x160
#define ROBOT_R_STATUS_RETURN_ID  	0x161
#define ROBOT_R_COMAND_REQUEST_ID  	0x162
#define ROBOT_R_COMAND_RETURN_ID  	0x163
#define ROBOT_R_Reset_ID		  	0x161

// Min Max Values for roboter position
#define BASIS_MIN		-75
#define BASIS_MAX	 	 75
#define SHOULDER_MIN	-25
#define SHOULDER_MAX	 25
#define ELBOW_MIN		-55
#define ELBOW_MAX		 70
#define HAND_MIN		-70
#define HAND_MAX         70
#define GRIPPER_MIN		  0
#define GRIPPER_MAX       1

// Position for every joint to reach a position


//
#define BLOCK_TIME_MIDDLE_POS 15 // block time for mutex midle position
#define MSG_QUEUE_SIZE 1
#define ARM_TASK_PRIORITY 3
#define ARM_TASK_STACKSIZE 256
#define TASK_DELAY 10

//----- Data types -------------------------------------------------------------


uint32_t *pcMsgBuffer;
//----- Function prototypes ----------------------------------------------------
extern  void  vMoveRoboter(void *pvData);
extern  void  waitUntilPos(uint8_t *pos);
extern  void  initArm();

//----- Data -------------------------------------------------------------------


#endif /* ARM_H_ */
