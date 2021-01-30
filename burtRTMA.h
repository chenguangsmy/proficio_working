/*  
 * This file includes code to control asynchronous control of the robot
 * 
 * included are:
 * 
 * recoverBurt -> recovers burt to home position upon error
 * 
 * movetoCenter -> moves the burt to a predefined center postion
 * 
 * Author(s): Ivana Stevens 2019, Chenguang Z. 2020
 * 
 */

#include "/home/robot/src/Proficio_Systems/magnitude.h" //...do we use these files?
#include "/home/robot/src/Proficio_Systems/normalize.h"
#include "/home/robot/rg2/include/RTMA_config.h"
#include <unistd.h>

#include "/home/robot/RTMA/include/RTMA.h"
#include "movingBurt.h"
// #include "hapticsDemoClass.h"
#include "CustomClass.h"

#include <string>
#include <signal.h>
#include <typeinfo>
#include <iostream>
#include <fstream>
#include <time.h>
#include <stdio.h>
#include <pthread.h>
#include <ctime>

#include <cmath>
#include <math.h>
#include <Eigen/Core>
#include <barrett/detail/ca_macro.h>
#include <barrett/systems/abstract/haptic_object.h>

#include <boost/ref.hpp>
#include <boost/bind.hpp>
#include <boost/tuple/tuple.hpp>

#include <barrett/detail/stl_utils.h>  // waitForEnter()

#include <barrett/log.h>
#include <barrett/math.h>
#include <barrett/units.h>
#include <barrett/config.h>
#include <barrett/systems.h>
#include <barrett/exception.h>
//#include <proficio/systems/utilities.h>
#include <barrett/products/product_manager.h>

// Networking
#include <netinet/in.h>
#include <sys/types.h>

//#include <proficio/systems/utilities.h>

BARRETT_UNITS_FIXED_SIZE_TYPEDEFS;
BARRETT_UNITS_TYPEDEFS(6);


//extern bool yDirectionError;
extern bool forceMet; //thresholdMet;
//extern double forceThreshold;
extern std::string fname_rtma;
extern bool fname_init; 

/**
 * Thread funtion to handle messages from RTMA 
 */
// replace DOF with 4, try
template <size_t DOF>
void respondToRTMA(barrett::systems::Wam<DOF>& wam,
              cp_type system_center,
              RTMA_Module &mod,
              //HapticsDemo<DOF> &ball)
              ControllerWarper<DOF> &cw,
              LoggerClass<DOF> &lg)
{ 
  cf_type cforce;
//  bool wamLocked = false; //.. this could be a garbage code -cg.
  bool read_rlt = false;
  
  bool freeMoving = false; //TODO: THIS SHOULD BE FALSE
  bool sendData = true;
  bool fnameInit = false;
  bool fdirInit = false;
  cp_type cp;
  cv_type cv;
  jp_type jp;
  jv_type jv;
  jt_type jt;
  double taskJ_center[4]; // task send joint position
  cp_type monkey_center(system_center);
  cp_type target;
	CMessage Consumer_M;

  // variables for save filename 
  char data_dir[MAX_DATA_DIR_LEN];
  std::string file_dir;
  char file_name[20];
  char subject_name[TAG_LENGTH];
  int session_num;

  while (true)  // Allow the user to stop and resume with pendant buttons
  {
    // Read Messages
    read_rlt = mod.ReadMessage( &Consumer_M, 0.1);
  /*  if (read_rlt) {
      printf("Msg readed! \n");
    }
    else {
      printf("No Msg can be read in this 0.1s\n");
    };
    */
    // Send Position Data
    if (Consumer_M.msg_type == MT_SAMPLE_GENERATED && sendData)
    {
      MDF_SAMPLE_GENERATED sample_generated_data;
      Consumer_M.GetData(&sample_generated_data);
      MDF_BURT_STATUS burt_status_data;
      
      burt_status_data.sample_header = sample_generated_data.sample_header;
      //burt_status_data.timestamp = getTimestamp();
      //burt_status_data.state = thresholdMet;
      //burt_status_data.error = (int) hasError;
      burt_status_data.RDT = cw.jj.get_rdt(); // ReaDTime for synchrony
      // Get Force Data //.. I doubt this line, how should cforce wrote? -CG
      burt_status_data.force_x = cforce[1];
      burt_status_data.force_y = cforce[0];
      burt_status_data.force_z = cforce[2];
      

      // Get Joint position, velocity and toque
      //For some reason, setting wam.getJointTorques to a variable of type jt_type Does not work. 
      //During make, will report error.
          
      burt_status_data.jt_1 = wam.getJointTorques()[0];
      burt_status_data.jt_2 = wam.getJointTorques()[1];
      burt_status_data.jt_3 = wam.getJointTorques()[2];
      burt_status_data.jt_4 = wam.getJointTorques()[3];

      burt_status_data.jv_1 = wam.getJointVelocities()[0];
      burt_status_data.jv_2 = wam.getJointVelocities()[1];
      burt_status_data.jv_3 = wam.getJointVelocities()[2];
      burt_status_data.jv_4 = wam.getJointVelocities()[3];

      burt_status_data.jp_1 = wam.getJointPositions()[0];
      burt_status_data.jp_2 = wam.getJointPositions()[1];
      burt_status_data.jp_3 = wam.getJointPositions()[2];
      burt_status_data.jp_4 = wam.getJointPositions()[3];
      
      // Get Position Data  TODO: MOVE CONVERSION ELSEWHERE
      cp = barrett::math::saturate(wam.getToolPosition(), 9.999);
      burt_status_data.pos_x = cp[0]; // * 1280 / 0.2; // Assume this is accurate
      burt_status_data.pos_y = cp[1]; // * 1280 / 0.2; // TODO: check that cp has the right value
      burt_status_data.pos_z = cp[2]; // * 1280 / 0.2; // and is set properly
      
      //populate velocity here
      cv = barrett::math::saturate(wam.getToolVelocity(), 19.999);
      burt_status_data.vel_x = cv[0]; // * 1280 / 0.2; // Assume this is accurate
      burt_status_data.vel_y = cv[1]; // * 1280 / 0.2; // TODO: check that cp has the right value
      burt_status_data.vel_z = cv[2]; // * 1280 / 0.2; // and is set properly

      //also populate the center location
      burt_status_data.center_x = system_center[0];
      burt_status_data.center_y = system_center[1];
      burt_status_data.center_z = system_center[2];

      // Send Message
      CMessage M( MT_BURT_STATUS );
      M.SetData( &burt_status_data, sizeof(burt_status_data) );
      mod.SendMessage( &M );
      // printf("M data has been sent! \n"); // debug code 
    }

    // Task State config, set stiffness and zero-force position
    if (Consumer_M.msg_type == MT_TASK_STATE_CONFIG)
    {
      //printf("M: MT_TASK_STATE_CONFIG \n");
      MDF_TASK_STATE_CONFIG task_state_data;
      Consumer_M.GetData( &task_state_data);
      //cout << "task id : " << task_state_data.id << endl;
      freeMoving = false;
      sendData  = true;
      switch(task_state_data.id)
      {
        case 1:   // set joint center and endpoint center
          cout << " ST 1, ";
          freeMoving = true;
          sendData = false;
          cw.setForceMet(false);
          // target: XYZ-IJK-0123456789
          monkey_center[0] = task_state_data.target[0]; // here we temperarily change to a const value, for tesging
          monkey_center[1] = task_state_data.target[1];
          monkey_center[2] = task_state_data.target[2];
          
          taskJ_center[0] = task_state_data.target[8];
          taskJ_center[1] = task_state_data.target[9];
          taskJ_center[2] = task_state_data.target[10];
          taskJ_center[3] = task_state_data.target[11];
          //cout << " case 1 Target : " << target[0] << "," << target[1] << "," << target[2] << endl;
          //cw.setCenter_joint(taskJ_center);
          //cw.setCenter_endpoint(monkey_center);
          break;
        case 2: // Present
          cout << " ST 2, ";
          
          //cout << " case 2 " << endl;
          break;
        case 3: //ForceRamp
          // wam.idle(); //try a remove, if it stiff the wam? -cg
          cout << " ST 3, ";
//          wamLocked = false;
          //forceThreshold = 0; //task_state_data.target[3]; //TODO: SEND FROM JUDGE MESSAGE? OR SEPARTE CONFIGURE
          //cout << "force threshold is: " << task_state_data.target[3] << endl;
          break;
        case 4: //Move
          cout << " ST 4, ";
          cw.setForceMet(true); //decrease the impedance suddenly
          //cw.setForceMet(false);//true); //debugging 
          break;
        case 5: // hold
          cout << " ST 5, ";
          break;
        case 6:
          cout << " ST 6, ";
          break;
        case 7:
          cout << " ST 7, " << endl;
          freeMoving = true;
          break;
        default:
          break;
      }      
      //printf("ST: %d, ", task_state_data.id);
    }

    // Have Burt move in steps toward home postion
    else if (Consumer_M.msg_type == MT_MOVE_HOME && freeMoving)
    {
      MDF_MOVE_HOME startButton;
      Consumer_M.GetData( &startButton); 
      cw.moveToq0(); //make sure on the right joint position
      // cout<<"move to: "<< monkey_center[0] << "; "<< monkey_center[1] << "; "<< monkey_center[2] <<endl;
      moveToCenter(wam, monkey_center, mod);  // do not fully delete this part! Msg contain! 
      // re-track force output here?  
      cw.trackSignal(); //maybe not needed as idle no longer exist. 

    }

    // Ping sent   Acknowlegde ping...
    else if (Consumer_M.msg_type == MT_PING) {
      cw.init();
    }

    // Exit the function
    else if (Consumer_M.msg_type == MT_EXIT) { // add finish recording here
      wam.moveHome();
      wam.idle();
      break;
    }
  
    else if (Consumer_M.msg_type == MT_SESSION_CONFIG)
    {
        printf("MT_SESSION_CONFIG. \n");
        MDF_SESSION_CONFIG ssconfig;
        Consumer_M.GetData(&ssconfig);
        strcpy(data_dir, ssconfig.data_dir);
        file_dir = data_dir;
        file_dir.replace(6,2,"robot");
        printf("data_dir is: %s", data_dir);
        fnameInit = true;
    }

    else if (Consumer_M.msg_type == MT_XM_START_SESSION)
    {
        printf("MT_XM_START_SESSION receieved! \n");
        MDF_XM_START_SESSION stsession;
        Consumer_M.GetData(&stsession);
        strcpy(subject_name, stsession.subject_name);
        session_num = stsession.calib_session_id;
        sprintf(file_name, "%sWAM%d.csv", subject_name, session_num);
        cout << "filename: " << file_name << endl;
        fdirInit = true;
    }

    //if (yDirectionError) { /*cout << "Y direction Error" << endl;*/ }
  }
  if (fnameInit && fdirInit){
        fname_rtma = file_dir + '/' + file_name;
        cout << "fname should be" << fname_rtma << endl;
        fname_init = true;
  }
}
