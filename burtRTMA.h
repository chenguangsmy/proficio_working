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
//#include "/home/robot/rg2/include/ipdProj_config.h"
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

// parport
#include <sys/ioctl.h>
#include <linux/parport.h>
#include <linux/ppdev.h>
#include <fcntl.h>
#include <stdlib.h>

//#include <proficio/systems/utilities.h>

BARRETT_UNITS_FIXED_SIZE_TYPEDEFS;
BARRETT_UNITS_TYPEDEFS(6);


//extern bool yDirectionError;
extern bool forceMet; //thresholdMet;
//extern double forceThreshold;
extern std::string fname_rtma;
extern bool fname_init; 

int ping_times = 5;
int trial_no = 0;

// for the sending hardware signals 
char dataL = 0x00;
char dataH = 0xFF;
int fd;
int running = 1;
void signalHandler(int sig){
    running = 0;
}
void parportinit(){
    printf("Parallel Port Interface Init\n");
    signal(SIGINT, signalHandler);

    // Open the parallel port for reading and writting
    fd = open("/dev/parport0", O_RDWR);

    if (fd == -1)
    {
      perror("Could not open parallel port");
    }

    // Try to claim port
    if (ioctl(fd, PPCLAIM, NULL)){
      perror("Could not claim parallel port");
      close(fd);
    }

    // Set the Mode
    int mode = IEEE1284_MODE_BYTE;
    if (ioctl(fd, PPSETMODE, &mode)){
      perror("Could not set mode");
      ioctl(fd, PPRELEASE);
      close(fd);
    }

    int dir = 0x00;
    if (ioctl(fd, PPDATADIR, &dir)){
      perror("Could not set parallel port direction");
      ioctl(fd, PPRELEASE);
      close(fd);
    }
}
void parportclose(){
  ioctl(fd, PPRELEASE);
  close(fd);
}

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
  bool ifPert = false;    // only perturb at certain trials
//  int pert_time = 0;    // don't need any more after use GatingForceJudge to trigger the pert
  cp_type cp;
  cv_type cv;
  jp_type jp;
  jv_type jv;
  jt_type jt;
  double taskJ_center[4]; // task send joint position
  double  tleading, tlasting;  // serve as a temp save for time
  double *twam, *ternie;
  int     trial_count = 0;    // as a marker for counting which trial perturb
  int     readyToMoveIter = 0;    // mark as send ready to move tiems
  int     task_state = 0;
  int     target_dir = 0;
  cp_type monkey_center(system_center);
  cp_type robot_center(system_center);
  cp_type perturbed_center(system_center);  // the perturbation position (at perturbed)
  cp_type target;
	CMessage Consumer_M;

  // variables for save filename 
  char data_dir[MAX_DATA_DIR_LEN];
  std::string file_dir;
  char file_name[20];
  char subject_name[TAG_LENGTH];
  int session_num; 
  bool readyToMove_nosent;
  bool sync_time_flag = false;
  double pert_small = 5; //5N
  double pert_big = -15;   //25N
  double force_thresh = 15; // force_tar in ballistic release
  double robot_x0 = 0; // should be: robot_x0 = force_tar/300; //(300 as the robot stiffness)
  parportinit();
  while (true)  // Allow the user to stop and resume with pendant buttons
  {
    // Read Messages
    read_rlt = mod.ReadMessage( &Consumer_M, 0.1);

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
      // Get Force Data //.. actually not wrote cforce. (only command)
      burt_status_data.force_x = cforce[1];
      burt_status_data.force_y = cforce[0];
      burt_status_data.force_z = cforce[2];
      

      // Get Joint position, velocity and toque
      burt_status_data.jt_1 = wam.getJointTorques()[0]; // for loop does not work here, write one by one
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
      
      // Get Position Data 
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


      // ... TODO... GET BOTH ROBOT TIME AND COMPUTER TIME
      //printf("before gettime \n");
      //cw.jj.gettime(twam, ternie);
      cw.jj.gettime(&(burt_status_data.wamt),  &(burt_status_data.erniet));
      //printf("after gettime \n");
      // alternative:
      //burt_status_data.wamt   = *twam;
      //burt_status_data.erniet = *ternie;
      //printf("wam_t is: %f, ernie_t is :%f", burt_status_data.wamt, burt_status_data.erniet); 

      // Send Message
      CMessage M( MT_BURT_STATUS );
      M.SetData( &burt_status_data, sizeof(burt_status_data) );
      mod.SendMessage( &M );
      // printf("M data has been sent! \n"); // debug code 
    }

    // Task State config, set stiffness and zero-force position
    if (Consumer_M.msg_type == MT_TASK_STATE_CONFIG)
    {
      MDF_TASK_STATE_CONFIG task_state_data;
      Consumer_M.GetData( &task_state_data);
      //cout << "task id : " << task_state_data.id << "pert_time:" <<task_state_data.pert_time<<endl;
      freeMoving = false;
      
      //cw.jj.setTaskState(task_state_data.id);
      task_state = task_state_data.id;
      switch(task_state_data.id)
      {
        case 1:   // set joint center and endpoint center
          cw.jj.setTaskState(1);
          cout << " ST 1, ";
          // ...TODO... RECORD TIME ONCE, GIVE OUT THE POSITIVE PULSE, RECORD AGAIN
          tleading = GetAbsTime(); 
          // change the bits here
          ioctl(fd, PPWDATA, &dataH);
          tlasting = GetAbsTime(); 
          sync_time_flag = true;
          // MAKE THE SENT_FLAG 0;
          barrett::btsleep(0.2); // the allocated time is to make sure the Netbox have calibrated the net force. 
          freeMoving = true;
          sendData = false;
          readyToMoveIter = 0;
          // target: XYZ-IJK-0123456789
          robot_center[0] = task_state_data.target[0]; // here we temperarily change to a const value, for testing
          robot_center[1] = task_state_data.target[1];
          robot_center[2] = task_state_data.target[2];

          force_thresh = task_state_data.target[7]; 
          
          taskJ_center[0] = task_state_data.target[8];
          taskJ_center[1] = task_state_data.target[9];
          taskJ_center[2] = task_state_data.target[10];
          taskJ_center[3] = task_state_data.target[11];

          monkey_center[0] = task_state_data.target[30]; // here we temperarily change to a const value, for tesging
          monkey_center[1] = task_state_data.target[31];
          monkey_center[2] = task_state_data.target[32];  
          
          printf("force for this target: %f N \n", force_thresh);
          robot_x0 = force_thresh/task_state_data.wamKp;
          switch(target_dir)
          {
              case 0: // right 
                robot_center[1] += robot_x0;
                break;
              case 2: // front
                robot_center[1] -= robot_x0;
                break;
              case 4: // left 
                robot_center[1] -= robot_x0;
                break;
              case 6: 
                robot_center[1] += robot_x0;
                break;
          }
          switch(target_dir)
          {
              case 0:
              case 6:
                pert_big = -task_state_data.pert_mag;
              break;
              case 2:
              case 4:
                pert_big = task_state_data.pert_mag;
              break;
          }
          pert_big = pert_big;

          //pert_small = -pert_small;
          //pert_big = -pert_big;
          //cout << " case 1 Target : " << target[0] << "," << target[1] << "," << target[2] << endl;
          //cw.setCenter_joint(taskJ_center);
          //cw.setCenter_endpoint(monkey_center);
          ifPert = int(task_state_data.ifpert) == 0 ? false : true;
          cw.jj.setPulsePert( int(task_state_data.ifpert) == 1); // if task_state_data_ifpert ~=1, stoc pert
          cw.jj.disablePertCount();
          //pert_time = int(double(task_state_data.pert_time) * 500.0); // to double
          //printf("task_state_data.ifpert is: %d, pert_time: %d\n", ifPert, pert_time);
          readyToMove_nosent = true;
          cw.setK1(task_state_data.wamKp);
          cw.setB1(task_state_data.wamBp);
          
          break;
        case 2: // Present
          cw.jj.setTaskState(2);
          //...TODO... SET THE PULSE NEGATIVE 
          ioctl(fd, PPWDATA, &dataL);
          //cw.jj.setPertTime(pert_time);
          cw.jj.setFoffset(-0.0);
          sendData = true;
          cout << " ST 2, ";
          // set input x0  
          cw.jj.setx0Gradual(robot_center);
          
          
          break;
        case 3: //ForceRamp
          cw.jj.setTaskState(3);
          // stiff the Wam and wait for perturb, 
          // after the perturbation, send messages to the GatingForceJudge. 
          cout << " ST 3, ";
          target_dir = task_state_data.target[4];
          
          printf("\n Direction: %d, force: %f\n\n", target_dir, force_thresh); 
          if (ifPert){ // should perturb
            cw.jj.setPertMag(pert_big); //
            cw.jj.setPertPositionMag(0); //cw.jj.setPertPositionMag(task_state_data.pertdx0_mag); // only when position perturb
            perturbed_center = monkey_center;
            perturbed_center[1] += task_state_data.pertdx0_mag;
            //wam.moveTo(perturbed_center, false, task_state_data.pert_mag/5.0); // what about the velocity?  
            //wam.moveTo(perturbed_center); // what about the velocity?  
            //btsleep(5);
            //wam.moveTo(monkey_center);
            // a direct moveto command
          }
          else{
            cw.jj.resetpretAmp();
            cw.jj.setPertMag(0.0);
          }
          
          break;
        case 4: //Move
          cw.jj.setTaskState(4);
          cout << " ST 4, ";
          cw.jj.resetpretAmp();
          cw.jj.setPertMag(0.0);
          readyToMove_nosent = false;  // have sent, hence no longer send the message.
          cw.setForceMet(true); // save the release in the buffer, wait finish pert to relese
          
          cw.jj.updateImpedanceWait();
          cw.jj.setFoffset(0.0);
          break;
        case 5: // hold
          cout << " ST 5, ";
          cw.jj.setTaskState(5);
          break;
        case 6:
          cw.jj.setTaskState(6);
          cout << " ST 6, ";
          cw.jj.resetpretAmp();
          cw.jj.setUpdateJaccobian(true);
          sendData  = false;
          
          break;
        case 7:
          cw.jj.setTaskState(7);
          cout << " ST 7, " << endl;
          cw.jj.setx0(robot_center);
          cw.jj.setPertMag(0);
          cw.setForceMet(false);
          cw.jj.disablePertCount(); // avoid perturbation at this time
          freeMoving = true;
          cw.moveToq0(); //make sure on the right joint position
          break;
        default:
          break;
      }      
      //printf("ST: %d, ", task_state_data.id);
      ///printf("task_state_data.ifpertRcv: %d, pert_time: %f, release_time: %f \n", int(task_state_data.ifpert), double(task_state_data.pert_time), double(task_state_data.release_time));
      //printf("task_state_data.ifpertRcv: %f, pert_time: %d, release_time: %d \n", double(task_state_data.ifpert), int(task_state_data.pert_time), int(task_state_data.release_time));
    }

    // Have Burt move in steps toward home postion
    else if (Consumer_M.msg_type == MT_MOVE_HOME && freeMoving)
    {
      MDF_MOVE_HOME startButton;
      Consumer_M.GetData( &startButton); 
      cout<<"move to: "<< monkey_center[0] << "; "<< monkey_center[1] << "; "<< monkey_center[2] <<endl; // testing
      moveToCenter(wam, robot_center, mod);  // do not fully delete this part! Msg contain! 
      // re-track force output here?  
      cw.trackSignal(); //maybe not needed as idle no longer exist. 

    }

    // Ping sent   Acknowlegde ping...
    else if (Consumer_M.msg_type == MT_PING) 
    {
      cw.init();
    }

    // Exit the function
    else if (Consumer_M.msg_type == MT_EXIT) 
    { // add finish recording here

      wam.moveHome();
      wam.idle();
      parportclose();
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
        cw.setForceMet(false);
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

    else if(Consumer_M.msg_type == MT_JUDGE_FEEDBACK)
    {
      // scan the Force Feedback, nothing to do here actually:
        MDF_JUDGE_FEEDBACK frc_fb; 
        Consumer_M.GetData(&frc_fb);
    }

    else if(Consumer_M.msg_type == MT_WAMPERT_STATUS)
    {
      MDF_WAMPERT_STATUS pert_sat;
      Consumer_M.GetData(&pert_sat);

      if (ifPert && pert_sat.perturb_start){
        cw.jj.enablePertCount();
        cw.jj.setPertTime(0); // start perturbation immediately
        cw.jj.setpretAmp(); // used in stochastic pert
      }
    }

    else if(Consumer_M.msg_type == MT_TRIAL_CONFIG){
      MDF_TRIAL_CONFIG trial_conf;
      Consumer_M.GetData(&trial_conf);
      trial_no = trial_conf.trial_no;
    }
  if (sync_time_flag){
    // pack Message
    MDF_TIME_SYNC sync_time_data;
    sync_time_data.tleading = tleading;
    sync_time_data.tlasting = tlasting;
    sync_time_data.trial_no = trial_no; 
    // Send Message
    CMessage M_timer( MT_TIME_SYNC );
    sync_time_flag = false;
    M_timer.SetData( &sync_time_data, sizeof(sync_time_data) );
    mod.SendMessage( &M_timer );
    printf("time data has been sent! \n"); // debug code 

  }

  if (cw.jj.getPertFinish() && readyToMove_nosent && readyToMoveIter<5) // finished the perturbation 
  {
    readyToMove(wam, robot_center, mod);   // boardcast readyToMove so that the `GatingForceJudge` knows
    readyToMoveIter++;
  }
  }
  if (fnameInit && fdirInit)
  {
        fname_rtma = file_dir + '/' + file_name;
        cout << "fname should be" << fname_rtma << endl;
        fname_init = true;
  }
  
}
