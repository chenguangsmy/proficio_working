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


BARRETT_UNITS_FIXED_SIZE_TYPEDEFS;
BARRETT_UNITS_TYPEDEFS(6);


extern bool forceMet; //thresholdMet;
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

/**********************************************************************
 * Thread funtion to handle messages from RTMA 
 *********************************************************************/

template <size_t DOF>
void respondToRTMA(barrett::systems::Wam<DOF>& wam,
              cp_type system_center,
              RTMA_Module &mod,
//              barrett::ProductManager& product_manager,
              ControllerWarper<DOF> &cw)
{ 
  cf_type cforce;
  bool read_rlt = false;
  
  bool freeMoving = false; 
  bool sendData = true;
  bool ifPert = false;        // only perturb at certain trials
  cp_type cp;
  cv_type cv;
  jp_type jp;
  jv_type jv;
  jt_type jt;
  double taskJ_center[4];     // task send joint position     ------------cg: will it change across trials? if not, why not delete it???
  double  tleading, tlasting; // saving hardware time for time alignment
  int     task_state = 0;
  int     target_dir = 0;
  int     num_pressEnd = 0;
  string session_num_s;
  cp_type robot_center(system_center);
  cp_type target;
	CMessage Consumer_M;
  

  //LoggerClass<DOF> *log1;
//  LoggerClass<DOF> log1(product_manager, wam, loggerfname, logtmpFile, cw);

  // variables for save filename 
  char data_dir[MAX_DATA_DIR_LEN];
  std::string file_dir;
  char file_name[20];
  char subject_name[TAG_LENGTH];
  int session_num; 
  int pert_type;                            // perturbation type: 1-pulse, 2-stoc, 3-slowRamp, 4-square, 5-pulseNoRelease 
  bool readyToMove_nosent;
  bool sync_time_flag = false;
  double pert_big = -15;                    // pert magnitude
  double force_thresh = 15;                 // use target force to calculate robot nominal position
  double robot_x0 = 0;                      // 
  parportinit();
  while (true)  
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
      
      burt_status_data.RDT = cw.jj.get_rdt(); // ReaDTime for synchrony
      
      /*
      burt_status_data.force_x = cforce[1];
      burt_status_data.force_y = cforce[0];
      burt_status_data.force_z = cforce[2];
      */ //This force is useless
      
      // Get Joint position, velocity and toque
      burt_status_data.jt_1 = wam.getJointTorques()[0]; // loop does not work here, write one by one
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

      cw.jj.gettime(&(burt_status_data.wamt),  &(burt_status_data.erniet)); // save time in message
      
      // Send Message
      CMessage M( MT_BURT_STATUS );
      M.SetData( &burt_status_data, sizeof(burt_status_data) );
      mod.SendMessage( &M );
    }

    // According to TS_config setting perturbation - related variables 
    if (Consumer_M.msg_type == MT_TASK_STATE_CONFIG)
    {
      MDF_TASK_STATE_CONFIG task_state_data;
      Consumer_M.GetData( &task_state_data);
      freeMoving = false;
      
      task_state = task_state_data.id;
      cw.jj.setTaskState( task_state);
      printf(" ST %d, ", task_state_data.id);
      switch(task_state_data.id)
      {
        case 1:   // Initialize, set center and make the hardware pulse
          // make hardware pulse 
          tleading = GetAbsTime(); 
          ioctl(fd, PPWDATA, &dataL);     // change the parports here
          tlasting = GetAbsTime(); 
          sync_time_flag = true;
          
          // task-related staff:
          barrett::btsleep(0.2);          // allow the Netbox have calibrated the net force. ------------cg: rethink the time---------------
          freeMoving = true;
          sendData = false;
          // target: XYZ-IJK-0123456789, refer to `executive module`
          robot_center[0] = task_state_data.target[30]; // use this to calculate robot x0
          robot_center[1] = task_state_data.target[31];
          robot_center[2] = task_state_data.target[32];

          force_thresh = task_state_data.target[7]; 
          
          taskJ_center[0] = task_state_data.target[8];
          taskJ_center[1] = task_state_data.target[9];
          taskJ_center[2] = task_state_data.target[10];
          taskJ_center[3] = task_state_data.target[11];
          
          printf("force for this target: %f N \n", force_thresh);
          robot_x0 = force_thresh/task_state_data.wamKp;
          target_dir = task_state_data.target[4];
          printf("\n Direction: %d, force: %f\n\n", target_dir, force_thresh); 
          pert_type = int(task_state_data.ifpert);

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

          cw.setK1(task_state_data.wamKp);  // the value at hold ---> THINK CHANGE IT INTO A MORE SMOOTH ONE
          cw.setB1(task_state_data.wamBp);
          break;

        case 2: // Present
          ioctl(fd, PPWDATA, &dataH);     //set pulse back
          
          sendData = true;
          
          // set robot x0  
          cw.jj.setx0Gradual(robot_center);
          break;

        case 3: //ForceRamp
          break;

        case 4: //ForceHold, perturbation only in this state
          ifPert = int(task_state_data.ifpert) == 0 ? false : true;
          cw.jj.setPertType( int(task_state_data.ifpert));  // let JJ know

          if (ifPert){ // should perturb
            switch (int (task_state_data.ifpert))
            {
              case 1: // pulse
              case 3:
              case 4:
              case 5:
              cw.jj.setPertMag(pert_big);                             // set mag 
              cw.jj.disablePertCount();                               // set the pulse 
              break;
            
              case 2: // stoc
              cw.jj.presetpretAmp(pert_big);
              // only wait for trigger
              break;
            }
          }
          else{
            cw.jj.resetpretAmp();
            cw.jj.setPertMag(0.0);
          }
          break;

        case 5: //Move
          cw.jj.resetpretAmp();
          cw.jj.setPertMag(0.0);
          if (pert_type != 5){
            cw.setForceMet(true); // save the release in the buffer, wait finish pert to relese
          }
          break;

        case 6: // hold
          break;

        case 7:
          sendData  = false;
          cw.jj.resetpretAmp(); // in case of trial stop at state 4
          ioctl(fd, PPWDATA, &dataH);     //set pulse back
          break;

        case 8:
          cw.jj.setx0(robot_center);
          cw.setForceMet(false);
          cw.jj.disablePertCount(); // avoid perturbation at this time
          cw.jj.setPertMag(0);
          freeMoving = true;
          cw.moveToq0(); //make sure on the right joint position
          break;

        default:
          break;
      }      
    }

    // Have Burt move in steps toward home postion
    else if (Consumer_M.msg_type == MT_MOVE_HOME && freeMoving)
    {
      MDF_MOVE_HOME startButton;
      Consumer_M.GetData( &startButton); 
      cout<<"move to: "<< robot_center[0] << "; "<< robot_center[1] << "; "<< robot_center[2] <<endl; // testing
      moveToCenter(wam, robot_center, mod);  // do not fully delete this part! Msg contain! 
      
      cw.trackSignal(); //do I need this code? try ----------------------------cg. 

    }

    // Ping sent Acknowlegde ping...
    else if (Consumer_M.msg_type == MT_PING) 
    {
      cw.init();
    }

    // Exit the function
    else if (read_rlt & Consumer_M.msg_type == MT_EXIT) 
    { // add finish recording here
      num_pressEnd = num_pressEnd + 1;
      if(num_pressEnd<=5) { // just end session
       cw.jj.moveAway();
//       log1.datalogger_end();
       //log1->datalogger_end();
       cw.closeLogger();
       barrett::btsleep(0.2);
       //delete log1;
      }
      
      if (num_pressEnd>5) { // truly exit
        printf("Hit 5 Ends, True Stop!\n");
        parportclose();
        break;
      }
    }
  
    else if (Consumer_M.msg_type == MT_SESSION_CONFIG)
    {
        printf("MT_SESSION_CONFIG. \n");
        MDF_SESSION_CONFIG ssconfig;
        Consumer_M.GetData(&ssconfig);
        strcpy(data_dir, ssconfig.data_dir);
        file_dir = data_dir;
        string fdir(data_dir);
        session_num_s = fdir.substr(fdir.size()-5);
        session_num = atoi(session_num_s.c_str());
        file_dir.replace(6,2,"robot");
        printf("data_dir is: %s", data_dir);
        sprintf(file_name, "datatmp/%sWAM%05d.csv", subject_name, session_num);
        //cw.log1->tmpFileName = file_name;
        cw.log1->setTmpFileName(file_name);
        cout << "filename: " << cw.log1->tmpFileName << endl;
        
        cw.setForceMet(false);
        sprintf(file_name, "%sWAM%05d.csv", subject_name, session_num);
        fname_rtma = file_dir + '/' + file_name;
        cout << "fname should be" << fname_rtma << endl;
        fname_init = true;
    }

    else if (Consumer_M.msg_type == MT_XM_START_SESSION)
    {
       /*
        log1 = new LoggerClass<DOF>(product_manager, wam, loggerfname, logtmpFile, cw);
        log1->datalogger_connect1();
        log1->datalogger_connect2();
        log1->datalogger_start();
        */
       cw.setupLogger();
        num_pressEnd = 0; // reset
//        log1.datalogger_connect1();
//        log1.datalogger_connect2();
//        log1.datalogger_start();
        
        printf("MT_XM_START_SESSION receieved! \n");
        MDF_XM_START_SESSION stsession;
        Consumer_M.GetData(&stsession);
        strcpy(subject_name, stsession.subject_name);

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
        cw.jj.enablePertCount();    // start perturbation immediately
        readyToMove_nosent = true;
        cw.jj.setpretAmp();         // used in stochastic pert
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

  if (cw.jj.getPertFinish() && readyToMove_nosent) // finished the perturbation 
  {
    readyToMove(wam, robot_center, mod);    // boardcast readyToMove so that the `GatingForceJudge` knows
    readyToMove_nosent = false;             // have sent, hence no longer send the message.
  }
  }
}
