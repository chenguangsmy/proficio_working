/*  
 * This file start the main function to control the wam robot
 * 
 * The function start two threads, one for WAM real-time control proram runs at 500Hz. 
 * The other one is the RTMA respond thread charge with messages.
 * 
 * recoverBurt -> recovers burt to home position upon error !!!!!! check that... !!!!!
 * 
 * 
 * Author(s): Chenguang Z. 2020
 * 
 */

//#include "wam_2dBalistic.h"  
//#include "/home/robot/src/Proficio_Systems/magnitude.h"
//#include "/home/robot/src/Proficio_Systems/normalize.h"
#include "/home/robot/rg2/include/RTMA_config.h" // TODO... change to some environment variables
#include <unistd.h>

#include "recordTrajectory.h"
#include "movingBurt.h"
#include "burtRTMA.h"
#include "CustomClass.h"

#include "/home/robot/RTMA/include/RTMA.h"

#include <signal.h>  // signal, raise, sig_atomic_t
#include <string.h>
#include <pthread.h>

// Networking
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>

// Barrett Library
#include <barrett/math.h>
#include <barrett/exception.h>
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>
#include <barrett/config.h>

#include <barrett/standard_main_function.h>
#define BARRETT_SMF_VALIDATE_ARGS

typedef typename ::barrett::math::Matrix<4,4> Matrix_4x4; //self-def matrix type
typedef typename ::barrett::math::Matrix<4,1> Matrix_4x1; 
typedef typename ::barrett::math::Matrix<2,1> Matrix_2x1; 
typedef typename ::barrett::math::Matrix<3,4> Matrix_3x4; 
typedef typename ::barrett::math::Matrix<3,3> Matrix_3x3; 
typedef typename ::barrett::math::Matrix<6,3, void> Matrix_6x3xv; 
typedef typename ::barrett::math::Matrix<3,1> Matrix_3x1; 
typedef typename ::barrett::math::Matrix<6,4> Matrix_6x4; 
using namespace barrett;
using detail::waitForEnter;


const char* remote_host = NULL;
v_type msg_tmp;
barrett::systems::ExposedOutput<v_type> message;

bool forceMet = false;
bool fname_init = false; 
bool trackOutput = false; // the variable prevent repeating printf 
std::string fname_rtma;
cp_type center_pos(-0.513, 0.482, -0.0);


// end mutex 
pthread_mutex_t beLock;
// Move mutex
pthread_mutex_t rmLock;

/*****************************************************************************************
 *  Set the IP address 
 ****************************************************************************************/
bool validate_args(int argc, char** argv) {
  if (argc != 2) {
    remote_host = "127.0.0.1";  //local host
    printf("Defaulting to 127.0.0.1\n");
  } else {
    remote_host = argv[1];
  }
  return true;
}


/** send this arg_struct in processing functions (threads) */
template <size_t DOF>
struct arg_struct {
  barrett::systems::Wam<DOF>& wam;
  cp_type system_center;
  RTMA_Module &mod;
  barrett::ProductManager& product_manager;
  ControllerWarper<DOF>& cw;

  arg_struct(barrett::systems::Wam<DOF>& wam,
              cp_type system_center,
              RTMA_Module &mod,
              barrett::ProductManager& product_manager,
              ControllerWarper<DOF>& cw
              ) : wam(wam), 
                system_center(system_center), 
                mod(mod), 
                product_manager(product_manager),
                cw(cw){}
};


/*****************************************************************************************
 * Wrapper function that calls respoder
 ****************************************************************************************/
template <size_t DOF> 
void * responderWrapper(void *arguments)
{
  struct arg_struct<DOF> *args = (arg_struct<DOF> *)arguments;
  //respondToRTMA(args->wam, args->system_center, args->mod, args->product_manager, args->cw);
  respondToRTMA(args->wam, args->system_center, args->mod, args->cw);
  return NULL;
}


/*****************************************************************************************
 * Thread function to processing the robot movement
 * Useful in the old version proficio, but may not as useful in customclass by JH and CZ
 * consider to change latter -CZ
 ****************************************************************************************/
/*
template <size_t DOF>
void * moveRobot(void *arguments)
{
  struct arg_struct<DOF> *args = (arg_struct<DOF> *)arguments;
  while (false)
  { // check if wam.trackReferenceSignal is false
    if (!args->cw.isTrackRef()) // if cw.isTrackRef is false
    {
      args->cw.trackSignal();
      if (trackOutput){
        printf("isTrackRef: false, track now! \n");
        trackOutput = false;
        }
    }
    // Check if you should be moving the robot
    else
    {
      if (!trackOutput){
        printf("isTrackRef: true, continue! \n");
        trackOutput = true;
        }

    }
    bool shouldClose = false;
    if (shouldClose)
    {
      cout << "Closing thread 2" << endl;
      break;
    }
    usleep(400);
  }
  return NULL;
}
*/

/*****************************************************************************************
 * wam_main
 *
 * Run experiment in BURT robot
 ****************************************************************************************/
template <size_t DOF>
int wam_main(int argc, char** argv, barrett::ProductManager& product_manager_, barrett::systems::Wam<DOF>& wam){
  printf("Begining of the main function!\n");  
  if (!validate_args(argc, argv)) return -1; // if not get right input, end.
  BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

  // Initializing RTMA
  RTMA_Module mod( 62, 0); //MID_BURT_ROBOT
  try
  {
    mod.DisconnectFromMMM();
    mod.ConnectToMMM((char *)"192.168.2.48:7112"); //RTMA host
    printf("RTMA connected!\n");  
  }
  catch( exception &e)
	{
		std::cout << "Unknown Exception!" << e.what() << std::endl;
	}
  char * loggerfname;
  try{
    loggerfname = argv[1];
  }
  catch (exception &e){
    std::cout << "cannot assign logger fname" << e.what() <<std::endl;
  }
  char logtmpFile[] = "datatmp/bt20200904XXXXXX";
	if (mkstemp(logtmpFile) == -1) {
		printf("ERROR: Couldn't create temporary file!\n");
	return 1;
	} 

  // Subscribe to executive messages
  mod.Subscribe( MT_TASK_STATE_CONFIG );
  mod.Subscribe( MT_TRIAL_CONFIG );
  mod.Subscribe( MT_MOVE_HOME ); // ...check this? what this do? --cg
  mod.Subscribe( MT_SAMPLE_GENERATED );
  mod.Subscribe( MT_EXIT ); 
  mod.Subscribe( MT_SESSION_CONFIG );
  mod.Subscribe( MT_XM_START_SESSION );
  mod.Subscribe( MT_JUDGE_FEEDBACK); 
  mod.Subscribe( MT_WAMPERT_STATUS);  // a message describing when to start perturbation
  printf("Module Supscription succeed!\n");  //

  wam.gravityCompensate();

  //wam.moveTo(center_pos);
  // set a series of initial value for the CustomController;
  Matrix_4x4 K_q0;  // joint_stiffness,     loose
	Matrix_4x4 K_q1;  //                      stiff
  Matrix_4x4 B_q0;  
  Matrix_4x4 B_q1;  
	Matrix_3x3 K_x0;  // endpoint_stiffness,  loose
  Matrix_3x3 K_x1;  //                      stiff
  Matrix_3x3 B_x0;
  Matrix_3x3 B_x1;
  jp_type prep_q_0, prep_q_1, prep_q_2, prep_q_3;
	jp_type input_q_0;
	cp_type input_x_0;

	K_q0(0,0) = 20.0; // keep wam upright
	K_q0(1,1) = 0.0;
	K_q0(2,2) = 10.0;
	K_q0(3,3) = 0.0;
  K_q1 = K_q0; 

  B_q0 = 0.0 * K_q0;; //0.1 * K_q0;
  B_q1 = 0.0 * K_q1;; //0.1 * K_q1;

	K_x0(0,0) = 300.0;
	K_x0(1,1) = 0.0;
	K_x0(2,2) = 300.0;
  K_x1(0,0) = 300.0; //300
	K_x1(1,1) = 300.0; //300
	K_x1(2,2) = 300.0; //300

  B_x0 = 0.0 * K_x0;
  B_x1(0,0) = 0.0; //10.0; //20.0;
  B_x1(1,1) = 0.0; //10.0; //20.0;
  B_x1(2,2) = 0.0; //10.0; //20.0;

	input_q_0[0] = 0.0;
	input_q_0[1] =-1.571;
	input_q_0[2] =-1.571;
	input_q_0[3] = 1.571;
	input_x_0[0] =-0.482;
	input_x_0[1] =-0.516;
	input_x_0[2] =-0.0;

  prep_q_0[0] = 0;
  prep_q_0[1] = 1.57;
  prep_q_0[2] = 0;
  prep_q_0[3] = 1.57;
  prep_q_1[0] = 0;
  prep_q_1[1] = 0;
  prep_q_1[2] = 0;
  prep_q_1[3] = 0;
  prep_q_2[0] = -1.57;
  prep_q_2[1] = 0;
  prep_q_2[2] = 0;
  prep_q_2[3] = 0;
  prep_q_3[0] = -1.57;
  prep_q_3[1] = 0;
  prep_q_3[2] = 0;
  prep_q_3[3] = 1.57;

  //wam.moveTo(prep_q_0);
  //wam.moveTo(prep_q_1);
  //wam.moveTo(prep_q_2);
  //wam.moveTo(prep_q_3);
  ControllerWarper<DOF> cw1(product_manager_, wam, K_q0, K_q1, B_q0, B_q1, K_x0, K_x1, B_x0, B_x1, input_q_0, input_x_0); 

  if (!cw1.init()) {
    printf("hptics_demo init failure!");
  return 1;
  }

  message.setValue(msg_tmp); //.. this is confusing, what do this do?

  cw1.connectForces();
  cout << "Connected Forces" << endl;


  // Spawn 2 threads for listening to RTMA and moving robot
  pthread_t rtmaThread, robotMoverThread;

  // Create thread arguments
  struct arg_struct<DOF> args(wam, center_pos, mod, product_manager_, cw1);

  //Start threads
  pthread_create(&rtmaThread, NULL, &responderWrapper<DOF>, (void *)&args);
  printf("RTMA thread created! \n");
  //pthread_create(&robotMoverThread, NULL, &moveRobot<DOF>, (void *)&args);
  //printf("robotMoverThread created! \n");
  // Wait for threads to finish
  pthread_join(rtmaThread, NULL );
  printf("The RTMA thread joined! \n");
  // pthread_join(robotMoverThread, NULL );
  cout << "Finished trials" << endl;
  barrett::btsleep(0.1);
  
  //wam.moveTo(prep_q_3);
  //wam.moveTo(prep_q_2);
  //wam.moveTo(prep_q_1);
  //wam.moveTo(prep_q_0);
  wam.moveHome();

  mod.DisconnectFromMMM();
  product_manager_.getPuck(1)->setProperty(product_manager_.getPuck(1)->getBus(), 1, 8, 3);
  product_manager_.getPuck(2)->setProperty(product_manager_.getPuck(2)->getBus(), 2, 8, 3);
  product_manager_.getPuck(3)->setProperty(product_manager_.getPuck(3)->getBus(), 3, 8, 3);
  barrett::systems::disconnect(wam.input);



  printf("\n\n");
  return 0;
}
