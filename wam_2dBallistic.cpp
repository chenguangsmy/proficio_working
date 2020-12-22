/*  
 * This file includes code to control asynchronous control of the robot
 * 
 * included are:
 * 
 * recoverBurt -> recovers burt to home position upon error
 * 
 * movetoCenter -> moves the burt to a predefined center postion
 * 
 * moveAstep -> moves the burt one step toward a position
 * 
 * Author(s): Ivana Stevens 2019, Chenguang Z. 2020
 * 
 */

//#include "wam_2dBalistic.h"  
//#include "/home/robot/src/Proficio_Systems/magnitude.h"
//#include "/home/robot/src/Proficio_Systems/normalize.h"
#include "/home/robot/rg2/include/RTMA_config.h"
#include <unistd.h>

#include "recordTrajectory.h"
#include "movingBurt.h"
// #include "hapticsDemoClass.h"
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

//#include <proficio/systems/utilities.h>
#include <barrett/standard_main_function.h>
#define BARRETT_SMF_VALIDATE_ARGS

//#include <proficio/standard_proficio_main.h>
typedef typename ::barrett::math::Matrix<4,4> Matrix_4x4; //self-def matrix type
typedef typename ::barrett::math::Matrix<4,1> Matrix_4x1; //self-def matrix type
typedef typename ::barrett::math::Matrix<2,1> Matrix_2x1; //self-def matrix type
typedef typename ::barrett::math::Matrix<3,4> Matrix_3x4; //self-def matrix type
typedef typename ::barrett::math::Matrix<3,3> Matrix_3x3; //self-def matrix type
typedef typename ::barrett::math::Matrix<6,3, void> Matrix_6x3xv; //self-def matrix type
typedef typename ::barrett::math::Matrix<3,1> Matrix_3x1; //self-def matrix type
typedef typename ::barrett::math::Matrix<6,4> Matrix_6x4; //self-def matrix type

using namespace barrett;
using detail::waitForEnter;

//BARRETT_UNITS_FIXED_SIZE_TYPEDEFS;
//BARRETT_UNITS_TYPEDEFS(10);

const char* remote_host = NULL;
v_type msg_tmp;
barrett::systems::ExposedOutput<v_type> message;

bool forceMet = false;
//const jp_type center_pos1(-1.5, 0, 0, 1.5);
cp_type center_pos(-0.448, -0.418, 0.010);


// end mutex
pthread_mutex_t beLock;
// Move mutex
pthread_mutex_t rmLock;

/*****************************************************************************************
 *  Set the IP address 
 ****************************************************************************************/
bool validate_args(int argc, char** argv) {
  if (argc != 2) {
    remote_host = "127.0.0.1";  //actually this IP is local host
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
              ControllerWarper<DOF>& cw) : wam(wam), 
                system_center(system_center), 
                mod(mod), 
                product_manager(product_manager),
                cw(cw) {}
};


/*****************************************************************************************
 * Wrapper function that calls respoder
 ****************************************************************************************/
template <size_t DOF> 
void * responderWrapper(void *arguments)
{
  struct arg_struct<DOF> *args = (arg_struct<DOF> *)arguments;
  respondToRTMA(args->wam, args->system_center, args->mod, args->cw);
  return NULL;
}


/*****************************************************************************************
 * Thread function to processing the robot movement
 * Useful in the old version proficio, but may not as useful in customclass by JH and CZ
 * consider to change latter -CZ
 ****************************************************************************************/
template <size_t DOF>
void * moveRobot(void *arguments)
{
  struct arg_struct<DOF> *args = (arg_struct<DOF> *)arguments;
  while (true)
  {
    if (false)
    {
      replayTrajectory(args->product_manager, args->wam);
      moveToCenter(args->wam, args->system_center, args->mod);
    }
    // Check if you should be moving the robot
    else if (true)
    {
      //moveToCenter(args->wam, args->system_center, args->mod);
      //moveAStep(args->wam, args->system_center, args->mod);
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

  // Subscribe to executive messages
  mod.Subscribe( MT_TASK_STATE_CONFIG );
  mod.Subscribe( MT_MOVE_HOME ); // ...check this? what this do? --cg
  mod.Subscribe( MT_SAMPLE_GENERATED );
  printf("Module Supscription succeed!\n");  //

  wam.gravityCompensate();
  // wam.moveTo(center_pos);
  // set a series of initial value for the CustomController;
  Matrix_4x4 K_q00; //... initialize these set of variables from RTMA system --cg
	Matrix_4x4 K_q01;
	Matrix_3x3 K_x00;
  Matrix_3x3 K_x01;
	jp_type input_q_000;
	cp_type input_x_000;

	K_q00(0,0) = 10;
	K_q00(1,1) = 10;
	K_q00(2,2) = 10;
	K_q00(3,3) = 20;
	K_x00(0,0) = 1000;
	K_x00(1,1) = 1000;
	K_x00(2,2) = 1000;

  K_x01(0,0) = 10;
	K_x01(1,1) = 10;
	K_x01(2,2) = 10;

	K_q01(0,0) = 10;
	K_q01(1,1) = 10;
	K_q01(2,2) = 10;
	K_q01(3,3) = 0;

	input_q_000[0] =-1.581;
	input_q_000[1] =-0.035;
	input_q_000[2] =-0.034;
	input_q_000[3] = 1.521;
	input_x_000[0] =-0.448;
	input_x_000[1] =-0.418;
	input_x_000[2] = 0.010;

  ControllerWarper<DOF> cw1(product_manager_, wam, K_q00, K_x00, K_x01, input_q_000,input_x_000); 

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
  pthread_create(&robotMoverThread, NULL, &moveRobot<DOF>, (void *)&args);
  printf("robotMoverThread created! \n");
  // Wait for threads to finish
  pthread_join(rtmaThread, NULL );
  printf("The RTMA thread joined! \n");
  // pthread_join(robotMoverThread, NULL );

  cout << "Finished trials" << endl;
  barrett::btsleep(0.1);

  mod.DisconnectFromMMM();
  product_manager_.getPuck(1)->setProperty(product_manager_.getPuck(1)->getBus(), 1, 8, 3);
  product_manager_.getPuck(2)->setProperty(product_manager_.getPuck(2)->getBus(), 2, 8, 3);
  product_manager_.getPuck(3)->setProperty(product_manager_.getPuck(3)->getBus(), 3, 8, 3);
  barrett::systems::disconnect(wam.input);

  wam.moveHome();
  printf("\n\n");
  return 0;
}
