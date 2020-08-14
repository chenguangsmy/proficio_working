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
 * Author(s): Ivana Stevens 2019
 * 
 */

#include "wam_2dBalistic.h"  
//#include "/home/robot/src/Proficio_Systems/magnitude.h"
//#include "/home/robot/src/Proficio_Systems/normalize.h"
#include "/home/robot/rg2/include/RTMA_config.h"
#include <unistd.h>

#include "recordTrajectory.h"
#include "movingBurt.h"
#include "hapticsDemoClass.h"
#include "burtRTMA.h"

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


//BARRETT_UNITS_FIXED_SIZE_TYPEDEFS;
//BARRETT_UNITS_TYPEDEFS(10);

const char* remote_host = NULL;
v_type msg_tmp;
barrett::systems::ExposedOutput<v_type> message;

bool forceMet = false;
//cp_type center_pos(0.50, -0.120, 0.250);
cp_type center_pos(-0.03, -0.32, 0.6);


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


/** */
template <size_t DOF>
struct arg_struct {
  barrett::systems::Wam<DOF>& wam;
  cp_type system_center;
  RTMA_Module &mod;
  barrett::ProductManager& product_manager;
  HapticsDemo<DOF>& ball;
  arg_struct(barrett::systems::Wam<DOF>& wam,
              cp_type system_center,
              RTMA_Module &mod,
              barrett::ProductManager& product_manager,
              HapticsDemo<DOF>& ball) : wam(wam), 
                system_center(system_center), 
                mod(mod), 
                product_manager(product_manager),
                ball(ball) {}
};


/*****************************************************************************************
 * Wrapper function to make calling respoder easier, for me....mentally
 ****************************************************************************************/
template <size_t DOF> 
void * responderWrapper(void *arguments)
{
  struct arg_struct<DOF> *args = (arg_struct<DOF> *)arguments;
  respondToRTMA(args->wam, args->system_center, args->mod, args->ball);
  return NULL;
}


/*****************************************************************************************
 * Thread function to move the robot
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
  printf("Begining of the main function!\n");  //
  if (!validate_args(argc, argv)) return -1; // if not get right input, end.
  BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

  // Initializing RTMA
  RTMA_Module mod( 62, 0); //MID_BURT_ROBOT
  try
  {
    mod.DisconnectFromMMM();
    mod.ConnectToMMM((char *)"192.168.2.48:7112");
    printf("RTMA connected!\n");  //
  }
  catch( exception &e)
	{
		std::cout << "Unknown Exception!" << e.what() << std::endl;
	}

  // Subscribe to executive messages
  mod.Subscribe( MT_TASK_STATE_CONFIG );
  mod.Subscribe( MT_MOVE_HOME );
  mod.Subscribe( MT_SAMPLE_GENERATED );
  printf("Module Supscription succeed!\n");  //
  // Instantiate robot
//  std::string fname = "calibration_data/wam3/";
//  if (side == LEFT) {  // Left Config
//    fname = fname + "LeftConfig.txt";
//  } else if (side == RIGHT) {
//    fname = fname + "RightConfig.txt";
//  }
//  proficio::systems::UserGravityCompensation<DOF> user_grav_comp_(barrett::EtcPathRelative(fname).c_str());
//  user_grav_comp_.setGainZero();
//HapticsDemo<DOF> haptics_demo(wam, product_manager_, &user_grav_comp_);
  // gravity compensation
  wam.gravityCompensate();
  wam.moveTo(center_pos);
  HapticsDemo<DOF> haptics_demo(wam, product_manager_);

//  if (!haptics_demo.setupNetworking()) return 1; //for debugging haptics demo function
    if (!haptics_demo.init()) {
      printf("hptics_demo init failure!");
    return 1;
  }
//  NetworkHaptics<DOF> nh(product_manager_.getExecutionManager(), remote_host, &user_grav_comp_);
  NetworkHaptics<DOF> nh(product_manager_.getExecutionManager(), remote_host);
  message.setValue(msg_tmp);

  barrett::systems::forceConnect(message.output, nh.input);
  haptics_demo.connectForces();
  cout << "Connected Forces" << endl;

  haptics_demo.ftOn = false;

  // Spawn 2 threads for listening to RTMA and moving robot
  pthread_t rtmaThread, robotMoverThread;

  // Create thread arguments
  struct arg_struct<DOF> args(wam, center_pos, mod, product_manager_, haptics_demo);

  //Start threads
  pthread_create(&rtmaThread, NULL, &responderWrapper<DOF>, (void *)&args);
  pthread_create(&robotMoverThread, NULL, &moveRobot<DOF>, (void *)&args);

  // Wait for threads to finish
  pthread_join(rtmaThread, NULL );
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
