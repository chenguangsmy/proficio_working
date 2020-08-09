/*  
 * Code to run Balistic Force Trials on the Proficio Robot
 * 
 * Author(s): Henry Friedlander 2017, Ivana Stevens 2018
 * 
 */
 
#include "proficio_2dBalistic.h"  
#include "/home/robot/src/Proficio_Systems/magnitude.h"
#include "/home/robot/src/Proficio_Systems/normalize.h"
#include "/home/robot/rg2/include/RTMA_config.h"
#include <unistd.h>

#include "recordTrajectory.h"
#include "movingBurt.h"
#include "ballisticForce.h"
#include "ballisticPlane.h"
#include "burtRTMA.h"

#include <RTMA/RTMA.h>

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
#include <proficio/systems/utilities.h>
#include <barrett/products/product_manager.h>

// Networking
#include <netinet/in.h>
#include <sys/types.h>

#include <proficio/systems/utilities.h>
#define BARRETT_SMF_VALIDATE_ARGS
//#define NO_CONTROL_PENDANT

#include <proficio/standard_proficio_main.h>    // NOLINT(build/include_order)

#define PI 3.14159265359

//using detail::waitForEnter;

BARRETT_UNITS_FIXED_SIZE_TYPEDEFS;
BARRETT_UNITS_TYPEDEFS(6);  // defines v_type to have length 6

const char* remoteHost = NULL;

// EDIT FOR VIBRATIONS
double kpLine = 3e3;
double kdLine = 4e1;

bool game_exit = false;
static bool burt_exit = false;
static bool robotMove = false;

// end mutex
pthread_mutex_t beLock;
// Move mutex
pthread_mutex_t rmLock;

bool thresholdMet = false;
bool yDirectionError = false;
double forceThreshold = 10;

  
bool taskComplete = false;
bool taskSuccess = false;
bool hasError = false;
bool shouldMove = false; //TODO: THIS SHOULD BE FALSE


/** 
 * validate_args
 */
bool validate_args(int argc, char** argv) {
  switch (argc) {
    case 4:
      kdLine = atof(argv[3]);
    case 3:
      kpLine = atof(argv[2]);
    case 2:
      remoteHost = argv[1];
      break;
    default:
      remoteHost = "127.0.0.1";
      printf("Defaulting to 127.0.0.1\n");
  }
  printf("Gains: kp = %f; kd = %f\n", kpLine, kdLine);
  return true;
}


namespace cube_sphere {
  /** 
   * exit_program_callback
   *
   * When killed from outside (by GUI), this allows a graceful exit.
   */
  void exit_program_callback(int signum) { game_exit = true; }
}  // namespace cube_sphere


/*****************************************************************************************
 * scale
 ****************************************************************************************/
cf_type scale(boost::tuple<cf_type, double> t) {
  return t.get<0>() * t.get<1>();
}


template <size_t DOF>
struct arg_struct {
  barrett::systems::Wam<DOF>& wam;
  cp_type system_center;
  RTMA_Module &mod;
  barrett::ProductManager& product_manager;
  barrett::systems::BalisticForce& ball;
  arg_struct(barrett::systems::Wam<DOF>& wam,
              cp_type system_center,
              RTMA_Module &mod,
              barrett::ProductManager& product_manager,
              barrett::systems::BalisticForce& ball) : wam(wam), 
                system_center(system_center), 
                mod(mod), 
                product_manager(product_manager),
                ball(ball) {}
};


/**
 * Wrapper function to make calling respoder easier, for me....mentally
 */
template <size_t DOF> 
void * responderWrapper(void *arguments)
{
  struct arg_struct<DOF> *args = (arg_struct<DOF> *)arguments;
  respondToRTMA(args->wam, args->system_center, args->mod, args->ball);
  return NULL;
}

/**
 * Thread function to move the robot
 * */
template <size_t DOF>
void * moveRobot(void *arguments)
{
  struct arg_struct<DOF> *args = (arg_struct<DOF> *)arguments;
  while (true)
  {
    if (yDirectionError)
    {
      replayTrajectory(args->product_manager, args->wam);
      yDirectionError = false;
      moveToCenter(args->wam, args->system_center, args->mod);
    }
    // Check if you should be moving the robot
    else if (robotMove)
    {
      //moveToCenter(args->wam, args->system_center, args->mod);
      //moveAStep(args->wam, args->system_center, args->mod);
    }
    bool shouldClose = burt_exit;
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
 * proficio_main
 *
 * Run experiment in BURT robot
 ****************************************************************************************/
template <size_t DOF>
int proficio_main(int argc, char** argv,
                  barrett::ProductManager& product_manager,  // NOLINT
                  barrett::systems::Wam<DOF>& wam,           // NOLINT
                  const Config& side) {  
  BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

  // Initializing RTMA
  RTMA_Module mod( 62, 0); //MID_BURT_ROBOT
  try
  {
    
    mod.DisconnectFromMMM();
    mod.ConnectToMMM((char *)"192.168.2.48:7112");
  }
  catch( exception &e)
	{
		std::cout << "Unknown Exception!" << e.what() << std::endl;
	}

  // Subscribe to executive messages
  mod.Subscribe( MT_TASK_STATE_CONFIG );
  mod.Subscribe( MT_MOVE_HOME );
  mod.Subscribe( MT_SAMPLE_GENERATED );
  //mod.Subscribe( MT_PING );
  //mod.Subscribe( MT_EXIT );

  // Instantiate Proficio
  //const cp_type system_center(0.450, -0.120, 0.250);
  const cp_type system_center(0.350, -0.120, 0.250);

  //======================================================================================
  wam.gravityCompensate();
  std::srand(time(NULL)); //initialize the random seed
  barrett::SafetyModule* safety_module = product_manager.getSafetyModule();
  barrett::SafetyModule::PendantState ps;
  safety_module->getPendantState(&ps);

  std::string filename = "calibration_data/wam3/";
  if (side == LEFT) {
    filename = filename + "LeftConfig.txt";
  } else if (side == RIGHT) {
    filename = filename + "RightConfig.txt";
  }

  // Catch kill signals if possible for a graceful exit.
  signal(SIGINT, cube_sphere::exit_program_callback);
  signal(SIGTERM, cube_sphere::exit_program_callback);
  signal(SIGKILL, cube_sphere::exit_program_callback);

  proficio::systems::UserGravityCompensation<DOF> gravity_comp(
      barrett::EtcPathRelative(filename).c_str());
  gravity_comp.setGainZero();

  // Instantiate Systems
  NetworkHaptics<DOF> network_haptics(product_manager.getExecutionManager(),
                                      remoteHost, &gravity_comp);

//  const cp_type ball_center(0.4, -0.15, 0.05);
  const cp_type ball_center(0.3, -0.15, 0.05);
  wam.moveTo(system_center);
  printf("Done Moving Arm! \n");
  // const cp_type box_center(0.35, 0.2, 0.0);
  const barrett::math::Vector<3>::type box_size(0.2, 0.2, 0.2);

  barrett::systems::BalisticForce ball(system_center); //ball_center);
  barrett::systems::HapticLine line(ball_center);
  barrett::systems::Summer<cf_type> direction_sum;
  barrett::systems::Summer<double> depth_sum;
  barrett::systems::PIDController<double, double> pid_controller;
  barrett::systems::Constant<double> zero(0.0);
  barrett::systems::TupleGrouper<cf_type, double> tuple_grouper;
  barrett::systems::Callback<boost::tuple<cf_type, double>, cf_type> mult(  // NOLINT
      scale);
  barrett::systems::ToolForceToJointTorques<DOF> tf2jt;
  barrett::systems::Summer<jt_type, 3> joint_torque_sum("+++");
  // EDIT FOR VIBRATIONS
  jt_type jtLimits(45.0);
  jtLimits[2] =35.0;
  jtLimits[0] = 55.0;
  proficio::systems::JointTorqueSaturation<DOF> joint_torque_saturation( jtLimits );
  // EDIT FOR VIBRATIONS
  v_type dampingConstants(50.0); //20;
  dampingConstants[2] = 10.0;    //10;
  dampingConstants[0] = 10.0;    //50;
  jv_type velocity_limits(1.7);
  proficio::systems::JointVelocitySaturation<DOF> velsat(dampingConstants, velocity_limits);

  barrett::systems::Normalize<cf_type> normalizeHelper;
  barrett::systems::Magnitude<cf_type, double> magnitudeHelper;

  jv_type joint_vel_filter_freq(20.0);
  barrett::systems::FirstOrderFilter<jv_type> joint_vel_filter;
  joint_vel_filter.setLowPass(joint_vel_filter_freq);

  // Configure Systems
  pid_controller.setKp(kpLine);
  pid_controller.setKd(kdLine);

  // Line up coordinate axis with python visualization
  barrett::systems::modXYZ<cp_type> mod_axes;
  mod_axes.negX();
  mod_axes.negY();
  mod_axes.xOffset(0.85);
  if (side == LEFT) {
    mod_axes.yOffset(0.27);
  } else if (side == RIGHT) {
    mod_axes.yOffset(-0.27);
  }
  mod_axes.zOffset(-0.2);  

  // Line up forces so that they correlate correctly with python visualization
  barrett::systems::modXYZ<cf_type> mod_force;
  mod_force.negX();
  mod_force.negY();
  barrett::systems::connect(wam.jpOutput, gravity_comp.input);
  barrett::systems::connect(wam.jvOutput, joint_vel_filter.input);
  barrett::systems::connect(joint_vel_filter.output, velsat.input);

  barrett::systems::connect(wam.toolPosition.output, mod_axes.input);
  barrett::systems::forceConnect(barrett::systems::message.output, network_haptics.input);
  barrett::systems::connect(mod_axes.output, ball.input);
  barrett::systems::connect(mod_axes.output, line.input);

  barrett::systems::connect(ball.directionOutput, direction_sum.getInput(0));
  barrett::systems::connect(line.directionOutput, direction_sum.getInput(1));

  barrett::systems::connect(wam.kinematicsBase.kinOutput, tf2jt.kinInput);
  barrett::systems::connect(direction_sum.output, magnitudeHelper.input);
  barrett::systems::connect(direction_sum.output, normalizeHelper.input);
  barrett::systems::connect(normalizeHelper.output, tuple_grouper.getInput<0>());

  barrett::systems::connect(magnitudeHelper.output, pid_controller.referenceInput);
  barrett::systems::connect(zero.output, pid_controller.feedbackInput);
  barrett::systems::connect(pid_controller.controlOutput, tuple_grouper.getInput<1>());

  barrett::systems::connect(tuple_grouper.output, mult.input);
  barrett::systems::connect(mult.output, mod_force.input);
  barrett::systems::connect(mod_force.output, tf2jt.input);
  barrett::systems::connect(tf2jt.output, joint_torque_sum.getInput(0));
  barrett::systems::connect(gravity_comp.output, joint_torque_sum.getInput(1));
  barrett::systems::connect(velsat.output, joint_torque_sum.getInput(2));
  barrett::systems::connect(joint_torque_sum.output, joint_torque_saturation.input);

  // Adjust velocity fault limit
  product_manager.getSafetyModule()->setVelocityLimit(1.5); //1.5
  product_manager.getSafetyModule()->setTorqueLimit(3.0);
  
  cout << "Begin idle wam" << endl;
  wam.idle();
  barrett::systems::connect(joint_torque_saturation.output, wam.input);
  //======================================================================================


  // Run Trial
  //bool freeMoving = false; //TODO: THIS SHOULD BE FALSE
  cp_type cp;
  cp_type target_center;
  jt_type jt;
  target_center[0] = 0.439;
  target_center[1] = 0.417;
  target_center[2] = 0.366;
  //double target_error = 0.03;

  std::deque<double> scores;
	CMessage Consumer_M;

  // Todo: these shouldn't be modifiable easily.
  //double targetWidth = 0.28125/2;
  //double trackLength = 0.53125/2; //0.5234375; actual reaching distance
  //double errorLimit = 0.008;

  // double zDepth = 0
  
  // Spawn 2 threads for listening to RTMA and moving robot
  pthread_t rtmaThread, robotMoverThread;
  
  // Create thread arguments
  struct arg_struct<DOF> args(wam, system_center, mod, product_manager, ball);
  
  //Start threads
  pthread_create(&rtmaThread, NULL, &responderWrapper<DOF>, (void *)&args);
  pthread_create(&robotMoverThread, NULL, &moveRobot<DOF>, (void *)&args);
  
  // Wait for threads to finish
  pthread_join(rtmaThread, NULL );
  // pthread_join(robotMoverThread, NULL );
  
  cout << "Finished trial" << endl;
  mod.DisconnectFromMMM();
  
  // Destroy mutexes
  //pthread_mutex_destroy(&beLock);
  //pthread_mutex_destroy(&rmLock);
  
  // ** END POSITION JUDGE ** 
  if (product_manager.getSafetyModule()->getMode() == barrett::SafetyModule::IDLE)
  {
    cout << "Experiement finished, moving home" << endl;
    wam.moveHome();
    return 0;
  }
  barrett::btsleep(0.02);
  #ifndef NO_CONTROL_PENDANT
  #endif
  return 0;
}
