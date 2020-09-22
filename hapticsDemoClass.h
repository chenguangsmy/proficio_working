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
 
#include "/home/robot/src/Proficio_Systems/magnitude.h"
#include "/home/robot/src/Proficio_Systems/normalize.h"
#include "/home/robot/rg2/include/RTMA_config.h"
#include <unistd.h>

#include "/home/robot/RTMA/include/RTMA.h"
#include "movingBurt.h"
#include "ballisticForce.h"

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

extern const char* remote_host;
extern bool forceMet;
extern cp_type center_pos;

#ifndef HAPTICS_STUFF
#define HAPTICS_STUFF

bool hapticCalcForcemetShow = false; // Cleave debug
/*****************************************************************************************
 *  Initialization Method 
 ****************************************************************************************/
cf_type hapticCalc(boost::tuple<cp_type, cv_type> haptics_tuple) {
  cf_type net_force;
  cp_type wam_pos = haptics_tuple.get<0>();
  cv_type wam_vel = haptics_tuple.get<1>();

  double stiffness = 1000.0;  // Spring Force (Kp). Was 1000.0 (Hongwei, 9/5/2019)
  double damping = 15.0;      // Damping Force (Kd). Was 15.0 (Hongwei, 9/5/2019)


  // Ensure Force in z direction is constant
  net_force[2] = stiffness * (center_pos[2] - wam_pos[2]);
  net_force[2] -= damping* wam_vel[2];
  
//  net_force[0] = 0;
//  net_force[1] = 0;
  // Resist deformation in the x-y directions
//  if (!forceMet)
  {
    net_force[0] = stiffness * (center_pos[0] - wam_pos[0]);  // Spring Portion
    net_force[1] = stiffness * (center_pos[1] - wam_pos[1]);

    net_force[0] -= damping * wam_vel[0];  // Damping portion
    net_force[1] -= damping * wam_vel[1];
    if (!hapticCalcForcemetShow) {
      printf("Force Not Met! \n");
      hapticCalcForcemetShow = true; // not show again
    }
  }
<<<<<<< HEAD
// else
=======
 else
>>>>>>> 20479182f148e860131f1d5af5d95ca6303ecf5a
  {
    /* display for force met! */
    if (hapticCalcForcemetShow) {
      printf("Force Met! Force Set to 0! \n");
      hapticCalcForcemetShow = false; // not show again
    }
  }
  

  return net_force;
}



/*****************************************************************************************
 *  Initialization Method 
 ****************************************************************************************/
template <size_t DOF>
class HapticsDemo {
  BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

 protected:
  /* Wam and ProductManager Instances Passed into Class */
  barrett::systems::Wam<DOF>& wam;
  barrett::ProductManager& product_manager_;

  /* Network Parameters */
//  struct sockaddr_in si_server;
//  int port, sock, slen;
//  char* buf;
//  char* srv_addr;

  /* Wam joint parameters */
  // jt_type jtLimits;
  barrett::systems::TupleGrouper<cp_type, cv_type> tuple_grouper;

  barrett::systems::ToolForceToJointTorques<DOF> cf_tf2jt;

  cf_type net_force;
  //jp_type init_joint_pos;
  //cf_type curForces;

  barrett::systems::Callback<boost::tuple<cp_type, cv_type>, cf_type> haptics;
  //proficio::systems::JointTorqueSaturation<DOF> jtsat;
  //proficio::systems::UserGravityCompensation<DOF>* user_grav_comp_;
  barrett::systems::Summer<jt_type, 2> jtSum;
  //v_type dampingConstants;
  //jv_type velocityLimits;
  //proficio::systems::JointVelocitySaturation<DOF> velsat;

  //barrett::systems::modXYZ<cp_type> invpos;
  //barrett::systems::modXYZ<cf_type> invforce;
  //barrett::systems::modXYZ<cv_type> invvel;
  //barrett::systems::modXYZ<cv_type> invvelSat;

  jv_type jvFiltFreq;
  cv_type cvFiltFreq;
  barrett::systems::FirstOrderFilter<jv_type> jvFilter;
  barrett::systems::FirstOrderFilter<cv_type> cvFilter;

 public:
  bool ftOn;
  double sumForces;


  /***************************************************************************************
  *  Initialization Method 
  **************************************************************************************/
  HapticsDemo(barrett::systems::Wam<DOF>& wam_arm, barrett::ProductManager& pm
//              , proficio::systems::UserGravityCompensation<DOF>* ugc)
      ): wam(wam_arm),
        product_manager_(pm),//{
     //   slen(sizeof(si_server)),
     //   jtLimits(55.0),
        haptics(hapticCalc),
//        jtsat(jtLimits),
//        user_grav_comp_(ugc),
          jtSum("++"),
    //    dampingConstants(20.0),
    //    velocityLimits(2.4),
//        velsat(dampingConstants, velocityLimits),
        jvFiltFreq(20.0),
        cvFiltFreq(20.0),
        sumForces(0.0) {
//    dampingConstants[2] = 10.0;
//    dampingConstants[0] = 30.0;
//    velsat.setDampingConstant(dampingConstants);
    
    // line up coordinate axis with python visualization
    //invpos.negX();
    //invpos.negY();
    //invpos.xOffset(1);
    //invpos.yOffset(-0.27);
    //invpos.zOffset(-0.2);
    //invforce.negX();
    //invforce.negY();
    //invvel.negX();
    //invvel.negY();
    //invvelSat.negX();
    //invvelSat.negY();
    jvFilter.setLowPass(jvFiltFreq);
    cvFilter.setLowPass(cvFiltFreq);
    wam.idle();
  }


/*****************************************************************************************
 *  Initialization Method 
 ****************************************************************************************/
//template <size_t DOF>
bool init() {
  //printf("Sstart initialization function \n");
  wam.gravityCompensate();
  product_manager_.getSafetyModule()->setVelocityLimit(2.5);  // Was 2.5 (Hongwei, 9/5/2019)
  product_manager_.getSafetyModule()->setTorqueLimit(4.5);    // Was 4.5 (Hongwei, 9/5/2019)

  wam.moveTo(center_pos);
  barrett::btsleep(0.5);

  wam.idle();
  //printf("Begin idle \n");
  return true;
}

/*****************************************************************************************
 *  Reset center position 
 ****************************************************************************************/
//template <size_t DOF>
void setCenter(cp_type newCenter) {
  printf("Enter function: setCenter.");
  center_pos = newCenter;
}

/*****************************************************************************************
 *  Set forceMet 
 ****************************************************************************************/
//template <size_t DOF>
void setForceMet(bool wasMet) {
  printf("Enter function: setForceMet.");
  forceMet = wasMet;
}


/*****************************************************************************************
 * 
 ****************************************************************************************/
//template <size_t DOF>
/*
bool setupNetworking() {
  printf("Enter function: setupNetworking.");
  buf = new char[1024];
  srv_addr = new char[16];
  memset(srv_addr, 0, 16);
  port = 5555;
  if ((sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) {
    printf("Failure creating the socket");
    return false;
  }
  memcpy(srv_addr, remote_host, strlen(remote_host));
  memset((char*)&si_server, 0, sizeof(si_server));
  si_server.sin_family = AF_INET;
  si_server.sin_port = htons(port);
  if (inet_aton(srv_addr, &si_server.sin_addr) == 0) {
    printf("inet_aton() failed - EXITING\n");
    return false;
  }
  return true;
}

*/
/*****************************************************************************************
 *
 * **************************************************************************************/
//template <size_t DOF>
void connectForces() {
    printf("Enter function: connectForces.");
  barrett::systems::modXYZ<cp_type> mod_axes;
  
  BARRETT_UNITS_FIXED_SIZE_TYPEDEFS;
  barrett::systems::connect(wam.kinematicsBase.kinOutput, cf_tf2jt.kinInput);
//  barrett::systems::connect(wam.jpOutput, user_grav_comp_->input);
  barrett::systems::connect(wam.jvOutput, jvFilter.input);
//  barrett::systems::connect(jvFilter.output, velsat.input);
  //barrett::systems::connect(wam.toolPosition.output, invpos.input);
  //barrett::systems::connect(invpos.output, tuple_grouper.getInput<0>());
  barrett::systems::connect(wam.toolPosition.output, tuple_grouper.getInput<0>()); //dont invert position
  //barrett::systems::connect(haptics.output, invforce.input);
  //barrett::systems::connect(invforce.output, cf_tf2jt.input);
  barrett::systems::connect(haptics.output, cf_tf2jt.input); // do not invert the force
  barrett::systems::connect(wam.toolVelocity.output, cvFilter.input);
  //barrett::systems::connect(cvFilter.output, invvel.input);
  //barrett::systems::connect(invvel.output, tuple_grouper.getInput<1>());
  barrett::systems::connect(cvFilter.output, tuple_grouper.getInput<1>());  // dont invert velocity
  barrett::systems::connect(tuple_grouper.output, haptics.input);
//  barrett::systems::connect(cf_tf2jt.output, jtSum.getInput(0));
  barrett::systems::connect(cf_tf2jt.output, wam.input);
//  barrett::systems::connect(cf_tf2jt.output, jtSum.getInput(0));
//  barrett::systems::connect(user_grav_comp_->output, jtSum.getInput(1));
//  barrett::systems::connect(wam.gravity.output, jtSum.getInput(1));
//  barrett::systems::connect(velsat.output, jtSum.getInput(2));
//  barrett::systems::connect(jvFilter.output, jtSum.getInput(2));
//  barrett::systems::connect(jtSum.output, jtsat.input);
//  barrett::systems::connect(jtSum.output, wam.input);
//  barrett::systems::connect(jtsat.output, wam.input);
}

};

#endif
