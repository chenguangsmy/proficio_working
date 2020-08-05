/*  Copyright 2019 
 *
 *  Code to run Balistic Force Trials on the Proficio Robot
 *
 *  This version of proficio_toolbox is free software: you can redistribute it
 *  and/or modify it under the terms of the GNU General Public License as
 *  published by the Free Software Foundation, either version 3 of the
 *  License, or (at your option) any later version.
 *
 *  <http://www.gnu.org/licenses/>.
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

#include <signal.h>  // signal, raise, sig_atomic_t
#include <string.h>

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

#include <proficio/systems/utilities.h>

#define BARRETT_SMF_VALIDATE_ARGS

#include <proficio/standard_proficio_main.h>


BARRETT_UNITS_FIXED_SIZE_TYPEDEFS;
BARRETT_UNITS_TYPEDEFS(10);

const char* remote_host = NULL;
v_type msg_tmp;
barrett::systems::ExposedOutput<v_type> message;
bool forceMet = false;
cp_type center_pos(0.50, -0.120, 0.250);;


/* Set the IP address */
bool validate_args(int argc, char** argv) {
  if (argc != 2) {
    remote_host = "127.0.0.1";
    printf("Defaulting to 127.0.0.1\n");
  } else {
    remote_host = argv[1];
  }
  return true;
}


cf_type hapticCalc(boost::tuple<cp_type, cv_type> haptics_tuple) {
  cf_type net_force;
  cp_type wam_pos = haptics_tuple.get<0>();
  cv_type wam_vel = haptics_tuple.get<1>();

  double stiffness = 1000.0;  // Spring Force (Kp)
  double damping = 15.0;      // Damping Force (Kd)


  // Ensure Force in z direction is constant
  net_force[2] = stiffness * (center_pos[2] - wam_pos[2]);
  net_force[2] -= damping* wam_vel[2];
  
  net_force[0] = 0;
  net_force[1] = 0;
  // Resist deformation in the x-y directions
  net_force[0] = stiffness * (center_pos[0] - wam_pos[0]);  // Spring Portion
  net_force[1] = stiffness * (center_pos[1] - wam_pos[1]);

  net_force[0] -= damping * wam_vel[0];  // Damping portion
  net_force[1] -= damping * wam_vel[1];

  return net_force;
}

enum ContactState {
  PLAYING,
  TARING,
  QUIT
} curState = PLAYING,
  lastState = PLAYING;


template <size_t DOF>
class HapticsDemo {
  BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

 protected:
  /* Wam and ProductManager Instances Passed into Class */
  barrett::systems::Wam<DOF>& wam;
  barrett::ProductManager& product_manager_;

  /* Network Parameters */
  struct sockaddr_in si_server;
  int port, sock, i, slen;
  char* buf;
  char* srv_addr;

  jt_type jtLimits;
  barrett::systems::TupleGrouper<cp_type, cv_type> tuple_grouper;

  barrett::systems::ToolForceToJointTorques<DOF> cf_tf2jt;

  cf_type net_force;
  jp_type init_joint_pos;
  //cp_type center_pos;
  //bool forceMet;
  cf_type curForces;

  barrett::systems::Callback<boost::tuple<cp_type, cv_type>, cf_type> haptics;
  proficio::systems::JointTorqueSaturation<DOF> jtsat;
  proficio::systems::UserGravityCompensation<DOF>* user_grav_comp_;
  barrett::systems::Summer<jt_type, 3> jtSum;
  v_type dampingConstants;
  jv_type velocityLimits;
  proficio::systems::JointVelocitySaturation<DOF> velsat;

  barrett::systems::modXYZ<cp_type> invpos;
  barrett::systems::modXYZ<cf_type> invforce;
  barrett::systems::modXYZ<cv_type> invvel;
  barrett::systems::modXYZ<cv_type> invvelSat;

  jv_type jvFiltFreq;
  cv_type cvFiltFreq;
  barrett::systems::FirstOrderFilter<jv_type> jvFilter;
  barrett::systems::FirstOrderFilter<cv_type> cvFilter;

 public:
  bool ftOn;
  double sumForces;

  HapticsDemo(barrett::systems::Wam<DOF>& wam_arm, barrett::ProductManager& pm,
              proficio::systems::UserGravityCompensation<DOF>* ugc)
      : wam(wam_arm),
        product_manager_(pm),
        slen(sizeof(si_server)),
        jtLimits(55.0),
        haptics(hapticCalc),
        jtsat(jtLimits),
        user_grav_comp_(ugc),
        jtSum("+++"),
        dampingConstants(20.0),
        velocityLimits(2.4),
        velsat(dampingConstants, velocityLimits),
        jvFiltFreq(20.0),
        cvFiltFreq(20.0),
        sumForces(0.0) {
    dampingConstants[2] = 10.0;
    dampingConstants[0] = 30.0;
    velsat.setDampingConstant(dampingConstants);
    
    // line up coordinate axis with python visualization
    invpos.negX();
    invpos.negY();    
    invpos.xOffset(1);
    invpos.yOffset(-0.27);
    //invpos.zOffset(-0.2); 
    
    
    
    invforce.negX();
    invforce.negY();
    invvel.negX();
    invvel.negY();
    invvelSat.negX();
    invvelSat.negY();
    jvFilter.setLowPass(jvFiltFreq);
    cvFilter.setLowPass(cvFiltFreq);
  }
  ~HapticsDemo() {}

  bool init(Config side);
  bool setupNetworking();
  void displayEntryPoint();
  void connectForces();
  void visualizationThread();
  void setCenter(cp_type newCenter);
  void setForceMet(bool wasMet);
};


/* Initialization Method */
template <size_t DOF>
bool HapticsDemo<DOF>::init(Config side) {
  wam.gravityCompensate();
  product_manager_.getSafetyModule()->setVelocityLimit(2.5);
  product_manager_.getSafetyModule()->setTorqueLimit(4.5);

  wam.moveTo(center_pos);
  
  
  barrett::btsleep(5);
  
  wam.idle();
  return true;
}

/* Reset center position */
template <size_t DOF>
void HapticsDemo<DOF>::setCenter(cp_type newCenter) {
  center_pos = newCenter;
}

/* Set forceMet */
template <size_t DOF>
void HapticsDemo<DOF>::setForceMet(bool wasMet) {
  forceMet = wasMet;
}

/* */
template <size_t DOF>
bool HapticsDemo<DOF>::setupNetworking() {
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

/*  */
template <size_t DOF>
void HapticsDemo<DOF>::connectForces() {
  barrett::systems::modXYZ<cp_type> mod_axes;
  
  BARRETT_UNITS_FIXED_SIZE_TYPEDEFS;
  barrett::systems::connect(wam.kinematicsBase.kinOutput, cf_tf2jt.kinInput);
  barrett::systems::connect(wam.jpOutput, user_grav_comp_->input);
  barrett::systems::connect(wam.jvOutput, jvFilter.input);
  barrett::systems::connect(jvFilter.output, velsat.input);
  barrett::systems::connect(wam.toolPosition.output, invpos.input);
  barrett::systems::connect(invpos.output, tuple_grouper.getInput<0>());
  barrett::systems::connect(haptics.output, invforce.input);
  barrett::systems::connect(invforce.output, cf_tf2jt.input);
  barrett::systems::connect(wam.toolVelocity.output, cvFilter.input);
  barrett::systems::connect(cvFilter.output, invvel.input);
  barrett::systems::connect(invvel.output, tuple_grouper.getInput<1>());
  barrett::systems::connect(tuple_grouper.output, haptics.input);
  barrett::systems::connect(cf_tf2jt.output, jtSum.getInput(0));
  barrett::systems::connect(user_grav_comp_->output, jtSum.getInput(1));
  barrett::systems::connect(velsat.output, jtSum.getInput(2));
  barrett::systems::connect(jtSum.output, jtsat.input);
  barrett::systems::connect(jtsat.output, wam.input);
}

/*  */
template <size_t DOF>
int proficio_main(int argc, char** argv, barrett::ProductManager& product_manager_,
                  barrett::systems::Wam<DOF>& wam, const Config& side) {
  BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
  std::string fname = "calibration_data/wam3/";
  if (side == LEFT) {  // Left Config
    fname = fname + "LeftConfig.txt";
  } else if (side == RIGHT) {
    fname = fname + "RightConfig.txt";
  }
  proficio::systems::UserGravityCompensation<DOF> user_grav_comp_(
      barrett::EtcPathRelative(fname).c_str());
  user_grav_comp_.setGainZero();
  HapticsDemo<DOF> haptics_demo(wam, product_manager_, &user_grav_comp_);
  if (!haptics_demo.setupNetworking()) {
    return 1;
  }
  if (!haptics_demo.init(side)) return 1;

  // instantiate Systems
  NetworkHaptics<DOF> nh(product_manager_.getExecutionManager(), remote_host,
                         &user_grav_comp_);
  message.setValue(msg_tmp);
  barrett::systems::forceConnect(message.output, nh.input);
  haptics_demo.connectForces();

  bool running = true;
  haptics_demo.ftOn = false;

  while (running) {
    switch (curState) {
      case QUIT:
        product_manager_.getPuck(1)->setProperty(product_manager_.getPuck(1)->getBus(), 1, 8, 3);
        product_manager_.getPuck(2)->setProperty(product_manager_.getPuck(2)->getBus(), 2, 8, 3);
        product_manager_.getPuck(3)->setProperty(product_manager_.getPuck(3)->getBus(), 3, 8, 3);
        barrett::systems::disconnect(wam.input);
        running = false;
        break;
      case PLAYING:
        lastState = PLAYING;
        barrett::btsleep(0.1);
        break;
      default:
        break;
    }
  }

  wam.moveHome();
  printf("\n\n");
  return 0;
}
