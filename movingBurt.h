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
 
#include "/home/robot/src/Proficio_Systems/magnitude.h" //... do we use these two files? -cg
#include "/home/robot/src/Proficio_Systems/normalize.h"
#include "/home/robot/rg2/include/RTMA_config.h"
#include <unistd.h>

#include "/home/robot/RTMA/include/RTMA.h"

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
//#include <barrett/products/product_manager.h>

// Networking
#include <netinet/in.h>
#include <sys/types.h>

//#include <proficio/systems/utilities.h>

BARRETT_UNITS_FIXED_SIZE_TYPEDEFS;
BARRETT_UNITS_TYPEDEFS(6);

#ifndef MOVING_BURT_STUFF
#define MOVING_BURT_STUFF


/*****************************************************************************************
 * Recovery after Ydirection error
 * 
 ****************************************************************************************/
template<size_t DOF>
void recoverBurt(barrett::ProductManager& pm, barrett::systems::Wam<DOF>& wam)
{ /*
	typedef boost::tuple<double, jp_type> jp_sample_type;

	char tmpFile[] = "moveBurtHome";

	// Build spline between recorded points
	barrett::log::Reader<jp_sample_type> lr(tmpFile);
	std::vector<jp_sample_type> vec;
	for (size_t i = 0; i < lr.numRecords(); ++i) {
	  vec.push_back(lr.getRecord());
	}
	barrett::math::Spline<jp_type> spline(vec);

	printf("Y direction error! Press [Enter] to return home.\n");
	barrett::detail::waitForEnter();

	// First, move to the starting position
  const jp_type firstPos = spline.eval(spline.initialS());
	wam.moveTo(firstPos);
  

	// Then play back the recorded motion
	barrett::systems::Ramp time(pm.getExecutionManager());
	// time.stop();
	time.setOutput(spline.initialS());

	barrett::systems::Callback<double, jp_type> trajectory(boost::ref(spline));
	barrett::systems::connect(time.output, trajectory.input);
	wam.trackReferenceSignal(trajectory.output);

	time.start();

	while (trajectory.input.getValue() < spline.finalS()) {
	  usleep(100000); 
  }
  

	wam.idle(); */
}




/*****************************************************************************************
 * moveToCenter
 * 
 * Move the wam to the center slowly while subject holding start button
 ****************************************************************************************/
template <size_t DOF>
void moveToCenter(barrett::systems::Wam<DOF>& wam,
              cp_type system_center,
              RTMA_Module &mod)
{
  /* //-cg debugging, 2020-12-28
  wam.moveTo(system_center);
  usleep(2000);
  cout << "Proficio reached home" << endl;
  */
  // Send Denso home message
  MDF_DENSO_MOVE_COMPLETE dmc;
  CMessage MDMC( MT_DENSO_MOVE_COMPLETE );
  MDMC.SetData( &dmc, sizeof(dmc) );
  mod.SendMessage( &MDMC );

}



/*****************************************************************************************
 * moveAStep
 * 
 * Move the wam to the center slowly while subject holding start button
 ****************************************************************************************/
template <size_t DOF>
void moveAStep(barrett::systems::Wam<DOF>& wam,
              cp_type system_center,
              RTMA_Module &mod)
{
  cout << "Press and hold start button, now" << endl;
  cp_type currentPos;
	CMessage Consumer_M;
  double stepSize = 0.03; // meters so 3cm...
  cp_type travelLen;
  cp_type unitVec;
  cp_type step;

  currentPos = barrett::math::saturate(wam.getToolPosition(), 9.999);

  // Calculate distance between point and home.
  travelLen = currentPos - system_center;

  if (travelLen.norm() > stepSize)
  {
    unitVec = travelLen/travelLen.norm();
    step = currentPos - stepSize*unitVec;    
    cout << "unit and step: " << unitVec << ": " << step << endl;
    wam.moveTo(step);
  }
  else
  { 
    wam.moveTo(system_center);
    cout << "Proficio reached home" << endl;
   
    MDF_PLAY_SOUND ps;
    ps.id = 2;
      
    // Send Denso home message
    MDF_DENSO_MOVE_COMPLETE dmc;
    CMessage MDMC( MT_DENSO_MOVE_COMPLETE );
    MDMC.SetData( &dmc, sizeof(dmc) );
    mod.SendMessage( &MDMC );
  }
  //return NULL;
}

#endif
