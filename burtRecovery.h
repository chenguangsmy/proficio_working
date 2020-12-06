/*****************************************************************************************
 * Recovery after Ydirection error
 * 
 ****************************************************************************************/
 
 
#include <iostream>
#include <vector>
#include <string>

#include <boost/ref.hpp>
#include <boost/bind.hpp>
#include <boost/tuple/tuple.hpp>

#include <barrett/detail/stl_utils.h>  // waitForEnter()
#include <barrett/math.h>
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/log.h>
#include <barrett/products/product_manager.h>

#include <barrett/standard_main_function.h>
#include <sys/types.h>


using namespace barrett;
using detail::waitForEnter;
using systems::connect;
using systems::disconnect;
using systems::reconnect;
 
 
template<size_t DOF>
void recoverBurt(ProductManager& pm, systems::Wam<DOF>& wam)
{
	typedef boost::tuple<double, jp_type> jp_sample_type;

	char tmpFile[] = "/home/robot/Proficio_Balistic_Release/Proficio_External/moveBurtHome";

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
  

	wam.idle(); 
}
