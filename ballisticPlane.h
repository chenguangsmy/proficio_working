/*  
 * Ballistic Plane
 * 
 * Holds the robot at given Z-plane
 * 
 * Author(s): Ivana Stevens 2018, 2019
 * 
 */

#include "/home/robot/src/Proficio_Systems/magnitude.h"
#include "/home/robot/src/Proficio_Systems/normalize.h"
#include "/home/robot/rg2/include/RTMA_config.h"
#include <unistd.h>

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

BARRETT_UNITS_FIXED_SIZE_TYPEDEFS;
BARRETT_UNITS_TYPEDEFS(6);


#ifndef BALLISTIC_PLANE_STUFF
#define BALLISTIC_PLANE_STUFF

extern bool yDirectionError;
extern bool thresholdMet;

namespace barrett {
  namespace systems {

    class HapticLine : public HapticObject {
      BARRETT_UNITS_FIXED_SIZE_TYPEDEFS;

    public:
      /** 
       * HapticLine
       */
      HapticLine(const cp_type& center, const std::string& sysName = "HapticLine") :
        HapticObject(sysName),
        c(center),
        depth(0.0), dir(0.0)
      {}
      virtual ~HapticLine() { mandatoryCleanUp(); }
      
      void setCenter(const cp_type& newCenter){
        BARRETT_SCOPED_LOCK(getEmMutex());
        c = newCenter;
      }

    protected:
      /** 
       * operate
       * 
       * Weird shaking when fully extended. Stop compensating for z 
       * direction at this point. Ensure whatever distance this is, no trial requires 
       * going past it { ie trial fails b/c !inTarget() } 
       */
      virtual void operate() {
        pos = input.getValue();
        inputForce = c - input.getValue();
        
        
        //int posy = inputForce[0];
        //int region = 0.25*(pow(2.7183,posy));
        
        /*
        // Outside defined exponenetial region
        if ((abs(inputForce[2]) - region) > 0 && pos[0] >= -.22 +.450) {
          inputForce[0] = 0;
          inputForce[1] = 0;
          depth = 0.25*inputForce.norm();
          dir = inputForce;
          
        // Waaaaaayyyyyyy too far; Collapse!
        } else if ((pos[0] < (-0.53125 / 2) + 0.450) || yDirectionError) { // -0.2656 (max distance) from .450 (y Center)
          thresholdMet = true; // (quit force) In theory this should already be true
          yDirectionError = true; // Force Quit 
          
          inputForce.setZero();
          
          depth = 0.0;
          dir = inputForce; 
          
        // Beyond y region, push back
        } else if (pos[0] < -.22 +.450) {
          inputForce[0] = 0;
          depth = 0.25*inputForce.norm();
          dir = inputForce;
          
        } else { // Within region, no added force
          inputForce[0] = 0;
          inputForce[1] = 0;
          depth = 0.0;
          dir = inputForce;
        }*/
        
        
        
        //MAGIC NUMBER!!! Y length greater than half trackLength
        if ((pos[0] < (-0.53125 / 2) + 0.450) || yDirectionError)
        {
          thresholdMet = true; // (quit force) In theory this should already be true
          yDirectionError = true; // Force Quit 
          
          inputForce.setZero();
          
          depth = 0.0;
          dir = inputForce;          
        }
        else { // Just set the X and Y forces to 0
          inputForce[0] = 0; // No force in Y direction
          inputForce[1] = 0; // No force in X direction
          
          depth = inputForce.norm();
          dir = inputForce;
        } 
        
        depthOutputValue->setData(&depth);
        directionOutputValue->setData(&dir); //haptic
      }

      cp_type c;
      cp_type pos;

      // state & temporaries
      cf_type inputForce;

      double depth;
      cf_type dir;

    private:
      DISALLOW_COPY_AND_ASSIGN(HapticLine);

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

  }
}

#endif
