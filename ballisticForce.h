/*  
 * Ballistic force
 * 
 * holds the robot at given center postion
 * 
 * Author(s): Ivana Stevens 2018, 2019
 * 
 */


#include "/home/robot/src/Proficio_Systems/magnitude.h"
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
#include <barrett/products/product_manager.h>

// Networking
#include <netinet/in.h>
#include <sys/types.h>

//#include <proficio/systems/utilities.h>

BARRETT_UNITS_FIXED_SIZE_TYPEDEFS;
BARRETT_UNITS_TYPEDEFS(6);


#ifndef BALLISTIC_FORCE_STUFF
#define BALLISTIC_FORCE_STUFF

extern bool yDirectionError;
extern bool thresholdMet;
extern double forceThreshold;

namespace barrett {
  namespace systems {
    v_type msg_tmp;
    barrett::systems::ExposedOutput<v_type> message;

    class BalisticForce : public HapticObject {
      BARRETT_UNITS_FIXED_SIZE_TYPEDEFS;

      public:
        /** 
         * BalisticForce
         */
        BalisticForce(const cp_type& center,
            const std::string& sysName = "BalisticForce") :
          HapticObject(sysName),
          c(center),
          depth(0.0), dir(0.0), forceMet(true)
        {}
        virtual ~BalisticForce() { mandatoryCleanUp(); }
        
        void setCenter(const cp_type& newCenter){
          BARRETT_SCOPED_LOCK(getEmMutex());
          c = newCenter;
        }
        
        const cp_type& getCenter() const { return c; }
        
        void setForceMet(bool newTM) {
          BARRETT_SCOPED_LOCK(getEmMutex());
          forceMet = newTM;
        }

      protected:
        /** 
         * operate
         */
        virtual void operate() 
        {
          //cp_type othercenter(0.350, -0.120, 0.250);
          error = c - input.getValue() ;
          error[2] = 0; // No force in Z direction
          double mag = error.norm();
          if (!forceMet ) //&& (math::abs(mag) > 0.15))
          {
              depth = mag;
              error = error/depth;
              
            /* If you haven't met the force threshold
            if (mag < forceThreshold)
            {
              depth = mag;
              error = error/depth;
            }
            // Else you've met the force threshold
            else 
            {
              BARRETT_SCOPED_LOCK(getEmMutex());
              forceMet = true;
              depth = 0.0;
              error.setZero();
            } */
          }
          else 
          {
            depth = 0.0;
            error.setZero();
          }        
          depthOutputValue->setData(&depth);
          directionOutputValue->setData(&error);        
        }
        
        bool forceMet;
        cp_type c;
        cf_type error;

        // state & temporaries
        double depth;
        cf_type dir;      

      private:
        DISALLOW_COPY_AND_ASSIGN(BalisticForce);

      public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

  }
}

#endif
