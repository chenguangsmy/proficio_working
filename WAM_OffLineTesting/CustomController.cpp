#include <iostream>
#include <string>

#include <barrett/systems.h>
#include <barrett/units.h>
#include <barrett/products/product_manager.h>
#include <barrett/detail/stl_utils.h>
#include <barrett/standard_main_function.h>
#include <barrett/math/matrix.h> 
#include <math.h>

// Required to run logger
#include <cstdlib>  // For mkstmp()
#include <cstdio>  // For remove()
#include <boost/tuple/tuple.hpp>
#include <barrett/log.h>

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

template<size_t DOF>
class JointControlClass : public systems::System{ 
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

public:
	Input<jp_type> wamJPInput;
	Input<jv_type> wamJVInput;
	Input<cp_type> wamCPInput;
	Input<cv_type> wamCVInput;
	Output<jt_type> wamJTOutput;
	Output<cf_type> wamCFPretOutput;
	jp_type input_q_0;
	cp_type input_x_0;

protected:
	typename Output<jt_type>::Value* outputValue1; 
	typename Output<cf_type>::Value* outputValue2; 
	systems::Wam<DOF>& wam;
	
public:
	explicit JointControlClass(systems::Wam<DOF>& wam, const std::string& sysName = "JointControlClass") :
		systems::System(sysName), wam(wam), wamJPInput(this), wamJVInput(this), wamCPInput(this), wamCVInput(this), wamJTOutput(this, &outputValue1), wamCFPretOutput(this, &outputValue2){
			
		// Joint stiffness
		K_q(0,0) = 5.0;
		K_q(1,1) = 5.0;
	 	K_q(2,2) = 5.0; 
		K_q(3,3) = 5.0;
		// Joint damping
	 	B_q = 0.1*K_q;

	 	// Nominal joint space postion
		input_q_0[0] = -1.581;
		input_q_0[1] = -0.035; 
		input_q_0[2] = -0.034; 
		input_q_0[3] = 1.521; 

		//End-effector stiffness
		K_x(0,0) = 100.0;
		K_x(1,1) = 100.0;
	 	K_x(2,2) = 200.0; 
		//End-effector damping
	 	B_x = 0.1*K_x;

		// Nominal end-effector potion	(NEED TO CHECK THIS BEFORE TESTING)
		input_x_0[0] = -0.448;  //position: raise hand on desk
		input_x_0[1] = 0.418;
		input_x_0[2] = 0.010;
		}

	virtual ~JointControlClass() { this->mandatoryCleanUp(); }

protected:
	jp_type input_q;
	jv_type input_q_dot;
	cp_type input_x;
	cv_type input_x_dot;
	jt_type torqueOutput;
	cf_type forceOutput;
	jt_type force2torqueOutput;
	cf_type f_pretOutput;

	// Initialize variables 
	Matrix_4x4 K_q; 
	Matrix_4x4 B_q; 
	Matrix_3x3 K_x; 
	Matrix_3x3 B_x; 
	Matrix_3x1 x_0; 	//Matrix_4x1 x_0;
	Matrix_3x1 x;   	//Matrix_4x1 x; 
	Matrix_3x1 x_dot;	//Matrix_4x1 x_dot;
	Matrix_4x1 q_0;
	Matrix_4x1 q; 
	Matrix_4x1 q_dot;
	Matrix_4x1 tau;
	Matrix_4x1 tau_q;
	Matrix_4x1 tau_x;
	Matrix_4x1 tau_pret;
	Matrix_3x1 f_pret;
	Matrix_6x4 J_tot;
	Matrix_3x4 J_x;

	virtual void operate() {
		
		input_q = wamJPInput.getValue();
 		input_q_dot = wamJVInput.getValue();
 		input_x = wamCPInput.getValue();
 		input_x_dot = wamCVInput.getValue();

		// Custom torque calculations
		
		// Convert imputs to matrix type
		// Define q
		q[0] = input_q[0];
		q[1] = input_q[1];
		q[2] = input_q[2];
		q[3] = input_q[3];

		// Define q_0
		q_0[0] = input_q_0[0]; 
		q_0[1] = input_q_0[1]; 
		q_0[2] = input_q_0[2]; 
		q_0[3] = input_q_0[3]; 

		// Define q_dot
		q_dot[0] = input_q_dot[0];
		q_dot[1] = input_q_dot[1];
		q_dot[2] = input_q_dot[2];
		q_dot[3] = input_q_dot[3];

		// x_0 get value
		x_0[0] = input_x_0[0];
		x_0[1] = input_x_0[1];
		x_0[2] = input_x_0[2];

		// x get value
		x[0] = input_x[0];
		x[1] = input_x[1];
		x[2] = input_x[2];

		//x_dot get value
		x_dot[0] = input_x_dot[0];
		x_dot[1] = input_x_dot[1];
		x_dot[2] = input_x_dot[2];

		// Import Jacobian
		J_tot.block(0,0,6,4) = wam.getToolJacobian(); // Entire 6D Jacobian
		J_x.block(0,0,3,4) = J_tot.block(0,0,3,4); // 3D Translational Jacobian
  
  
		// Control Law Implamentation

		// Joint impedance controller
		tau_q = K_q*(q_0 - q) - B_q*(q_dot);

		// End-effector impedance controller
		tau_x = J_x.transpose()*(K_x*(x_0 - x) - B_x*(x_dot)); 	

		// Random Preturbation
		f_pretOutput.setRandom();
    f_pretOutput[0] = f_pretOutput[0] * 2.0;
    f_pretOutput[1] = f_pretOutput[1] * 2.0;
    f_pretOutput[2] = f_pretOutput[2] * 2.0;
		f_pretOutput[2] = 0.0;
		f_pret[0] = f_pretOutput[0];
		f_pret[1] = f_pretOutput[1];
		f_pret[2] = f_pretOutput[2]; 

		tau_pret = J_x.transpose()*(f_pret);

		// Sum torque commands
		tau = tau_q + tau_x + tau_pret;

		// printf("tau: %.5f, %.5f, %.5f, \n", f_pret[0],f_pret[1],f_pret[2]);

		// Save outputs
		torqueOutput[0] = tau[0];
		torqueOutput[1] = tau[1];
		torqueOutput[2] = tau[2];
		torqueOutput[3] = tau[3];

		this->outputValue1->setData(&torqueOutput);
		this->outputValue2->setData(&f_pretOutput);
	}

private:
	DISALLOW_COPY_AND_ASSIGN(JointControlClass);
};



template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm, systems::Wam<DOF>& wam) {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

	wam.gravityCompensate();
	char tmpFile[] = "bt20200904XXXXXX";
	if (mkstemp(tmpFile) == -1) {
		printf("ERROR: Couldn't create temporary file!\n");
	return 1;
	} 
	//Initilaize jj class
	JointControlClass<DOF> jj(wam);

	// Move to nominal joint postion before starting control
  	wam.moveTo(jj.input_q_0);

	// //Set up data logging
	systems::Ramp time(pm.getExecutionManager(), 1.0);
	systems::TupleGrouper<double, jp_type, jv_type, cp_type, cv_type, jt_type, cf_type> tg;
	systems::connect(time.output, tg.template getInput<0>());
	systems::connect(wam.jpOutput, tg.template getInput<1>());
	systems::connect(wam.jvOutput, tg.template getInput<2>());
	systems::connect(wam.toolPosition.output, tg.template getInput<3>());
	systems::connect(wam.toolVelocity.output, tg.template getInput<4>());
	//systems::connect(wam.toolOrientation.output, tg.template getInput<6>());
	systems::connect(jj.wamJTOutput, tg.template getInput<5>());
	systems::connect(jj.wamCFPretOutput, tg.template getInput<6>());


	typedef boost::tuple<double, jp_type, jv_type, cp_type, cv_type, jt_type, cf_type> tuple_type;

	const size_t PERIOD_MULTIPLIER = 1;
	systems::PeriodicDataLogger<tuple_type> logger(
			pm.getExecutionManager(),
			new log::RealTimeWriter<tuple_type>(tmpFile, PERIOD_MULTIPLIER * pm.getExecutionManager()->getPeriod()),
			PERIOD_MULTIPLIER);

	time.start();
	systems::connect(tg.output, logger.input);
	printf("Logging started.\n");


	// Connect torque controller
	printf("Press [Enter] to start your custom system.");
	waitForEnter();

 	systems::connect(wam.jpOutput, jj.wamJPInput);
 	systems::connect(wam.jvOutput, jj.wamJVInput);
	systems::connect(wam.toolPosition.output, jj.wamCPInput);	
 	systems::connect(wam.toolVelocity.output, jj.wamCVInput);

	// Track refrence
	wam.trackReferenceSignal(jj.wamJTOutput);	

	pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);

	//Turn off data logger
	logger.closeLog();
	printf("Logging stopped.\n");
	log::Reader<tuple_type> lr(tmpFile);
	lr.exportCSV(argv[1]);
	printf("Output written to %s.\n", argv[1]);
	std::remove(tmpFile);

	return 0;
}
