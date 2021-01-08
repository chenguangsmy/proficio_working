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
	Input<double> timeInput;
	Input<jp_type> wamJPInput;
	Input<jv_type> wamJVInput;
	Input<cp_type> wamCPInput;
	Input<cv_type> wamCVInput;
	Output<jt_type> wamJTOutput;
	Output<cf_type> wamCFPretOutput;
	Output<int> wamIterationOutput;
	jp_type input_q_0;
	cp_type input_x_0;

protected:
	typename Output<jt_type>::Value* outputValue1; 
	typename Output<cf_type>::Value* outputValue2; 
	typename Output<int>::Value* outputValue3; 

	systems::Wam<DOF>& wam;
	
public:
	explicit JointControlClass(systems::Wam<DOF>& wam, const std::string& sysName = "JointControlClass") :
		systems::System(sysName), wam(wam), timeInput(this), 
		wamJPInput(this), wamJVInput(this), wamCPInput(this), wamCVInput(this), wamJTOutput(this, &outputValue1), 
		wamCFPretOutput(this, &outputValue2)
		, wamIterationOutput(this, &outputValue3)
		{
			
		// Joint stiffness
		K_q(0,0) = 100.0;
		K_q(1,1) = 0.0;
	 	K_q(2,2) = 100.0; 
		K_q(3,3) = 0.0;

		// Joint damping
	 	B_q = 0.1*K_q;

	 	// Nominal joint space postion
		input_q_0[0] = -1.570;
		input_q_0[1] =  0.008; 
		input_q_0[2] =  0.008; 
		input_q_0[3] =  1.570; 

		//End-effector stiffness
		K_x(0,0) = 2000.0;
		K_x(1,1) = 2000.0;
	 	K_x(2,2) = 0.0; 

		//End-effector damping
	 	B_x = 0.02*K_x;

		// Nominal end-effector potion	(NEED TO CHECK THIS BEFORE TESTING)
		input_x_0[0] = -0.409;  //position: raise hand on desk
		input_x_0[1] = 0.482;
		input_x_0[2] = -0.04;

		iteration_MAX = 4;
		iteration = 0; // do not input and output here
		prevPret[0] = 0.0;
		prevPret[1] = 0.0;
		prevPret[2] = 0.0;		
		}

	virtual ~JointControlClass() { this->mandatoryCleanUp(); }

protected:
	double input_time;
	double input_iteration;
	double pretAmplitude;
	double rampTime; // ramp stiffnes duing this duration
	int iteration;
	int iteration_MAX;
	cf_type input_prevPret;
	cf_type prevPret;
	jp_type input_q;
	jv_type input_q_dot;
	cp_type input_x;
	cv_type input_x_dot;
	jt_type torqueOutput;
	cf_type forceOutput;
	jt_type force2torqueOutput;
	cf_type f_pretOutput;
	double output_iteration;

	// Initialize variables 
	Matrix_4x4 K_q; 
	Matrix_4x4 K_q0; 	// stand alone K_q value
	Matrix_4x4 B_q; 
	Matrix_3x3 K_x; 
	Matrix_3x3 K_x0;
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

		input_time = timeInput.getValue();
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

		rampTime = 10.0;

		// Import Jacobian
		J_tot.block(0,0,6,4) = wam.getToolJacobian(); // Entire 6D Jacobian
		J_x.block(0,0,3,4) = J_tot.block(0,0,3,4); // 3D Translational Jacobian
  
		// Control Law Implamentation

		// Ramp up stiffness for first 10 seconds

		// Joint impedance controller
		if (input_time < rampTime ) {
			tau_q = (input_time/rampTime)*K_q*(q_0 - q) - B_q*(q_dot);
		}
		else {
			tau_q = K_q*(q_0 - q) - B_q*(q_dot);
		}

		// End-effector impedance controller
		if (input_time < rampTime ) {
			tau_x = J_x.transpose()*((input_time/rampTime)*K_x*(x_0 - x) - B_x*(x_dot)); 	
		}
		else {
			tau_x = J_x.transpose()*(K_x*(x_0 - x) - B_x*(x_dot)); 
		}

		// Random Preturbation
		// Less than iteration_MAX iterations since last update
		if (iteration < iteration_MAX){
			iteration++;
			f_pretOutput[0] = prevPret[0];
			f_pretOutput[1] = prevPret[1];
			f_pretOutput[2] = prevPret[2];

			f_pret[0] = f_pretOutput[0];
			f_pret[1] = f_pretOutput[1];
			f_pret[2] = f_pretOutput[2]; 
		}
		// More than iteration_MAX iterations since last update get new preturbaiton amplitude
		else if (iteration >= iteration_MAX) {
			// Reset the count
			iteration = 1;
			f_pretOutput.setRandom();
    		f_pretOutput[0] = f_pretOutput[0];
    		f_pretOutput[1] = f_pretOutput[1];
    		f_pretOutput[2] = f_pretOutput[2];
			f_pretOutput[2] = 0.0;

			// Make Preturbation unifore amplitude
			pretAmplitude = 2.0;
			if (f_pretOutput[0] >= 0 ) {
				f_pretOutput[0] = pretAmplitude;
			} else if (f_pretOutput[0] < 0 ){
				f_pretOutput[0] = -pretAmplitude;
			}

			if (f_pretOutput[1] >= 0 ) {
				f_pretOutput[1] = pretAmplitude;
			} else if (f_pretOutput[1] < 0 ){
				f_pretOutput[1] = -pretAmplitude;
			}
		
			f_pret[0] = f_pretOutput[0];
			f_pret[1] = f_pretOutput[1];
			f_pret[2] = f_pretOutput[2]; 
		}

		tau_pret = J_x.transpose()*(f_pret);

		// Sum torque commands
		tau = tau_q + tau_x + tau_pret;

		// printf("tau: %.5f, %.5f, %.5f, \n", f_pret[0],f_pret[1],f_pret[2]);
		// printf("time: %.5f, \n", input_time);
		
		// Save outputs
		prevPret[0] = f_pretOutput[0];
		prevPret[1] = f_pretOutput[1];
		prevPret[2] = f_pretOutput[2]; 

		torqueOutput[0] = tau[0];
		torqueOutput[1] = tau[1];
		torqueOutput[2] = tau[2];
		torqueOutput[3] = tau[3];

		//printf("tau output: %.5f, %.5f, %.5f, interation: %d \n", tau[0],tau[1],tau[2], tau[3], iteration);
		
		this->outputValue1->setData(&torqueOutput);
		this->outputValue2->setData(&f_pretOutput);
		this->outputValue3->setData(&iteration);
		
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
  
	systems::TupleGrouper<double, jp_type, jv_type, cp_type, cv_type, jt_type, cf_type, int> tg;
	systems::connect(time.output, tg.template getInput<0>());
	systems::connect(wam.jpOutput, tg.template getInput<1>());
	systems::connect(wam.jvOutput, tg.template getInput<2>());
	systems::connect(wam.toolPosition.output, tg.template getInput<3>());
	systems::connect(wam.toolVelocity.output, tg.template getInput<4>());
	//systems::connect(wam.toolOrientation.output, tg.template getInput<6>());
	systems::connect(jj.wamJTOutput, tg.template getInput<5>());
	systems::connect(jj.wamCFPretOutput, tg.template getInput<6>());
	systems::connect(jj.wamIterationOutput, tg.template getInput<7>());


	typedef boost::tuple<double, jp_type, jv_type, cp_type, cv_type, jt_type, cf_type, int> tuple_type;

	const size_t PERIOD_MULTIPLIER = 1;
	systems::PeriodicDataLogger<tuple_type> logger(
			pm.getExecutionManager(),
			new log::RealTimeWriter<tuple_type>(tmpFile, PERIOD_MULTIPLIER * pm.getExecutionManager()->getPeriod()),
			PERIOD_MULTIPLIER);

	// Connect torque controller
	systems::connect(time.output, jj.timeInput);
 	systems::connect(wam.jpOutput, jj.wamJPInput);
 	systems::connect(wam.jvOutput, jj.wamJVInput);
	systems::connect(wam.toolPosition.output, jj.wamCPInput);	
 	systems::connect(wam.toolVelocity.output, jj.wamCVInput);

	printf("Press [Enter] to start your custom system.");
	waitForEnter();
	systems::connect(tg.output, logger.input);
	time.start();
  
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
