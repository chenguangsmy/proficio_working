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

#define IS_PULSE_PERT 1

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

#ifndef CONTROLLERWARPER_STUFF
#define CONTROLLERWARPER_STUFF

extern std::string fname_rtma;
extern bool fname_init; 

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
    Output<int> wamRDTOutput; // read-time to synchronize stand-alone data and wam data
	jp_type input_q_0;
	cp_type input_x_0;
	systems::Ramp time;
	Output<int>	wamTaskState;

protected:
	typename Output<jt_type>::Value* outputValue1; 
	typename Output<int>::Value* outputValue2;
	typename Output<cf_type>::Value* outputValue3; 	// should be CFPretOutput 
	typename Output<int>::Value* outputValue4; 		// should be IterationOutput
	typename Output<int>::Value* outputValue5; 		// should be holeOutput
	
	systems::Wam<DOF>& wam;
	ProductManager& pm;
	
	
public:
	explicit JointControlClass(ProductManager& pm, Matrix_4x4 K_q, Matrix_4x4 B_q, Matrix_3x3 K_x, Matrix_3x3 B_x,
	 Matrix_4x4 K_q1, Matrix_4x4 B_q1,
	 jp_type input_q_0, cp_type input_x_0, 
	 systems::Wam<DOF>& wam, const std::string& sysName = "JointControlClass") :
		pm(pm), systems::System(sysName), wam(wam), timeInput(this),
		wamJPInput(this), wamJVInput(this), wamCPInput(this), wamCVInput(this),
		time(pm.getExecutionManager(), 1.0), 
		wamJTOutput(this, &outputValue1), 
		wamRDTOutput(this, &outputValue2),
		wamCFPretOutput(this, &outputValue3), 
		wamIterationOutput(this, &outputValue4),
		wamTaskState(this, &outputValue5),

		K_q(K_q), B_q(B_q), K_q1(K_q1), B_q1(B_q1),
		input_q_0(input_q_0), K_x(K_x), B_x(B_x), input_x_0(input_x_0){
			loop_iteration = 0;
			loop_itMax = 500*0.1; 	// freq*s
      		rampTime = 2.5;
			rdt = 0;
		 	if_set_JImp = false; 	// 
			if_set_Imp = false;
			K_qQuantum = (K_q1 - K_q0)/double(loop_itMax);
			iteration_MAX = 8;
			iteration = 0;
			task_state = 0;
      		//printf("K_qQ is: %.3f, %.3f, %.3f, %.3f\n", K_qQuantum(0,0), K_qQuantum(1,1), K_qQuantum(2,2), K_qQuantum(3,3));
		}

	virtual ~JointControlClass() { this->mandatoryCleanUp(); }

	void setImpedance(Matrix_3x3 K_x1, Matrix_3x3 B_x1){ 
		K_x = K_x1;
		B_x = B_x1; 
	}

	void setTaskState(int ts){
		task_state = ts;
	}

	void setJointImpedance(Matrix_4x4 K_q1, Matrix_3x3 B_q1){ //higher one, ST_HOLD
		// how to avoid continuous increasing?
		K_q = K_q1;
		B_q = B_q1;
	}

	void setImpedance_inc(Matrix_3x3 K_x1, Matrix_3x3 B_x1){ 
		K_xQuantum = (K_x1 - K_x)/loop_itMax;
		B_xQuantum = (B_x1 - B_x)/loop_itMax;
		if_set_Imp = true;
	}

	void setJointImpedance_inc(Matrix_4x4 K_q1, Matrix_3x3 B_q1){ //higher one, ST_HOLD
		// how to avoid continuous increasing?
		K_qQuantum = (K_q1 - K_q)/loop_itMax;
		B_qQuantum = (B_q1 - B_q)/loop_itMax;
		if_set_JImp = true;
	}

	void setx0(cp_type center_pos){
		input_x_0 = center_pos;
	}

	void setq0(jp_type center_pos){
		input_q_0 = center_pos;
	}

	int get_rdt(){
		return rdt;
	}

	int update_input_time0(){
		input_time0 = input_time;
		return 1;
	}

	int setpretAmp(){
		pretAmplitude_x = 6.0;
		pretAmplitude_y = 6.0;
	}

	int resetpretAmp(){
		pretAmplitude_x = 0.0;
		pretAmplitude_y = 0.0;
	}

	int resetpretFlip(bool flip){
		pert_flip = flip; // 0 for no pert, 1 for pert
	}

	int setPertMag(double mag){
		pert_mag = mag;
		return 1;
	}
protected:
	double	input_time;
	double  input_time0;	 // give a time offset when increase
	double	input_iteration;
	double 	pretAmplitude_x;
	double 	pretAmplitude_y;
	double 	rampTime;
	int 	iteration;
	int 	iteration_MAX;
	int 	loop_iteration;  // these are my code different from James
	int		loop_itMax;
	int		task_state;
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
    double pert_mag; //impulse-perturbation magnitude (Newton)
	int 	rdt; 
	bool 	if_set_JImp;
	bool 	if_set_Imp;
	bool    pert_flip;

	// Initialize variables 
	Matrix_4x4 K_q;
	Matrix_4x4 B_q;
	Matrix_4x4 K_q0; 	//lower value of K_q, because K_q would be set to 0 when forceMet.
	Matrix_4x4 B_q0;
	Matrix_4x4 K_q1;	//higher value of K_q
	Matrix_4x4 B_q1;
	Matrix_4x4 K_qQuantum; // each small part
	Matrix_4x4 B_qQuantum;
	Matrix_3x3 K_xQuantum;
	Matrix_3x3 B_xQuantum;
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
	//Matrix_2x1 callRand;
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

		// Import Jacobian
		J_tot.block(0,0,6,4) = wam.getToolJacobian(); // Entire 6D Jacobian
		J_x.block(0,0,3,4) = J_tot.block(0,0,3,4); // 3D Translational Jacobian

		// Updating K_q incrementally
		/*if (if_set_JImp){// slowly ramp the impedance
			loop_iteration++;
			K_q = K_q + K_qQuantum;
			B_q = B_q + B_qQuantum;
		}
		if (if_set_Imp){// ramp the endpoint impedance
			loop_iteration++;
			K_x = K_x + K_xQuantum;
			B_x = B_x + B_xQuantum;
		}
		if (loop_iteration >= loop_itMax){
			loop_iteration = 0;
			if_set_Imp = false;
			if_set_JImp = false;
		}
		*/
		// Joint impedance controller
		if ((input_time-input_time0) < rampTime ) {
			tau_q = ((input_time-input_time0)/rampTime)*K_q*(q_0 - q) - B_q*(q_dot);
		}
		else {
			tau_q = K_q*(q_0 - q) - B_q*(q_dot);
		}

		// End-effector impedance controller
		if ((input_time-input_time0) < rampTime ) {
			tau_x = J_x.transpose()*(((input_time-input_time0)/rampTime)*K_x*(x_0 - x) - B_x*(x_dot)); 	
		}
		else {
			tau_x = J_x.transpose()*(K_x*(x_0 - x) - B_x*(x_dot)); 
		}

		//printf("K_q is: %.3f, %.3f, %.3f, %.3f\n", K_q(0,0), K_q(1,1), K_q(2,2), K_q(3,3));
		//printf("K_x is: %.3f, %.3f, %.3f; B_x is: %.3f, %.3f, %.3f \n", K_x(0,0), K_x(1,1), K_x(2,2), B_x(0,0), B_x(1,1), B_x(2,2));
		// Control Law Implamentation
/*
		// End-effector impedance controller
		tau_x = J_x.transpose()*(K_x*(x_0 - x) - B_x*(x_dot)); 	
		// printf("K_x are: %.3f, %.3f, %.3f \n", K_x(0,0), K_x(1,1), K_x(2,2));
*/
// Less than iteration_MAX iterations since last update
		if (IS_PULSE_PERT) {
      //printf("-");
			//f_pretOutput.setRandom();
      //printf("Pert_flip: %s\n", pert_flip ? "true" : "false");
		if(pert_flip == true)
			{
				iteration = 0;
				resetpretFlip(false);
			}
		iteration++;
      //  printf("Pert_flip: %s\n", pert_flip ? "true" : "false");
        
      	if (iteration < 1000)
      		{
       		f_pretOutput[0] = 0;
        	f_pretOutput[1] = 0;
       		f_pretOutput[2] = 0;
      		}
        else if(iteration == 1000)
        {
			f_pretOutput[1] = pert_mag/2;

        }
       	else if(iteration == 1000+75)
       	{
			f_pretOutput[1] = pert_mag/2;

       	}
		else if (iteration < 1000 + 75) // as a fix start time for 2.85s.  // have a try by doing this
			{
     		//printf("Start the force impulse perturbation! \n");
				f_pretOutput[1] = pert_mag;
      		//  pert_on = 1;
			}
		else		// The impulse should last 150 ms
			{
        		//pert_on = 0;
      			//  printf("End of the force impulse perturbation! \n");
				f_pretOutput[0] = 0;
				f_pretOutput[1] = 0;
				f_pretOutput[2] = 0;
			}
      	

			f_pret[0] = f_pretOutput[0];
			f_pret[1] = f_pretOutput[1];
			f_pret[2] = f_pretOutput[2];

		}
		else{
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
			//pretAmplitude_x = 0.0;
			if (f_pretOutput[0] >= 0 ) {
				f_pretOutput[0] = pretAmplitude_x;
			} else if (f_pretOutput[0] < 0 ){
				f_pretOutput[0] = -pretAmplitude_x;
			}
			//pretAmplitude_y = 0.0;
			if (f_pretOutput[1] >= 0 ) {
				f_pretOutput[1] = pretAmplitude_y;
			} else if (f_pretOutput[1] < 0 ){
				f_pretOutput[1] = -pretAmplitude_y;
			}

			// only in x, y direction
			// f_pretOutput[0] = 0;
			// f_pretOutput[1] = 0;
		
			f_pret[0] = f_pretOutput[0];
			f_pret[1] = f_pretOutput[1];
			f_pret[2] = f_pretOutput[2]; 

		}
		}

		tau_pret = J_x.transpose()*(f_pret);


		// Sum torque commands
		tau = tau_q + tau_x + tau_pret;
		// Save outputs
		// Save outputs
		prevPret[0] = f_pretOutput[0];
		prevPret[1] = f_pretOutput[1];
		prevPret[2] = f_pretOutput[2]; 

		torqueOutput[0] = tau[0];
		torqueOutput[1] = tau[1];
		torqueOutput[2] = tau[2];
		torqueOutput[3] = tau[3];

		// update readtime variable
		rdt++;

		this->outputValue1->setData(&torqueOutput);
		this->outputValue2->setData(&rdt);
		this->outputValue3->setData(&f_pretOutput);
		this->outputValue4->setData(&iteration);
		this->outputValue5->setData(&task_state);
	}

private:
	DISALLOW_COPY_AND_ASSIGN(JointControlClass);
};


template<size_t DOF>
class ControllerWarper{
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
	private:
	// wam variables
	ProductManager& pm;
	systems::Wam<DOF>& wam;
	systems::ToolForceToJointTorques<DOF> tf2jt;
	systems::Summer<jt_type, 2> jtSum;
	// controller variables
	jp_type input_q_00;
	cp_type input_x_00;
	cp_type	center_pos; 	// keep this inorder to same with the old code.
	cp_type center_pos0; 	// the reference center [-0.448, 0.418, 0]
	Matrix_4x4 K_q0;		// free-moving stiffness and damping
	Matrix_4x4 B_q0; 
	Matrix_3x3 K_x0;			
	Matrix_3x3 B_x0;
	Matrix_4x4 K_q1;		// locked stiffness and damping
	Matrix_4x4 B_q1;
	Matrix_3x3 K_x1; 		
	Matrix_3x3 B_x1;
	bool forceMet;
	bool TrackRef;
	public:
	JointControlClass<DOF> jj;
	ControllerWarper(ProductManager& pm, systems::Wam<DOF>& wam, 
		Matrix_4x4 K_q0, Matrix_4x4 K_q1, Matrix_4x4 B_q0, Matrix_4x4 B_q1,
		Matrix_3x3 K_x0, Matrix_3x3 K_x1, Matrix_3x3 B_x0, Matrix_3x3 B_x1,
		jp_type input_q_00, cp_type input_x_00):
	pm(pm), wam(wam),
	K_q0(K_q0), B_q0(B_q0), K_q1(K_q1), B_q1(B_q1),
	K_x0(K_x0), B_x0(B_x0), K_x1(K_x1), B_x1(B_x1),
	input_q_00(input_q_00), input_x_00(input_x_00), center_pos(input_x_00), center_pos0(input_x_00),
	jj(pm, K_q0, B_q0, K_x0, B_x0, K_q1, B_q1, input_q_00, input_x_00, wam),
	forceMet(false), TrackRef(false){
	// after initilization, mvoeTo
	printf("Move to joint controller position, in CustomClass:: Controller Wrapper.");
	wam.moveTo(jj.input_q_0);
	barrett::btsleep(5.0);
  //pert_on = 0;
	}

	~ControllerWarper(){}

	bool init() {
		wam.gravityCompensate();
  		pm.getSafetyModule()->setVelocityLimit(2.5);  // Was 2.5 (Hongwei, 9/5/2019)
  		pm.getSafetyModule()->setTorqueLimit(4.5);    // Was 4.5 (Hongwei, 9/5/2019)

  		wam.moveTo(jj.input_q_0); //center_pos
		TrackRef = false;
  		barrett::btsleep(0.5);
		
  		wam.idle();
  		//printf("Begin idle \n");
  		return true;
	}

	void setCenter_endpoint(cp_type newCenter) {
  		//printf("Enter function: setCenter.");
		center_pos = newCenter; 
		jj.setx0(center_pos);
	}

	void setCenter_joint(double *jointCenter) {
		// copy the value of each element
		input_q_00[0] = jointCenter[0];
		input_q_00[1] = jointCenter[1];
		input_q_00[2] = jointCenter[2];
		input_q_00[3] = jointCenter[3]; 
		// set to jj
		jj.setq0(input_q_00);
	}

	void startController(){
		// connect 
	}

	void setForceMet(bool wasMet){
		forceMet = wasMet;
		
		if (wasMet){
			// change the K_q to a low value here
			jj.setImpedance(K_x0, B_x0);
			printf("\nset impedance to: %.3f, %.3f, %.3f\n", K_x0(0,0), K_x0(1,1), K_x0(2,2));
			//jj.setJointImpedance(K_q0); // decrease impedance suddenly
			//printf("\nset impedance to: %.3f, %.3f, %.3f, %.3f\n", K_q1(0,0), K_q1(1,1), K_q1(2,2), K_q1(3,3));
		}
		else {
			// change the K_q to a high value here
			jj.update_input_time0();			// initializing ramp
			jj.setImpedance(K_x1, B_x1);
			printf("\nset impedance to: %.3f, %.3f, %.3f\n", K_x1(0,0), K_x1(1,1), K_x1(2,2));
			//jj.setJointImpedance(K_q1); // increase impedance steadily
			//printf("\nset impedance to: %.3f, %.3f, %.3f, %.3f\n", K_q0(0,0), K_q0(1,1), K_q0(2,2), K_q0(3,3));
		}
	}

	void moveToq0(void ){
		wam.moveTo(jj.input_q_0); //center_pos
		TrackRef = false;
  		barrett::btsleep(0.5);
	}

	void connectForces() {
    	printf("Enter function: connectForces.");
  		barrett::systems::modXYZ<cp_type> mod_axes;
		systems::connect(wam.kinematicsBase.kinOutput, tf2jt.kinInput);
		systems::connect(jj.time.output, jj.timeInput);
 		systems::connect(wam.jpOutput, jj.wamJPInput);
 		systems::connect(wam.jvOutput, jj.wamJVInput);
		systems::connect(wam.toolPosition.output, jj.wamCPInput);	
 		systems::connect(wam.toolVelocity.output, jj.wamCVInput);
		// track reference
		// wam.trackReferenceSignal(jtSum.output);
		wam.trackReferenceSignal(jj.wamJTOutput);
		TrackRef = true;
  		BARRETT_UNITS_FIXED_SIZE_TYPEDEFS;
	}

	void trackSignal(){ //enable the tracking signal out of CustomClass
		wam.trackReferenceSignal(jj.wamJTOutput);
		TrackRef = true;
  		BARRETT_UNITS_FIXED_SIZE_TYPEDEFS;
		printf("Track Ref! \n");
	}

	bool isTrackRef(){
		return TrackRef;
	}

};

template<size_t DOF>
class LoggerClass{
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
public:
  	ProductManager& pm;
	systems::Wam<DOF>& wam;
	//systems::Ramp time;
	systems::TupleGrouper<double, jp_type, jv_type, cp_type, cv_type, jt_type, cf_type, int, int, int> tg;
	typedef boost::tuple<double, jp_type, jv_type, cp_type, cv_type, jt_type, cf_type, int, int, int> tuple_type;
	const size_t PERIOD_MULTIPLIER;
	char *tmpFile;
	const char* tmpFileName;
	systems::PeriodicDataLogger<tuple_type> logger;
  ControllerWarper<DOF> & controller1; 
	explicit LoggerClass(ProductManager& pm, systems::Wam<DOF>& wam, char *fname, char *tmpfname, ControllerWarper<DOF>& controller):
  				PERIOD_MULTIPLIER(1), 
				//time(pm.getExecutionManager(), 1.0),
				tmpFile(tmpfname),
  				logger(pm.getExecutionManager(), new log::RealTimeWriter<tuple_type>(tmpFile, PERIOD_MULTIPLIER * pm.getExecutionManager()->getPeriod()), PERIOD_MULTIPLIER),
				pm(pm), wam(wam), 
  				controller1(controller),
				tmpFileName(fname){
    	printf("Entered the class Constructor!\n");
		printf("Fished the Constructor.");
	}
	
	virtual ~LoggerClass(){}

public:
	void datalogger_connect(){
		printf("Start connecting! \n");
		systems::connect(controller1.jj.time.output, tg.template getInput<0>());
		systems::connect(wam.jpOutput, tg.template getInput<1>());
		systems::connect(wam.jvOutput, tg.template getInput<2>());
		systems::connect(wam.toolPosition.output, tg.template getInput<3>());
		systems::connect(wam.toolVelocity.output, tg.template getInput<4>());
		systems::connect(controller1.jj.wamJTOutput, tg.template getInput<5>());
		systems::connect(controller1.jj.wamCFPretOutput, tg.template getInput<6>());
		systems::connect(controller1.jj.wamIterationOutput, tg.template getInput<7>());
		systems::connect(controller1.jj.wamRDTOutput, tg.template getInput<8>());
		systems::connect(controller1.jj.wamTaskState, tg.template getInput<9>());
	}
	void datalogger_start(){
		printf("Start timming! \n");
		controller1.jj.time.start();
		printf("Connect input! \n");
		connect(tg.output, logger.input);
		printf("Logging started.\n");
	}
	void datalogger_end(){
		logger.closeLog();
		printf("Logging stopped.\n");
		log::Reader<tuple_type> lr(tmpFile);
		if (fname_init){
			lr.exportCSV(fname_rtma.c_str());
			printf("save to: %s",fname_rtma.c_str());
		}
		else{
			lr.exportCSV(tmpFileName);
			printf("Output written to %s.\n", tmpFileName);
		}
		
		std::remove(tmpFile);
	}
};

#endif
