# proficio_working
This is a working proficio 2dballistic program repo. 
This project is not really tidy. Some of the files is useless but not deleted, so it may cause hard to read.


## main function
* proficio_2dBalistic.cpp 
  Where main function locates.
  * Important: should enter the *validate_args* function, in order to have the remote_host IP address.
## useful *.h files
* CustomClass.h
  * class `JointControlClass`: robot controller. It is a mixture controller  containing jp, cp, and pert (joint position, cartesian tool position, and perturbation controller). In the controller, the convert between joint and cartesian positions are using Jacobian matrix , which is acquired from the wam. 
  * class `ControllerWarper`: contains the contorller and other nessesary functions serving the starting and moving of the robot. 
  * class `LoggerClass`: in charge of recording wam robot variables and write it in a seperate *.cvs file. 

* hapticsDemoClass.h
  * Contain files about Task logic with robot. 
  * These are old codes from proficio robot, I have not tidied it up. -- CG

* proficio_2dBalistic.h
  * Where is networkhaptics class declaration. 
  * This class is copied from the libbarrett example 10, but do not really have function in our tasks. 
  * <font 'color' = 0xaaaaaa > *operate()* function in NetworkHaptics class is its update part. It has a singleIO object, but what this function do is mainly send the information to the remote host (actually the same computer), and receieve information, to update the gravity compensation gain. 
  * <font 'color' = 0xaaaaaa > The input and output of this SingleIO object is not correlate with the hapticsDemoClass, where the control algorithm work. </font>

* burtRTMA.h
  * Where the respond to RTMA system code locates.
  * Containing the task logic and change data of the robot. 
      * Task logic: 
          * Case1: 
          * Case2: 
          * Case3: begin
          * Case4: 
          * Case5: final
          * Case6: intertrial
          * Case7: move?

## download command 
$ git clone https://github.com/chenguangsmy/proficio_working.git

## location on Chenguang's laptop: 
`/Users/cleave/Documents/proj/BallisticReleaseTaskCode/proficio_working`

## related files
* RTMA task config: `rg2/config/Wam.2dDebug.KingKong/XM.target_simple.config`, sending target into `burtRTMA.h`
* RTMA task config: `rg2/config/Wam.2dDebug.KingKong/TrialStatus.conf`, sending target code into `burtRTMA.h`
* RTMA task config: `rg2/config/Wam.2dDebug.KingKong/XM.target_simple.config`, sending target `burtRTMA.h`
    * This item may conflict with the `XM.target_simple.config`, I need to check the `executive.m` file to have a fine look. 