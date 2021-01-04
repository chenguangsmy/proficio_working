# proficio_working
This is a working proficio 2D ballistic program repo. 
This project need to be refined later. 

## important functions
* `wam_main()`  
  Main function, locates in `wam_2dBallistic`.
  Responsble for starting up this module, 
  subscribing messages from RTMA system,
  initializing customcontroller values,
  creating respondRTMA threads, 
  and finish. 
  * Important: should enter the *validate_args* function, in order to have the remote_host IP address.

* `CustomClass::operate()`   
  Execution fuction directly updates joints' torques. 
  Locates in `CustomClass.h`. 
  Calculate joints' torques by adding up Cartesian Force, Joint Torque and Perturbation force. 
  Using `wam.trackReferenceSignal()` to let wam rum. 
  (<font color=red>Question to Barrett here </font>)

* `LoggerClass::LoggerClass()`   
  Locates in `CustomClass.h`. 
  Initialize which variable to track in a seperate file. 
  (<font color=red>Need Synchronize time with RTMA here! </font>)   

* `respondToRTMA(...)`  
  A ever-looping function responding RTMA.  
  Locates in `burtRTMA.h`.  
  Response for: 
  sending WAM information to RTMA system, 
  receiving task states, 
  changing low-level WAM controller values (
    K_x
  ).
  (<font color=red>Do we need to change K_q better? </font>)   

## useful *.h files
* CustomClass.h
  * class `JointControlClass`: robot controller. It is a mixture controller containing jp, cp, and pert (joint position, cartesian tool position, and perturbation controller). In the controller, the convert between joint and cartesian positions are using Jacobian matrix , which is acquired from the wam. 
  * class `ControllerWarper`: contains the contorller and other nessesary functions serving the starting and moving of the robot. 
  * class `LoggerClass`: in charge of recording wam robot variables and write it in a seperate *.cvs file. 

* burtRTMA.h
  * Where the respond to RTMA system code locates.
  * Containing the task logic and change data of the robot. 
      * Task logic: 
          * Case1: Begin (set Robot Impedance High)
          * Case2: Present
          * Case3: ForceRamp
          * Case4: Move  (set Robot Impedance Low)
          * Case5: Hold
          * Case6: End
          * Case7: Reset

* movingBurt.h
  * `moveToCenter()`, Used
  * Change the `moveToCenter()` part into emergency. --CG

* recordTrajectory.h
  * A libbarrett function

## download command 
$ git clone https://github.com/chenguangsmy/proficio_working.git

## location on cleave.mbp: 
`/Users/cleave/Documents/proj/BallisticReleaseTaskCode/proficio_working`

## location on cleave.aln:
`/home/cleave/proj.prac/proficio_working`

## related files
* RTMA task config: `rg2/config/Wam.2dDebug.KingKong/XM.target_simple.config`, sending target into `burtRTMA.h`
* RTMA task config: `rg2/config/Wam.2dDebug.KingKong/TrialStatus.conf`, sending target code into `burtRTMA.h`
* RTMA task config: `rg2/config/Wam.2dDebug.KingKong/XM.target_simple.config`, sending target `burtRTMA.h`
    * This item may conflict with the `XM.target_simple.config`, I need to check the `executive.m` file to have a fine look. 