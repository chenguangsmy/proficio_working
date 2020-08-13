# proficio_working
This is a working proficio 2dballistic program repo. 
This project is not really tidy. Some of the files is useless but not deleted, so it may cause hard to read.


## main function
* proficio_2dBalistic.cpp 
  Where main function locates.
  * Important: should enter the *validate_args* function, in order to have the remote_host IP address.
## useful *.h files
* hapticsDemoClass.h
  * Where robot control algorithm locates.
* proficio_2dBalistic.h
  * Where is networkhaptics class declaration. 
  * *operate()* function in NetworkHaptics class is its update part. It has a singleIO object, but what this function do is mainly send the information to the remote host (actually the same computer), and receieve information, to update the gravity compensation gain.
  * The input and output of this SingleIO object is not correlate with the hapticsDemoClass, where the control algorithm work. 
* burtRTMA.h
  * Where the respond to RTMA system code locates.

## download command 
$ git clone https://github.com/chenguangsmy/proficio_working.git
