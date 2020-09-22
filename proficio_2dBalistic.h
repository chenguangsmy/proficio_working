/*  Copyright 2016 Barrett Technology <support@barrett.com>
 *
 *  This file is part of proficio_toolbox.
 *
 *  This version of proficio_toolbox is free software: you can redistribute it
 *  and/or modify it under the terms of the GNU General Public License as
 *  published by the Free Software Foundation, either version 3 of the
 *  License, or (at your option) any later version.
 *
 *  This version of proficio_toolbox is distributed in the hope that it will be
 *  useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this version of proficio_toolbox.  If not, see
 *  <http://www.gnu.org/licenses/>.
 * 
 * 
 */

/** @file haptic_world.cpp
 *
 *  I/O    | Description
 *  ------ | ------------
 *  input  | Cartesian velocity of the robot joints
 *  output | Cartesian force for user gravity compensation
 *
 *  NOTICE: This program is for demonstration purposes only. It is not approved
 *  for clinical use.
 *
 *  This program does not comply with the Barrett C++ Coding standard
 */

#ifndef PROFICIO_DEMOS_CUBE_SPHERE_H
#define PROFICIO_DEMOS_CUBE_SPHERE_H

#include <stdexcept>
#include <errno.h>

#include <syslog.h>
#include <unistd.h>     // for close()
#include <sys/socket.h> // For sockets
#include <fcntl.h>      // To change socket to nonblocking mode
#include <arpa/inet.h>  // For inet_pton()

#include <barrett/detail/ca_macro.h>
#include <barrett/os.h>                          // NOLINT(build/include_order)
#include <barrett/systems/abstract/single_io.h>
#include <barrett/thread/disable_secondary_mode_warning.h>
#include <barrett/units.h>
#include <barrett/math.h>

//#include <proficio/systems/utilities.h>          // NOLINT(build/include_order)


template <size_t DOF>
class NetworkHaptics
    : public barrett::systems::SingleIO<
          barrett::math::Vector<6>::type,
          boost::tuple<double, barrett::math::Vector<6>::type> > {
  BARRETT_UNITS_TYPEDEFS(6);

 public:
  static const int SIZE_OF_MSG = 6 * sizeof(double);
  static const int SIZE_OF_MSG_RECV = 5 * sizeof(double);

  /** Set up networking:
   *  - Create socket
   *  - Set socket to non-blocking and set flags
   *  - Set up buffer size
   *  - Set up the bind address
   *  - Set up address of remote host
   *  - Call "connect" to set datagram destination
   */
  explicit NetworkHaptics(barrett::systems::ExecutionManager* exec_manager,
                          const char* remote_host,
//                          proficio::systems::UserGravityCompensation<DOF>* gc,
                          int port_src = 5557, int port_dest = 5556,
                          const std::string& sys_name = "NetworkHaptics")
      : barrett::systems::SingleIO<v_type, boost::tuple<double, v_type> >(
            sys_name),
        sock_(-1),
        curr_vel_(0.0){
 //       user_grav_comp_(gc) {
    int err;
    long flags;
    int buflen;
    char* errResp;
    unsigned int buflenlen;
    struct sockaddr_in bind_addr;
    struct sockaddr_in their_addr;
    v_type zero_v(0.0);
    tuple_msg_.get<1>() = zero_v;
    tuple_msg_.get<0>() = 10;
    printf("STP1\n");
    /* Create socket */
    sock_ = socket(PF_INET, SOCK_DGRAM, 0);
    if (sock_ == -1) { 
      ctor_error_handler("Could not create socket", __func__);
    }

    /* Set socket to non-blocking, set flag associated with open file */
    flags = fcntl(sock_, F_GETFL, 0);
    if (flags < 0) {
      ctor_error_handler("Could not get socket flags.", __func__);
    }

    flags |= O_NONBLOCK;
    err = fcntl(sock_, F_SETFL, flags);
    if (err < 0) {
      ctor_error_handler("Could not set socket flags.", __func__);
    }

    /* Maybe set UDP buffer size? */
    buflenlen = sizeof(buflen);
    err = getsockopt(sock_, SOL_SOCKET, SO_SNDBUF, (char*)&buflen, &buflenlen);  // NOLINT
    if (err) {
      ctor_error_handler("Could not get output buffer size.", __func__);
    }
    barrett::logMessage("%s: Note, output buffer is %d bytes.") % __func__ %
        buflen;

    buflenlen = sizeof(buflen);
    buflen = 11 * SIZE_OF_MSG_RECV;
    err = setsockopt(sock_, SOL_SOCKET, SO_SNDBUF, (char*)&buflen, buflenlen);
    if (err)
    {
      ctor_error_handler("Could not set output buffer size.", __func__);

    }

    /* Set reuse port */
    int iSetOption = 1;
    err = setsockopt(sock_, SOL_SOCKET, SO_REUSEADDR, (char*)&iSetOption, sizeof(iSetOption));
    if (err)
    {
      ctor_error_handler("Could not set reuse.", __func__);
    }
    printf("STP2\n");
    /* Set up the bind address */
    bind_addr.sin_family = AF_INET;
    bind_addr.sin_port = htons(port_src);
    bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    err = bind(sock_, (struct sockaddr*)&bind_addr, sizeof(bind_addr));
    if (err == -1)
    {
      // Determine Errno
      errResp = (char *)"Unknown cause";
      switch(errno)
      {
        case EACCES: errResp = (char *)"Protected address"; break;
        case EADDRINUSE: errResp = (char *)"Address in use"; break;
        case EBADF: errResp = (char *)"Sockfd not valid"; break;
        case EINVAL: errResp = (char *)"Socket already bound"; break;
        case ENOTSOCK: errResp = (char *)"Socket not socket"; break;
        case EADDRNOTAVAIL: errResp = (char *)"Addr not local"; break;
        case EFAULT: errResp = (char *)"Addr not accessible by user"; break;
        case ELOOP: errResp = (char *)"Too many symbolic links"; break;
        case ENAMETOOLONG: errResp = (char *)"Addr name too long"; break;
        case ENOENT: errResp = (char *)"File doesnt exists"; break;
        case ENOMEM: errResp = (char *)"Not enough kernel memory"; break;
        case ENOTDIR: errResp = (char *)"Path prefix not directory"; break;
        case EROFS: errResp = (char *)"Socket in read only file system"; break;
      }
      //(barrett::logMessage("(NetworkHaptics::NetworkHaptics): Ctor failed %s: Could not bind to socket on port %d") % __func__ % port_src).raise<std::runtime_error>();
      barrett::logMessage(
        "(NetworkHaptics::NetworkHaptics): Ctor failed %s: Couldn't "
        "bind to socket on port 5557: %s") % __func__ % errResp;
      throw std::runtime_error( errResp );
    }

    /* Set up the other guy's address */
    their_addr.sin_family = AF_INET; // address family...
    their_addr.sin_port = htons(port_dest); //unsigned short from host to network byte order
    err = !inet_pton(AF_INET, remote_host, &their_addr.sin_addr); // convert IP to human readable address
    if (err) {
      barrett::logMessage(
          "(NetworkHaptics::NetworkHaptics): Constructor failed %s: "
          "Bad IP argument '%s'.") %
          __func__ % remote_host;
      throw std::runtime_error(
          "(NetworkHaptics::NetworkHaptics): Bad IP argument.");
    }

    /* Call "connect" to set datagram destination */
    err = connect(sock_, (struct sockaddr*)&their_addr, sizeof(struct sockaddr));
    if (err) {
      ctor_error_handler("Could not set datagram destination.", __func__);
    }

    if (exec_manager != NULL) {
      exec_manager->startManaging(*this);
    }
    printf("STP3\n");
  }

  virtual ~NetworkHaptics() {
    mandatoryCleanUp();
    close(sock_);
  }


  /** Handles basic errors in the constructor by logging them with
   *  barrett::logMessage, and throwing a runtime error
   *
   *  @param failure_type   brief description of failure
   *  @param func_name      name of the function where the failure occurred
   */
  void ctor_error_handler(const std::string& failure_type,
                          const std::string& func_name) {
    std::string log_msg =
        "(NetworkHaptics::NetworkHaptics): Constructor failed ";
    log_msg.append(func_name);
    log_msg.append(failure_type);
    std::string error_msg = "(NetworkHaptics::NetworkHaptics): ";
    error_msg.append(failure_type);
    barrett::logMessage(log_msg);
    throw std::runtime_error(error_msg);
  }

 protected:
  int sock_;
  v_type curr_vel_;
  boost::tuple<double, v_type> tuple_msg_;
  // proficio::systems::UserGravityCompensation<DOF>* user_grav_comp_;

  /** Send current velocity. Get user gravity compensation message
   *  and set/increment/decrement the gain as specified
   */
  virtual void operate() {
    curr_vel_ = input.getValue();
    {
      // send() and recv() cause switches to secondary mode. The socket is
      // non-blocking, so this *probably* won't impact the control-loop
      // timing that much...
      barrett::thread::DisableSecondaryModeWarning dsmw;
      send(sock_, curr_vel_.data(), SIZE_OF_MSG, 0);
      recv(sock_, &tuple_msg_, SIZE_OF_MSG_RECV, 0);
/*      if (tuple_msg_.get<0>() == 0)
        user_grav_comp_->setGainZero();
      else if (tuple_msg_.get<0>() == 1)
        user_grav_comp_->incrementGain();
      else if (tuple_msg_.get<0>() == 2)
        user_grav_comp_->decrementGain();
        */
    }
    outputValue->setData(&tuple_msg_);
  }

 private:
  DISALLOW_COPY_AND_ASSIGN(NetworkHaptics);
};

#endif 
