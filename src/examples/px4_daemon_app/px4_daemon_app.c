/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file px4_daemon_app.c
 * daemon application example for PX4 autopilot
 *
 * @author Example User <mail@example.com>
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <px4_config.h>
#include <nuttx/sched.h>

#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/uav_position_setpoint.h>
#include <uORB/topics/uav_position_feedback.h>
 #include <uORB/topics/uav_type.h>
#include <uORB/topics/vehicle_local_position.h>

#include <systemlib/systemlib.h>
#include <systemlib/err.h>

static bool thread_should_exit = false;		/**< daemon exit flag */
static bool thread_running = false;		/**< daemon status flag */
static int daemon_task;				/**< Handle of daemon task / thread */

/**
 * daemon management function.
 */
__EXPORT int px4_daemon_app_main(int argc, char *argv[]);

/**
 * Mainloop of daemon.
 */
int px4_daemon_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

static void
usage(const char *reason)
{
	if (reason) {
		warnx("%s\n", reason);
	}

	warnx("usage: daemon {start|stop|status} [-p <additional params>]\n\n");
}

/**
 * The daemon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_create().
 */
int px4_daemon_app_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage("missing command");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			warnx("daemon already running\n");
			/* this is not an error */
			return 0;
		}

		thread_should_exit = false;
		daemon_task = px4_task_spawn_cmd("daemon",
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_DEFAULT,
						 1000,
						 px4_daemon_thread_main,
						 (argv) ? (char *const *)&argv[2] : (char *const *)NULL);

/*        daemon_task = px4_task_spawn_cmd("daemon_send",
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_DEFAULT,
						 2000,
						 px4_daemon_send_thread_main,
						 (argv) ? (char *const *)&argv[2] : (char *const *)NULL);
*/
		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			warnx("\trunning\n");

		} else {
			warnx("\tnot started\n");
		}

		return 0;
	}

	usage("unrecognized command");
	return 1;
}



//-------------------------------------------------------------------------------------------------------------
int px4_daemon_thread_main(int argc, char *argv[])
{

	warnx("[daemon] starting\n");

    int uav_position_feedback_sub;
    struct uav_position_feedback_s mocap_data;
    memset(&mocap_data, 0, sizeof(mocap_data));
    uav_position_feedback_sub = orb_subscribe(ORB_ID(uav_position_feedback)); 


    int uav_position_setpoint_sub;
    struct uav_position_setpoint_s xyz_d;
    memset(&xyz_d, 0, sizeof(xyz_d));
    uav_position_setpoint_sub = orb_subscribe(ORB_ID(uav_position_setpoint));

    int uav_type_sub;
    struct uav_type_s xyz_type;
    memset(&xyz_type, 0, sizeof(xyz_type));
    uav_type_sub = orb_subscribe(ORB_ID(uav_type));
/*
    int vehicle_attitude_sub;
    struct vehicle_attitude_s att_data;
    memset(&att_data, 0, sizeof(att_data));
    vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));

    
    int vehicle_local_position_sub;
    struct vehicle_local_position_s xyz;
    memset(&xyz, 0, sizeof(xyz));
    vehicle_local_position_sub = orb_subscribe(ORB_ID(vehicle_local_position));
*/

    bool updated;

	thread_running = true;
    int i = 0;
	while (!thread_should_exit) {
		//warnx("Hello daemon!");
        printf("Hello daemon:%d\n", ++i);

        orb_check(uav_position_feedback_sub, &updated);
        if(updated)
        {
            orb_copy(ORB_ID(uav_position_feedback), uav_position_feedback_sub, &mocap_data);
            printf("---------------mocap data is--------------\n");
            printf("\tx=%8.4f, y=%8.4f, z=%8.4f\n", (double)mocap_data.x, (double)mocap_data.y, (double)mocap_data.z);
        }
        
        orb_check(uav_position_setpoint_sub, &updated);
        if(updated)
        {
            orb_copy(ORB_ID(uav_position_setpoint), uav_position_setpoint_sub, &xyz_d);
            printf("---------------uav position setpoint is-------------------\n");
            printf("\tx_d=%8.4f, y_d=%8.4f, z_d=%8.4f, yaw_d=%8.4f \n", (double)xyz_d.x_d, (double)xyz_d.y_d, (double)xyz_d.z_d, (double)xyz_d.yaw_d);
        


        }


        orb_check(uav_type_sub, &updated);
        if(updated)
        {
                        printf("---------------uav type is-------------------\n");
            orb_copy(ORB_ID(uav_type), uav_type_sub, &xyz_type);
            if ((int)xyz_type.type == 0)
            printf("type : normal\n");
            else
                if ((int)xyz_type.type == 1)
                    printf("type : idel\n");
                    else
                        if ((int)xyz_type.type == 2)
                            printf("type : land\n");
                             else
                                if ((int)xyz_type.type == 3)
                                    printf("type : takeoff\n");
                                else
                                    printf("type mistake !!!\n");

            if ((bool)xyz_type.flage_position)
                printf("flag_position: 1\n");
            else
                printf("flag_position: 0\n");

        }
        /*
        orb_check(vehicle_attitude_sub, &updated);
        if(updated)
        {
            orb_copy(ORB_ID(vehicle_attitude), vehicle_attitude_sub, &att_data);
            printf("---------------attitude data is--------------\n");
            printf("\troll=%8.4f, pitch=%8.4f, yaw=%8.4f\n", (double)att_data.roll, (double)att_data.pitch, (double)att_data.yaw);
        }

        orb_check(vehicle_local_position_sub, &updated);
        if(updated)
        {
            orb_copy(ORB_ID(vehicle_local_position), vehicle_local_position_sub, &xyz);
            printf("---------------local position data is--------------\n");
            printf("\tx=%8.4f, y=%8.4f, z=%8.4f\n", (double)xyz.x, (double)xyz.y, (double)xyz.z);
        }
*/
		usleep(100000);
	}

	warnx("[daemon] exiting.\n");

	thread_running = false;

	return 0;
}

//-------------------------------------------------------------------------------------------------------------

/*
int px4_daemon_send_thread_main(int argc, char *argv[])
{

	warnx("[daemon_send] starting\n");

    orb_advert_t uav_position_and_reference_pub;
    struct uav_position_and_reference_s xyz_data;
    xyz_data.x = 0.1;
    xyz_data.y = 0.2;
    xyz_data.z = 0.3;
    xyz_data.x_d = 0.4;
    xyz_data.y_d = 0.5;
    xyz_data.z_d = 0.6;
    xyz_data.yaw_d = 0.7;
    uav_position_and_reference_pub = orb_advertise(ORB_ID(uav_position_and_reference), &xyz_data);

    thread_running = true;

	while (!thread_should_exit)
    {
        xyz_data.x += 1;
        xyz_data.y += 1;
        xyz_data.z += 1;
        xyz_data.x_d += 1;
        xyz_data.y_d += 1;
        xyz_data.z_d += 1;
        xyz_data.yaw_d += 1;
        orb_publish(ORB_ID(uav_position_and_reference), uav_position_and_reference_pub, &xyz_data);
        sleep(4);
    }

	warnx("[daemon_send] exiting.\n");

	thread_running = false;

	return 0;
}
*/


