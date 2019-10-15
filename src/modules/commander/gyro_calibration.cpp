 /****************************************************************************
 *
 *   Copyright (c) 2013-2017 PX4 Development Team. All rights reserved.
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
 * @file gyro_calibration.cpp
 *
 * Gyroscope calibration routine
 */

#include <px4_config.h>
#include "gyro_calibration.h"
#include "calibration_messages.h"
#include "calibration_routines.h"
#include "commander_helper.h"

#include <px4_posix.h>
#include <px4_defines.h>
#include <px4_time.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <cmath>
#include <string.h>
#include <drivers/drv_hrt.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/sensor_correction.h>
#include <drivers/drv_gyro.h>
#include <systemlib/mavlink_log.h>
#include <parameters/param.h>
#include <systemlib/err.h>

static const char *sensor_name = "gyro";

static const unsigned max_gyros = 3;

/// Data passed to calibration worker routine
typedef struct  {
    orb_advert_t		*mavlink_log_pub;//传递消息出去
    int32_t			device_id[max_gyros];//设备id号
    int			gyro_sensor_sub[max_gyros];//sensor.msg订阅句柄
    int			sensor_correction_sub;//sensor_correct.nsg订阅句柄，但是好像最后没有发布
    struct gyro_calibration_s	gyro_scale[max_gyros];//校正参数结构体
    struct gyro_report	gyro_report_0;//sensor_0数据，用来探测校正中是否发生移动
} gyro_worker_data_t;

static calibrate_return gyro_calibration_worker(int cancel_sub, void* data)//
{
    gyro_worker_data_t*	worker_data = (gyro_worker_data_t*)(data);//将data的类型强制转换
    unsigned		calibration_counter[max_gyros] = { 0 }, slow_count = 0;//初始化计数器和slow_count
	const unsigned		calibration_count = 5000;
    struct gyro_report	gyro_report;//结构体实例居然可以一模一样哈哈
    unsigned		poll_errcount = 0;//订阅的错误数

    struct sensor_correction_s sensor_correction; /**< sensor thermal corrections**/
    //订阅sensor_correction.msg改正值和比例值重新初始化，比例为1，平移为0(之前赋值的不是msg包是word_data)
	if (orb_copy(ORB_ID(sensor_correction), worker_data->sensor_correction_sub, &sensor_correction) != 0) {
		/* use default values */
        memset(&sensor_correction, 0, sizeof(sensor_correction));
		for (unsigned i = 0; i < 3; i++) {
			sensor_correction.gyro_scale_0[i] = 1.0f;
			sensor_correction.gyro_scale_1[i] = 1.0f;
			sensor_correction.gyro_scale_2[i] = 1.0f;
		}
	}

    px4_pollfd_struct_t fds[max_gyros];//定义订阅的句柄结构体数组
	for (unsigned s = 0; s < max_gyros; s++) {
        fds[s].fd = worker_data->gyro_sensor_sub[s];//赋值
        fds[s].events = POLLIN;//阻塞等待
	}
    //先初始化
	memset(&worker_data->gyro_report_0, 0, sizeof(worker_data->gyro_report_0));


	/* use slowest gyro to pace, but count correctly per-gyro for statistics */
	while (slow_count < calibration_count) {
        if (calibrate_cancel_check(worker_data->mavlink_log_pub, cancel_sub)) {//可能检查校正命令是否被取消，并提示
			return calibrate_return_cancelled;
		}

		/* check if there are new thermal corrections */
		bool updated;
        orb_check(worker_data->sensor_correction_sub, &updated);//检查sensor_correction.msg包是否更新数据

		if (updated) {
			orb_copy(ORB_ID(sensor_correction), worker_data->sensor_correction_sub, &sensor_correction);
		}

        int poll_ret = px4_poll(&fds[0], max_gyros, 1000);//判断是否句柄准备好了读写操作？

        if (poll_ret > 0) {//如果通道没错，开始读取数据
            unsigned update_count = calibration_count;//这个是为了找最慢的传感器更新次数而设置的
            //
            //这循环是进行采样数据，将三个传感器采集到规定的数据个数就停止，【其中有个处理比较好，就是对每个传感器进行判断，不同采样频率的传感器也能一起校正】
            for (unsigned s = 0; s < max_gyros; s++) {//循环
                if (calibration_counter[s] >= calibration_count) {//判断采样数据是否足够
					// Skip if instance has enough samples
					continue;
				}

                bool changed;//对订阅的gyro的句柄进行判断是否进行了更新
				orb_check(worker_data->gyro_sensor_sub[s], &changed);

				if (changed) {
                    orb_copy(ORB_ID(sensor_gyro), worker_data->gyro_sensor_sub[s], &gyro_report);//数据读取

                    if (s == 0) {//然后就是3个传感器进行累加
						// take a working copy
						worker_data->gyro_scale[s].x_offset += (gyro_report.x - sensor_correction.gyro_offset_0[0]) * sensor_correction.gyro_scale_0[0];
						worker_data->gyro_scale[s].y_offset += (gyro_report.y - sensor_correction.gyro_offset_0[1]) * sensor_correction.gyro_scale_0[1];
						worker_data->gyro_scale[s].z_offset += (gyro_report.z - sensor_correction.gyro_offset_0[2]) * sensor_correction.gyro_scale_0[2];

						// take a reference copy of the primary sensor including correction for thermal drift
                        orb_copy(ORB_ID(sensor_gyro), worker_data->gyro_sensor_sub[s], &worker_data->gyro_report_0);//这个写法看不懂，暂时不知道干什么用
						worker_data->gyro_report_0.x = (gyro_report.x - sensor_correction.gyro_offset_0[0]) * sensor_correction.gyro_scale_0[0];
						worker_data->gyro_report_0.y = (gyro_report.y - sensor_correction.gyro_offset_0[1]) * sensor_correction.gyro_scale_0[1];
						worker_data->gyro_report_0.z = (gyro_report.z - sensor_correction.gyro_offset_0[2]) * sensor_correction.gyro_scale_0[2];

					} else if (s == 1) {
						worker_data->gyro_scale[s].x_offset += (gyro_report.x - sensor_correction.gyro_offset_1[0]) * sensor_correction.gyro_scale_1[0];
						worker_data->gyro_scale[s].y_offset += (gyro_report.y - sensor_correction.gyro_offset_1[1]) * sensor_correction.gyro_scale_1[1];
						worker_data->gyro_scale[s].z_offset += (gyro_report.z - sensor_correction.gyro_offset_1[2]) * sensor_correction.gyro_scale_1[2];

					} else if (s == 2) {
						worker_data->gyro_scale[s].x_offset += (gyro_report.x - sensor_correction.gyro_offset_2[0]) * sensor_correction.gyro_scale_2[0];
						worker_data->gyro_scale[s].y_offset += (gyro_report.y - sensor_correction.gyro_offset_2[1]) * sensor_correction.gyro_scale_2[1];
						worker_data->gyro_scale[s].z_offset += (gyro_report.z - sensor_correction.gyro_offset_2[2]) * sensor_correction.gyro_scale_2[2];

					} else {
						worker_data->gyro_scale[s].x_offset += gyro_report.x;
						worker_data->gyro_scale[s].y_offset += gyro_report.y;
						worker_data->gyro_scale[s].z_offset += gyro_report.z;

					}

                    calibration_counter[s]++;//校正计数增加，增加到max_counter就跳出了

				}

				// Maintain the sample count of the slowest sensor 
                //判断改正计数不为0并且采样计数小于update_count为了找出采样率最低的传感器所采集的个数
				if (calibration_counter[s] && calibration_counter[s] < update_count) {
					update_count = calibration_counter[s];
				}

            }//采集for循环结束

            if (update_count % (calibration_count / 20) == 0) {//还需要在进行累加
				calibration_log_info(worker_data->mavlink_log_pub, CAL_QGC_PROGRESS_MSG, (update_count * 100) / calibration_count);
			}

			// Propagate out the slowest sensor's count
            if (slow_count < update_count) {//如果slow_count<0,即updaue_count没有被赋值，增加错误次数
				slow_count = update_count;
			}

		} else {
			poll_errcount++;
		}

        if (poll_errcount > 1000) {//如果数据输入错误1000次，提示
			calibration_log_critical(worker_data->mavlink_log_pub, CAL_ERROR_SENSOR_MSG);
			return calibrate_return_error;
		}
	}

	for (unsigned s = 0; s < max_gyros; s++) {
        if (worker_data->device_id[s] != 0 && calibration_counter[s] < calibration_count / 2) {//如果有采样的数据小于规定数报错
			calibration_log_critical(worker_data->mavlink_log_pub, "ERROR: missing data, sensor %d", s)
			return calibrate_return_error;
		}
        //直接计算
		worker_data->gyro_scale[s].x_offset /= calibration_counter[s];
		worker_data->gyro_scale[s].y_offset /= calibration_counter[s];
		worker_data->gyro_scale[s].z_offset /= calibration_counter[s];
	}

	return calibrate_return_ok;
}

int do_gyro_calibration(orb_advert_t *mavlink_log_pub)//可能该参数是为了传出信息给mavlink
{
	int			res = PX4_OK;
    gyro_worker_data_t	worker_data = {};//陀螺仪工作数据结构体实例化

    calibration_log_info(mavlink_log_pub, CAL_QGC_STARTED_MSG, sensor_name);//我的理解是把[cal] calibration started: 2 gyro传进入传出信息

	worker_data.mavlink_log_pub = mavlink_log_pub;

    struct gyro_calibration_s gyro_scale_zero;//gyro改正数结构体实例化
    gyro_scale_zero.x_offset = 0.0f;//初始化
	gyro_scale_zero.x_scale = 1.0f;
	gyro_scale_zero.y_offset = 0.0f;
	gyro_scale_zero.y_scale = 1.0f;
	gyro_scale_zero.z_offset = 0.0f;
	gyro_scale_zero.z_scale = 1.0f;

	int device_prio_max = 0;
	int32_t device_id_primary = 0;

    worker_data.sensor_correction_sub = orb_subscribe(ORB_ID(sensor_correction));//订阅改正数包，暂时不知道干什么用

    for (unsigned s = 0; s < max_gyros; s++) {//这个循环主要是（通过串口）1、将gyro的id重新获取了一遍 2、设置gyro的比例系数为1
        char str[30];

		// Reset gyro ids to unavailable.
		worker_data.device_id[s] = 0;
		// And set default subscriber values.
		worker_data.gyro_sensor_sub[s] = -1;
        (void)sprintf(str, "CAL_GYRO%u_ID", s);
        //para_set_no_notification是重置id为0
        res = param_set_no_notification(param_find(str), &(worker_data.device_id[s]));
		if (res != PX4_OK) {
			calibration_log_critical(mavlink_log_pub, "Unable to reset CAL_GYRO%u_ID", s);
			return PX4_ERROR;
		}

		// Reset all offsets to 0 and scales to 1
		(void)memcpy(&worker_data.gyro_scale[s], &gyro_scale_zero, sizeof(gyro_scale_zero));
#ifdef __PX4_NUTTX
        sprintf(str, "%s%u", GYRO_BASE_DEVICE_PATH, s);//设备路径
		int fd = px4_open(str, 0);
		if (fd >= 0) {
            worker_data.device_id[s] = px4_ioctl(fd, DEVIOCGDEVICEID, 0);//（ioctl(int fd,cmd)）返回fd的设备号
            res = px4_ioctl(fd, GYROIOCSSCALE, (long unsigned int)&gyro_scale_zero);//设置gyro的比例系数
            px4_close(fd);//关闭设备进程

			if (res != PX4_OK) {
				calibration_log_critical(mavlink_log_pub, CAL_ERROR_RESET_CAL_MSG, s);
				return PX4_ERROR;
			}
		}
#else
		(void)sprintf(str, "CAL_GYRO%u_XOFF", s);
		res = param_set_no_notification(param_find(str), &gyro_scale_zero.x_offset);
		if (res != PX4_OK) {
			PX4_ERR("unable to reset %s", str);
		}
		(void)sprintf(str, "CAL_GYRO%u_YOFF", s);
		res = param_set_no_notification(param_find(str), &gyro_scale_zero.y_offset);
		if (res != PX4_OK) {
			PX4_ERR("unable to reset %s", str);
		}
		(void)sprintf(str, "CAL_GYRO%u_ZOFF", s);
		res = param_set_no_notification(param_find(str), &gyro_scale_zero.z_offset);
		if (res != PX4_OK) {
			PX4_ERR("unable to reset %s", str);
		}
		(void)sprintf(str, "CAL_GYRO%u_XSCALE", s);
		res = param_set_no_notification(param_find(str), &gyro_scale_zero.x_scale);
		if (res != PX4_OK) {
			PX4_ERR("unable to reset %s", str);
		}
		(void)sprintf(str, "CAL_GYRO%u_YSCALE", s);
		res = param_set_no_notification(param_find(str), &gyro_scale_zero.y_scale);
		if (res != PX4_OK) {
			PX4_ERR("unable to reset %s", str);
		}
		(void)sprintf(str, "CAL_GYRO%u_ZSCALE", s);
		res = param_set_no_notification(param_find(str), &gyro_scale_zero.z_scale);
		if (res != PX4_OK) {
			PX4_ERR("unable to reset %s", str);
		}
		param_notify_changes();
#endif

	}

	// We should not try to subscribe if the topic doesn't actually exist and can be counted.
    const unsigned orb_gyro_count = orb_group_count(ORB_ID(sensor_gyro));//返回gyro的个数

	// Warn that we will not calibrate more than max_gyros gyroscopes
    if (orb_gyro_count > max_gyros) {//如果gyro的个数超出了提示
		calibration_log_critical(mavlink_log_pub, "Detected %u gyros, but will calibrate only %u", orb_gyro_count, max_gyros);
	}


    //这个循环检测是否能订阅到数据，判断id号是否一致，返回主传感器，赋值gyro数据句柄
	for (unsigned cur_gyro = 0; cur_gyro < orb_gyro_count && cur_gyro < max_gyros; cur_gyro++) {

		// Lock in to correct ORB instance
        bool found_cur_gyro = false;//这个参数用来判断是否有订阅到数据
		for(unsigned i = 0; i < orb_gyro_count && !found_cur_gyro; i++) {
            //订阅gyro的数据
			worker_data.gyro_sensor_sub[cur_gyro] = orb_subscribe_multi(ORB_ID(sensor_gyro), i);

            struct gyro_report report;//实例化gyro数据结构体
            orb_copy(ORB_ID(sensor_gyro), worker_data.gyro_sensor_sub[cur_gyro], &report);//将订阅的数据赋值给结构体实例

#ifdef __PX4_NUTTX

			// For NuttX, we get the UNIQUE device ID from the sensor driver via an IOCTL
			// and match it up with the one from the uORB subscription, because the
			// instance ordering of uORB and the order of the FDs may not be the same.
            //从驱动通过IOCTL获取有可能和uORB获取到的数据不同，需要判断是否相同
			if(report.device_id == worker_data.device_id[cur_gyro]) {
				// Device IDs match, correct ORB instance for this gyro
				found_cur_gyro = true;
			} else {
                orb_unsubscribe(worker_data.gyro_sensor_sub[cur_gyro]);//如果不同解除订阅
			}

#else

			// For the DriverFramework drivers, we fill device ID (this is the first time) by copying one report.
			worker_data.device_id[cur_gyro] = report.device_id;
			found_cur_gyro = true;

#endif
		}

        if(!found_cur_gyro) {//如果没有找到，就提示报个错
			calibration_log_critical(mavlink_log_pub, "Gyro #%u (ID %u) no matching uORB devid", cur_gyro, worker_data.device_id[cur_gyro]);
            res = calibrate_return_error;//并返回结果，erro
			break;
		}

		if (worker_data.device_id[cur_gyro] != 0) {
			// Get priority
			int32_t prio;
            orb_priority(worker_data.gyro_sensor_sub[cur_gyro], &prio);//(fd,&prio)获取主传感器

            if (prio > device_prio_max) {//因为device_prio_max初始化设置为0，才能够完成遍历
				device_prio_max = prio;
				device_id_primary = worker_data.device_id[cur_gyro];
			}
		} else {
            //否则报个错，id不存在，因为基本上device_id是不会等于0
            calibration_log_critical(mavlink_log_pub, "Gyro #%u no device id, abort", cur_gyro);
		}
	}

    int cancel_sub  = calibrate_cancel_subscribe();//传回的是有vechile_commander的句柄

	unsigned try_count = 0;
	unsigned max_tries = 20;
    res = PX4_ERROR;//初始化先赋值为错误

	do {
		// Calibrate gyro and ensure user didn't move
        //该函数中进行了数据获取的步骤，并进行解算
        calibrate_return cal_return = gyro_calibration_worker(cancel_sub, &worker_data);

		if (cal_return == calibrate_return_cancelled) {
			// Cancel message already sent, we are done here
			res = PX4_ERROR;
			break;

		} else if (cal_return == calibrate_return_error) {
			res = PX4_ERROR;

		} else {
            /* check offsets 这就找到了为什么在工作的时候要记录一个gyro的数据*/
			float xdiff = worker_data.gyro_report_0.x - worker_data.gyro_scale[0].x_offset;
			float ydiff = worker_data.gyro_report_0.y - worker_data.gyro_scale[0].y_offset;
			float zdiff = worker_data.gyro_report_0.z - worker_data.gyro_scale[0].z_offset;

			/* maximum allowable calibration error in radians */
            const float maxoff = 0.01f;//rad/s 0.6°/s
            //原来在这个地方是否进行移动判断
			if (!PX4_ISFINITE(worker_data.gyro_scale[0].x_offset) ||
			    !PX4_ISFINITE(worker_data.gyro_scale[0].y_offset) ||
			    !PX4_ISFINITE(worker_data.gyro_scale[0].z_offset) ||
			    fabsf(xdiff) > maxoff ||
			    fabsf(ydiff) > maxoff ||
			    fabsf(zdiff) > maxoff) {

                calibration_log_critical(mavlink_log_pub, "motion, retrying..");//提示（这样的话可以自定义给地面站提示）
				res = PX4_ERROR;

			} else {
				res = PX4_OK;
			}
		}
		try_count++;

    } while (res == PX4_ERROR && try_count <= max_tries);//条件是res结果not ok，并未到尝试次数

	if (try_count >= max_tries) {
		calibration_log_critical(mavlink_log_pub, "ERROR: Motion during calibration");
		res = PX4_ERROR;
	}

    calibrate_cancel_unsubscribe(cancel_sub);//取消vechile_commander订阅

	for (unsigned s = 0; s < max_gyros; s++) {
        px4_close(worker_data.gyro_sensor_sub[s]);//取消三个gyro.msg订阅
	}

	if (res == PX4_OK) {

		/* set offset parameters to new values */
		bool failed = false;
        //在这里也没看懂为什么需要进行或运算；将主传感器参数设置为标定到的
        failed = failed || (PX4_OK != param_set_no_notification(param_find("CAL_GYRO_PRIME"), &(device_id_primary)));

		bool tc_locked[3] = {false}; // true when the thermal parameter instance has already been adjusted by the calibrator

		for (unsigned uorb_index = 0; uorb_index < max_gyros; uorb_index++) {
            if (worker_data.device_id[uorb_index] != 0) {//这得判断下数据是否正确
				char str[30];

				/* check if thermal compensation is enabled */
                int32_t tc_enabled_int;//热补偿
                param_get(param_find("TC_G_ENABLE"), &(tc_enabled_int));//获取是否进行温度补偿
                if (tc_enabled_int == 1) {//上面的参数找到了，但是下面具体补偿值找不到
					/* Get struct containing sensor thermal compensation data */
					struct sensor_correction_s sensor_correction; /**< sensor thermal corrections */
					memset(&sensor_correction, 0, sizeof(sensor_correction));
					orb_copy(ORB_ID(sensor_correction), worker_data.sensor_correction_sub, &sensor_correction);

					/* don't allow a parameter instance to be calibrated again by another uORB instance */
					if (!tc_locked[sensor_correction.gyro_mapping[uorb_index]]) {
						tc_locked[sensor_correction.gyro_mapping[uorb_index]] = true;

						/* update the _X0_ terms to include the additional offset */
						int32_t handle;
						float val;
						for (unsigned axis_index = 0; axis_index < 3; axis_index++) {
							val = 0.0f;
							(void)sprintf(str, "TC_G%u_X0_%u", sensor_correction.gyro_mapping[uorb_index], axis_index);
							handle = param_find(str);
							param_get(handle, &val);
							if (axis_index == 0) {
								val += worker_data.gyro_scale[uorb_index].x_offset;

							} else if (axis_index == 1) {
								val += worker_data.gyro_scale[uorb_index].y_offset;

							} else if (axis_index == 2) {
								val += worker_data.gyro_scale[uorb_index].z_offset;

							}
							failed |= (PX4_OK != param_set_no_notification(handle, &val));
						}
						param_notify_changes();
					}

					// Ensure the calibration values used the driver are at default settings
					worker_data.gyro_scale[uorb_index].x_offset = 0.f;
					worker_data.gyro_scale[uorb_index].y_offset = 0.f;
					worker_data.gyro_scale[uorb_index].z_offset = 0.f;
				}
                //设置偏差参数值
				(void)sprintf(str, "CAL_GYRO%u_XOFF", uorb_index);
				failed |= (PX4_OK != param_set_no_notification(param_find(str), &(worker_data.gyro_scale[uorb_index].x_offset)));
				(void)sprintf(str, "CAL_GYRO%u_YOFF", uorb_index);
				failed |= (PX4_OK != param_set_no_notification(param_find(str), &(worker_data.gyro_scale[uorb_index].y_offset)));
				(void)sprintf(str, "CAL_GYRO%u_ZOFF", uorb_index);
				failed |= (PX4_OK != param_set_no_notification(param_find(str), &(worker_data.gyro_scale[uorb_index].z_offset)));
                //设置id
				(void)sprintf(str, "CAL_GYRO%u_ID", uorb_index);
				failed |= (PX4_OK != param_set_no_notification(param_find(str), &(worker_data.device_id[uorb_index])));

#ifdef __PX4_NUTTX
                /* apply new scaling and offsets？ */
				(void)sprintf(str, "%s%u", GYRO_BASE_DEVICE_PATH, uorb_index);
				int fd = px4_open(str, 0);

				if (fd < 0) {
					failed = true;
					continue;
				}
                //设置比例平移参数（但是之前设置过一次了，重复？）【这次是设置的校正之后的实际没有校正比例哈哈】
				res = px4_ioctl(fd, GYROIOCSSCALE, (long unsigned int)&worker_data.gyro_scale[uorb_index]);
				px4_close(fd);

				if (res != PX4_OK) {
					calibration_log_critical(mavlink_log_pub, CAL_ERROR_APPLY_CAL_MSG, 1);
				}
#endif
			}
		}

        if (failed) {//如果失败提示
			calibration_log_critical(mavlink_log_pub, "ERROR: failed to set offset params");
			res = PX4_ERROR;
		}
	}

	/* if there is a any preflight-check system response, let the barrage of messages through */
	usleep(200000);

	if (res == PX4_OK) {
		calibration_log_info(mavlink_log_pub, CAL_QGC_DONE_MSG, sensor_name);
	} else {
		calibration_log_info(mavlink_log_pub, CAL_QGC_FAILED_MSG, sensor_name);
	}

    orb_unsubscribe(worker_data.sensor_correction_sub);//但是好像没有存进sensor_correct.msg包里

	/* give this message enough time to propagate */
	usleep(600000);

	return res;
}
