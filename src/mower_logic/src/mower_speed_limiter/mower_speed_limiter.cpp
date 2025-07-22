// Created by Clemens Elflein on 2/18/22, 5:37 PM.
// Copyright (c) 2022 Clemens Elflein. All rights reserved.
//
// This work is licensed under a Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
//
// Feel free to use the design in your private/educational projects, but don't try to sell the design or products based on it without getting my consent first.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
//
#include "ros/ros.h"

// Include messages for mower control
#include "mower_msgs/Status.h"
#include "mower_msgs/ESCStatus.h"
#include "mower_logic/SpeedLimiterDebug.h"
#include "mower_logic/VelocityLimit.h"
#include "mower_logic/MowerSpeedLimiterConfig.h"
#include "dynamic_reconfigure/server.h"

ros::Publisher debug_pub;
ros::Publisher velLim_pub;
mower_msgs::Status status;
mower_msgs::ESCStatus left_esc_status, right_esc_status;
bool left_esc_received = false, right_esc_received = false;
mower_speed_limiter::MowerSpeedLimiterConfig config;
dynamic_reconfigure::Server<mower_speed_limiter::MowerSpeedLimiterConfig> *reconfig_server;


void reconfigureCB(mower_speed_limiter::MowerSpeedLimiterConfig &c, uint32_t level) {
    config = c;
}

void leftESCReceived(const mower_msgs::ESCStatus::ConstPtr &msg) {
    left_esc_status = *msg;
    left_esc_received = true;
}

void rightESCReceived(const mower_msgs::ESCStatus::ConstPtr &msg) {
    right_esc_status = *msg;
    right_esc_received = true;
}


void statusReceived(const mower_msgs::Status::ConstPtr &msg) {
    static bool firstData = true;
    static mower_msgs::Status last_status;   

    // we need the differences, so initialize in the first run
    if (firstData) {

        last_status = *msg;
        firstData = false;

        return;
    }

    double dt = (msg->stamp - last_status.stamp).toSec();
    last_status = *msg;

    //velocity limiting based on mow current (use if erpm not available)
    static double i_cc_error = 1.0, last_cc_error = 0.0;
    double cc_velocity_limit = 1.0, cc_error = 0.0, d_cc = 0.0;
    if(!config.enable_cc) {
        i_cc_error = 0.0; last_cc_error = 0.0;
    } else {
        cc_error = config.mow_current_limit - msg->mower_esc_current;
        i_cc_error += cc_error * dt;
        if ((i_cc_error * config.ki_cc) > 1.0)
            i_cc_error = 1.0/config.ki_cc;
        else if (i_cc_error < 0.0)
            i_cc_error = 0.0;
        d_cc = (cc_error - last_cc_error) / dt;
        last_cc_error = cc_error;
        cc_velocity_limit = cc_error * config.kp_cc + i_cc_error * config.ki_cc + d_cc * config.kd_cc;

        if (cc_velocity_limit > 1.0)
            cc_velocity_limit = 1.0;
        else if (cc_velocity_limit < 0.0)
            cc_velocity_limit = 0.0;
    }

    //velocity limiting based on mow erpm
    static double i_cs_error = 1.0, last_cs_error = 0.0;
    double cs_velocity_limit = 1.0, cs_error = 0.0, d_cs = 0.0;
    if(!config.enable_cs || !msg->mow_enabled) {
        i_cs_error = 0.0; last_cs_error = 0.0;
    } else {
        cs_error = (std::abs(msg->mower_motor_rpm) - config.mow_erpm_limit)/100; //divide to keep PID constants similar order to others
        if(cs_error > 4.0)
            cs_error = 4.0;
        else if(cs_error < -4.0)
            cs_error = -4.0;
        i_cs_error += cs_error * dt;
        if ((i_cs_error * config.ki_cs) > 1.0)
            i_cs_error = 1.0/config.ki_cs;
        else if (i_cs_error < 0.0)
            i_cs_error = 0.0;
        d_cs = (cs_error - last_cs_error) / dt;
        last_cs_error = cs_error;
        cs_velocity_limit = cs_error * config.kp_cs + i_cs_error * config.ki_cs + d_cs * config.kd_cs;

        if (cs_velocity_limit > 1.0)
            cs_velocity_limit = 1.0;
        else if (cs_velocity_limit < 0.0)
            cs_velocity_limit = 0.0;
    }

    //velocity limiting based on wheel DC
    static double i_wdc_error = 1.0, last_wdc_error = 0.0;
    double wdc_velocity_limit = 1.0, wdc_error = 0.0, d_wdc = 0.0;
    if(!config.enable_wdc) {
        i_wdc_error = 0.0; last_wdc_error = 0.0;
    } else {
        // Only proceed if we have received ESC status data
        if (!left_esc_received || !right_esc_received) {
            wdc_velocity_limit = 1.0; // No limiting if no ESC data
        } else {
            double dc = std::max(left_esc_status.duty_cycle, right_esc_status.duty_cycle);
            wdc_error = (config.wheel_dc_limit - dc) * 20; //bit of gain to keep PID constants similar order
            if(wdc_error > 1.0)
                wdc_error = 1.0;
            else if(wdc_error < -1.0)
                wdc_error = -1.0;
            i_wdc_error += wdc_error * dt;
            if ((i_wdc_error * config.ki_wdc) > 1.0)
                i_wdc_error = 1.0/config.ki_wdc;
            else if (i_wdc_error < 0.0)
                i_wdc_error = 0.0;
            d_wdc = (wdc_error - last_wdc_error) / dt;
            last_wdc_error = wdc_error;
            wdc_velocity_limit = wdc_error * config.kp_wdc + i_wdc_error * config.ki_wdc + d_wdc * config.kd_wdc;

            if (wdc_velocity_limit > 1.0)
                wdc_velocity_limit = 1.0;
            else if (wdc_velocity_limit < 0.0)
                wdc_velocity_limit = 0.0;
        }
    }

    //velocity limiting based on esc temperature
    static double i_me_error = 1.0, last_me_error = 0.0;
    double me_velocity_limit = 1.0, me_error = 0.0, d_me = 0.0;
    if(!config.enable_me) {
        i_me_error = 0.0; last_me_error = 0.0;
    } else {
        me_error = (config.mow_esc_limit - msg->mower_esc_temperature);
        i_me_error += me_error * dt;
        if ((i_me_error * config.ki_me) > 1.0)
            i_me_error = 1.0/config.ki_me;
        else if (i_me_error < 0.0)
            i_me_error = 0.0;
        d_me = (me_error - last_me_error) / dt;
        last_me_error = me_error;
        me_velocity_limit = me_error * config.kp_me + i_me_error * config.ki_me + d_me * config.kd_me;

        if (me_velocity_limit > 1.0)
            me_velocity_limit = 1.0;
        else if (me_velocity_limit < 0.0)
            me_velocity_limit = 0.0;
    }

    //velocity limiting based on wheel DC
    static double i_mm_error = 1.0, last_mm_error = 0.0;
    double mm_velocity_limit = 1.0, mm_error = 0.0, d_mm = 0.0;
    if(!config.enable_mm) {
        i_mm_error = 0.0; last_mm_error = 0.0;
    } else {
        mm_error = (config.mow_motor_limit - msg->mower_motor_temperature);
        i_mm_error += mm_error * dt;
        if ((i_mm_error * config.ki_mm) > 1.0)
            i_mm_error = 1.0/config.ki_mm;
        else if (i_mm_error < 0.0)
            i_mm_error = 0.0;
        d_mm = (mm_error - last_mm_error) / dt;
        last_mm_error = mm_error;
        mm_velocity_limit = mm_error * config.kp_mm + i_mm_error * config.ki_mm + d_mm * config.kd_mm;

        if (mm_velocity_limit > 1.0)
            mm_velocity_limit = 1.0;
        else if (mm_velocity_limit < 0.0)
            mm_velocity_limit = 0.0;
    }

    double velocity_limit = std::min(cc_velocity_limit, cs_velocity_limit);
    velocity_limit = std::min(velocity_limit,wdc_velocity_limit);
    velocity_limit = std::min(velocity_limit,me_velocity_limit);
    velocity_limit = std::min(velocity_limit,mm_velocity_limit);

    mower_logic::VelocityLimit velLimMsg;
    velLimMsg.stamp = ros::Time::now();
    velLimMsg.velocity_limit = velocity_limit;
    velLim_pub.publish(velLimMsg);


    if (config.debug_pid)
    {
        mower_logic::SpeedLimiterDebug debugMsg;
        debugMsg.stamp = ros::Time::now();
        debugMsg.kp_cc_set = cc_error * config.kp_cc;
        debugMsg.ki_cc_set = i_cc_error * config.ki_cc;
        debugMsg.kd_cc_set = d_cc * config.kd_cc;
        debugMsg.cc_err = cc_error;
        debugMsg.kp_cs_set = cs_error * config.kp_cs;
        debugMsg.ki_cs_set = i_cs_error * config.ki_cs;
        debugMsg.kd_cs_set = d_cs * config.kd_cs;
        debugMsg.cs_err = cs_error;
        debugMsg.kp_wdc_set = wdc_error * config.kp_wdc;
        debugMsg.ki_wdc_set = i_wdc_error * config.ki_wdc;
        debugMsg.kd_wdc_set = d_wdc * config.kd_wdc;
        debugMsg.wdc_err = wdc_error;
        debugMsg.kp_me_set = wdc_error * config.kp_me;
        debugMsg.ki_me_set = i_wdc_error * config.ki_me;
        debugMsg.kd_me_set = d_wdc * config.kd_me;
        debugMsg.me_err = me_error;
        debugMsg.kp_mm_set = wdc_error * config.kp_mm;
        debugMsg.ki_mm_set = i_wdc_error * config.ki_mm;
        debugMsg.kd_mm_set = d_wdc * config.kd_mm;
        debugMsg.mm_err = mm_error;

        debugMsg.cc_velocity_limit = cc_velocity_limit;
        debugMsg.cs_velocity_limit = cs_velocity_limit;
        debugMsg.wdc_velocity_limit = wdc_velocity_limit;
        debugMsg.me_velocity_limit = me_velocity_limit;
        debugMsg.mm_velocity_limit = mm_velocity_limit;
        debugMsg.velocity_limit = velocity_limit;

        debug_pub.publish(debugMsg);
    }
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "mower_speed_limiter");

    ros::NodeHandle n;
    ros::NodeHandle paramNh("~");

    reconfig_server = new dynamic_reconfigure::Server<mower_speed_limiter::MowerSpeedLimiterConfig>(paramNh);
    reconfig_server->setCallback(reconfigureCB);

    ros::Subscriber status_sub;
    status_sub = n.subscribe("/ll/mower_status", 100, statusReceived);
    
    ros::Subscriber left_esc_sub;
    left_esc_sub = n.subscribe("/ll/diff_drive/left_esc_status", 10, leftESCReceived);
    
    ros::Subscriber right_esc_sub;
    right_esc_sub = n.subscribe("/ll/diff_drive/right_esc_status", 10, rightESCReceived);

    debug_pub = n.advertise<mower_logic::SpeedLimiterDebug>("mower_logic/speed_limiter_debug", 1);

    velLim_pub = n.advertise<mower_logic::VelocityLimit>("mower/velocity_limit", 1);

    ros::spin();
    delete (reconfig_server);
    return 0;
}
