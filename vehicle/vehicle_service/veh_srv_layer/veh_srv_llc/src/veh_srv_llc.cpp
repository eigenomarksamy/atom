/* 
 * Author:  Omar K. Samy
 * Date:    27-12-2020
 */

/* ros standard libraries inclusion */
#include "ros/ros.h"
#include "std_msgs/String.h"

/* defined msg and srv types inclusion */
#include "veh_srv_ros_types/AudiCmd.h"

/* pkg-level header inclusion */
#include "veh_srv_llc.h"

/* language-based inclusion */
#include <iostream>
#include <limits>

/* defines */
#define PI          M_PI
#define STEER_RATIO (17.3f)
#define MAX_STEER   2*PI
#define MIN_STEER   -2*PI

/* global variables */
static struct cmd_in_data_S st_cmd_in_data;

/* local function prototypes */
void audiCmdCallback(const veh_srv_ros_types::AudiCmd::ConstPtr&);
static void fillComCfg(struct ros_com_S*);
static void initRos(ros::Subscriber&, ros::NodeHandle&, struct ros_com_cfg_S, int, char **);
static void fillCmdOut(struct cmd_out_data_S*, struct cmd_in_data_S);

/* implementation of local functions */
void audiCmdCallback(const veh_srv_ros_types::AudiCmd::ConstPtr& msg)
{
    st_cmd_in_data.time = msg->timestamp.toSec();
    st_cmd_in_data.seq_counter = msg->seq_counter;
    st_cmd_in_data.throt_in_cmd = static_cast<uint64_t>(msg->throttle_cmd);
    st_cmd_in_data.brake_in_cmd = static_cast<uint64_t>(msg->brake_cmd);
    st_cmd_in_data.steer_in_cmd = static_cast<sint64_t>(msg->steering_cmd);
    st_cmd_in_data.gear_in_cmd = static_cast<Gear_CMD_E>(msg->gear_cmd);
}

static void fillComCfg(struct ros_com_cfg_S* p_rosComCfg)
{
    p_rosComCfg->nodeName = "audibot_veh_llc_node";
    p_rosComCfg->topicInName = "/audibot/vehicle_srv/llc";
    p_rosComCfg->topicOutNames[VEH_THROTTLE] = "/audibot/vehicle_srv/veh_throttle";
    p_rosComCfg->topicOutNames[VEH_BRAKE] = "/audibot/vehicle_srv/veh_brake";
    p_rosComCfg->topicOutNames[VEH_STEERING] = "/audibot/vehicle_srv/veh_steering";
    p_rosComCfg->topicOutNames[VEH_GEAR] = "/audibot/vehicle_srv/veh_gear";
    p_rosComCfg->maxBuffLen = 1000;
    p_rosComCfg->rosRate = 100;
}

static void initRos(ros::Subscriber& sub, ros::NodeHandle& node_handler,
                    struct ros_com_cfg_S ros_com_cfg, int argc, char **argv)
{
    ros::init(argc, argv, ros_com_cfg.nodeName);
    node_handler.subscribe(ros_com_cfg.topicInName, ros_com_cfg.maxBuffLen, audiCmdCallback);
}

static void fillCmdOut(struct cmd_out_data_S* p_cmdOutData, struct cmd_in_data_S cmdInData)
{
    p_cmdOutData->time = ros::Time::now().toSec();
    bool is_movable = (cmdInData.gear_in_cmd == GEAR_R || cmdInData.gear_in_cmd == GEAR_D) ? true : false;
    if (is_movable)
    {
        /* convert from uint64_t to float64 (cmd * MAX_CMD<1> / UINT64_MAX) */
        p_cmdOutData->throt_out_cmd = static_cast<float>(cmdInData.throt_in_cmd / UINT64_MAX);
        p_cmdOutData->brake_out_cmd = static_cast<float>(cmdInData.brake_in_cmd / UINT64_MAX);
        if (cmdInData.steer_in_cmd > 0)
        {
            p_cmdOutData->steer_out_cmd = static_cast<float>((cmdInData.steer_in_cmd * MAX_STEER) / SINT16_MAX);
        }
        else if (cmdInData.steer_in_cmd < 0)
        {
            p_cmdOutData->steer_out_cmd = static_cast<float>((cmdInData.steer_in_cmd * MIN_STEER) / SINT16_MIN);
        }
        else /* steering_cmd == 0 */
        {
            p_cmdOutData->steer_out_cmd = static_cast<float>(0);
        }
        
        p_cmdOutData->gear_out_cmd = static_cast<uint8_t>((cmdInData.gear_in_cmd == GEAR_R) ? 1 : 0);
    }
    else
    {
        p_cmdOutData->throt_out_cmd = static_cast<float>(0);
        p_cmdOutData->brake_out_cmd = static_cast<float>(0);
        p_cmdOutData->steer_out_cmd = static_cast<float>(0);
        p_cmdOutData->gear_out_cmd = static_cast<uint8_t>(0);
    }
    p_cmdOutData->seq_counter++;
}

/* main function */
int32_t main(int argc, char **argv)
{
    struct ros_com_cfg_S ros_com_cfg;
    struct cmd_out_data_S out_data;
    ros::NodeHandle node_handler;
    ros::Subscriber node_sub;
    ros::Rate node_rate(ros_com_cfg.rosRate);
    out_data.seq_counter = 0;
    fillComCfg(&ros_com_cfg);
    /* initialize ros node */
    initRos(node_sub, node_handler, ros_com_cfg, argc, argv);
    while (ros::ok())
    {

        ros::spinOnce();
        node_rate.sleep();
    }
    return 0;
}