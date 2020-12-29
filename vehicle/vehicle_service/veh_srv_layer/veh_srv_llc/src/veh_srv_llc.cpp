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
#include <stdio.h>
#include <iostream>

/* local function prototypes */
void audiCmdCallback(const veh_srv_ros_types::AudiCmd::ConstPtr&);
static void fillComCfg(const struct ros_com_S*);
static void initRos(ros::Subscriber&, struct ros_com_cfg_S, int, char **);

void audiCmdCallback(const veh_srv_ros_types::AudiCmd::ConstPtr& msg)
{
    
}

static void fillComCfg(struct ros_com_cfg_S* ros_com_cfg)
{
    ros_com_cfg->nodeName = "audibot_veh_llc_node";
    ros_com_cfg->topicInName = "/audibot/vehicle_srv/llc";
    ros_com_cfg->topicOutNames[VEH_THROTTLE] = "/audibot/vehicle_srv/veh_throttle";
    ros_com_cfg->topicOutNames[VEH_BRAKE] = "/audibot/vehicle_srv/veh_brake";
    ros_com_cfg->topicOutNames[VEH_STEERING] = "/audibot/vehicle_srv/veh_steering";
    ros_com_cfg->topicOutNames[VEH_GEAR] = "/audibot/vehicle_srv/veh_gear";
    ros_com_cfg->maxBuffLen = 1000;
}

static void initRos(ros::Subscriber& sub, struct ros_com_cfg_S ros_com_cfg, int argc, char **argv)
{
    ros::init(argc, argv, ros_com_cfg.nodeName);
    ros::NodeHandle node_handle;
    node_handle.subscribe(ros_com_cfg.topicInName, ros_com_cfg.maxBuffLen, audiCmdCallback);
}

int32_t main(int argc, char **argv)
{
    struct ros_com_cfg_S ros_com_cfg;
    fillComCfg(&ros_com_cfg);
    /* initialize ros node */
    ros::Subscriber node_sub;
    initRos(node_sub, ros_com_cfg, argc, argv);
    ros::spin();
    return 0;
}