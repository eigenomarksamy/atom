/* 
 * Author:  Omar K. Samy
 * Date:    27-12-2020
 */

/********************************************/
/*                  Includes                */
/********************************************/
/* ros standard libraries inclusion */
#include "ros/ros.h"
#include "std_msgs/String.h"

/* defined msg and srv types inclusion */
#include "veh_srv_ros_types/AudiCmd.h"
#include "veh_srv_ros_types/VehThrottle.h"
#include "veh_srv_ros_types/VehBrake.h"
#include "veh_srv_ros_types/VehSteering.h"
#include "veh_srv_ros_types/VehGear.h"

/* pkg-level header inclusion */
#include "veh_srv_llc.h"

/* language-based inclusion */
#include <iostream>


/********************************************/
/*          Defines and globals             */
/********************************************/
/* defines */
#define PI          M_PI
#define STEER_RATIO (17.3f)
#define MAX_STEER   2*PI
#define MIN_STEER   -2*PI
#define INVALID_MAX 200u

/* global variables */
static struct cmd_in_data_S gst_cmd_in_data;
static struct cmd_in_data_S gst_cmd_in_pre_data;
static bool_t st_is_first_cycle = TRUE;
static uint8_t st_invalid_count = 0;


/********************************************/
/*          Local Functions                 */
/********************************************/
/* local functions prototypes */
/*! \brief callback function */
void audiCmdCallback(const veh_srv_ros_types::AudiCmd::ConstPtr&);
/*! \brief static function that fills the initialization */
static void fillComCfg(struct ros_com_S*);
/*! \brief initializes the ros communication */
static void initRos(ros::Publisher&, ros::Publisher&, ros::Publisher&, ros::Publisher&,
                    ros::Publisher&, ros::Subscriber&, ros::NodeHandle&, struct ros_com_cfg_S,
                    int, char **);
/*! \brief validation function */
static bool_t validateInput(const struct cmd_in_data_S* const, const struct cmd_in_data_S* const);
/*! \brief fills the output structure */
static void fillCmdOut(struct cmd_out_data_S*, const struct cmd_in_data_S* const);
/*! \brief publishes data */
static void publishCmd(ros::Publisher&, const struct cmd_out_data_S*, enum cmd_sys_E);

/* implementation of local functions */
/*! \details triggered when the ros detects a new message 
 * on the topic "/audibot/vehicle_srv/llc".
 * it fills a global static structure.
 * \param[in] msg - the msg of the type on the topic
 */
void audiCmdCallback(const veh_srv_ros_types::AudiCmd::ConstPtr& msg)
{
    if (st_is_first_cycle)
    {
        st_is_first_cycle = FALSE;
    }
    else
    {
        gst_cmd_in_pre_data = gst_cmd_in_data;
    }
    gst_cmd_in_data.time = msg->timestamp.toSec();
    gst_cmd_in_data.seq_counter = msg->seq_counter;
    gst_cmd_in_data.throt_in_cmd = static_cast<uint64_t>(msg->throttle_cmd);
    gst_cmd_in_data.brake_in_cmd = static_cast<uint64_t>(msg->brake_cmd);
    gst_cmd_in_data.steer_in_cmd = static_cast<sint64_t>(msg->steering_cmd);
    gst_cmd_in_data.gear_in_cmd = static_cast<Gear_CMD_E>(msg->gear_cmd);
}

/*! \details initializes the communication configurations
 * \param[out] p_rosComCfg - the configurations to be filled
 */
static void fillComCfg(struct ros_com_cfg_S* p_rosComCfg)
{
    p_rosComCfg->nodeName = "audibot_veh_llc_node";
    p_rosComCfg->topicInName = "/audibot/vehicle_srv/llc";
    p_rosComCfg->topicOutNames[VEH_CMD_NONE] = "/audibot/veh_srv/veh_cmd_feedback";
    p_rosComCfg->topicOutNames[VEH_THROTTLE] = "/audibot/vehicle_srv/veh_throttle";
    p_rosComCfg->topicOutNames[VEH_BRAKE] = "/audibot/vehicle_srv/veh_brake";
    p_rosComCfg->topicOutNames[VEH_STEERING] = "/audibot/vehicle_srv/veh_steering";
    p_rosComCfg->topicOutNames[VEH_GEAR] = "/audibot/vehicle_srv/veh_gear";
    p_rosComCfg->maxBuffLen = 1000;
    p_rosComCfg->rosRate = 100;
}

/*! \details initializes the ros communication
 * \param[out] node_pub_throt - ros node publisher
 * \param[out] node_pub_brake - ros node publisher
 * \param[out] node_pub_steer - ros node publisher
 * \param[out] node_pub_gear - ros node publisher
 * \param[out] node_sub - ros node subscriber
 * \param[in, out] node_handler - ros node handler object
 * \param[in] ros_com_cfg - configurations of ros communication
 * \param[in] argc - arguments added in launch
 * \param[in] argv - arguments added in launch
 */
static void initRos(ros::Publisher& node_pub_all, ros::Publisher& node_pub_throt,
                    ros::Publisher& node_pub_brake, ros::Publisher& node_pub_steer,
                    ros::Publisher& node_pub_gear, ros::Subscriber& node_sub,
                    ros::NodeHandle& node_handler, struct ros_com_cfg_S ros_com_cfg,
                    int argc, char **argv)
{
    ros::init(argc, argv, ros_com_cfg.nodeName);
    node_pub_all = node_handler.advertise<veh_srv_ros_types::AudiCmd>(ros_com_cfg.topicOutNames[VEH_CMD_NONE],
                                                                        ros_com_cfg.maxBuffLen);
    node_pub_throt = node_handler.advertise<veh_srv_ros_types::VehThrottle>(ros_com_cfg.topicOutNames[VEH_THROTTLE],
                                                                            ros_com_cfg.maxBuffLen);
    node_pub_brake = node_handler.advertise<veh_srv_ros_types::VehBrake>(ros_com_cfg.topicOutNames[VEH_BRAKE],
                                                                        ros_com_cfg.maxBuffLen);
    node_pub_steer = node_handler.advertise<veh_srv_ros_types::VehSteering>(ros_com_cfg.topicOutNames[VEH_STEERING],
                                                                            ros_com_cfg.maxBuffLen);
    node_pub_gear = node_handler.advertise<veh_srv_ros_types::VehGear>(ros_com_cfg.topicOutNames[VEH_GEAR],
                                                                        ros_com_cfg.maxBuffLen);
    node_sub = node_handler.subscribe(ros_com_cfg.topicInName, ros_com_cfg.maxBuffLen, audiCmdCallback);
}

/*! \details validates the input msg according to the previous msg.
 * \param[in] msg - the msg of the type on the topic
 */
static bool_t validateInput(const struct cmd_in_data_S* const p_cmdInDataPre, 
                            const struct cmd_in_data_S* const p_cmdInData)
{
    bool_t validInput = FALSE;
    if (p_cmdInData->seq_counter >= p_cmdInDataPre->seq_counter)
    {
        validInput = TRUE;
    }
    return validInput;
}

/*! \details the output structure which is published on the corresponding topics.
 * \param[out] p_cmdOutData - the publishing message
 * \param[in] p_cmdInData - the input to the node (subscription)
 */
static void fillCmdOut(struct cmd_out_data_S* p_cmdOutData,
                        const struct cmd_in_data_S* const p_cmdInData)
{
    p_cmdOutData->time = ros::Time::now().toSec();
    bool_t is_movable = (p_cmdInData->gear_in_cmd == GEAR_R || p_cmdInData->gear_in_cmd == GEAR_D) ? TRUE : FALSE;
    if (is_movable)
    {
        /* convert from uint64_t to float64 (cmd * MAX_CMD<1> / UINT64_MAX) */
        p_cmdOutData->throt_out_cmd = static_cast<float>(p_cmdInData->throt_in_cmd / UINT64_MAX);
        p_cmdOutData->brake_out_cmd = static_cast<float>(p_cmdInData->brake_in_cmd / UINT64_MAX);
        if (p_cmdInData->steer_in_cmd > 0)
        {
            p_cmdOutData->steer_out_cmd = static_cast<float>((p_cmdInData->steer_in_cmd * MAX_STEER) / SINT16_MAX);
        }
        else if (p_cmdInData->steer_in_cmd < 0)
        {
            p_cmdOutData->steer_out_cmd = static_cast<float>((p_cmdInData->steer_in_cmd * MIN_STEER) / SINT16_MIN);
        }
        else /* steering_cmd == 0 */
        {
            p_cmdOutData->steer_out_cmd = static_cast<float>(0);
        }
        
        p_cmdOutData->gear_out_cmd = static_cast<uint8_t>((p_cmdInData->gear_in_cmd == GEAR_R) ? 1 : 0);
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

/*! \details the output structure which is published on the corresponding topics.
 * \param[in] publisher - the publishing message
 * \param[in] p_cmdOut - the input to the node (subscription)
 * \param[in] subsystem - the input to the node (subscription)
 */
static void publishCmd(ros::Publisher& publisher, const struct cmd_out_data_S* p_cmdOut, enum cmd_sys_E subsystem)
{
    veh_srv_ros_types::AudiCmd msg_all;
    veh_srv_ros_types::VehThrottle msg_throt;
    veh_srv_ros_types::VehBrake msg_brake;
    veh_srv_ros_types::VehSteering msg_steer;
    veh_srv_ros_types::VehGear msg_gear;
    switch (subsystem)
    {
    case VEH_CMD_NONE:
        msg_all.throttle_cmd = gst_cmd_in_data.throt_in_cmd;
        msg_all.brake_cmd = gst_cmd_in_data.brake_in_cmd;
        msg_all.steering_cmd = gst_cmd_in_data.steer_in_cmd;
        msg_all.gear_cmd = gst_cmd_in_data.gear_in_cmd;
        msg_all.seq_counter = p_cmdOut->seq_counter;
        msg_all.timestamp = ros::Time::now();
        publisher.publish(msg_all);
        break;
    case VEH_THROTTLE:
        msg_throt.throttle_cmd.data = p_cmdOut->throt_out_cmd;
        msg_throt.seq_counter = p_cmdOut->seq_counter;
        msg_throt.timestamp = ros::Time::now();
        publisher.publish(msg_throt);
        break;
    case VEH_BRAKE:
        msg_brake.brake_cmd.data = p_cmdOut->brake_out_cmd;
        msg_brake.seq_counter = p_cmdOut->seq_counter;
        msg_brake.timestamp = ros::Time::now();
        publisher.publish(msg_brake);
        break;
    case VEH_STEERING:
        msg_steer.steering_cmd.data = p_cmdOut->steer_out_cmd;
        msg_steer.seq_counter = p_cmdOut->seq_counter;
        msg_steer.timestamp = ros::Time::now();
        publisher.publish(msg_steer);
        break;
    case VEH_GEAR:
        msg_gear.gear_cmd.data = p_cmdOut->gear_out_cmd;
        msg_gear.seq_counter = p_cmdOut->seq_counter;
        msg_gear.timestamp = ros::Time::now();
        publisher.publish(msg_gear);
        break;
    default:
        break;
    }
}

/********************************************/
/*              Main Function               */
/********************************************/
/*! \brief node's main function
 * \details executes the node
 * \param[in] argc - arguments from launch
 * \param[in] argv - arguments from launch
 */
int32_t main(int argc, char **argv)
{
    struct ros_com_cfg_S ros_com_cfg;
    struct cmd_out_data_S out_data;
    ros::NodeHandle node_handler;
    ros::Subscriber node_sub;
    ros::Publisher node_pub[VEH_CMD_LEN];
    ros::Rate node_rate(ros_com_cfg.rosRate);
    out_data.seq_counter = 0;
    fillComCfg(&ros_com_cfg);
    (void)memset(&gst_cmd_in_pre_data, 0, sizeof(gst_cmd_in_pre_data));
    /* initialize ros node */
    initRos(node_pub[VEH_CMD_NONE], node_pub[VEH_THROTTLE], node_pub[VEH_BRAKE],
            node_pub[VEH_STEERING], node_pub[VEH_GEAR], node_sub, node_handler,
            ros_com_cfg, argc, argv);
    while ((ros::ok()) && (INVALID_MAX > st_invalid_count))
    {
        if (validateInput(&gst_cmd_in_pre_data, &gst_cmd_in_data))
        {
            fillCmdOut(&out_data, &gst_cmd_in_data);
            for (uint8_t it = VEH_CMD_NONE; it < VEH_CMD_LEN; it++)
            {
                publishCmd(node_pub[it], &out_data, static_cast<cmd_sys_E>(it));
            }
        }
        else
        {
            st_invalid_count++;
        }
        ros::spinOnce();
        node_rate.sleep();
    }
    return 0;
}