/* 
 * Author:  Omar K. Samy
 * Date:    27-12-2020
 */


/********************************************/
/*                  Includes                */
/********************************************/

/* pkg-level header inclusion */
#include "veh_srv_llc.h"


/********************************************/
/*  Defines, Structs, Enums, Globals, ETC.  */
/********************************************/

/* defines */
#define STEER_RATIO     (17.3f)
#define MAX_STEER       M_PI * 2
#define MIN_STEER       M_PI * -2
#define INVALID_MAX     200u
#define VEH_CMD_ALL     VEH_CMD_NONE


/* global variables */
static struct cmd_in_data_S gst_cmd_in_data;
static bool_t gst_is_first_cycle = TRUE;



/********************************************/
/*          Local Functions                 */
/********************************************/
/* local functions prototypes */

/*! \brief callback function */
void audiCmdCallback(const veh_srv_ros_types::AudiCmd::ConstPtr&);
/*! \brief static function that fills the initialization */
static void initComCfg(struct ros_com_S*);
/*! \brief initializes the ros communication */
static void initRos(struct ros_types_conf_S*, const struct ros_com_cfg_S* const, int, char **);
/*! \brief vehicle configurations */
static void initVehPhyCfg(struct veh_phy_cfg_S *);
/*! \brief validation function */
static bool_t validateInput(const struct cmd_in_data_S* const, const struct cmd_in_data_S* const);
/*! \brief fills the output structure */
static void fillCmdOut(struct cmd_out_data_S*, const struct cmd_in_data_S* const, const struct veh_phy_cfg_S* const);
/*! \brief publishes data */
static void publishCmd(ros::Publisher&, const struct cmd_out_data_S*, enum cmd_sys_E);
/*! \brief publishes data */
static void assignPrevMsg(struct cmd_in_data_S*);
/*! \brief node's initialization function */
void init(struct ros_com_cfg_S*,
          struct ros_types_conf_S*,
          struct veh_phy_cfg_S*,
          struct cmd_in_data_S*,
          struct cmd_out_data_S*,
          int, char **);
/*! \brief node's main function */
int32_t main(int, char**);


/* implementation of local functions */

/*! \details triggered when the ros detects a new message 
 * on the topic "/audibot/vehicle_srv/llc".
 * it fills a global static structure.
 * \param[in] msg - the msg of the type on the topic
 */
void audiCmdCallback(const veh_srv_ros_types::AudiCmd::ConstPtr& msg)
{
    if (gst_is_first_cycle)
    {
        gst_is_first_cycle = FALSE;
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
static void initComCfg(struct ros_com_cfg_S* p_rosComCfg)
{
    p_rosComCfg->nodeName = "audibot_veh_llc_node";
    p_rosComCfg->topicInName = "/audibot/vehicle_srv/llc";
    p_rosComCfg->topicOutNames[VEH_CMD_ALL] = "/audibot/veh_srv/veh_cmd_feedback";
    p_rosComCfg->topicOutNames[VEH_THROTTLE] = "/audibot/vehicle_srv/veh_throttle";
    p_rosComCfg->topicOutNames[VEH_BRAKE] = "/audibot/vehicle_srv/veh_brake";
    p_rosComCfg->topicOutNames[VEH_STEERING] = "/audibot/vehicle_srv/veh_steering";
    p_rosComCfg->topicOutNames[VEH_GEAR] = "/audibot/vehicle_srv/veh_gear";
    p_rosComCfg->maxBuffLen = 1000;
    p_rosComCfg->rosRate = 100;
    p_rosComCfg->selfRate = TRUE;
}


/*! \details initializes the ros communication
 * \param[in, out] p_rosTypeCfg - ros node handler, publisher & subscriber
 * \param[in] p_rosComCfg - configurations of ros communication
 * \param[in] argc - arguments added in launch
 * \param[in] argv - arguments added in launch
 */
static void initRos(struct ros_types_conf_S* p_rosTypeCfg, const struct ros_com_cfg_S* const p_rosComCfg,
                    int argc, char **argv)
{
    ros::init(argc, argv, p_rosComCfg->nodeName);
    ros::NodeHandle node_handler;
    p_rosTypeCfg->nodePublishers[VEH_CMD_ALL] = node_handler.advertise<veh_srv_ros_types::AudiCmd>(p_rosComCfg->topicOutNames[VEH_CMD_ALL], p_rosComCfg->maxBuffLen);
    p_rosTypeCfg->nodePublishers[VEH_THROTTLE] = node_handler.advertise<veh_srv_ros_types::VehThrottle>(p_rosComCfg->topicOutNames[VEH_THROTTLE], p_rosComCfg->maxBuffLen);
    p_rosTypeCfg->nodePublishers[VEH_BRAKE] = node_handler.advertise<veh_srv_ros_types::VehBrake>(p_rosComCfg->topicOutNames[VEH_BRAKE], p_rosComCfg->maxBuffLen);
    p_rosTypeCfg->nodePublishers[VEH_STEERING] = node_handler.advertise<veh_srv_ros_types::VehSteering>(p_rosComCfg->topicOutNames[VEH_STEERING], p_rosComCfg->maxBuffLen);
    p_rosTypeCfg->nodePublishers[VEH_GEAR] = node_handler.advertise<veh_srv_ros_types::VehGear>(p_rosComCfg->topicOutNames[VEH_GEAR], p_rosComCfg->maxBuffLen);
    p_rosTypeCfg->nodeSubscriber = node_handler.subscribe(p_rosComCfg->topicInName, p_rosComCfg->maxBuffLen, audiCmdCallback);
    p_rosTypeCfg->p_nodRate = new ros::Rate(p_rosComCfg->rosRate);
}


/*! \details initializes the vehicle cfg
 * \param[out] p_vehPhyCfg - vehicle's physical configurations
 */
static void initVehPhyCfg(struct veh_phy_cfg_S * p_vehPhyCfg)
{
    p_vehPhyCfg->maxSteer = MAX_STEER;
    p_vehPhyCfg->minSteer = MIN_STEER;
    p_vehPhyCfg->steerRatio = STEER_RATIO;
}


static bool_t validateInput(const struct cmd_in_data_S* const p_cmdInData, 
                            const struct cmd_in_data_S* const p_cmdInDataPre)
{
    bool_t validInput = FALSE;
    if (p_cmdInData->seq_counter >= p_cmdInDataPre->seq_counter)
    {
        validInput = TRUE;
    }
    return validInput;
}


/*! \details the output structure which is published on the corresponding topics.
 * \param[out] p_cmdOutData - pointer to the output cmd
 * \param[in] p_cmdInData - the input to the node (subscription)
 * \param[in] p_vehCfg - vehicle's physical configurations
 */
static void fillCmdOut(struct cmd_out_data_S* p_cmdOutData,
                        const struct cmd_in_data_S* const p_cmdInData, const struct veh_phy_cfg_S* const p_vehCfg)
{
    p_cmdOutData->time = ros::Time::now().toSec();
    bool_t is_movable = (p_cmdInData->gear_in_cmd == GEAR_R || p_cmdInData->gear_in_cmd == GEAR_D) ? TRUE : FALSE;
    if (is_movable)
    {
        /* convert from uint64_t to float64 (cmd * MAX_CMD<1> / UINT64_MAX) */
        p_cmdOutData->throt_out_cmd = static_cast<float>(p_cmdInData->throt_in_cmd / UINT64_MAX);
        p_cmdOutData->brake_out_cmd = static_cast<float>(p_cmdInData->brake_in_cmd / UINT64_MAX);
        if (0 < p_cmdInData->steer_in_cmd)
        {
            p_cmdOutData->steer_out_cmd = \
                                        static_cast<float>((p_cmdInData->steer_in_cmd * p_vehCfg->maxSteer) / SINT16_MAX);
        }
        else if (0 > p_cmdInData->steer_in_cmd)
        {
            p_cmdOutData->steer_out_cmd = \
                                        static_cast<float>((p_cmdInData->steer_in_cmd * p_vehCfg->minSteer) / SINT16_MIN);
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
 * \param[in] publisher - the publisher object
 * \param[in] p_cmdOut - pointer to the output cmd struct
 * \param[in] subsystem - the subsystem to be published
 */
static void publishCmd(ros::Publisher& publisher, const struct cmd_out_data_S* p_cmdOut, enum cmd_sys_E subsystem)
{
    struct ros_msg_types_S msg;
    switch (subsystem)
    {
    case VEH_CMD_ALL:
        msg.msgAll.throttle_cmd = gst_cmd_in_data.throt_in_cmd;
        msg.msgAll.brake_cmd = gst_cmd_in_data.brake_in_cmd;
        msg.msgAll.steering_cmd = gst_cmd_in_data.steer_in_cmd;
        msg.msgAll.gear_cmd = gst_cmd_in_data.gear_in_cmd;
        msg.msgAll.seq_counter = p_cmdOut->seq_counter;
        msg.msgAll.timestamp = ros::Time::now();
        publisher.publish(msg.msgAll);
        break;
    case VEH_THROTTLE:
        msg.msgThrot.throttle_cmd.data = p_cmdOut->throt_out_cmd;
        msg.msgThrot.seq_counter = p_cmdOut->seq_counter;
        msg.msgThrot.timestamp = ros::Time::now();
        publisher.publish(msg.msgThrot);
        break;
    case VEH_BRAKE:
        msg.msgBrake.brake_cmd.data = p_cmdOut->brake_out_cmd;
        msg.msgBrake.seq_counter = p_cmdOut->seq_counter;
        msg.msgBrake.timestamp = ros::Time::now();
        publisher.publish(msg.msgBrake);
        break;
    case VEH_STEERING:
        msg.msgSteer.steering_cmd.data = p_cmdOut->steer_out_cmd;
        msg.msgSteer.seq_counter = p_cmdOut->seq_counter;
        msg.msgSteer.timestamp = ros::Time::now();
        publisher.publish(msg.msgSteer);
        break;
    case VEH_GEAR:
        msg.msgGear.gear_cmd.data = p_cmdOut->gear_out_cmd;
        msg.msgGear.seq_counter = p_cmdOut->seq_counter;
        msg.msgGear.timestamp = ros::Time::now();
        publisher.publish(msg.msgGear);
        break;
    default:
        break;
    }
}


/*! \details assigns the previous received msg for error checking.
 * \param[out] p_preCmd - pointer to the previous msg
 */
static void assignPrevMsg(struct cmd_in_data_S* p_preCmd)
{
    *p_preCmd = gst_cmd_in_data;
}


/*! \details initializes the node
 * \param[out] p_rosComCfg - ros communication configurations
 * \param[out] p_rosTypesConf - ros types configurations
 * \param[out] p_vehPhyCfg - vehicle's physical configurations
 * \param[out] p_prevData - msg in previous data
 * \param[out] p_outData - output data after processing
 * \param[in] argc - arguments from launch
 * \param[in] argv - arguments from launch
 */
void init(struct ros_com_cfg_S* p_rosComCfg, struct ros_types_conf_S* p_rosTypesConf,
            struct veh_phy_cfg_S* p_vehPhyCfg, struct cmd_in_data_S* p_prevData,
            struct cmd_out_data_S* p_outData, int argc, char **argv)
{
    /* fill the node configuration structure */
    initComCfg(p_rosComCfg);
    /* initialize ros node */
    initRos(p_rosTypesConf, p_rosComCfg, argc, argv);
    /* initialize vehicle physical configrations */
    initVehPhyCfg(p_vehPhyCfg);
    /* initialization of output data */
    (void)memset(p_outData, 0, sizeof(cmd_out_data_S));
    /* initialization of previous cycle data */
    (void)memset(p_prevData, 0, sizeof(cmd_in_data_S));
}


/********************************************/
/*              Main Function               */
/********************************************/

/*! \details executes the node
 * \param[in] argc - arguments from launch
 * \param[in] argv - arguments from launch
 */
int32_t main(int argc, char **argv)
{
    struct ros_com_cfg_S rosComCfg;
    struct ros_types_conf_S rosTypesCfg;
    struct veh_phy_cfg_S vehPhyCfg;
    struct cmd_in_data_S inPrevData;
    struct cmd_out_data_S outData;
    bool_t cntr_inValid = 0;
    /* initialization of the whole node */
    init(&rosComCfg, &rosTypesCfg, &vehPhyCfg, &inPrevData, &outData, argc, argv);
    /* 
     * equivalent to while(1) as long as ros communication is up
     * and we are receiving valid messages
    */
    while ((ros::ok()) && (INVALID_MAX > cntr_inValid))
    {
        if(!gst_is_first_cycle)
        {
            if (validateInput(&gst_cmd_in_data, &inPrevData))
            {
                /* fill output of cmd */
                fillCmdOut(&outData, &gst_cmd_in_data, &vehPhyCfg);
                /* publish all subsystems */
                for (uint8_t it = 0; it < VEH_CMD_LEN; it++)
                {
                    publishCmd(rosTypesCfg.nodePublishers[it], &outData, static_cast<cmd_sys_E>(it));
                }
            }
            else
            {
                cntr_inValid++;
            }
            assignPrevMsg(&inPrevData);
        }
        ros::spinOnce();
        if(rosComCfg.selfRate)
        {
            rosTypesCfg.p_nodRate->sleep();
        }
    }
    return 0;
}