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
#define AUDI_STEER_RATIO    (17.3f)
#define AUDI_MAX_STEER      (M_PI * 2)
#define AUDI_MIN_STEER      (-AUDI_MAX_STEER)
#define AUDI_WHEEL_BASE     (2.65f)
#define AUDI_TRACK_WIDTH    (1.638f)
#define AUDI_WHEEL_RADIUS   (0.36f)
#define MAX_STEER           AUDI_MAX_STEER
#define MIN_STEER           AUDI_MIN_STEER
#define STEER_RATIO         AUDI_STEER_RATIO
#define WHEEL_BASE          AUDI_WHEEL_BASE
#define TRACK_WIDTH         AUDI_TRACK_WIDTH
#define WHEEL_RADIUS        AUDI_WHEEL_RADIUS

#define INVALID_MAX         UINT8_MAX
#define CMD_SYS_ALL         CMD_SYS_NONE


/* global variables */
static struct cmd_InData_S gst_cmd_in_data;
static bool_t gst_is_first_cycle = TRUE;



/********************************************/
/*          Local Functions                 */
/********************************************/
/* local functions prototypes */

/*! \brief callback function */
void audiCmdCallback(const veh_srv_ros_types::AudiCmd::ConstPtr&);
/*! \brief static function that fills the initialization */
static void initComCfg(struct cfg_RosComm_S*,
                       std::string veh_ns="audibot",
                       uint8_t veh_str_len=1,
                       uint8_t veh_id=0,
                       std::string veh_name="audibot");
/*! \brief initializes the ros communication */
static void initRos(struct cfg_RosNode_S*, const struct cfg_RosComm_S* const, int, char **);
/*! \brief vehicle configurations */
static void initVehPhyCfg(struct cfg_VehPhy_S *);
/*! \brief validation function */
static bool_t validateInput(const struct cmd_InData_S* const, const struct cmd_InData_S* const);
/*! \brief fills the output structure */
static void fillCmdOut(struct cmd_OutData_S*, const struct cmd_InData_S* const, const struct cfg_VehPhy_S* const);
/*! \brief publishes data */
static void publishCmd(ros::Publisher&, const struct cmd_OutData_S*, enum CMD_Sys_E);
/*! \brief publishes data */
static void assignPrevMsg(struct cmd_InData_S*);
/*! \brief node's initialization function */
void init(struct cfg_RosComm_S*,
          struct cfg_RosNode_S*,
          struct cfg_VehPhy_S*,
          struct cmd_InData_S*,
          struct cmd_OutData_S*,
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
    gst_cmd_in_data.gear_in_cmd = static_cast<CMD_Gear_E>(msg->gear_cmd);
}


/*! \details initializes the communication configurations
 * \param[out] p_rosComCfg - the configurations to be filled
 */
static void initComCfg(struct cfg_RosComm_S* p_rosComCfg,
                       std::string veh_ns,
                       uint8_t veh_str_len,
                       uint8_t veh_id,
                       std::string veh_name)
{
    std::string veh_id_str = std::to_string(veh_id);
    if ((1 == veh_str_len) && (veh_ns == veh_name))
    {
        p_rosComCfg->nodeName = veh_ns + "_veh_llc_node";
        p_rosComCfg->topicInName = "/" + veh_ns + "/vehicle_srv/llc";
        p_rosComCfg->topicOutNames[CMD_SYS_ALL] = "/" + veh_ns + "/veh_srv/veh_cmd_feedback";
        p_rosComCfg->topicOutNames[CMD_SYS_THROT] = "/" + veh_ns + "/vehicle_srv/veh_throttle";
        p_rosComCfg->topicOutNames[CMD_SYS_BRAKE] = "/" + veh_ns + "/vehicle_srv/veh_brake";
        p_rosComCfg->topicOutNames[CMD_SYS_STEER] = "/" + veh_ns + "/vehicle_srv/veh_steering";
        p_rosComCfg->topicOutNames[CMD_SYS_GEAR] = "/" + veh_ns + "/vehicle_srv/veh_gear";
    }
    else if ((1 != veh_str_len) && (veh_ns == veh_name))
    {
        p_rosComCfg->nodeName = veh_ns + "_ID_" + veh_id_str + "_veh_llc_node";
        p_rosComCfg->topicInName = "/" + veh_ns + "_ID_" + veh_id_str + "/vehicle_srv/llc";
        p_rosComCfg->topicOutNames[CMD_SYS_ALL] = "/" + veh_ns + "_ID_" + veh_id_str + "/veh_srv/veh_cmd_feedback";
        p_rosComCfg->topicOutNames[CMD_SYS_THROT] = "/" + veh_ns + "_ID_" + veh_id_str + "/vehicle_srv/veh_throttle";
        p_rosComCfg->topicOutNames[CMD_SYS_BRAKE] = "/" + veh_ns + "_ID_" + veh_id_str + "/vehicle_srv/veh_brake";
        p_rosComCfg->topicOutNames[CMD_SYS_STEER] = "/" + veh_ns + "_ID_" + veh_id_str + "/vehicle_srv/veh_steering";
        p_rosComCfg->topicOutNames[CMD_SYS_GEAR] = "/" + veh_ns + "_ID_" + veh_id_str + "/vehicle_srv/veh_gear";
    }
    else if ((1 == veh_str_len) && (veh_ns != veh_name))
    {
        p_rosComCfg->nodeName = veh_ns + "_" + veh_name + "_veh_llc_node";
        p_rosComCfg->topicInName = "/" + veh_ns + "_" + veh_name + "/vehicle_srv/llc";
        p_rosComCfg->topicOutNames[CMD_SYS_ALL] = "/" + veh_ns + "_" + veh_name + "/veh_srv/veh_cmd_feedback";
        p_rosComCfg->topicOutNames[CMD_SYS_THROT] = "/" + veh_ns + "_" + veh_name + "/vehicle_srv/veh_throttle";
        p_rosComCfg->topicOutNames[CMD_SYS_BRAKE] = "/" + veh_ns + "_" + veh_name + "/vehicle_srv/veh_brake";
        p_rosComCfg->topicOutNames[CMD_SYS_STEER] = "/" + veh_ns + "_" + veh_name + "/vehicle_srv/veh_steering";
        p_rosComCfg->topicOutNames[CMD_SYS_GEAR] = "/" + veh_ns + "_" + veh_name + "/vehicle_srv/veh_gear";
    }
    else /* if ((1 != veh_str_len) && (veh_ns != veh_name)) */
    {
        p_rosComCfg->nodeName = veh_ns + "_" + veh_name + "_ID_" + veh_id_str + "_veh_llc_node";
        p_rosComCfg->topicInName = "/" + veh_ns + "_" + veh_name + "_ID_" + veh_id_str + "/vehicle_srv/llc";
        p_rosComCfg->topicOutNames[CMD_SYS_ALL] = \
        "/" + veh_ns + "/" + veh_name + "_ID_" + veh_id_str + "/veh_srv/veh_cmd_feedback";
        p_rosComCfg->topicOutNames[CMD_SYS_THROT] = \
        "/" + veh_ns + "/" + veh_name + "_ID_" + veh_id_str + "/vehicle_srv/veh_throttle";
        p_rosComCfg->topicOutNames[CMD_SYS_BRAKE] = \
        "/" + veh_ns + "/" + veh_name + "_ID_" + veh_id_str + "/vehicle_srv/veh_brake";
        p_rosComCfg->topicOutNames[CMD_SYS_STEER] = \
        "/" + veh_ns + "/" + veh_name + "_ID_" + veh_id_str + "/vehicle_srv/veh_steering";
        p_rosComCfg->topicOutNames[CMD_SYS_GEAR] = \
        "/" + veh_ns + "/" + veh_name + "_ID_" + veh_id_str + "/vehicle_srv/veh_gear";
    }
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
static void initRos(struct cfg_RosNode_S* p_rosTypeCfg, const struct cfg_RosComm_S* const p_rosComCfg,
                    int argc, char **argv)
{
    ros::init(argc, argv, p_rosComCfg->nodeName);
    ros::NodeHandle node_handler;
    p_rosTypeCfg->nodePublishers[CMD_SYS_ALL] = node_handler.advertise<veh_srv_ros_types::AudiCmd>(p_rosComCfg->topicOutNames[CMD_SYS_ALL], p_rosComCfg->maxBuffLen);
    p_rosTypeCfg->nodePublishers[CMD_SYS_THROT] = node_handler.advertise<veh_srv_ros_types::VehThrottle>(p_rosComCfg->topicOutNames[CMD_SYS_THROT], p_rosComCfg->maxBuffLen);
    p_rosTypeCfg->nodePublishers[CMD_SYS_BRAKE] = node_handler.advertise<veh_srv_ros_types::VehBrake>(p_rosComCfg->topicOutNames[CMD_SYS_BRAKE], p_rosComCfg->maxBuffLen);
    p_rosTypeCfg->nodePublishers[CMD_SYS_STEER] = node_handler.advertise<veh_srv_ros_types::VehSteering>(p_rosComCfg->topicOutNames[CMD_SYS_STEER], p_rosComCfg->maxBuffLen);
    p_rosTypeCfg->nodePublishers[CMD_SYS_GEAR] = node_handler.advertise<veh_srv_ros_types::VehGear>(p_rosComCfg->topicOutNames[CMD_SYS_GEAR], p_rosComCfg->maxBuffLen);
    p_rosTypeCfg->nodeSubscriber = node_handler.subscribe(p_rosComCfg->topicInName, p_rosComCfg->maxBuffLen, audiCmdCallback);
    p_rosTypeCfg->p_nodRate = new ros::Rate(p_rosComCfg->rosRate);
}


/*! \details initializes the vehicle cfg
 * \param[out] p_vehPhyCfg - vehicle's physical configurations
 */
static void initVehPhyCfg(struct cfg_VehPhy_S * p_vehPhyCfg)
{
    p_vehPhyCfg->maxSteer       = MAX_STEER;
    p_vehPhyCfg->minSteer       = MIN_STEER;
    p_vehPhyCfg->steerRatio     = STEER_RATIO;
    p_vehPhyCfg->wheelBase      = WHEEL_BASE;
    p_vehPhyCfg->trackWidth     = TRACK_WIDTH;
    p_vehPhyCfg->wheelRadius    = WHEEL_RADIUS;
}


static bool_t validateInput(const struct cmd_InData_S* const p_cmdInData, 
                            const struct cmd_InData_S* const p_cmdInDataPre)
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
static void fillCmdOut(struct cmd_OutData_S* p_cmdOutData,
                        const struct cmd_InData_S* const p_cmdInData, const struct cfg_VehPhy_S* const p_vehCfg)
{
    p_cmdOutData->time = ros::Time::now().toSec();
    bool_t is_movable = (p_cmdInData->gear_in_cmd == CMD_GEAR_R || p_cmdInData->gear_in_cmd == CMD_GEAR_D) ? TRUE : FALSE;
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
        
        p_cmdOutData->gear_out_cmd = static_cast<uint8_t>((p_cmdInData->gear_in_cmd == CMD_GEAR_R) ? 1 : 0);
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
static void publishCmd(ros::Publisher& publisher, const struct cmd_OutData_S* p_cmdOut, enum CMD_Sys_E subsystem)
{
    struct cfg_RosMsgs_S msg;
    switch (subsystem)
    {
    case CMD_SYS_ALL:
        msg.msgAll.throttle_cmd = gst_cmd_in_data.throt_in_cmd;
        msg.msgAll.brake_cmd = gst_cmd_in_data.brake_in_cmd;
        msg.msgAll.steering_cmd = gst_cmd_in_data.steer_in_cmd;
        msg.msgAll.gear_cmd = gst_cmd_in_data.gear_in_cmd;
        msg.msgAll.seq_counter = p_cmdOut->seq_counter;
        msg.msgAll.timestamp = ros::Time::now();
        publisher.publish(msg.msgAll);
        break;
    case CMD_SYS_THROT:
        msg.msgThrot.throttle_cmd.data = p_cmdOut->throt_out_cmd;
        msg.msgThrot.seq_counter = p_cmdOut->seq_counter;
        msg.msgThrot.timestamp = ros::Time::now();
        publisher.publish(msg.msgThrot);
        break;
    case CMD_SYS_BRAKE:
        msg.msgBrake.brake_cmd.data = p_cmdOut->brake_out_cmd;
        msg.msgBrake.seq_counter = p_cmdOut->seq_counter;
        msg.msgBrake.timestamp = ros::Time::now();
        publisher.publish(msg.msgBrake);
        break;
    case CMD_SYS_STEER:
        msg.msgSteer.steering_cmd.data = p_cmdOut->steer_out_cmd;
        msg.msgSteer.seq_counter = p_cmdOut->seq_counter;
        msg.msgSteer.timestamp = ros::Time::now();
        publisher.publish(msg.msgSteer);
        break;
    case CMD_SYS_GEAR:
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
static void assignPrevMsg(struct cmd_InData_S* p_preCmd)
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
void init(struct cfg_RosComm_S* p_rosComCfg, struct cfg_RosNode_S* p_rosTypesConf,
            struct cfg_VehPhy_S* p_vehPhyCfg, struct cmd_InData_S* p_prevData,
            struct cmd_OutData_S* p_outData, int argc, char **argv)
{
    /* fill the node configuration structure */
    initComCfg(p_rosComCfg);
    /* initialize ros node */
    initRos(p_rosTypesConf, p_rosComCfg, argc, argv);
    /* initialize vehicle physical configrations */
    initVehPhyCfg(p_vehPhyCfg);
    /* initialization of output data */
    (void)memset(p_outData, 0, sizeof(cmd_OutData_S));
    /* initialization of previous cycle data */
    (void)memset(p_prevData, 0, sizeof(cmd_InData_S));
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
    struct cfg_RosComm_S rosComCfg;
    struct cfg_RosNode_S rosTypesCfg;
    struct cfg_VehPhy_S vehPhyCfg;
    struct cmd_InData_S inPrevData;
    struct cmd_OutData_S outData;
    uint8_t inValid_cntr = 0;
    /* initialization of the whole node */
    init(&rosComCfg, &rosTypesCfg, &vehPhyCfg, &inPrevData, &outData, argc, argv);
    /* 
     * equivalent to while(1) as long as ros communication is up
     * and we are receiving valid messages
    */
    while ((ros::ok()) && (INVALID_MAX > inValid_cntr))
    {
        if(!gst_is_first_cycle)
        {
            if (validateInput(&gst_cmd_in_data, &inPrevData))
            {
                /* fill output of cmd */
                fillCmdOut(&outData, &gst_cmd_in_data, &vehPhyCfg);
                /* publish all subsystems */
                for (uint8_t it = 0; it < CMD_SYS_LEN; it++)
                {
                    publishCmd(rosTypesCfg.nodePublishers[it], &outData, static_cast<CMD_Sys_E>(it));
                }
                inValid_cntr = 0;
            }
            else
            {
                inValid_cntr++;
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