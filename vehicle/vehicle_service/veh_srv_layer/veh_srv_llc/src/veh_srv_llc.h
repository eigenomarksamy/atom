/* 
 * Author:  Omar K. Samy
 * Date:    29-12-2020
 */

/* header guard */
#ifndef VEH_SRV_LLC_H_
#define VEH_SRV_LLC_H_

/* ros standard libraries inclusion */
#include "ros/ros.h"
/* defined msg and srv types inclusion */
#include "veh_srv_ros_types/AudiCmd.h"
#include "veh_srv_ros_types/VehThrottle.h"
#include "veh_srv_ros_types/VehBrake.h"
#include "veh_srv_ros_types/VehSteering.h"
#include "veh_srv_ros_types/VehGear.h"
/* language-based inclusion */
#include <string>


/* typedefs */
typedef bool    bool_t;
typedef int8_t  sint8_t;
typedef int16_t sint16_t;
typedef int32_t sint32_t;
typedef int64_t sint64_t;
typedef double  float64_t;
typedef float   float32_t;


/* definition of enums, structs, defines, etc. */
#define SINT8_MIN           INT8_MIN
#define SINT8_MAX           INT8_MAX
#define SINT16_MIN          INT16_MIN
#define SINT16_MAX          INT16_MAX
#define SINT32_MIN          INT32_MIN
#define SINT32_MAX          INT32_MAX
#define SINT64_MIN          INT64_MIN
#define SINT64_MAX          INT64_MAX

#define TRUE                true
#define FALSE               false

#define MAX_STR_LEN         UINT8_MAX
#define MAX_NODE_NAME_LEN   MAX_STR_LEN
#define MAX_TOPIC_NAME_LEN  MAX_STR_LEN


/*! \brief defines the system actuators */
enum CMD_Sys_E {
    CMD_SYS_NONE = 0,
    CMD_SYS_THROT,
    CMD_SYS_BRAKE,
    CMD_SYS_STEER,
    CMD_SYS_GEAR,
    CMD_SYS_LEN
};

/*! \brief defines the gear commands */
enum CMD_Gear_E {
    CMD_GEAR_NONE = 0,
    CMD_GEAR_P,
    CMD_GEAR_R,
    CMD_GEAR_N,
    CMD_GEAR_D,
    CMD_GEAR_LEN
};


/*! \brief defines the communication-related parameters */
struct cfg_RosComm_S {
    std::string nodeName;
    std::string topicInName;
    std::string topicOutNames[CMD_SYS_LEN];
    uint16_t    maxBuffLen;
    uint8_t     rosRate;
    bool_t      selfRate;
};

/*! \brief defines the node-related ros parameters */
struct cfg_RosNode_S {
    ros::Subscriber nodeSubscriber;
    ros::Publisher  nodePublishers[CMD_SYS_LEN];
    ros::Rate*      p_nodRate;
};

/*! \brief defines the msg types used */
struct cfg_RosMsgs_S {
    veh_srv_ros_types::AudiCmd      msgAll;
    veh_srv_ros_types::VehThrottle  msgThrot;
    veh_srv_ros_types::VehBrake     msgBrake;
    veh_srv_ros_types::VehSteering  msgSteer;
    veh_srv_ros_types::VehGear      msgGear;
};

/*! \brief defines the vehicle's physical configuration */
struct cfg_VehPhy_S {
    float32_t maxSteer;
    float32_t minSteer;
    float32_t steerRatio;   /* = 17.3:1 */
    float32_t wheelBase;    /* = 2.65 meters */
    float32_t trackWidth;   /* = 1.638 meters */
    float32_t wheelRadius;  /* = 0.36 meters */
};

/*! \brief defines the receiving msg parameters */
struct cmd_InData_S {
    float64_t   time;                   /* ros::Time::now().toSec() */
    uint64_t    seq_counter;
    sint64_t    steer_in_cmd;           /* from SINT64_MIN to SINT64_MAX */
    uint64_t    throt_in_cmd;           /* from UINT64_MIN to UINT64_MAX */
    uint64_t    brake_in_cmd;           /* from UINT64_MIN to UINT64_MAX */
    CMD_Gear_E  gear_in_cmd;
};

/*! \brief defines the output msgs parameters */
struct cmd_OutData_S {
    uint64_t    seq_counter;
    float64_t   time;               /* ros::Time::now().toSec() */
    float32_t   steer_out_cmd;
    float32_t   throt_out_cmd;
    float32_t   brake_out_cmd;
    uint8_t     gear_out_cmd;
};


#endif /* VEH_SRV_LLC_H_ */