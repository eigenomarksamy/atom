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

#define MAX_STR_LEN         100u
#define MAX_NODE_NAME_LEN   MAX_STR_LEN
#define MAX_TOPIC_NAME_LEN  MAX_STR_LEN


enum Gear_CMD_E {
    GEAR_NONE = 0,
    GEAR_P,
    GEAR_R,
    GEAR_N,
    GEAR_D,
    GEAR_CMD_LEN
};

enum cmd_sys_E {
    VEH_CMD_NONE = 0,
    VEH_THROTTLE,
    VEH_BRAKE,
    VEH_STEERING,
    VEH_GEAR,
    VEH_CMD_LEN
};


struct ros_com_cfg_S {
    std::string nodeName;
    std::string topicInName;
    std::string topicOutNames[VEH_CMD_LEN];
    uint16_t    maxBuffLen;
    uint8_t     rosRate;
    bool_t      selfRate;
};

struct ros_types_conf_S {
    ros::NodeHandle nodeHandler;
    ros::Subscriber nodeSubscriber;
    ros::Publisher  nodePublishers[VEH_CMD_LEN];
};

struct ros_msg_types_S {
    veh_srv_ros_types::AudiCmd      msgAll;
    veh_srv_ros_types::VehThrottle  msgThrot;
    veh_srv_ros_types::VehBrake     msgBrake;
    veh_srv_ros_types::VehSteering  msgSteer;
    veh_srv_ros_types::VehGear      msgGear;
};

struct cmd_in_data_S {
    float64_t   time;                   /* ros::Time::now().toSec() */
    uint64_t    seq_counter;
    sint64_t    steer_in_cmd;           /* from SINT64_MIN to SINT64_MAX */
    uint64_t    throt_in_cmd;           /* from UINT64_MIN to UINT64_MAX */
    uint64_t    brake_in_cmd;           /* from UINT64_MIN to UINT64_MAX */
    Gear_CMD_E  gear_in_cmd;
};

struct cmd_out_data_S {
    uint64_t    seq_counter;
    float64_t   time;               /* ros::Time::now().toSec() */
    float32_t   steer_out_cmd;
    float32_t   throt_out_cmd;
    float32_t   brake_out_cmd;
    uint8_t     gear_out_cmd;
};

#endif /* VEH_SRV_LLC_H_ */