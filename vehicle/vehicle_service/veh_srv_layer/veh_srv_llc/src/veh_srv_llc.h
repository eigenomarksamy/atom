/* 
 * Author:  Omar K. Samy
 * Date:    29-12-2020
 */

/* header guard */
#ifndef VEH_SRV_LLC_H_
#define VEH_SRV_LLC_H_

/* ros standard libraries inclusion */

/* defined msg and srv types inclusion */

/* pkg-level header inclusion */

/* language-based inclusion */
#include <string>

/* definition of enums, structs, defines, etc. */
#define MAX_STR_LEN         100u
#define MAX_NODE_NAME_LEN   MAX_STR_LEN
#define MAX_TOPIC_NAME_LEN  MAX_STR_LEN

enum cmd_output_E {
    VEH_NONE = 0,
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
    uint16_t maxBuffLen;
};

#endif /* VEH_SRV_LLC_H_ */