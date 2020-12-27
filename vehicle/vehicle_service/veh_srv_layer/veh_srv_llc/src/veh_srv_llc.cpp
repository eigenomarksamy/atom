/* 
 * Author:  Omar K. Samy
 * Date:    27-12-2020
 */

/* ros standard libraries inclusion */
#include "ros/ros.h"
#include "std_msgs/String.h"

/* defined msg and srv types inclusion */


/* pkg-level header inclusion */
#include "srv_llc_type.h"

/* language-based inclusion */
#include <stdio.h>
#include <iostream>

void callback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int32_t main(int argc, char **argv)
{
    /* initialize ros node */
    ros::init(argc, argv, "node_name");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("topic_name", 1000, callback);
    ros::spin();
    return 0;
}