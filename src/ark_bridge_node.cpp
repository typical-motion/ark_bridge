#include <ros/ros.h>
#include <iostream>
#include "ark_bridge/value_get.h"
#include "ark_bridge/values_list.h"
#include "ark_bridge/valuewatch.h"
#include "ark_bridge/arkui_update.h"
#include "ark_bridge/arkui.h"
#include "std_srvs/Empty.h"
std::map<std::string, ark_bridge::value_desc *> value_desc_map;
std::map<std::string, ark_bridge::ui_node *> ui_node_map;

bool onValueRegisterRequest(ark_bridge::value_register::Request &request, ark_bridge::value_register::Response &response)
{
    if (value_desc_map[request.detial.name])
    {
        delete value_desc_map[request.detial.name];
    }
    ark_bridge::value_desc *local_desc = new ark_bridge::value_desc(request.detial);
    value_desc_map[request.detial.name] = local_desc;
    return true;
}
bool onValuesListRequest(ark_bridge::values_list::Request &request, ark_bridge::values_list::Response &response)
{
    ark_bridge::values_list::Response msg;
    msg.length = value_desc_map.size();

    for (auto const &x : value_desc_map)
    {
        msg.values.push_back(*x.second);
    }
    response = msg;
    return true;
}
bool onValueGetRequest(ark_bridge::value_get::Request &request, ark_bridge::value_get::Response &response)
{
    if (value_desc_map[request.key])
    {
        response.detial = *value_desc_map[request.key];
    }
    else
    {
        return false;
    }
    return true;
}

bool onValueUpdateRequest(ark_bridge::value_update::Request &request, ark_bridge::value_update::Response &response)
{
    if (value_desc_map[request.key])
    {
        value_desc_map[request.key]->value = request.value;
    }
    else
    {
        return false;
    }
    return true;
}

bool onArkUIUpdate(ark_bridge::arkui_update::Request &request, ark_bridge::arkui_update::Response &response)
{
    if (request.operation == 2)
    {
        // GET
        if (request.key.length() == 0)
        {
            std::map<std::string, ark_bridge::ui_node *>::iterator iter = ui_node_map.begin();
            while (iter != ui_node_map.end())
            {
                if (iter->second->level == 0)
                {
                    ark_bridge::ui_node *ui_node = iter->second;
                    response.detials.push_back(*ui_node);
                }
                iter++;
            }
            return true;
        }
        else
        {
            if (ui_node_map[request.key])
            {
                response.detials.push_back(*ui_node_map[request.key]);
                return true;
            }
            else
            {
                return false;
            }
        }
    }
    else if (request.operation == 1)
    {
        // Delete
        std::map<std::string, ark_bridge::ui_node *>::iterator iter = ui_node_map.begin();
        while (iter != ui_node_map.end())
        {
            if (iter->first.rfind(request.key, 0) == 0)
            { // Starts with key
                iter = ui_node_map.erase(iter);
            }
            else
            {
                ++iter;
            }
        }
        return true;
    }
    else if (request.operation == 0)
    {
        // Update
        if (ui_node_map.count(request.key) > 0)
        {
            delete ui_node_map[request.key];
        }
        ark_bridge::ui_node *local_desc = new ark_bridge::ui_node(request.detial);
        ui_node_map[request.key] = local_desc;
        return true;
    }
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "ark_bridge");
    ros::NodeHandle nh;

    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
    {
        ros::console::notifyLoggerLevelsChanged();
    }
    ros::ServiceServer serviceValueWatchRegister = nh.advertiseService("/ark/bridge/valuewatcher/register", onValueRegisterRequest);
    ros::ServiceServer serviceValueWatchList = nh.advertiseService("/ark/bridge/valuewatcher/list", onValuesListRequest);
    ros::ServiceServer serviceValueWatchGet = nh.advertiseService("/ark/bridge/valuewatcher/get", onValueGetRequest);
    ros::ServiceServer serviceValueWatchUpdate = nh.advertiseService("/ark/bridge/valuewatcher/update", onValueUpdateRequest);

    ros::ServiceServer serviceArkUIUpdate = nh.advertiseService("/ark/bridge/arkui/update", onArkUIUpdate);

    ros::Rate(300);
    ros::spin();
}