#ifndef H_VALUE_WATCHER
#define H_VALUE_WATCHER
#include <ros/ros.h>
#include "subscriber.h"
namespace ark
{
    const uint8_t TYPE_INT = 0;
    const uint8_t TYPE_FLOAT = 1;
    const uint8_t TYPE_DOUBLE = 2;
    const uint8_t TYPE_BOOL = 3;

    template <class T>
    ValueSubscriber<T> Subscribe(ros::NodeHandle &nh, const std::string name, T &var, ark_bridge::value_desc desc, void (*localCallback)(T) = 0)
    {
        ValueSubscriber<T> *subscriber = new ValueSubscriber<T>(nh, name, var, desc, localCallback);
        return *subscriber;
    }

    template <class T>
    ValueSubscriber<T> Subscribe(ros::NodeHandle &nh, const std::string name, T &var, uint8_t type, void (*localCallback)(T) = 0, float max = 0, float min = 0)
    {
        ark_bridge::value_desc desc;
        desc.name = name;
        desc.value.value = var;
        desc.max_value = max;
        desc.min_value = min;

        return Subscribe(nh, name, var, desc, localCallback);
    }
}

#endif