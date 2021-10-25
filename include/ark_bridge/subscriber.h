#ifndef H_SUBSCRIBER
#define H_SUBSCRIBER
#include <ros/ros.h>
#include <ark_bridge/value_update.h>
#include <ark_bridge/value_register.h>
#include <ark_bridge/value_value.h>
#include <ark_bridge/value_desc.h>
namespace ark
{
    template <class K>
    class ValueSubscriber
    {
    private:
        ros::NodeHandle &handle;
        ros::Subscriber subscriber;
        K &ref;
        void (*localCallback)(K);
        void registerSubscriber()
        {
            ark_bridge::value_register msg;
            msg.request.detial = desc;
            ros::service::call("/ark/bridge/valuewatcher/register", msg);
        }

    public:
        const std::string name;
        ark_bridge::value_desc desc;
        ValueSubscriber(ros::NodeHandle &nh, const std::string name, K &var, const ark_bridge::value_desc &desc, void (*localCallback)(K) = 0) : handle(nh), name(name), ref(var), desc(desc), localCallback(localCallback)
        {
            this->subscriber = nh.subscribe("/ark/bridge/valuewatcher/" + name, 1, &ValueSubscriber<K>::callback, this);
            this->registerSubscriber();
        }
        ValueSubscriber<K> update();

        void callback(const ark_bridge::value_value::ConstPtr &msg)
        {
            memcpy(&this->ref, &msg->value, sizeof(this->ref));
            if (localCallback)
                localCallback(this->ref);
            ark_bridge::value_update update_msg;
            update_msg.request.key = desc.name;
            update_msg.request.value = *msg;
            ros::service::call("/ark/bridge/valuewatcher/update", update_msg);
        }
    };
}

#endif
