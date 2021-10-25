#ifndef H_ARKUI
#define H_ARKUI
#include <ros/ros.h>
#include <ark_bridge/value_update.h>
#include <ark_bridge/value_desc.h>
#include <ark_bridge/ui_node.h>
#include <ark_bridge/arkui_update.h>
#include <ark_bridge/subscriber.h>
#include <ark_ui/Item.h>
#include <ark_ui/Folder.h>
#include <ark_ui/Tab.h>
#include <ark_ui/Category.h>
#include <image_transport/image_transport.h>
#include <initializer_list>

namespace ark
{
    namespace ui
    {

        void waitForBridge()
        {
            ROS_INFO("Waiting for ark_bridge...");
            ark_bridge::arkui_update data;
            data.request.operation = -1;
            data.request.key = "";
            ros::service::call("/ark/bridge/valuewatcher/register", data);
            ROS_INFO("ark_bridge is up.");
        }

        class Item
        {
        public:
            ark_bridge::ui_node container;
            template <class T>
            Item(ark::ValueSubscriber<T> &subscriber)
            {
                container.key = subscriber.desc.name; // Encode this
                container.name = subscriber.desc.name;
                container.level = -1;
                container.children.clear();
                container.children.push_back("ark_valuewatch://" + subscriber.desc.name);
            }
            template <class T>
            Item(ark::ValueSubscriber<T> &sub1, ark::ValueSubscriber<T> &sub2)
            {
                container.key = sub1.desc.name; // Encode this
                container.name = sub1.desc.name;
                container.level = -1;
                container.children.clear();
                container.children.push_back("ark_valuewatch://" + sub1.desc.name);
                container.children.push_back("ark_valuewatch://" + sub2.desc.name);
            }
            Item(image_transport::Publisher &publisher)
            {
                container.key = publisher.getTopic(); // Encode this
                container.name = publisher.getTopic();
                container.level = -1;
                container.children.clear();
                container.children.push_back("image_transport://" + publisher.getTopic());
            }

            void update()
            {

                ark_bridge::arkui_update data;
                data.request.operation = 0;
                data.request.key = container.key;
                data.request.detial = container;
                ros::service::call("/ark/bridge/arkui/update", data);
                // Update
            }
        };
        class Folder
        {
        private:
            std::vector<Item *> children;

        public:
            ark_bridge::ui_node container;

            Folder(const std::string &name, const std::string &desc = "", const std::string &key_prefix = "")
            {
                container.key = key_prefix + "/" + name; // This could be encoded.
                container.name = name;
                container.desc = desc;
                container.level = 2;
            }

            template <class T>
            Folder &append(ark::ValueSubscriber<T> &valueSubscriber)
            {
                Item *child = new Item(valueSubscriber);
                child->container.key = this->container.key + "/" + child->container.key;
                container.children.push_back(child->container.key);
                children.push_back(child);
                return *this;
            }
            template <class T>
            Folder &append(ark::ValueSubscriber<T> &sub1, ark::ValueSubscriber<T> &sub2)
            {
                Item *child = new Item(sub1, sub2);
                child->container.key = this->container.key + "/" + child->container.key;
                container.children.push_back(child->container.key);
                children.push_back(child);
                return *this;
            }
            Folder &append(image_transport::Publisher &publisher)
            {
                Item *child = new Item(publisher);
                child->container.key = this->container.key + "/" + child->container.key;
                container.children.push_back(child->container.key);
                children.push_back(child);
                return *this;
            }

            void update()
            {
                for (auto &iter : children)
                {
                    iter->update();
                }
                ark_bridge::arkui_update data;
                data.request.operation = 0;
                data.request.key = container.key;
                data.request.detial = container;
                ros::service::call("/ark/bridge/arkui/update", data);
            }
        };

        class Tab
        {
        private:
            std::vector<Folder *> children;

        public:
            ark_bridge::ui_node container;
            Tab(const std::string &name, const std::string &desc = "", const std::string &key_prefix = "")
            {
                container.key = key_prefix + "/" + name; // This could be encoded.
                container.name = name;
                container.desc = desc;
                container.level = 1;
                folder("default");
            }
            Folder &folder(const std::string &name, const std::string &desc = "")
            {
                Folder *child = new Folder(name, desc, this->container.key);
                container.children.push_back(child->container.key);
                children.push_back(child);
                return *child;
            }

            Folder &folder()
            {
                return named("default");
            }

            Folder &named(const std::string &name)
            {
                for (auto &iter : children)
                {
                    if (name == iter->container.name)
                    {
                        return *iter;
                    }
                }
                return folder(name);
            }
            void update()
            {
                for (auto &iter : children)
                {
                    iter->update();
                }
                ark_bridge::arkui_update data;
                data.request.operation = 0;
                data.request.key = container.key;
                data.request.detial = container;
                ros::service::call("/ark/bridge/arkui/update", data);
                // Update
            }
        };

        class Category
        {
        private:
            std::vector<Tab *> children;

        public:
            ark_bridge::ui_node container;

            Category(const std::string &name, const std::string &desc = "", const std::string &key_prefix = "")
            {
                container.key = key_prefix + "/" + name; // This could be encoded.
                container.name = name;
                container.desc = desc;
                container.level = 0;
                tab("default");
            }

            Tab &tab(const std::string &name, const std::string &desc = "")
            {
                Tab *child = new Tab(name, desc, this->container.key);
                container.children.push_back(child->container.key);
                children.push_back(child);
                return *child;
            }
            Tab &tab()
            {
                return named("default");
            }

            Tab &named(const std::string &name)
            {
                for (auto &iter : children)
                {
                    if (name == iter->container.name)
                    {
                        return *iter;
                    }
                }
                return tab(name);
            }

            void update()
            {
                // TODO: Clear remote children
                for (auto &iter : children)
                {
                    iter->update();
                }
                ark_bridge::arkui_update data;
                data.request.operation = 0;
                data.request.key = container.key;
                data.request.detial = container;
                ros::service::call("/ark/bridge/arkui/update", data);
            }
        };

    }

}

#endif
