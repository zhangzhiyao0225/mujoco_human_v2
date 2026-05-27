#ifndef HANDOP_INTERFACE_HPP
#define HANDOP_INTERFACE_HPP

#include <map>
#include <pugixml.hpp>
#include <thread>

#include "dds/dds_pubsub_data.hpp"

namespace handop
{
    using namespace std::chrono_literals;

    template <typename PubSubType>
    class SubscriberTpl
    {
    public:
        using SubType = bitbot::DataSubscriber<PubSubType>;
        using MsgType = SubType::MsgType;
        using Ptr = std::shared_ptr<SubscriberTpl<PubSubType>>;

    public:
        /**
         * @brief Constructor by config node
         * @param config_node in, config node @ subscriber type="DataSubType"
         */
        SubscriberTpl(const pugi::xml_node &config_node, bool using_proxy_thread = false)
        {
            subscriber_ = std::make_shared<SubType>(
                config_node.attribute("domain_id").as_int(),
                config_node.attribute("participant_name").as_string(),
                config_node.attribute("topic_name").as_string());
        }

        ~SubscriberTpl()
        {
        }

        SubType::Ptr GetSubscriber()
        {
            return subscriber_;
        }

        SubType::MsgType GetData()
        {
            return data_;
        }

    private:
        // DDS members
        SubType::Ptr subscriber_;
        SubType::MsgType data_{};

        std::atomic_bool system_ready_{false};
    };

    template <typename PubSubType>
    class PublisherTpl
    {
    public:
        using PubType = bitbot::DataPublisher<PubSubType>;
        using MsgType = PubType::MsgType;
        using MsgPtr = PubType::MsgPtr;

    public:
        using Ptr = std::shared_ptr<PublisherTpl<PubSubType>>;

    public:
        /**
         * @brief Constructor by config node
         * @param config_node in, config node @ publisher type="RobotNotificationPubType"
         */
        PublisherTpl(const pugi::xml_node &config_node, bool using_proxy_thread = false)
        {
            publisher_ = std::make_shared<PubType>(
                config_node.attribute("domain_id").as_int(),
                config_node.attribute("participant_name").as_string(),
                config_node.attribute("topic_name").as_string());

            if (!using_proxy_thread)
            {
            }
        }

        ~PublisherTpl()
        {
            // if (thread_.joinable())
            // {
            //     thread_.join();
            // }
        }

        void Publish(MsgType &&data)
        {
            publisher_->Publish(std::forward<MsgType>(data));
        }

    private:
        PubType::Ptr publisher_;
    };

} // namespace handop

#endif // !HANDOP_INTERFACE_HPP
