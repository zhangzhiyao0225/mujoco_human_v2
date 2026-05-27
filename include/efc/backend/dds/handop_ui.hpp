#ifndef HANDOP_USER_INTERFACE_HPP
#define HANDOP_USER_INTERFACE_HPP

#include <map>
#include <pugixml.hpp>
#include <thread>

#include "HandOp/HandOp.h"
#include "HandOp/HandOpPubSubTypes.h"

#include "dds/dds_pubsub_data.hpp"
#include "dds/handop_interface.hpp"
#include "utils/backend_event_decl.hpp"
#include <readerwriterqueue.h>

namespace handop
{
    using namespace std::chrono_literals;

    using TwistSubscriber = SubscriberTpl<TwistPubSubType>;
    using ActCmdSubscriber = SubscriberTpl<ActCmdPubSubType>;
    using RobotNotificationPublisher = PublisherTpl<RobotNotificationPubSubType>;

    using TwistSubType = TwistSubscriber::SubType;
    using ActCmdSubType = ActCmdSubscriber::SubType;
    using RobotNotificationPubType = RobotNotificationPublisher::PubType;

    class HandlerSubBase
    {
    public:
        HandlerSubBase(moodycamel::ReaderWriterQueue<std::vector<std::pair<bitbot::EventId, bitbot::EventValue>>> &sq,
                       bitbot::bitbot_init_param &initparam)
            : sq_(sq),
              initparam_(initparam)
        {
        }

        ~HandlerSubBase()
        {
        }

    protected:
        moodycamel::ReaderWriterQueue<std::vector<std::pair<bitbot::EventId, bitbot::EventValue>>> &sq_;
        bitbot::bitbot_init_param &initparam_;
    };

    class TwistHandler : public HandlerSubBase
    {
    public:
        TwistHandler(moodycamel::ReaderWriterQueue<std::vector<std::pair<bitbot::EventId, bitbot::EventValue>>> &sq,
                     bitbot::bitbot_init_param &initparam,
                     TwistSubType::Ptr subscriber)
            : HandlerSubBase(sq, initparam), subscriber_(subscriber)
        {

            twists_ = new Twist *[twist_count_];
            for (unsigned int i = 0; i < twist_count_; i++)
            {
                twists_[i] = new Twist();
            }

            subscriber_->SetProcFn(std::move(std::bind(&TwistHandler::handle, this, std::placeholders::_1)));
        }

        ~TwistHandler()
        {
            for (unsigned int i = 0; i < twist_count_; i++)
            {
                delete twists_[i];
            }
            delete[] twists_;
        }

        void handle(Twist &&twist)
        {
            if (!initparam_.can_do(BACKEND_EVENT_SET_ALL_VELO))
            {
                return;
            }

            Twist *current = twists_[current_twist_idx_];
            *current = std::move(twist);
            current_twist_idx_ = (current_twist_idx_ + 1) % twist_count_;
            sq_.enqueue({std::pair<bitbot::EventId, bitbot::EventValue>(BACKEND_EVENT_SET_ALL_VELO, reinterpret_cast<bitbot::EventValue>(current))});
        }

    private:
        TwistSubType::Ptr subscriber_;

        Twist **twists_ = nullptr;
        unsigned int current_twist_idx_ = 0;
        static constexpr unsigned int twist_count_ = 10;
    };

    class ActCmdHandler : public HandlerSubBase
    {
    public:
        ActCmdHandler(moodycamel::ReaderWriterQueue<std::vector<std::pair<bitbot::EventId, bitbot::EventValue>>> &sq,
                      bitbot::bitbot_init_param &initparam,
                      ActCmdSubType::Ptr subscriber)
            : HandlerSubBase(sq, initparam), subscriber_(subscriber)
        {
            subscriber_->SetProcFn(std::move(std::bind(&ActCmdHandler::handle, this, std::placeholders::_1)));
        }

        ~ActCmdHandler()
        {
        }

        void handle(ActCmd &&cmd)
        {
            // std::cout << "listen ActCmdHandler Data available, request " << cmd.act() << ", mapping " << (*initparam_.key_2_evts)[cmd.act()] << std::endl;
            bitbot::EventValue ev = 0;
            bool ok = sq_.enqueue({std::pair<bitbot::EventId, bitbot::EventValue>((*initparam_.key_2_evts)[cmd.act()], static_cast<bitbot::EventValue>(0))});
            if (!ok)
            {
                std::cout << "fail to enqueue data" << std::endl;
            }
        }

    private:
        ActCmdSubType::Ptr subscriber_;
    };

    class HandOpHandler
    {
    public:
        using Ptr = std::shared_ptr<HandOpHandler>;
        HandOpHandler(moodycamel::ReaderWriterQueue<std::vector<std::pair<bitbot::EventId, bitbot::EventValue>>>
                          &sq,
                      bitbot::bitbot_init_param &initparam,
                      const pugi::xml_node &config_node)
            : sq_(sq)
        {
            pugi::xml_node dds_node = config_node.child("dds");
            // std::cout << "cfg node name:" << config_node.name() << ", dds " << (!dds_node ? "(null)" : dds_node.name()) << std::endl;
            if (dds_node)
            {
                pugi::xml_node handop_node = dds_node.child("handop");
                if (handop_node)
                {
                    pugi::xml_node publishers_node = handop_node.child("publishers");
                    pugi::xml_node subscribers_node = handop_node.child("subscribers");

                    for (pugi::xml_node node = publishers_node.child("publisher"); node; node = node.next_sibling("publisher"))
                    {
                        auto node_attr_name = node.attribute("type").as_string();
                        if (strcmp(node_attr_name, "RobotNotificationPubType") == 0)
                        {
                            notify_publisher_ = std::make_shared<RobotNotificationPublisher>(node);
                            break;
                        }
                    }

                    for (pugi::xml_node node = subscribers_node.child("subscriber"); node; node = node.next_sibling("subscriber"))
                    {
                        auto node_attr_name = node.attribute("type").as_string();
                        if (strcmp(node_attr_name, "TwistSubType") == 0)
                        {
                            twist_subscriber_ = std::make_shared<TwistSubscriber>(node);
                            twist_handler_.reset(new TwistHandler(sq_, initparam, twist_subscriber_->GetSubscriber()));
                        }
                        else if (strcmp(node_attr_name, "ActCmdSubType") == 0)
                        {
                            actcmd_subscriber_ = std::make_shared<ActCmdSubscriber>(node);
                            actcmd_handler_.reset(new ActCmdHandler(sq_, initparam, actcmd_subscriber_->GetSubscriber()));
                        }
                    }
                }
            }

            // std::cout << "HandOpHandler::HandOpHandler()" << std::endl;
        }

        ~HandOpHandler()
        {
        }

        RobotNotificationPublisher::Ptr GetRobotNotificationPublisher()
        {
            return notify_publisher_;
        }

    private:
        moodycamel::ReaderWriterQueue<std::vector<std::pair<bitbot::EventId, bitbot::EventValue>>>
            &sq_;
        // DDS members
        TwistSubscriber::Ptr twist_subscriber_;
        ActCmdSubscriber::Ptr actcmd_subscriber_;
        std::unique_ptr<TwistHandler> twist_handler_{};
        std::unique_ptr<ActCmdHandler> actcmd_handler_{};
        RobotNotificationPublisher::Ptr notify_publisher_;
    };

} // namespace handop

#endif // !HANDOP_INTERFACE_HPP
