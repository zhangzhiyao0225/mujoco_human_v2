#ifndef DDS_PUBSUB_HPP
#define DDS_PUBSUB_HPP

#include <chrono>
#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/publisher/DataWriterListener.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/subscriber/DataReaderListener.hpp>
#include <fastdds/dds/subscriber/SampleInfo.hpp>
#include <fastdds/dds/subscriber/Subscriber.hpp>
#include <fastdds/dds/subscriber/qos/DataReaderQos.hpp>
#include <fastdds/dds/topic/Topic.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>
#include <iostream>
#include <string>

using namespace eprosima::fastdds::dds;
namespace bitbot
{

    template <typename PubSubT>
    class DdsPublisher
    {
    public:
        using PublisherPtr = std::shared_ptr<DdsPublisher<PubSubT>>;
        using MsgType = typename PubSubT::type;
        using MsgPtr = std::shared_ptr<typename PubSubT::type>;

        DdsPublisher(int const domain_id, std::string const &participant_name,
                     std::string const &topic_name)
        {
            eprosima::fastdds::dds::DomainParticipantQos participant_qos;
            participant_qos.name(participant_name);
            participant_ =
                eprosima::fastdds::dds::DomainParticipantFactory::get_instance()
                    ->create_participant(domain_id, participant_qos);

            type_ = TypeSupport(new PubSubT());
            type_.register_type(participant_);
            publisher_ = participant_->create_publisher(PUBLISHER_QOS_DEFAULT, nullptr);
            topic_ = participant_->create_topic(topic_name, type_.get_type_name(),
                                                TOPIC_QOS_DEFAULT);
            ::eprosima::fastdds::dds::DataWriterQos wQos = ::eprosima::fastdds::dds::DATAWRITER_QOS_DEFAULT;
            wQos.reliability().kind = ::eprosima::fastdds::dds::ReliabilityQosPolicyKind::BEST_EFFORT_RELIABILITY_QOS;
            wQos.history().kind = ::eprosima::fastdds::dds::HistoryQosPolicyKind::KEEP_LAST_HISTORY_QOS;
            wQos.history().depth = 1;

            writer_ = publisher_->create_datawriter(topic_, wQos);
        }

        ~DdsPublisher()
        {
            if (writer_ != nullptr)
            {
                publisher_->delete_datawriter(writer_);
            }
            if (publisher_ != nullptr)
            {
                participant_->delete_publisher(publisher_);
            }
            if (topic_ != nullptr)
            {
                participant_->delete_topic(topic_);
            }
        }

        bool Publish(MsgPtr msg)
        {
            writer_->write(msg.get());
            return true;
        }

    private:
        DomainParticipant *participant_;
        Publisher *publisher_;
        DataWriter *writer_;
        Topic *topic_;
        TypeSupport type_;
    };

    template <typename PubSubT>
    class DdsSubscriber
    {
    public:
        using SubscriberPtr = std::shared_ptr<DdsSubscriber<PubSubT>>;
        using MsgPtr = std::shared_ptr<typename PubSubT::type>;

        DdsSubscriber(int const domain_id, std::string const &participant_name,
                      std::string const &topic_name)
        {
            eprosima::fastdds::dds::DomainParticipantQos participant_qos;
            participant_qos.name(participant_name);
            participant_ =
                eprosima::fastdds::dds::DomainParticipantFactory::get_instance()
                    ->create_participant(domain_id, participant_qos);
            type_ = TypeSupport(new PubSubT());
            type_.register_type(participant_);

            subscriber_ =
                participant_->create_subscriber(SUBSCRIBER_QOS_DEFAULT, nullptr);
            topic_ = participant_->create_topic(topic_name, type_.get_type_name(),
                                                TOPIC_QOS_DEFAULT);

            ::eprosima::fastdds::dds::DataReaderQos rQos = ::eprosima::fastdds::dds::DATAREADER_QOS_DEFAULT;
            rQos.reliability().kind = ::eprosima::fastdds::dds::ReliabilityQosPolicyKind::BEST_EFFORT_RELIABILITY_QOS;
            rQos.history().kind = ::eprosima::fastdds::dds::HistoryQosPolicyKind::KEEP_LAST_HISTORY_QOS;
            rQos.history().depth = 1;
            reader_ = subscriber_->create_datareader(topic_, rQos,
                                                     &listener_);
        }

        ~DdsSubscriber()
        {
            if (reader_ != nullptr)
            {
                subscriber_->delete_datareader(reader_);
            }
            if (subscriber_ != nullptr)
            {
                participant_->delete_subscriber(subscriber_);
            }
            if (topic_ != nullptr)
            {
                participant_->delete_topic(topic_);
            }
        }

        bool MessageAvailable() { return listener_.data_flag_.load(); }

        MsgPtr GetMessage()
        {
            std::lock_guard<std::mutex> lock(listener_.mutex_);
            listener_.data_flag_.store(false);
            return listener_.msg_;
        }

    private:
        DomainParticipant *participant_;
        Subscriber *subscriber_;
        DataReader *reader_;
        Topic *topic_;
        TypeSupport type_;

        class SubListener : public DataReaderListener
        {
        public:
            void on_data_available(DataReader *reader) override
            {
                SampleInfo info;
                std::lock_guard<std::mutex> lock(mutex_);

                msg_ = std::make_shared<typename PubSubT::type>();
                if (reader->take_next_sample(msg_.get(), &info) ==
                    eprosima::fastrtps::types::ReturnCode_t::RETCODE_OK)
                {
                    if (info.valid_data)
                    {
                        data_flag_ = true;
                    }
                }
            }
            std::mutex mutex_;
            std::atomic_bool data_flag_{false};
            MsgPtr msg_{nullptr};
        } listener_;
    };

} // namespace bitbot

#endif // !DDS_PUBSUB_HPP
