#ifndef DDS_PUBSUB_DATA_HPP
#define DDS_PUBSUB_DATA_HPP

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
  class DataPublisher
  {
  public:
    using Ptr = std::shared_ptr<DataPublisher<PubSubT>>;
    using MsgType = typename PubSubT::type;
    using MsgPtr = std::shared_ptr<MsgType>;

    DataPublisher(int const domain_id, std::string const &participant_name,
                  std::string const &topic_name)
    {
      eprosima::fastdds::dds::DomainParticipantQos participant_qos = ::eprosima::fastdds::dds::PARTICIPANT_QOS_DEFAULT;
      participant_qos.name(participant_name);
      participant_ =
          eprosima::fastdds::dds::DomainParticipantFactory::get_instance()
              ->create_participant(domain_id, participant_qos);

      type_ = TypeSupport(new PubSubT());
      type_.register_type(participant_);
      publisher_ = participant_->create_publisher(PUBLISHER_QOS_DEFAULT, nullptr);
      topic_ = participant_->create_topic(topic_name, type_.get_type_name(),
                                          TOPIC_QOS_DEFAULT);

      std::cout << "Publisher created " << domain_id << ", " << topic_name << ", " << participant_name << ", " << type_.get_type_name() << std::endl;
      ::eprosima::fastdds::dds::DataWriterQos wQos = ::eprosima::fastdds::dds::DATAWRITER_QOS_DEFAULT;
      // wQos.reliability().kind = ::eprosima::fastdds::dds::ReliabilityQosPolicyKind::BEST_EFFORT_RELIABILITY_QOS;
      // wQos.history().kind = ::eprosima::fastdds::dds::HistoryQosPolicyKind::KEEP_LAST_HISTORY_QOS;
      // wQos.history().depth = 1;
      wQos.endpoint().history_memory_policy = ::eprosima::fastrtps::rtps::PREALLOCATED_WITH_REALLOC_MEMORY_MODE;
      wQos.history().kind = ::eprosima::fastdds::dds::HistoryQosPolicyKind::KEEP_LAST_HISTORY_QOS;
      wQos.history().depth = 50;
      wQos.resource_limits().max_samples = 200;
      wQos.resource_limits().max_instances = 10;
      wQos.resource_limits().max_samples_per_instance = 200;
      writer_ = publisher_->create_datawriter(topic_, wQos);
    }

    ~DataPublisher()
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

    bool CanPublish()
    {
      ::eprosima::fastdds::dds::PublicationMatchedStatus pub_status;
      writer_->get_publication_matched_status(pub_status);
      if (pub_status.current_count > 0)
      {
        // std::cout << "Publisher: subscriber matched..." << std::endl;
        return true;
      }

      static int wait_count = 0;
      if (wait_count++ == 0)
        std::cout << wait_count << "Publisher: waiting for subscriber..." << std::endl;
      return false;
    }

    bool Publish(MsgPtr msg)
    {
      // std::cout << "publish " << msg->data() << "..." << std::endl;
      if (!CanPublish())
      {
        return false;
      }
      writer_->write(msg.get());
      return true;
    }

    bool Publish(MsgType &&msg)
    {
      if (!CanPublish())
      {
        return false;
      }
      // std::cout << "publish " << msg.data() << "..." << std::endl;
      writer_->write(&msg);
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
  class DataSubscriber
  {
  public:
    using Ptr = std::shared_ptr<DataSubscriber<PubSubT>>;
    using MsgType = typename PubSubT::type;
    using MsgPtr = std::shared_ptr<MsgType>;

    DataSubscriber(int const domain_id, std::string const &participant_name,
                   std::string const &topic_name)
    {
      eprosima::fastdds::dds::DomainParticipantQos participant_qos = ::eprosima::fastdds::dds::PARTICIPANT_QOS_DEFAULT;
      participant_qos.name(participant_name);
      participant_ =
          eprosima::fastdds::dds::DomainParticipantFactory::get_instance()
              ->create_participant(domain_id, participant_qos);
      type_ = TypeSupport(new PubSubT());
      type_.register_type(participant_);

      subscriber_ =
          participant_->create_subscriber(::eprosima::fastdds::dds::SUBSCRIBER_QOS_DEFAULT, nullptr);
      topic_ = participant_->create_topic(topic_name, type_.get_type_name(),
                                          TOPIC_QOS_DEFAULT);

      std::cout << "Subscriber created with topic " << topic_name << std::endl;
      // ::eprosima::fastdds::dds::DataReaderQos rQos = ::eprosima::fastdds::dds::DATAREADER_QOS_DEFAULT;
      // rQos.reliability().kind = ::eprosima::fastdds::dds::ReliabilityQosPolicyKind::BEST_EFFORT_RELIABILITY_QOS;
      // rQos.history().kind = ::eprosima::fastdds::dds::HistoryQosPolicyKind::KEEP_LAST_HISTORY_QOS;
      // rQos.history().depth = 1;
      // reader_ = subscriber_->create_datareader(topic_, rQos, &listener_);
      reader_ = subscriber_->create_datareader(topic_, ::eprosima::fastdds::dds::DATAREADER_QOS_DEFAULT, &listener_);
    }

    ~DataSubscriber()
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

    void SetProcFn(std::function<void(MsgType &&)> &&fn)
    {
      listener_.SetProcFn(std::forward<std::function<void(MsgType &&)>>(fn));
    }

    bool MessageAvailable() { return listener_.data_flag_.load(); }

  private:
    DomainParticipant *participant_;
    Subscriber *subscriber_;
    DataReader *reader_;
    Topic *topic_;
    TypeSupport type_;

    class SubListener : public DataReaderListener
    {
    public:
      SubListener() {}

      void on_subscription_matched(
          ::eprosima::fastdds::dds::DataReader *,
          const ::eprosima::fastdds::dds::SubscriptionMatchedStatus &info)
      {
        if (info.current_count_change == 1)
        {
          std::cout << "Subscriber: match publisher ok." << std::endl;
        }
        else if (info.current_count_change == -1)
        {
          std::cout << "Subscriber: unmatch publisher." << std::endl;
        }
        else
        {
          std::cout << info.current_count_change
                    << " is not a valid value for SubscriptionMatchedStatus current count change" << std::endl;
        }
      }

      void on_data_available(DataReader *reader) override
      {
        SampleInfo info;
        // std::cout << "listen Data received..." << std::endl;

        // std::lock_guard<std::mutex> lock(mutex_);

        MsgType rc;
        if (reader->take_next_sample(&rc, &info) ==
            eprosima::fastrtps::types::ReturnCode_t::RETCODE_OK)
        {
          if (info.valid_data)
          {
            on_try_proc_data_(std::move(rc));

            data_flag_ = true;
          }
        }
      }

      void SetProcFn(std::function<void(MsgType &&)> &&fn)
      {
        on_try_proc_data_ = std::move(fn);
      }

      // std::mutex mutex_;
      std::atomic_bool data_flag_{false};
      std::function<void(MsgType &&)> on_try_proc_data_;
    } listener_;
  };

} // namespace bitbot

#endif // !DDS_PUBSUB_HPP
