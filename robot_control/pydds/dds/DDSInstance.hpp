/*
 * @Non-code author: Alvin_kalu
 * @Date: 2024-09-18 13:44:33
 * @LastEditors: Alvin_kalu guanwu.li@fftai.com
 * @LastEditTime: 2024-10-11 15:27:14
 * @FilePath: /data/robot_system/sim/pydds/dds/DDSInstance.hpp
 * @Description: 
 * 
 * Copyright (c) 2024 by Fourier Intelligence Co. Ltd , All Rights Reserved. 
 */
#ifndef __DDS_INSTANCE_HPP__
#define __DDS_INSTANCE_HPP__

#include <memory>
#include <functional>
#include <condition_variable>

#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/publisher/DataWriterListener.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>

#include <fastdds/dds/subscriber/SampleInfo.hpp>
#include <fastdds/dds/subscriber/DataReaderListener.hpp>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>

#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/subscriber/Subscriber.hpp>

#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/publisher/qos/DataWriterQos.hpp>
#include <fastdds/dds/publisher/qos/PublisherQos.hpp>

namespace fftai{
namespace dds{

namespace fastdds = eprosima::fastdds::dds;

class Context{
public:
    virtual ~Context(){
        if (nullptr != participant_){
            // Delete DDS entities contained within the DomainParticipant
            participant_->delete_contained_entities();
            factory_->delete_participant(participant_);
        }
    }

    fastdds::Topic* create_topic(
            const std::string& topic_name,
            const std::string& type_name,
            const fastdds::TopicQos& qos,
            fastdds::TopicListener* listener = nullptr,
            const fastdds::StatusMask& mask = fastdds::StatusMask::all()){
        auto description = participant_->lookup_topicdescription(topic_name);
        if(description != nullptr){
            return static_cast<fastdds::Topic*>(description);
        }
        return participant_->create_topic(topic_name, type_name, qos, listener, mask);
    }

    virtual fastdds::DomainParticipant* participant(){
        return participant_;
    }

    bool initialize(const int domain_id){
        fastdds::DomainParticipantQos pqos = fastdds::PARTICIPANT_QOS_DEFAULT;
        pqos.name("participant");

        factory_ = fastdds::DomainParticipantFactory::get_shared_instance();
        participant_ = factory_->create_participant(domain_id, pqos, nullptr, fastdds::StatusMask::none());
        return participant_ != nullptr;
    }

protected:
    std::shared_ptr<fastdds::DomainParticipantFactory> factory_;
    fastdds::DomainParticipant* participant_ = nullptr;
};

class Instance{
public:
    virtual ~Instance() = default;
};

class Publisher : public Instance{
public:
    virtual ~Publisher() = default;
};

class Subscriber : public Instance{
public:
    virtual ~Subscriber() = default;
};

template<typename MessageSubType>
class PublisherInstance : public Publisher, public fastdds::DataWriterListener{
public:
    typedef typename MessageSubType::type MessageType;
    virtual ~PublisherInstance(){
        auto participant = context_ != nullptr ? context_->participant() : nullptr;
        if(participant != nullptr){
            if(publisher_ != nullptr){
                if(writer_ != nullptr)
                    publisher_->delete_datawriter(writer_);
                participant->delete_publisher(publisher_);
            }

            if(topic_ != nullptr)
                participant->delete_topic(topic_);
        }
    }

    virtual void on_publication_matched(fastdds::DataWriter* writer, const fastdds::PublicationMatchedStatus& info) override{
        std::unique_lock<std::mutex> lock(matched_mutex_);
        if(info.current_count_change == 1){
            matched_.store(true);
        }else if(info.current_count_change == -1){
            matched_.store(info.current_count > 0);
        }
        matched_cv_.notify_one();
    }

    virtual bool matched(){
        return matched_.load();
    }

    virtual bool publish_message(const MessageType& message){
        return this->writer_->write(&message) == fastdds::RETCODE_OK;
    }

    virtual bool initialize(std::shared_ptr<Context> context, const std::string& topic_name, bool wait_for_matched=false, int wait_timeout_ms=5000){
        context_ = context;
        auto participant = context_->participant();
        type_ = fastdds::TypeSupport(new MessageSubType());
        type_.register_type(participant);

        // Create the publisher
        fastdds::PublisherQos pub_qos = fastdds::PUBLISHER_QOS_DEFAULT;
        participant->get_default_publisher_qos(pub_qos);
        publisher_ = participant->create_publisher(pub_qos, nullptr, fastdds::StatusMask::none());
        if (publisher_ == nullptr){
            printf("%s Publisher initialization failed\n", topic_name.c_str());
            return false;
        }

        // Create the topic
        fastdds::TopicQos topic_qos = fastdds::TOPIC_QOS_DEFAULT;
        participant->get_default_topic_qos(topic_qos);
        topic_qos.deadline().period = fastdds::Duration_t(0, 1000);
        topic_ = context->create_topic(topic_name, type_.get_type_name(), topic_qos);
        if (topic_ == nullptr){
            printf("%s Topic initialization failed\n", topic_name.c_str());
            return false;
        }

        // Create the data writer
        fastdds::DataWriterQos writer_qos = fastdds::DATAWRITER_QOS_DEFAULT;
        publisher_->get_default_datawriter_qos(writer_qos);
        writer_qos.reliability().kind = fastdds::ReliabilityQosPolicyKind::BEST_EFFORT_RELIABILITY_QOS;
        writer_qos.durability().kind = fastdds::DurabilityQosPolicyKind::VOLATILE_DURABILITY_QOS;
        writer_qos.history().kind = fastdds::HistoryQosPolicyKind::KEEP_LAST_HISTORY_QOS;
        writer_qos.history().depth = 1;
        writer_ = publisher_->create_datawriter(topic_, writer_qos, this, fastdds::StatusMask::all());
        if (writer_ == nullptr){
            printf("%s DataWriter initialization failed\n", topic_name.c_str());
            return false;
        }

        if(wait_for_matched){
            {
                std::unique_lock<std::mutex> lock(matched_mutex_);
                matched_cv_.wait_for(lock, std::chrono::milliseconds(wait_timeout_ms), [&](){
                    return matched_.load();
                });
            }
            auto matched = matched_.load();
            if(!matched){
                printf("Timeout for found matched publication with %d ms, topic_name = %s\n", wait_timeout_ms, topic_name.c_str());
            }
            return matched;
        }
        return true;
    }

private:
    fastdds::Publisher* publisher_ = nullptr;
    fastdds::Topic* topic_ = nullptr;
    fastdds::DataWriter* writer_ = nullptr;
    fastdds::TypeSupport type_;
    std::shared_ptr<Context> context_;
    std::atomic<bool> matched_{false};
    std::condition_variable matched_cv_;
    std::mutex matched_mutex_;
};

template<typename MessageSubType>
class SubscriberInstance : public Subscriber, public fastdds::DataReaderListener{
public:
    typedef typename MessageSubType::type MessageType;
    virtual ~SubscriberInstance(){
        auto participant = context_ != nullptr ? context_->participant() : nullptr;
        if(participant != nullptr){
            if(subscriber_ != nullptr){
                if(reader_ != nullptr)
                    subscriber_->delete_datareader(reader_);
                participant->delete_subscriber(subscriber_);
            }

            if(topic_ != nullptr)
                participant->delete_topic(topic_);
        }
    }

    virtual bool matched(){
        return matched_.load();
    }

    virtual void on_subscription_matched(fastdds::DataReader* reader, const fastdds::SubscriptionMatchedStatus& info) override{
        std::unique_lock<std::mutex> lock(matched_mutex_);
        if(info.current_count_change == 1){
            matched_.store(true);
        }else if(info.current_count_change == -1){
            matched_.store(info.current_count > 0);
        }
        matched_cv_.notify_one();
    }

    void on_data_available(fastdds::DataReader* reader) override{
        while (fastdds::RETCODE_OK == reader->take_next_sample(&current_message_, &current_message_info_)){
            if ((current_message_info_.instance_state == fastdds::ALIVE_INSTANCE_STATE) && current_message_info_.valid_data){
                if(callback_){
                    callback_(current_message_);
                }
            }
        }
    }

    virtual bool initialize(std::shared_ptr<Context> context, const std::string& topic_name, const std::function<void(const MessageType& msg)>& callback, bool wait_for_matched=false, int wait_timeout_ms=5000){

        context_ = context;
        auto participant = context->participant();
        type_ = fastdds::TypeSupport(new MessageSubType());
        type_.register_type(participant);
        callback_ = callback;

        // Create the subscriber
        fastdds::SubscriberQos sub_qos = fastdds::SUBSCRIBER_QOS_DEFAULT;
        participant->get_default_subscriber_qos(sub_qos);
        subscriber_ = participant->create_subscriber(sub_qos, nullptr, fastdds::StatusMask::none());
        if (subscriber_ == nullptr){
            printf("%s Subscriber initialization failed\n", topic_name.c_str());
            return false;
        }

        // Create the topic
        fastdds::TopicQos topic_qos = fastdds::TOPIC_QOS_DEFAULT;
        participant->get_default_topic_qos(topic_qos);
        topic_qos.deadline().period = fastdds::Duration_t(0, 1000);
        topic_ = context->create_topic(topic_name, type_.get_type_name(), topic_qos);
        if (topic_ == nullptr){
            printf("%s Topic initialization failed\n", topic_name.c_str());
            return false;
        }

        // Create the reader
        fastdds::DataReaderQos reader_qos = fastdds::DATAREADER_QOS_DEFAULT;
        subscriber_->get_default_datareader_qos(reader_qos);
        reader_qos.reliability().kind = fastdds::ReliabilityQosPolicyKind::BEST_EFFORT_RELIABILITY_QOS;
        reader_qos.durability().kind = fastdds::DurabilityQosPolicyKind::VOLATILE_DURABILITY_QOS;
        reader_qos.history().kind = fastdds::HistoryQosPolicyKind::KEEP_LAST_HISTORY_QOS;
        reader_qos.history().depth = 1;
        reader_ = subscriber_->create_datareader(topic_, reader_qos, this, fastdds::StatusMask::all());
        if (reader_ == nullptr){
            printf("%s DataReader initialization failed\n", topic_name.c_str());
            return false;
        }
        
        if(wait_for_matched){
            {
                std::unique_lock<std::mutex> lock(matched_mutex_);
                matched_cv_.wait_for(lock, std::chrono::milliseconds(wait_timeout_ms), [&](){
                    return matched_.load();
                });
            }
            auto matched = matched_.load();
            if(!matched){
                printf("Timeout for found matched publication with %d ms, topic_name = %s\n", wait_timeout_ms, topic_name.c_str());
            }
            return matched;
        }
        return true;
    }

private:
    fastdds::Subscriber* subscriber_;
    fastdds::Topic* topic_;
    fastdds::DataReader* reader_;
    fastdds::TypeSupport type_;
    MessageType current_message_;
    fastdds::SampleInfo current_message_info_;
    std::function<void(const MessageType& msg)> callback_;
    std::shared_ptr<Context> context_;
    std::atomic<bool> matched_{false};
    std::condition_variable matched_cv_;
    std::mutex matched_mutex_;
};

static std::shared_ptr<Context> create_dds_context(const int domain_id){
    std::shared_ptr<Context> instance(new Context());
    if(!instance->initialize(domain_id)){
        instance.reset();
    }
    return instance;
}

template<typename MessageSubType>
static std::shared_ptr<PublisherInstance<MessageSubType>> create_dds_publisher(
    std::shared_ptr<Context> context, 
    const std::string& topic_name,
    bool wait_for_matched = false, 
    int wait_timeout_ms   = 5000
){
    std::shared_ptr<PublisherInstance<MessageSubType>> instance(new PublisherInstance<MessageSubType>());
    if(!instance->initialize(context, topic_name, wait_for_matched, wait_timeout_ms))
        instance.reset();
    return instance;
}

template<typename MessageSubType>
std::shared_ptr<SubscriberInstance<MessageSubType>> create_dds_subscriber(
    std::shared_ptr<Context> context, 
    const std::string& topic_name, 
    const std::function<void(const typename MessageSubType::type& msg)>& callback,
    bool wait_for_matched = false, 
    int wait_timeout_ms   = 5000
){
    std::shared_ptr<SubscriberInstance<MessageSubType>> instance(new SubscriberInstance<MessageSubType>());
    if(!instance->initialize(context, topic_name, callback, wait_for_matched, wait_timeout_ms))
        instance.reset();
    return instance;
}

}; // namespace DDS
}; // namespace fftai

#endif // __DDS_INSTANCE_HPP__