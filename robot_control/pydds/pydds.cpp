#include <pybind11/pybind11.h>
#include <csignal>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <future>
#include <string.h>
#include <fstream>
#include <csignal>
#include <unordered_map>
#include <memory>

#include <dds/RobotSystemPubSubTypes.hpp>
#include <dds/DDSInstance.hpp>

namespace py = pybind11;

std::string format(const char* fmt, ...) {
    va_list vl;
    va_start(vl, fmt);
    char buffer[2048];
    vsnprintf(buffer, sizeof(buffer), fmt, vl);
    return buffer;
}

int64_t get_current_timestamp(){
    return std::chrono::high_resolution_clock::now().time_since_epoch().count();
}

template<typename MessageSubType>
class SubscriberMessageCollection{
private:
    typedef typename MessageSubType::type MessageType;
    std::shared_ptr<fftai::dds::SubscriberInstance<MessageSubType>> subscriber_;
    std::mutex data_ready_lock_;
    std::unordered_map<std::string, MessageType> current_messages_data_ready_;
    std::unordered_map<std::string, std::atomic<bool>> have_new_message_;
    std::unordered_map<std::string, int64_t> last_message_time_ns_;

public:
    virtual ~SubscriberMessageCollection() {
        subscriber_.reset();
    };
    virtual void on_message_available(const MessageType& msg){
        if(msg.status() != "OK"){
            // printf("Got a message that status is not OK. source = %s\n", msg.source().c_str());
            return;
        }

        std::unique_lock<std::mutex> lock(data_ready_lock_);
        current_messages_data_ready_[msg.source()] = msg;
        have_new_message_[msg.source()].store(true);
        last_message_time_ns_[msg.source()] = get_current_timestamp();
    }

    virtual bool initialize(const std::shared_ptr<fftai::dds::Context>& context, const std::string& topic_name, bool wait_for_matched, int wait_timeout_ms){
        std::function<void(const MessageType& msg)> callback = std::bind(&SubscriberMessageCollection<MessageSubType>::on_message_available, this, std::placeholders::_1);
        subscriber_ = fftai::dds::create_dds_subscriber<MessageSubType>(context, topic_name, callback, wait_for_matched, wait_timeout_ms);
        return subscriber_ != nullptr;
    }

    bool have_new_message(const std::string& source){
        std::unique_lock<std::mutex> lock(data_ready_lock_);
        return have_new_message_[source].load();
    }

    int64_t message_latency_ns(const std::string& source){
        std::unique_lock<std::mutex> lock(data_ready_lock_);
        return get_current_timestamp() - last_message_time_ns_[source];
    }

    MessageType get_current_message(const std::string& source){
        std::unique_lock<std::mutex> lock(data_ready_lock_);
        have_new_message_[source].store(false);
        return current_messages_data_ready_[source];
    }
};

template<typename MessageSubType>
std::shared_ptr<SubscriberMessageCollection<MessageSubType>> create_dds_subscriber_message_collection(
    const std::shared_ptr<fftai::dds::Context>& context, 
    const std::string& topic_name,
    bool wait_for_matched, 
    int wait_timeout_ms
){
    std::shared_ptr<SubscriberMessageCollection<MessageSubType>> instance(new SubscriberMessageCollection<MessageSubType>());
    if(!instance->initialize(context, topic_name, wait_for_matched, wait_timeout_ms))
        instance.reset();
    return instance;
}

PYBIND11_MODULE(libpydds, m) {
    py::class_<fftai::dds::Context, std::shared_ptr<fftai::dds::Context>>(m, "Context")
		.def(py::init([](int domain){return fftai::dds::create_dds_context(domain);}), py::arg("domain")=0)
		.def("__repr__", [](fftai::dds::Context& self){
			return format("<Context this=%p>", &self);
		});

    //1. SystemState
    py::class_<RobotSystem::SystemStateRequest>(m, "SystemStateRequest")
        .def(py::init([](std::string target){
            RobotSystem::SystemStateRequest result;
            result.target(target);
            return result;
        }))
		.def_property_readonly("target", [](RobotSystem::SystemStateRequest& self){return self.target();});

    py::class_<RobotSystem::SystemStateResponse>(m, "SystemStateResponse")
        .def(py::init([](std::string source, int timestamp, int state, std::string status){
            RobotSystem::SystemStateResponse result;
            result.source(source);
            result.timestamp(timestamp);
            result.state(state);
            result.status(status);
            return result;
        }))
		.def_property_readonly("source", [](RobotSystem::SystemStateResponse& self){return self.source();})
		.def_property_readonly("timestamp", [](RobotSystem::SystemStateResponse& self){return self.timestamp();})
		.def_property_readonly("state", [](RobotSystem::SystemStateResponse& self){return self.state();})
		.def_property_readonly("status", [](RobotSystem::SystemStateResponse& self){return self.status();})
        .def("__repr__", [](RobotSystem::SystemStateResponse& self){
			return format("<source=%s, timestamp=%ld, state=%d, status=%s>", self.source().c_str(), self.timestamp(), self.state(), self.status().c_str());
		});

    using SystemStateRequestPublisher = fftai::dds::PublisherInstance<RobotSystem::SystemStateRequestPubSubType>;
    py::class_<SystemStateRequestPublisher, std::shared_ptr<SystemStateRequestPublisher>>(m, "SystemStateRequestPublisher")
		.def(py::init([](std::shared_ptr<fftai::dds::Context> context, std::string topic_name, bool wait_for_matched, int wait_timeout_ms){
            return fftai::dds::create_dds_publisher<RobotSystem::SystemStateRequestPubSubType>(context, topic_name, wait_for_matched, wait_timeout_ms);
        }))
        .def("publish", [](SystemStateRequestPublisher& self, const RobotSystem::SystemStateRequest& msg){ return self.publish_message(msg); })
		.def("__repr__", [](SystemStateRequestPublisher& self){
			return format("<SystemStateRequestPublisher this=%p>", &self);
		});
    
    using SystemStateResponseSubscriber = SubscriberMessageCollection<RobotSystem::SystemStateResponsePubSubType>;
    py::class_<SystemStateResponseSubscriber, std::shared_ptr<SystemStateResponseSubscriber>>(m, "SystemStateResponseSubscriber")
		.def(py::init([](std::shared_ptr<fftai::dds::Context> context, std::string topic_name, bool wait_for_matched, int wait_timeout_ms){
            return create_dds_subscriber_message_collection<RobotSystem::SystemStateResponsePubSubType>(context, topic_name, wait_for_matched, wait_timeout_ms);
        }))
        .def("current_message", [](SystemStateResponseSubscriber& self, std::string source){return self.get_current_message(source);})
        .def("have_new_message", [](SystemStateResponseSubscriber& self, std::string source){return self.have_new_message(source);})
        .def("message_latency_ns", [](SystemStateResponseSubscriber& self, std::string source){return self.message_latency_ns(source);})
		.def("__repr__", [](SystemStateResponseSubscriber& self){
			return format("<SystemStateResponseSubscriber this=%p>", &self);
		});

    //2. PVCState
    py::class_<RobotSystem::PVCStateRequest>(m, "PVCStateRequest")
        .def(py::init([](std::string target, bool position, bool velocity, bool current){
            RobotSystem::PVCStateRequest result;
            result.target(target);
            result.position(position);
            result.velocity(velocity);
            result.current(current);
            return result;
        }))
		.def_property_readonly("target", [](RobotSystem::PVCStateRequest& self){return self.target();})
        .def("__repr__", [](RobotSystem::PVCStateRequest& self){
			return format("<position=%s, velocity=%s, current=%s>", 
                self.position() ? "true" : "false", self.velocity() ? "true" : "false", self.current() ? "true" : "false"
            );
		});

    py::class_<RobotSystem::PVCStateResponse>(m, "PVCStateResponse")
        .def(py::init([](std::string source, int timestamp, std::string status, float position, float velocity, float current){
            RobotSystem::PVCStateResponse result;
            result.source(source);
            result.timestamp(timestamp);
            result.status(status);
            result.position(position);
            result.velocity(velocity);
            result.current(current);
            return result;
        }))
		.def_property_readonly("source", [](RobotSystem::PVCStateResponse& self){return self.source();})
		.def_property_readonly("timestamp", [](RobotSystem::PVCStateResponse& self){return self.timestamp();})
		.def_property_readonly("status", [](RobotSystem::PVCStateResponse& self){return self.status();})
		.def_property_readonly("position", [](RobotSystem::PVCStateResponse& self){return self.position();})
		.def_property_readonly("velocity", [](RobotSystem::PVCStateResponse& self){return self.velocity();})
		.def_property_readonly("current", [](RobotSystem::PVCStateResponse& self){return self.current();})
        .def("__repr__", [](RobotSystem::PVCStateResponse& self){
			return format("<source=%s, timestamp=%ld, status=%s, position=%f, velocity=%f, current=%f>", self.source().c_str(), self.timestamp(), self.status().c_str(), self.position(), self.velocity(), self.current());
		});

    using PVCStateRequestPublisher = fftai::dds::PublisherInstance<RobotSystem::PVCStateRequestPubSubType>;
    py::class_<PVCStateRequestPublisher, std::shared_ptr<PVCStateRequestPublisher>>(m, "PVCStateRequestPublisher")
		.def(py::init([](std::shared_ptr<fftai::dds::Context> context, std::string topic_name, bool wait_for_matched, int wait_timeout_ms){
            return fftai::dds::create_dds_publisher<RobotSystem::PVCStateRequestPubSubType>(context, topic_name, wait_for_matched, wait_timeout_ms);
        }))
        .def("publish", [](PVCStateRequestPublisher& self, const RobotSystem::PVCStateRequest& msg){ return self.publish_message(msg); })
		.def("__repr__", [](PVCStateRequestPublisher& self){
			return format("<PVCStateRequestPublisher this=%p>", &self);
		});
    
    using PVCStateResponseSubscriber = SubscriberMessageCollection<RobotSystem::PVCStateResponsePubSubType>;
    py::class_<PVCStateResponseSubscriber, std::shared_ptr<PVCStateResponseSubscriber>>(m, "PVCStateResponseSubscriber")
		.def(py::init([](std::shared_ptr<fftai::dds::Context> context, std::string topic_name, bool wait_for_matched, int wait_timeout_ms){
            return create_dds_subscriber_message_collection<RobotSystem::PVCStateResponsePubSubType>(context, topic_name, wait_for_matched, wait_timeout_ms);
        }))
        .def("current_message", [](PVCStateResponseSubscriber& self, std::string source){return self.get_current_message(source);})
        .def("have_new_message", [](PVCStateResponseSubscriber& self, std::string source){return self.have_new_message(source);})
        .def("message_latency_ns", [](PVCStateResponseSubscriber& self, std::string source){return self.message_latency_ns(source);})
		.def("__repr__", [](PVCStateResponseSubscriber& self){
			return format("<PVCStateResponseSubscriber this=%p>", &self);
		});

    //3. PositionControl
    py::class_<RobotSystem::PositionControlRequest>(m, "PositionControlRequest")
        .def(py::init([](std::string target, float position){
            RobotSystem::PositionControlRequest result;
            result.target(target);
            result.position(position);
            return result;
        }))
		.def_property_readonly("target", [](RobotSystem::PositionControlRequest& self){return self.target();})
        .def_property_readonly("position", [](RobotSystem::PositionControlRequest& self) { return self.position(); })
        .def("__repr__", [](RobotSystem::PositionControlRequest& self){
			return format("<target=%s, position=%f>", self.target().c_str(), self.position());
		});
    
    py::class_<RobotSystem::PositionControlResponse>(m, "PositionControlResponse")
        .def(py::init([](std::string source, int timestamp, std::string status, float position, float velocity, float current, int current_id){
            RobotSystem::PositionControlResponse result;
            result.source(source);
            result.timestamp(timestamp);
            result.status(status);
            result.position(position);
            result.velocity(velocity);
            result.current(current);
            result.current_id(current_id);
            return result;
        }))
		.def_property_readonly("source", [](RobotSystem::PositionControlResponse& self){return self.source();})
		.def_property_readonly("timestamp", [](RobotSystem::PositionControlResponse& self){return self.timestamp();})
		.def_property_readonly("status", [](RobotSystem::PositionControlResponse& self){return self.status();})
		.def_property_readonly("position", [](RobotSystem::PositionControlResponse& self){return self.position();})
		.def_property_readonly("velocity", [](RobotSystem::PositionControlResponse& self){return self.velocity();})
		.def_property_readonly("current", [](RobotSystem::PositionControlResponse& self){return self.current();})
		.def_property_readonly("current_id", [](RobotSystem::PositionControlResponse& self){return self.current_id();})
        .def("__repr__", [](RobotSystem::PositionControlResponse& self){
			return format("<source=%s, timestamp=%ld, status=%s, position=%f, velocity=%f, current=%f, current_id=%d>", 
                          self.source().c_str(), self.timestamp(), self.status().c_str(), self.position(), self.velocity(), self.current(), self.current_id());
		});

    using PositionControlRequestPublisher = fftai::dds::PublisherInstance<RobotSystem::PositionControlRequestPubSubType>;
    py::class_<PositionControlRequestPublisher, std::shared_ptr<PositionControlRequestPublisher>>(m, "PositionControlRequestPublisher")
		.def(py::init([](std::shared_ptr<fftai::dds::Context> context, std::string topic_name, bool wait_for_matched, int wait_timeout_ms){
            return fftai::dds::create_dds_publisher<RobotSystem::PositionControlRequestPubSubType>(context, topic_name, wait_for_matched, wait_timeout_ms);
        }))
        .def("publish", [](PositionControlRequestPublisher& self, const RobotSystem::PositionControlRequest& msg){ return self.publish_message(msg); })
		.def("__repr__", [](PositionControlRequestPublisher& self){
			return format("<PositionControlRequestPublisher this=%p>", &self);
		});
    
    using PositionControlResponseSubscriber = SubscriberMessageCollection<RobotSystem::PositionControlResponsePubSubType>;
    py::class_<PositionControlResponseSubscriber, std::shared_ptr<PositionControlResponseSubscriber>>(m, "PositionControlResponseSubscriber")
		.def(py::init([](std::shared_ptr<fftai::dds::Context> context, std::string topic_name, bool wait_for_matched, int wait_timeout_ms){
            return create_dds_subscriber_message_collection<RobotSystem::PositionControlResponsePubSubType>(context, topic_name, wait_for_matched, wait_timeout_ms);
        }))
        .def("current_message", [](PositionControlResponseSubscriber& self, std::string source){return self.get_current_message(source);})
        .def("have_new_message", [](PositionControlResponseSubscriber& self, std::string source){return self.have_new_message(source);})
        .def("message_latency_ns", [](PositionControlResponseSubscriber& self, std::string source){return self.message_latency_ns(source);})
		.def("__repr__", [](PositionControlResponseSubscriber& self){
			return format("<PositionControlResponseSubscriber this=%p>", &self);
		});
    
    //4. MotorControl
    py::class_<RobotSystem::MotorControlRequest>(m, "MotorControlRequest")
        .def(py::init([](std::string target, int control_word){
            RobotSystem::MotorControlRequest result;
            result.target(target);
            result.control_word(control_word);
            return result;
        }))
		.def_property_readonly("target", [](RobotSystem::MotorControlRequest& self){return self.target();})
		.def_property_readonly("control_word", [](RobotSystem::MotorControlRequest& self){return self.control_word();})
        .def("__repr__", [](RobotSystem::MotorControlRequest& self){
			return format("<target=%s, control_word=%d>", self.target().c_str(), self.control_word());
		});

    py::class_<RobotSystem::MotorControlResponse>(m, "MotorControlResponse")
        .def(py::init([](std::string source, int timestamp, std::string status){
            RobotSystem::MotorControlResponse result;
            result.source(source);
            result.timestamp(timestamp);
            result.status(status);
            return result;
        }))
		.def_property_readonly("source", [](RobotSystem::MotorControlResponse& self){return self.source();})
		.def_property_readonly("timestamp", [](RobotSystem::MotorControlResponse& self){return self.timestamp();})
		.def_property_readonly("status", [](RobotSystem::MotorControlResponse& self){return self.status();})
        .def("__repr__", [](RobotSystem::MotorControlResponse& self){
			return format("<source=%s, timestamp=%ld, status=%s>", self.source().c_str(), self.timestamp(), self.status().c_str());
		});

    using MotorControlRequestPublisher = fftai::dds::PublisherInstance<RobotSystem::MotorControlRequestPubSubType>;
    py::class_<MotorControlRequestPublisher, std::shared_ptr<MotorControlRequestPublisher>>(m, "MotorControlRequestPublisher")
		.def(py::init([](std::shared_ptr<fftai::dds::Context> context, std::string topic_name, bool wait_for_matched, int wait_timeout_ms){
            return fftai::dds::create_dds_publisher<RobotSystem::MotorControlRequestPubSubType>(context, topic_name, wait_for_matched, wait_timeout_ms);
        }))
        .def("publish", [](MotorControlRequestPublisher& self, const RobotSystem::MotorControlRequest& msg){ return self.publish_message(msg); })
		.def("__repr__", [](MotorControlRequestPublisher& self){
			return format("<MotorControlRequestPublisher this=%p>", &self);
		});
    
    using MotorControlResponseSubscriber = SubscriberMessageCollection<RobotSystem::MotorControlResponsePubSubType>;
    py::class_<MotorControlResponseSubscriber, std::shared_ptr<MotorControlResponseSubscriber>>(m, "MotorControlResponseSubscriber")
		.def(py::init([](std::shared_ptr<fftai::dds::Context> context, std::string topic_name, bool wait_for_matched, int wait_timeout_ms){
            return create_dds_subscriber_message_collection<RobotSystem::MotorControlResponsePubSubType>(context, topic_name, wait_for_matched, wait_timeout_ms);
        }))
        .def("current_message", [](MotorControlResponseSubscriber& self, std::string source){return self.get_current_message(source);})
        .def("have_new_message", [](MotorControlResponseSubscriber& self, std::string source){return self.have_new_message(source);})
        .def("message_latency_ns", [](MotorControlResponseSubscriber& self, std::string source){return self.message_latency_ns(source);})
		.def("__repr__", [](MotorControlResponseSubscriber& self){
			return format("<MotorControlResponseSubscriber this=%p>", &self);
		});

    //5. OperationMode
    py::class_<RobotSystem::OperationModeRequest>(m, "OperationModeRequest")
        .def(py::init([](std::string target, int mode_of_operation){
            RobotSystem::OperationModeRequest result;
            result.target(target);
            result.mode_of_operation(mode_of_operation);
            return result;
        }))
		.def_property_readonly("target", [](RobotSystem::OperationModeRequest& self){return self.target();})
		.def_property_readonly("mode_of_operation", [](RobotSystem::OperationModeRequest& self){return self.mode_of_operation();})
        .def("__repr__", [](RobotSystem::OperationModeRequest& self){
			return format("<target=%s, mode_of_operation=%d>", self.target().c_str(), self.mode_of_operation());
		});

    py::class_<RobotSystem::OperationModeResponse>(m, "OperationModeResponse")
        .def(py::init([](std::string source, int timestamp, std::string status){
            RobotSystem::OperationModeResponse result;
            result.source(source);
            result.timestamp(timestamp);
            result.status(status);
            return result;
        }))
		.def_property_readonly("source", [](RobotSystem::OperationModeResponse& self){return self.source();})
		.def_property_readonly("timestamp", [](RobotSystem::OperationModeResponse& self){return self.timestamp();})
		.def_property_readonly("status", [](RobotSystem::OperationModeResponse& self){return self.status();})
        .def("__repr__", [](RobotSystem::OperationModeResponse& self){
			return format("<source=%s, timestamp=%ld, status=%s>", self.source().c_str(), self.timestamp(), self.status().c_str());
		});

    using OperationModeRequestPublisher = fftai::dds::PublisherInstance<RobotSystem::OperationModeRequestPubSubType>;
    py::class_<OperationModeRequestPublisher, std::shared_ptr<OperationModeRequestPublisher>>(m, "OperationModeRequestPublisher")
		.def(py::init([](std::shared_ptr<fftai::dds::Context> context, std::string topic_name, bool wait_for_matched, int wait_timeout_ms){
            return fftai::dds::create_dds_publisher<RobotSystem::OperationModeRequestPubSubType>(context, topic_name, wait_for_matched, wait_timeout_ms);
        }))
        .def("publish", [](OperationModeRequestPublisher& self, const RobotSystem::OperationModeRequest& msg){ return self.publish_message(msg); })
		.def("__repr__", [](OperationModeRequestPublisher& self){
			return format("<OperationModeRequestPublisher this=%p>", &self);
		});

    using OperationModeResponseSubscriber = SubscriberMessageCollection<RobotSystem::OperationModeResponsePubSubType>;
    py::class_<OperationModeResponseSubscriber, std::shared_ptr<OperationModeResponseSubscriber>>(m, "OperationModeResponseSubscriber")
		.def(py::init([](std::shared_ptr<fftai::dds::Context> context, std::string topic_name, bool wait_for_matched, int wait_timeout_ms){
            return create_dds_subscriber_message_collection<RobotSystem::OperationModeResponsePubSubType>(context, topic_name, wait_for_matched, wait_timeout_ms);
        }))
        .def("current_message", [](OperationModeResponseSubscriber& self, std::string source){return self.get_current_message(source);})
        .def("have_new_message", [](OperationModeResponseSubscriber& self, std::string source){return self.have_new_message(source);})
        .def("message_latency_ns", [](OperationModeResponseSubscriber& self, std::string source){return self.message_latency_ns(source);})
		.def("__repr__", [](OperationModeResponseSubscriber& self){
			return format("<OperationModeResponseSubscriber this=%p>", &self);
		});

    //6. IMUState
    py::class_<RobotSystem::IMUStateRequest>(m, "IMUStateRequest")
        .def(py::init([](std::string target){
            RobotSystem::IMUStateRequest result;
            result.target(target);
            return result;
        }))
		.def_property_readonly("target", [](RobotSystem::IMUStateRequest& self){return self.target();});

    py::class_<RobotSystem::IMUStateResponse>(m, "IMUStateResponse")
        .def(py::init([](std::string source, int timestamp, std::string status, std::string frame_type, 
                         float temperature, float pressure, int system_time_ms, float sync_time, 
                         float roll, float pitch, float yaw, 
                         float acceleration_x, float acceleration_y, float acceleration_z, 
                         float gyroscope_x, float gyroscope_y, float gyroscope_z, 
                         float magnetometer_x, float magnetometer_y, float magnetometer_z, 
                         float quaternion_x, float quaternion_y, float quaternion_z, float quaternion_w){
            RobotSystem::IMUStateResponse result;
            result.source(source);
            result.timestamp(timestamp);
            result.status(status);
            result.frame_type(frame_type);
            result.temperature(temperature);
            result.pressure(pressure);
            result.system_time_ms(system_time_ms);
            result.sync_time(sync_time);
            result.roll(roll);
            result.pitch(pitch);
            result.yaw(yaw);
            result.acceleration_x(acceleration_x);
            result.acceleration_y(acceleration_y);
            result.acceleration_z(acceleration_z);
            result.gyroscope_x(gyroscope_x);
            result.gyroscope_y(gyroscope_y);
            result.gyroscope_z(gyroscope_z);
            result.magnetometer_x(magnetometer_x);
            result.magnetometer_y(magnetometer_y);
            result.magnetometer_z(magnetometer_z);
            result.quaternion_x(quaternion_x);
            result.quaternion_y(quaternion_y);
            result.quaternion_z(quaternion_z);
            result.quaternion_w(quaternion_w);
            return result;
        }))
		.def_property_readonly("source", [](RobotSystem::IMUStateResponse& self){return self.source();})
		.def_property_readonly("timestamp", [](RobotSystem::IMUStateResponse& self){return self.timestamp();})
		.def_property_readonly("status", [](RobotSystem::IMUStateResponse& self){return self.status();})
		.def_property_readonly("frame_type", [](RobotSystem::IMUStateResponse& self){return self.frame_type();})
		.def_property_readonly("temperature", [](RobotSystem::IMUStateResponse& self){return self.temperature();})
		.def_property_readonly("pressure", [](RobotSystem::IMUStateResponse& self){return self.pressure();})
		.def_property_readonly("system_time_ms", [](RobotSystem::IMUStateResponse& self){return self.system_time_ms();})
		.def_property_readonly("sync_time", [](RobotSystem::IMUStateResponse& self){return self.sync_time();})
		.def_property_readonly("roll", [](RobotSystem::IMUStateResponse& self){return self.roll();})
		.def_property_readonly("pitch", [](RobotSystem::IMUStateResponse& self){return self.pitch();})
		.def_property_readonly("yaw", [](RobotSystem::IMUStateResponse& self){return self.yaw();})
		.def_property_readonly("acceleration_x", [](RobotSystem::IMUStateResponse& self){return self.acceleration_x();})
		.def_property_readonly("acceleration_y", [](RobotSystem::IMUStateResponse& self){return self.acceleration_y();})
		.def_property_readonly("acceleration_z", [](RobotSystem::IMUStateResponse& self){return self.acceleration_z();})
		.def_property_readonly("gyroscope_x", [](RobotSystem::IMUStateResponse& self){return self.gyroscope_x();})
		.def_property_readonly("gyroscope_y", [](RobotSystem::IMUStateResponse& self){return self.gyroscope_y();})
		.def_property_readonly("gyroscope_z", [](RobotSystem::IMUStateResponse& self){return self.gyroscope_z();})
		.def_property_readonly("magnetometer_x", [](RobotSystem::IMUStateResponse& self){return self.magnetometer_x();})
		.def_property_readonly("magnetometer_y", [](RobotSystem::IMUStateResponse& self){return self.magnetometer_y();})
		.def_property_readonly("magnetometer_z", [](RobotSystem::IMUStateResponse& self){return self.magnetometer_z();})
		.def_property_readonly("quaternion_x", [](RobotSystem::IMUStateResponse& self){return self.quaternion_x();})
		.def_property_readonly("quaternion_y", [](RobotSystem::IMUStateResponse& self){return self.quaternion_y();})
		.def_property_readonly("quaternion_z", [](RobotSystem::IMUStateResponse& self){return self.quaternion_z();})
		.def_property_readonly("quaternion_w", [](RobotSystem::IMUStateResponse& self){return self.quaternion_w();})
        .def("__repr__", [](RobotSystem::IMUStateResponse& self){
			return format("<source=%s, timestamp=%ld, status=%s, frame_type=%s, "
                          "temperature=%f, pressure=%f, system_time_ms=%d, sync_time=%f, "
                          "roll=%f, pitch=%f, yaw=%f, "
                          "acceleration_x=%f, acceleration_y=%f, acceleration_z=%f, "
                          "gyroscope_x=%f, gyroscope_y=%f, gyroscope_z=%f, "
                          "magnetometer_x=%f, magnetometer_y=%f, magnetometer_z=%f, "
                          "quaternion_x=%f, quaternion_y=%f, quaternion_z=%f, quaternion_w=%f>", 
                          self.source().c_str(), self.timestamp(), self.status().c_str(), self.frame_type().c_str(), 
                          self.temperature(), self.pressure(), self.system_time_ms(), self.sync_time(), 
                          self.roll(), self.pitch(), self.yaw(), 
                          self.acceleration_x(), self.acceleration_y(), self.acceleration_z(), 
                          self.gyroscope_x(), self.gyroscope_y(), self.gyroscope_z(), 
                          self.magnetometer_x(), self.magnetometer_y(), self.magnetometer_z(), 
                          self.quaternion_x(), self.quaternion_y(), self.quaternion_z(), self.quaternion_w());
		});

    using IMUStateRequestPublisher = fftai::dds::PublisherInstance<RobotSystem::IMUStateRequestPubSubType>;
    py::class_<IMUStateRequestPublisher, std::shared_ptr<IMUStateRequestPublisher>>(m, "IMUStateRequestPublisher")
		.def(py::init([](std::shared_ptr<fftai::dds::Context> context, std::string topic_name, bool wait_for_matched, int wait_timeout_ms){
            return fftai::dds::create_dds_publisher<RobotSystem::IMUStateRequestPubSubType>(context, topic_name, wait_for_matched, wait_timeout_ms);
        }))
        .def("publish", [](IMUStateRequestPublisher& self, const RobotSystem::IMUStateRequest& msg){ return self.publish_message(msg); })
		.def("__repr__", [](IMUStateRequestPublisher& self){
			return format("<IMUStateRequestPublisher this=%p>", &self);
		});
    
    using IMUStateResponseSubscriber = SubscriberMessageCollection<RobotSystem::IMUStateResponsePubSubType>;
    py::class_<IMUStateResponseSubscriber, std::shared_ptr<IMUStateResponseSubscriber>>(m, "IMUStateResponseSubscriber")
		.def(py::init([](std::shared_ptr<fftai::dds::Context> context, std::string topic_name, bool wait_for_matched, int wait_timeout_ms){
            return create_dds_subscriber_message_collection<RobotSystem::IMUStateResponsePubSubType>(context, topic_name, wait_for_matched, wait_timeout_ms);
        }))
        .def("current_message", [](IMUStateResponseSubscriber& self, std::string source){return self.get_current_message(source);})
        .def("have_new_message", [](IMUStateResponseSubscriber& self, std::string source){return self.have_new_message(source);})
        .def("message_latency_ns", [](IMUStateResponseSubscriber& self, std::string source){return self.message_latency_ns(source);})
		.def("__repr__", [](IMUStateResponseSubscriber& self){
			return format("<IMUStateResponseSubscriber this=%p>", &self);
		});

    //7. EncoderState
    py::class_<RobotSystem::EncoderStateRequest>(m, "EncoderStateRequest")
        .def(py::init([](std::string target){
            RobotSystem::EncoderStateRequest result;
            result.target(target);
            return result;
        }))
		.def_property_readonly("target", [](RobotSystem::EncoderStateRequest& self){return self.target();});

    py::class_<RobotSystem::EncoderStateResponse>(m, "EncoderStateResponse")
        .def(py::init([](std::string source, int timestamp, std::string status, float angle, float radian){
            RobotSystem::EncoderStateResponse result;
            result.source(source);
            result.timestamp(timestamp);
            result.status(status);
            result.angle(angle);
            result.radian(radian);
            return result;
        }))
		.def_property_readonly("source", [](RobotSystem::EncoderStateResponse& self){return self.source();})
		.def_property_readonly("timestamp", [](RobotSystem::EncoderStateResponse& self){return self.timestamp();})
		.def_property_readonly("status", [](RobotSystem::EncoderStateResponse& self){return self.status();})
		.def_property_readonly("angle", [](RobotSystem::EncoderStateResponse& self){return self.angle();})
		.def_property_readonly("radian", [](RobotSystem::EncoderStateResponse& self){return self.radian();})
        .def("__repr__", [](RobotSystem::EncoderStateResponse& self){
			return format("<source=%s, timestamp=%ld, status=%s, angle=%f, radian=%f>", 
                          self.source().c_str(), self.timestamp(), self.status().c_str(), self.angle(), self.radian());
		});

    using EncoderStateRequestPublisher = fftai::dds::PublisherInstance<RobotSystem::EncoderStateRequestPubSubType>;
    py::class_<EncoderStateRequestPublisher, std::shared_ptr<EncoderStateRequestPublisher>>(m, "EncoderStateRequestPublisher")
		.def(py::init([](std::shared_ptr<fftai::dds::Context> context, std::string topic_name, bool wait_for_matched, int wait_timeout_ms){
            return fftai::dds::create_dds_publisher<RobotSystem::EncoderStateRequestPubSubType>(context, topic_name, wait_for_matched, wait_timeout_ms);
        }))
        .def("publish", [](EncoderStateRequestPublisher& self, const RobotSystem::EncoderStateRequest& msg){ return self.publish_message(msg); })
		.def("__repr__", [](EncoderStateRequestPublisher& self){
			return format("<EncoderStateRequestPublisher this=%p>", &self);
		});
    
    using EncoderStateResponseSubscriber = SubscriberMessageCollection<RobotSystem::EncoderStateResponsePubSubType>;
    py::class_<EncoderStateResponseSubscriber, std::shared_ptr<EncoderStateResponseSubscriber>>(m, "EncoderStateResponseSubscriber")
		.def(py::init([](std::shared_ptr<fftai::dds::Context> context, std::string topic_name, bool wait_for_matched, int wait_timeout_ms){
            return create_dds_subscriber_message_collection<RobotSystem::EncoderStateResponsePubSubType>(context, topic_name, wait_for_matched, wait_timeout_ms);
        }))
        .def("current_message", [](EncoderStateResponseSubscriber& self, std::string source){return self.get_current_message(source);})
        .def("have_new_message", [](EncoderStateResponseSubscriber& self, std::string source){return self.have_new_message(source);})
        .def("message_latency_ns", [](EncoderStateResponseSubscriber& self, std::string source){return self.message_latency_ns(source);})
		.def("__repr__", [](EncoderStateResponseSubscriber& self){
			return format("<EncoderStateResponseSubscriber this=%p>", &self);
		});

    //8. PIDIMMSet
    py::class_<RobotSystem::PIDIMMSetRequest>(m, "PIDIMMSetRequest")
        .def(py::init([](std::string target, float control_position_kp_imm, float control_velocity_kp_imm, float control_velocity_ki_imm){
            RobotSystem::PIDIMMSetRequest result;
            result.target(target);
            result.control_position_kp_imm(control_position_kp_imm);
            result.control_velocity_kp_imm(control_velocity_kp_imm);
            result.control_velocity_ki_imm(control_velocity_ki_imm);
            return result;
        }))
		.def_property_readonly("target", [](RobotSystem::PIDIMMSetRequest& self){return self.target();})
		.def_property_readonly("control_position_kp_imm", [](RobotSystem::PIDIMMSetRequest& self){return self.control_position_kp_imm();})
		.def_property_readonly("control_velocity_kp_imm", [](RobotSystem::PIDIMMSetRequest& self){return self.control_velocity_kp_imm();})
		.def_property_readonly("control_velocity_ki_imm", [](RobotSystem::PIDIMMSetRequest& self){return self.control_velocity_ki_imm();});

    py::class_<RobotSystem::PIDIMMSetResponse>(m, "PIDIMMSetResponse")
        .def(py::init([](std::string source, int timestamp, std::string status){
            RobotSystem::PIDIMMSetResponse result;
            result.source(source);
            result.timestamp(timestamp);
            result.status(status);
            return result;
        }))
		.def_property_readonly("source", [](RobotSystem::PIDIMMSetResponse& self){return self.source();})
		.def_property_readonly("timestamp", [](RobotSystem::PIDIMMSetResponse& self){return self.timestamp();})
		.def_property_readonly("status", [](RobotSystem::PIDIMMSetResponse& self){return self.status();})
        .def("__repr__", [](RobotSystem::PIDIMMSetResponse& self){
			return format("<source=%s, timestamp=%ld, status=%s",
                          self.source().c_str(), self.timestamp(), self.status().c_str());
		});

    using PIDIMMSetRequestPublisher = fftai::dds::PublisherInstance<RobotSystem::PIDIMMSetRequestPubSubType>;
    py::class_<PIDIMMSetRequestPublisher, std::shared_ptr<PIDIMMSetRequestPublisher>>(m, "PIDIMMSetRequestPublisher")
		.def(py::init([](std::shared_ptr<fftai::dds::Context> context, std::string topic_name, bool wait_for_matched, int wait_timeout_ms){
            return fftai::dds::create_dds_publisher<RobotSystem::PIDIMMSetRequestPubSubType>(context, topic_name, wait_for_matched, wait_timeout_ms);
        }))
        .def("publish", [](PIDIMMSetRequestPublisher& self, const RobotSystem::PIDIMMSetRequest& msg){ return self.publish_message(msg); })
		.def("__repr__", [](PIDIMMSetRequestPublisher& self){
			return format("<PIDIMMSetRequestPublisher this=%p>", &self);
		});
    
    using PIDIMMSetResponseSubscriber = SubscriberMessageCollection<RobotSystem::PIDIMMSetResponsePubSubType>;
    py::class_<PIDIMMSetResponseSubscriber, std::shared_ptr<PIDIMMSetResponseSubscriber>>(m, "PIDIMMSetResponseSubscriber")
		.def(py::init([](std::shared_ptr<fftai::dds::Context> context, std::string topic_name, bool wait_for_matched, int wait_timeout_ms){
            return create_dds_subscriber_message_collection<RobotSystem::PIDIMMSetResponsePubSubType>(context, topic_name, wait_for_matched, wait_timeout_ms);
        }))
        .def("current_message", [](PIDIMMSetResponseSubscriber& self, std::string source){return self.get_current_message(source);})
        .def("have_new_message", [](PIDIMMSetResponseSubscriber& self, std::string source){return self.have_new_message(source);})
        .def("message_latency_ns", [](PIDIMMSetResponseSubscriber& self, std::string source){return self.message_latency_ns(source);})
		.def("__repr__", [](PIDIMMSetResponseSubscriber& self){
			return format("<PIDIMMSetResponseSubscriber this=%p>", &self);
		});
    
    //9. PIDIMMGet
    py::class_<RobotSystem::PIDIMMGetRequest>(m, "PIDIMMGetRequest")
        .def(py::init([](std::string target){
            RobotSystem::PIDIMMGetRequest result;
            result.target(target);
            return result;
        }))
		.def_property_readonly("target", [](RobotSystem::PIDIMMGetRequest& self){return self.target();});

    py::class_<RobotSystem::PIDIMMGetResponse>(m, "PIDIMMGetResponse")
        .def(py::init([](std::string source, int timestamp, std::string status, 
                         float control_position_kp_imm, float control_velocity_kp_imm, float control_velocity_ki_imm,
                         float control_current_kp_imm, float control_current_ki_imm){
            RobotSystem::PIDIMMGetResponse result;
            result.source(source);
            result.timestamp(timestamp);
            result.status(status);
            result.control_position_kp_imm(control_position_kp_imm);
            result.control_velocity_kp_imm(control_velocity_kp_imm);
            result.control_velocity_ki_imm(control_velocity_ki_imm);
            result.control_current_kp_imm(control_current_kp_imm);
            result.control_current_ki_imm(control_current_ki_imm);
            return result;
        }))
		.def_property_readonly("source", [](RobotSystem::PIDIMMGetResponse& self){return self.source();})
		.def_property_readonly("timestamp", [](RobotSystem::PIDIMMGetResponse& self){return self.timestamp();})
		.def_property_readonly("status", [](RobotSystem::PIDIMMGetResponse& self){return self.status();})
		.def_property_readonly("control_position_kp_imm", [](RobotSystem::PIDIMMGetResponse& self){return self.control_position_kp_imm();})
		.def_property_readonly("control_velocity_kp_imm", [](RobotSystem::PIDIMMGetResponse& self){return self.control_velocity_kp_imm();})
		.def_property_readonly("control_velocity_ki_imm", [](RobotSystem::PIDIMMGetResponse& self){return self.control_velocity_ki_imm();})
		.def_property_readonly("control_current_kp_imm", [](RobotSystem::PIDIMMGetResponse& self){return self.control_current_kp_imm();})
		.def_property_readonly("control_current_ki_imm", [](RobotSystem::PIDIMMGetResponse& self){return self.control_current_ki_imm();})
        .def("__repr__", [](RobotSystem::PIDIMMGetResponse& self){
			return format("<source=%s, timestamp=%ld, control_position_kp_imm=%f, control_velocity_kp_imm=%f, control_velocity_ki_imm=%f, control_current_kp_imm=%f, control_current_ki_imm:%f>", 
                          self.source().c_str(), self.timestamp(), self.status().c_str(), 
                          self.control_position_kp_imm(), self.control_velocity_kp_imm(), self.control_velocity_ki_imm(), 
                          self.control_current_kp_imm(), self.control_current_ki_imm());
		});

    using PIDIMMGetRequestPublisher = fftai::dds::PublisherInstance<RobotSystem::PIDIMMGetRequestPubSubType>;
    py::class_<PIDIMMGetRequestPublisher, std::shared_ptr<PIDIMMGetRequestPublisher>>(m, "PIDIMMGetRequestPublisher")
		.def(py::init([](std::shared_ptr<fftai::dds::Context> context, std::string topic_name, bool wait_for_matched, int wait_timeout_ms){
            return fftai::dds::create_dds_publisher<RobotSystem::PIDIMMGetRequestPubSubType>(context, topic_name, wait_for_matched, wait_timeout_ms);
        }))
        .def("publish", [](PIDIMMGetRequestPublisher& self, const RobotSystem::PIDIMMGetRequest& msg){ return self.publish_message(msg); })
		.def("__repr__", [](PIDIMMGetRequestPublisher& self){
			return format("<PIDIMMGetRequestPublisher this=%p>", &self);
		});
    
    using PIDIMMGetResponseSubscriber = SubscriberMessageCollection<RobotSystem::PIDIMMGetResponsePubSubType>;
    py::class_<PIDIMMGetResponseSubscriber, std::shared_ptr<PIDIMMGetResponseSubscriber>>(m, "PIDIMMGetResponseSubscriber")
		.def(py::init([](std::shared_ptr<fftai::dds::Context> context, std::string topic_name, bool wait_for_matched, int wait_timeout_ms){
            return create_dds_subscriber_message_collection<RobotSystem::PIDIMMGetResponsePubSubType>(context, topic_name, wait_for_matched, wait_timeout_ms);
        }))
        .def("current_message", [](PIDIMMGetResponseSubscriber& self, std::string source){return self.get_current_message(source);})
        .def("have_new_message", [](PIDIMMGetResponseSubscriber& self, std::string source){return self.have_new_message(source);})
        .def("message_latency_ns", [](PIDIMMGetResponseSubscriber& self, std::string source){return self.message_latency_ns(source);})
		.def("__repr__", [](PIDIMMGetResponseSubscriber& self){
			return format("<PIDIMMGetResponseSubscriber this=%p>", &self);
		});
    
}