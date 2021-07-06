#pragma once

#include <condition_variable>
#include <memory>
#include <mutex>
#include <set>
#include <thread>
#include <unordered_map>
#include <vector>
#include <atomic>

#include "cyber/common/log.h"
#include "cyber/time/time.h"
#include "modules/common/proto/error_code.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace bridge {

template<typename T>
class ProtobufManager{
public:
    ProtobufManager() =default;
    std::unique_ptr<T>> CreateObj(double timeStamp);

    bool DestructObj(double timeStamp);

    int64_t getTimeStamp(double timeStamp)const {return timeMap_[timeStamp];}

private:
    std::unordered_map<double, std::unique_ptr<T>> protoMap_;
    std::unordered_map<double, int64_t> timeMap_;
    int64_t timestamp_;
    std::mutex mtx_;

}

template<typename T>
std::unique_ptr<T>> ProtobufManager<T>::CreateObj(double timeStamp){
    std::lock_guard<std::mutex> lock(mtx_);
    if(protoMap_.find(timeStamp) == protoMap_.end()){
        timeMap_[timeStamp] = cyber::Time::Now().ToMicrosecond();
        std::unique_ptr<T> protoPtr(new T);
        protoMap_[timeStamp] = protoPtr;
        return protoPtr.get();
    }else{
        return protoMap_[timeStamp].get();
    }
}

template<typename T>
bool ProtobufManager<T>::DestructObj(double timeStamp){
    return protoMap_.erase(timeStamp) == 1;
}


}// namespace bridge
}// namespace apollo