#pragma once

#include <memory>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <vector>
#include <string>

#include "cyber/time/time.h"

namespace apollo {
namespace bridge {

template<typename T>
class ProtobufManager{
public:
    ProtobufManager() =default;
    std::shared_ptr<T> CreateObj(double timeStamp);

    bool DestructObj(double timeStamp);

    int64_t getTimeStamp(double timeStamp){return timeMap_[timeStamp];}

private:
    std::unordered_map<double, std::shared_ptr<T>> protoMap_;
    std::unordered_map<double, int64_t> timeMap_;
    int64_t timestamp_;
    std::mutex mtx_;

};

template<typename T>
std::shared_ptr<T> ProtobufManager<T>::CreateObj(double timeStamp){
    std::lock_guard<std::mutex> lock(mtx_);
    if(protoMap_.find(timeStamp) == protoMap_.end()){
        timeMap_[timeStamp] = cyber::Time::Now().ToMicrosecond();
        protoMap_[timeStamp] = std::make_shared<T>();
        return protoMap_[timeStamp];
    }else{
        return protoMap_[timeStamp];
    }
}

template<typename T>
bool ProtobufManager<T>::DestructObj(double timeStamp){
    return protoMap_.erase(timeStamp) == 1;
}


}// namespace bridge
}// namespace apollo