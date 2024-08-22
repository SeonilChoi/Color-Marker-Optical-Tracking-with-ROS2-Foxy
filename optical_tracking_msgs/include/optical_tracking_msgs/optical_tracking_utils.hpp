#ifndef OPTICAL_TRACKING_UTILS_HPP_
#define OPTICAL_TRACKING_UTILS_HPP_

#include <vector>

namespace optical_tracking_msgs
{

template<typename T1, typename T2>
void convert_vector_to_msg_2d(const std::vector<T1> vec, std::vector<T2> & data)
{
    T2 msg;
    data.reserve(vec.size());
    
    for (const auto & pt : vec){
        msg.x = pt(0);
        msg.y = pt(1);

        data.push_back(msg);
    }
}

template<typename T1, typename T2>
void convert_vector_to_msg_3d(const std::vector<T1> vec, std::vector<T2> & data)
{
    T2 msg;
    data.reserve(vec.size());

    for (const auto & pt : vec){
        msg.x = pt(0);
        msg.y = pt(1);
        msg.z = pt(2);

        data.push_back(msg);
    }
}

template<typename T1, typename T2>
void convert_msg_to_vector_2d(const std::vector<T1> data, std::vector<T2> & vec)
{
    T2 pt;
    vec.reserve(data.size());

    for (const auto & msg : data){
        pt(0) = msg.x;
        pt(1) = msg.y;

        vec.push_back(pt);
    }
}

template<typename T1, typename T2>
void convert_msg_to_vector_3d(const std::vector<T1> data, std::vector<T2> & vec)
{
    T2 pt;
    vec.reserve(data.size());

    for (const auto & msg : data){
        pt(0) = msg.x;
        pt(1) = msg.y;
        pt(2) = msg.z;

        vec.push_back(pt);
    }
}

} //namespace
#endif
