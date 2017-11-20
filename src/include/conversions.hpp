/***************************************************************************
* Copyright (c) 2016, Wolf Vollprecht, Johan Mabille and Sylvain Corlay    *
*                                                                          *
* Distributed under the terms of the BSD 3-Clause License.                 *
*                                                                          *
* The full license is in the file LICENSE, distributed with this software. *
****************************************************************************/

#ifndef XTENSOR_ROS_CONVERSIONS
#define XTENSOR_ROS_CONVERSIONS

#include <ros/message_traits.h>
#include <ros/serialization.h>

#include <xtensor/xarray.hpp>
#include <xtensor/xtensor.hpp>
#include <xtensor/xadapt.hpp>
#include <xtensor/xutils.hpp>

// Including the messages
#include "xtensor_ros/f64.h"
#include "xtensor_ros/u8.h"


xtensor_ros::f64 as_msg(xt::xarray<double>& arr)
{
    using msg_type = xtensor_ros::f64;
    using _data_type = typename msg_type::_data_type;

    xtensor_ros::f64 msg;

    msg.strides = arr.strides();
    msg.shape = arr.shape();
    msg.data = _data_type(arr.data().begin(), arr.data().end());
    return msg;
}

auto from_msg(const xtensor_ros::f64& arr)
{
    using msg_type = xtensor_ros::f64;
    using _data_type = typename msg_type::_data_type;

    xtensor_ros::f64 msg;

    return xt::xadapt(arr.data, arr.shape, arr.strides);
}


namespace xmsg
{
    using namespace xtensor_ros;
}

template <class T>
struct get_xmsg_type;

template <>
struct get_xmsg_type<double>
{
    using type = xmsg::f64;
};

template <>
struct get_xmsg_type<uint8_t>
{
    using type = xmsg::u8;
};

namespace ros
{
    namespace message_traits
    {

    template <class T>
    struct MD5Sum<xt::xarray<T>>
    {
        using msg_type = typename get_xmsg_type<T>::type;
        using msg_md5 = MD5Sum<msg_type>;

        static const char* value()
        {
            return msg_md5::value();
        }

        static const char* value(const xt::xarray<T>& ) { return value(); }
        static const uint64_t static_value1 = msg_md5::static_value1;
        static const uint64_t static_value2 = msg_md5::static_value2;
    };

    template <class T>
    struct DataType<xt::xarray<T>>
    {
        static const char* value()
        {
            return DataType<xtensor_ros::f64>::value();
        }

        static const char* value(const xt::xarray<T>& m)
        {
            return DataType<xtensor_ros::f64>::value();
        }
    };

    template <class T>
    struct Definition<xt::xarray<T>>
    {
        static const char* value()
        {
            return Definition<xtensor_ros::f64>::value();
        }

        static const char* value(const xt::xarray<T>& m)
        {
            return Definition<xtensor_ros::f64>::value();
        }
    };

    template <class T>
    struct IsFixedSize<xt::xarray<T>> : public FalseType {};

    template <class T>
    struct IsSimple<xt::xarray<T>> : public FalseType {};
}

namespace serialization
{
    template<typename T, class Enabled = void>
    struct UVectorSerializer
    {};

    template<typename T>
    struct UVectorSerializer<xt::uvector<T>,
                             std::enable_if_t<xt::xtrivially_default_constructible<T>::value>>
    {
        typedef xt::uvector<T> vec_type;
        typedef typename vec_type::iterator iterator;
        typedef typename vec_type::const_iterator const_iterator;

        template<typename Stream>
        inline static void write(Stream& stream, const vec_type& v)
        {
            uint32_t len = (uint32_t)v.size();
            stream.next(len);
            if (!v.empty())
            {
                const uint32_t data_len = len * sizeof(T);
                memcpy(stream.advance(data_len), &v.front(), data_len);
            }
        }

        template<typename Stream>
        inline static void read(Stream& stream, vec_type& v)
        {
            uint32_t len;
            stream.next(len);
            v.resize(len);

            if (len > 0)
            {
                const uint32_t data_len = sizeof(T) * len;
                memcpy(&v.front(), stream.advance(data_len), data_len);
            }
        }

        inline static uint32_t serializedLength(const vec_type& v)
        {
            return 4 + v.size() * sizeof(T);
        }
    };

    template<class T>
    struct Serializer<xt::xarray<T>>
    {
        using xtype = xt::xarray<T>;
        using shape_type = typename xtype::shape_type;
        using shape_serializer_type = VectorSerializer<typename shape_type::value_type,
        typename shape_type::allocator_type>;

        using data_serializer_type = UVectorSerializer<typename xtype::container_type>;

        template<typename Stream>
        inline static void write(Stream& stream, const xtype& t)
        {
            shape_serializer_type::write(stream, t.shape());
            shape_serializer_type::write(stream, t.strides());
            data_serializer_type::write(stream, t.data());
        }

        template<typename Stream>
        inline static void read(Stream& stream, xtype& t)
        {
            std::vector<std::size_t> shape, strides;
            shape_serializer_type::read(stream, shape);
            shape_serializer_type::read(stream, strides);
            t.reshape(std::move(shape));
            data_serializer_type::read(stream, t.data());
        }

        inline static uint32_t serializedLength(const xtype& t)
        {
            uint32_t size = 0;
            size += shape_serializer_type::serializedLength(t.shape()) * 2;
            size += data_serializer_type::serializedLength(t.data());
            return size;
        }
    };

}

}

#endif