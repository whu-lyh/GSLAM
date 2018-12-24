#ifndef GSLAM_NODEJSON_H
#define GSLAM_NODEJSON_H

#include <nbind/TypeTransformer.h>
#include <v8.h>
#include <GSLAM/core/JSON.h>

namespace nbind{

template <>
struct BindingType<GSLAM::Json> {
    typedef v8::Handle<v8::Value> WireType;

        typedef GSLAM::Json Type;

        static inline bool checkType(WireType arg) {
            return true;
        }

        static inline Type fromWire(WireType arg){
            if(arg->IsNumber())  return GSLAM::Json(convertFromWire<double>(arg));
            if(arg->IsBoolean()) return GSLAM::Json(convertFromWire<bool>(arg));
            if(arg->IsString())  return GSLAM::Json(convertFromWire<std::string>(arg));
            if(arg->IsArray())   return GSLAM::Json(convertFromWire<GSLAM::Json::array>(arg));
            if(arg->IsObject())  return GSLAM::Json(convertFromWire<GSLAM::Json::object>(arg));
            return GSLAM::Json();
        }
        static inline Type fromWireType(WireType arg) {
            std::cerr<<"tried JSON fromWireType ";

            Type ret=fromWire(arg);
            std::cerr<<"Obtained "<<ret.dump();
            return ret;
        }

        static inline WireType toWireType(Type &&arg)
        {
            switch (arg.type()) {
            case GSLAM::Json::NUL:
                return convertToWire(nullptr);
                break;
            case GSLAM::Json::NUMBER:
                return convertToWire(arg.number_value());
            case GSLAM::Json::BOOL:
                return convertToWire(arg.bool_value());
            case GSLAM::Json::STRING:
                return convertToWire(arg.string_value());
            case GSLAM::Json::ARRAY:
                return convertToWire(arg.array_items());
            case GSLAM::Json::OBJECT:
                return convertToWire(arg.object_items());
            default:
                return convertToWire(nullptr);
                break;
            }
            return convertToWire(nullptr);
        }
};

}

#endif // NODEJSON_H
