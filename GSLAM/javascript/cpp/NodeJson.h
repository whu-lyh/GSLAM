#ifndef GSLAM_NODEJSON_H
#define GSLAM_NODEJSON_H

#include <nbind/TypeID.h>
#include <nbind/v8/BindingStd.h>
#include <nbind/TypeTransformer.h>
#include <v8.h>
#include <GSLAM/core/JSON.h>
#include <GSLAM/core/GSLAM.h>

namespace nbind{


template <typename KeyType, typename ArgType>
struct BindingType<std::map<KeyType,ArgType>> {
    typedef v8::Handle<v8::Value> WireType;
    typedef std::map<KeyType,ArgType> Type;

    static inline bool checkType(WireType arg) {
        return(arg->IsObject());
    }

    static inline Type fromWireType(WireType arg) {

        Type result;

        v8::Local<v8::Object> obj = arg.template As<v8::Object>();
        v8::Local<v8::Array>  arr = obj->GetOwnPropertyNames();
        uint32_t count = arr->Length();

        for(uint32_t num = 0; num < count; ++num) {
            v8::Local<v8::Value> item;

            if(
                Nan::Get(arr, num).ToLocal(&item) &&
                BindingType<KeyType>::checkType(item)
            ) {

                auto value=obj->Get(item);
                if(!BindingType<ArgType>::checkType(value))
                    throw(std::runtime_error("Error converting object element"));

                result.insert(std::make_pair(convertFromWire<KeyType>(item),
                                             convertFromWire<ArgType>(value)));

            } else {
                throw(std::runtime_error("Error converting object element"));
            }
        }

        return result;
    }

    static inline WireType toWireType(Type &&arg) {
        v8::Local<v8::Object> arr = Nan::New<v8::Object>();

        for(std::pair<KeyType,ArgType> it:arg) {
            arr->Set(convertToWire(it.first),convertToWire(it.second));
        }

        return(arr);
    }

};


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
            Type ret=fromWire(arg);
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
