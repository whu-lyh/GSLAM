#ifndef LAMDAMETHODSIGNATURE_H
#define LAMDAMETHODSIGNATURE_H
#include <nbind/signature/BaseSignature.h>

// This file is part of nbind, copyright (C) 2014-2016 BusFaster Ltd.
// Released under the MIT license, see LICENSE.

// This file is very similar to FunctionSignature.h and AccessorSignature.h
// so modify them together.

namespace nbind {
template<typename ReturnType, typename ArgList> struct LambdaCaller;

template<typename ReturnType, typename... Args>
struct LambdaCaller<ReturnType, TypeList<Args...>> {

        template <class Bound, typename MethodType, typename NanArgs>
        static WireType callMethod(Bound &target, MethodType func, NanArgs &args) noexcept(false) {
                (void)args; // Silence possible compiler warning about unused parameter.

                // Note that Args().get may throw.
                return(MethodResultConverter<ReturnType>::toWireType(
                        func(target,Args(args).get(args)...),
                        target,
                        0.0
                ));
        }

        template <typename Function, typename NanArgs>
        static WireType callFunction(Function func, NanArgs &args) noexcept(false) {
                (void)args; // Silence possible compiler warning about unused parameter.

                // Note that Args().get may throw.
                return(convertToWire<ReturnType>(
                        (*func)(Args(args).get(args)...)
                ));
        }

};

// Specialize Caller for void return type, because toWireType needs a non-void
// argument.

template<typename... Args>
struct LambdaCaller<void, TypeList<Args...>> {

        template <class Bound, typename MethodType, typename NanArgs>
        static WireType callMethod(Bound &target, MethodType func, NanArgs &args) noexcept(false) {
                (void)args; // Silence possible compiler warning about unused parameter.

                // Note that Args().get may throw.
                func(target,Args(args).get(args)...);

                return(Nan::Undefined());
        }

        template <typename Function, typename NanArgs>
        static WireType callFunction(Function func, NanArgs &args) noexcept(false) {
                (void)args; // Silence possible compiler warning about unused parameter.

                // Note that Args().get may throw.
                (*func)(Args(args).get(args)...);

                return(Nan::Undefined());
        }

};

// Wrapper for all C++ methods with matching class, argument and return types.

template <typename PtrType, class Bound, typename PolicyList, typename ReturnType, typename... Args>
class LambdaMethodSignature : public TemplatedBaseSignature<LambdaMethodSignature<PtrType, Bound, PolicyList, ReturnType, Args...>, PolicyList, ReturnType, Args...> {

public:

    typedef PtrType MethodType;

    typedef TemplatedBaseSignature<LambdaMethodSignature, PolicyList, ReturnType, Args...> Parent;

    static constexpr auto typeExpr = BaseSignature :: SignatureType :: method;

    typedef LambdaCaller<
            ReturnType,
            typename emscripten::internal::MapWithIndex<
                    PolicyList,
                    TypeList,
                    ArgFromWire,
                    Args...
            >::type
    > CallWrapper;

#if defined(BUILDING_NODE_EXTENSION)

    template <typename V8Args, typename NanArgs>
    static void callInner(const typename Parent::MethodInfo &method, V8Args &args, NanArgs &nanArgs, Bound *target) {
        nanArgs.GetReturnValue().Set(CallWrapper::callMethod(
                *target,
                method.func,
                args
        ));
    }

    static void call(const Nan::FunctionCallbackInfo<v8::Value> &args) {
        Parent::template callInnerSafely<Bound>(
            args,
            args,
            SignatureParam::get(args)->methodNum
        );
    }

#elif defined(__EMSCRIPTEN__)

    static typename TypeTransformer<ReturnType, PolicyList>::Binding::WireType call(
        uint32_t num,
        Bound *target,
        typename TypeTransformer<Args, PolicyList>::Binding::WireType... args
    ) {
        auto method = Parent::getMethod(num).func;

        return(Caller<PolicyList, ReturnType, Args...>::callMethod(*target, method, args...));
    }

#endif // BUILDING_NODE_EXTENSION, __EMSCRIPTEN__

};

} // namespace

#endif // LAMDAMETHODSIGNATURE_H
