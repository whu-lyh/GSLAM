#pragma once
#include <type_traits>
#include <list>
#include <sstream>

#undef timer
#include "nbind/api.h"
#define NBIND_MULTIMETHOD(name, args, ...) definer.overloaded args ().method(name, ## __VA_ARGS__)

#include "nbind/noconflict.h"
#include "nbind/v8/BindWrapper.h"
#include "LambdaMethodSignature.h"

#define NBIND_MODULE class JSRegister{\
public:\
    JSRegister();\
}instance;\
JSRegister::JSRegister()

namespace nbind {
template <bool B, typename T = void> using enable_if_t = typename std::enable_if<B, T>::type;
template <bool B, typename T, typename F> using conditional_t = typename std::conditional<B, T, F>::type;
template <typename T> using remove_cv_t = typename std::remove_cv<T>::type;
template <typename T> using remove_reference_t = typename std::remove_reference<T>::type;


/// Strip the class from a method type
template <typename T> struct remove_class { };
template <typename C, typename R, typename... A> struct remove_class<R (C::*)(A...)> { typedef R type(A...); };
template <typename C, typename R, typename... A> struct remove_class<R (C::*)(A...) const> { typedef R type(A...); };

template <typename F> struct strip_function_object {
    using type = typename remove_class<decltype(&F::operator())>::type;
};
// Extracts the function signature from a function, function pointer or lambda.
template <typename Function, typename F = remove_reference_t<Function>>
using function_signature_t = conditional_t<
    std::is_function<F>::value,
    F,
    typename conditional_t<
        std::is_pointer<F>::value || std::is_member_pointer<F>::value,
        std::remove_pointer<F>,
        strip_function_object<F>
    >::type
>;


template <>
struct BindingType<WireType> {

        typedef WireType Type;

        static inline bool checkType(WireType arg) {
                return true;
        }

        static inline Type fromWireType(WireType arg) {
                return arg;
        }

        static inline WireType toWireType(Type &&arg)
        {
            return arg;
        }

};

template <class Bound>
class class_ {

public:

        class_(const char *name) : bindClass(BindClass<Bound>::getInstance()) {
                bindClass.init(name);

#		if defined(BUILDING_NODE_EXTENSION)
                        // Set up handler to wrap object pointers instantiated in C++
                        // for use in JavaScript.

                        bindClass.wrapPtr = BindWrapper<Bound>::wrapPtr;
#		endif

                registerClass(bindClass);
        }

        template <class SuperType>
        class_ &inherit() {
                bindClass.template addSuperClass<SuperType>();

                return(*this);
        }

        template <
                class Signature,
                typename MethodType
        > void addMethod(
                const char *name,
                MethodType method,
                TypeFlags flags = TypeFlags::none
        ) {
                bindClass.addMethod(
                        name,
                        &Signature::getInstance(),
                        Signature::getDirect(method),
                        Signature::addMethod(method, flags),
                        flags
                );
        }

        template <
                template <typename, class, typename, typename...> class Signature,
                typename ReturnType,
                typename... Args,
                typename... Policies
        > void addMethodMaybeConst(
                const char* name,
                ReturnType(Bound::*method)(Args...),
                Policies... policies
        ) {
                addMethod<
                        Signature<
                                decltype(method),
                                Bound,
                                typename SkipNamePolicy<PolicyListType<Policies...>>::Type,
                                ReturnType,
                                Args...
                        >,
                        decltype(method)
                >(executeNamePolicy(name, policies...), method);
        }

        template <
                template <typename, class, typename, typename...> class Signature,
                typename ReturnType,
                typename... Args,
                typename... Policies
        > void addMethodMaybeConst(
                const char* name,
                ReturnType(Bound::*method)(Args...) const,
                Policies... policies
        ) {
                addMethod<
                        Signature<
                                decltype(method),
                                Bound,
                                typename SkipNamePolicy<PolicyListType<Policies...>>::Type,
                                ReturnType,
                                Args...
                        >,
                        decltype(method)
                >(
                        executeNamePolicy(name, policies...),
                        method,
                        TypeFlags::isConst
                );
        }

        template <typename... Args, typename... Policies>
        class_ &constructor(Policies...) {
                typedef ConstructorSignature<
                        Bound,
                        typename SkipNamePolicy<PolicyListType<Policies...>>::Type,
                        Args...
                > Signature;

                bindClass.addConstructor(&Signature::getInstance());

                return(*this);
        }

        // Static method.

        template <typename ReturnType, typename... Args, typename... Policies>
        class_ &method(
                const char* name,
                ReturnType(*func)(Args...),
                Policies... policies
        ) {
                addMethod<
                        FunctionSignature<
                                decltype(func),
                                std::nullptr_t,
                                typename SkipNamePolicy<PolicyListType<Policies...>>::Type,
                                ReturnType,
                                Args...
                        >,
                        decltype(func)
                >(executeNamePolicy(name, policies...), func);

                return(*this);
        }

        // Dynamic method.

        template <typename MethodType, typename... Policies>
        class_ &method(
                const char* name,
                MethodType (Bound::*method),
                Policies... policies
        ) {
                addMethodMaybeConst<MethodSignature>(name, method, policies...);

                return(*this);
        }

        template <typename... Args>
        struct Overloaded {

                Overloaded(class_ &parentDefiner) : parentDefiner(parentDefiner), definer(*this) {}

                Overloaded(const Overloaded &other) : parentDefiner(other.parentDefiner), definer(*this) {}

                Overloaded(Overloaded &&other) : parentDefiner(other.parentDefiner), definer(*this) {}

                // Overloaded static method.

                template <typename ReturnType, typename... Policies>
                class_ &method(
                        const char* name,
                        ReturnType(*func)(Args...),
                        Policies... policies
                ) {
                        return(parentDefiner.method(name, func, policies...));
                }

                // Overloaded dynamic non-const method.

                template <typename ReturnType, typename... Policies>
                class_ &method(
                        const char* name,
                        ReturnType (Bound::*method)(Args...),
                        Policies... policies
                ) {
                        return(parentDefiner.method(name, method, policies...));
                }

                // Overloaded dynamic const method.

                template <typename ReturnType, typename... Policies>
                class_ &method(
                        const char* name,
                        ReturnType (Bound::*method)(Args...) const,
                        Policies... policies
                ) {
                        return(parentDefiner.method(name, method, policies...));
                }

                class_ &parentDefiner;
                Overloaded &definer;
        };

        template <typename... Args>
        Overloaded<Args...> overloaded() {
                return(Overloaded<Args...>(*this));
        }

        template <typename GetterType, typename... Policies>
        class_ &property(
                const char* name,
                GetterType (Bound::*getter),
                Policies... policies
        ) {
                addMethodMaybeConst<GetterSignature>(name, getter, policies...);

                bindClass.addMethod(emptySetter);

                return(*this);
        }

        template <typename GetterType, typename SetterType, typename... Policies>
        class_ &property(
                const char* name,
                GetterType (Bound::*getter),
                SetterType (Bound::*setter),
                Policies... policies
        ) {
                addMethodMaybeConst<SetterSignature>(name, setter, policies...);
                addMethodMaybeConst<GetterSignature>(name, getter, policies...);

                return(*this);
        }

        template <typename Func,typename ReturnType,typename... Args,
                  typename... Policies>
        class_<Bound> &lamda(
            const char* name,
            Func&& method,
            ReturnType (*func) (Bound& bound,Args...),
            Policies... policies
        ) {
            typedef LambdaMethodSignature<
            Func,
            Bound,
            typename SkipNamePolicy<PolicyListType<Policies...>>::Type,
            ReturnType,
            Args...> Signature;
            auto& instance=Signature::getInstance();
            BindClass<Bound>::getInstance()
                    .addMethod(name,&instance,
                               Signature::getDirect(method),
                               Signature::addMethod(method, TypeFlags::isConst),
                               TypeFlags::isConst);
            return *this;
        }

        template <typename Func,typename ReturnType,typename... Args,
                  typename... Policies>
        class_<Bound> &lamda(
            const char* name,
            Func&& method,
            ReturnType (*func) (const Bound& bound,Args...),
            Policies... policies
        ) {
            typedef LambdaMethodSignature<
            Func,
            Bound,
            typename SkipNamePolicy<PolicyListType<Policies...>>::Type,
            ReturnType,
            Args...> Signature;
            auto& instance=Signature::getInstance();
            BindClass<Bound>::getInstance()
                    .addMethod(name,&instance,
                               Signature::getDirect(method),
                               Signature::addMethod(method, TypeFlags::isConst),
                               TypeFlags::isConst);
            return *this;
        }

        template <typename Func,typename... Policies>
        class_<Bound> &method(
            const char* name,
            Func&&  method,
            Policies... policies
        ) {
            return lamda(name,method,
                          (function_signature_t<Func> *)nullptr,policies...);
        }

        template <typename Func,typename... Policies>
        class_<Bound> &def(
            const char* name,
            Func&&  method,
            Policies... policies
        ) {
            return lamda(name,method,
                          (function_signature_t<Func> *)nullptr,policies...);
        }

        // Static method.

        template <typename ReturnType, typename... Args, typename... Policies>
        class_ &def_static(
                const char* name,
                ReturnType(*func)(Args...),
                Policies... policies
        ) {
                addMethod<
                        FunctionSignature<
                                decltype(func),
                                std::nullptr_t,
                                typename SkipNamePolicy<PolicyListType<Policies...>>::Type,
                                ReturnType,
                                Args...
                        >,
                        decltype(func)
                >(executeNamePolicy(name, policies...), func);

                return(*this);
        }

        // Dynamic method.

        template <typename MethodType, typename... Policies>
        class_ &def(
                const char* name,
                MethodType (Bound::*method),
                Policies... policies
        ) {
                addMethodMaybeConst<MethodSignature>(name, method, policies...);

                return(*this);
        }


private:

        BindClass<Bound> &bindClass;

};

}
