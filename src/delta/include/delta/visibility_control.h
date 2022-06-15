#ifndef DELTA__VISIBILITY_CONTROL_H_
#define DELTA__VISIBILITY_CONTROL_H_
#ifdef __cplusplus
extern "C"
{
#endif
// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
// https://gcc.gnu.org/wiki/Visibility
#if defined _WIN32 || defined __CYGWIN__
    #ifdef __GNUC__
        #define DELTA_EXPORT __attribute__ ((dllexport))
        #define DELTA_IMPORT __attribute__ ((dllimport))
    #else
        #define DELTA_EXPORT __declspec(dllexport)
        #define DELTA_IMPORT __declspec(dllimport)
    #endif
    #ifdef DELTA_BUILDING_DLL
        #define DELTA_PUBLIC DELTA_EXPORT
    #else
        #define DELTA_PUBLIC DELTA_IMPORT
    #endif
    #define DELTA_PUBLIC_TYPE DELTA_PUBLIC
    #define DELTA_LOCAL
#else
    #define DELTA_EXPORT __attribute__ ((visibility("default")))
    #define DELTA_IMPORT
    #if __GNUC__ >= 4
        #define DELTA_PUBLIC __attribute__ ((visibility("default")))
        #define DELTA_LOCAL __attribute__ ((visibility("hidden")))
    #else
        #define DELTA_PUBLIC
        #define DELTA_LOCAL
    #endif
    #define DELTA_PUBLIC_TYPE
#endif
#ifdef __cplusplus
}
#endif
#endif
// DELTA__VISIBILITY_CONTROL_H_