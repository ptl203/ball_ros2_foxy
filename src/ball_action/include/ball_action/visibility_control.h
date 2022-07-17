#ifndef BALL_ACTION__VISIBILITY_CONTROL_H_
#define BALL_ACTION__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define BALL_ACTION_EXPORT __attribute__ ((dllexport))
    #define BALL_ACTION_IMPORT __attribute__ ((dllimport))
  #else
    #define BALL_ACTION_EXPORT __declspec(dllexport)
    #define BALL_ACTION_IMPORT __declspec(dllimport)
  #endif
  #ifdef BALL_ACTION_BUILDING_DLL
    #define BALL_ACTION_PUBLIC BALL_ACTION_EXPORT
  #else
    #define BALL_ACTION_PUBLIC BALL_ACTION_IMPORT
  #endif
  #define BALL_ACTION_PUBLIC_TYPE BALL_ACTION_PUBLIC
  #define BALL_ACTION_LOCAL
#else
  #define BALL_ACTION_EXPORT __attribute__ ((visibility("default")))
  #define BALL_ACTION_IMPORT
  #if __GNUC__ >= 4
    #define BALL_ACTION_PUBLIC __attribute__ ((visibility("default")))
    #define BALL_ACTION_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define BALL_ACTION_PUBLIC
    #define BALL_ACTION_LOCAL
  #endif
  #define BALL_ACTION_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // BALL_ACTION__VISIBILITY_CONTROL_H_