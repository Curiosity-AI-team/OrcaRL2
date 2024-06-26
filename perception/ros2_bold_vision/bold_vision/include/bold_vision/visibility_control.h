#ifndef BOLD_VISION__VISIBILITY_CONTROL_H_
#define BOLD_VISION__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define BOLD_VISION_EXPORT __attribute__ ((dllexport))
    #define BOLD_VISION_IMPORT __attribute__ ((dllimport))
  #else
    #define BOLD_VISION_EXPORT __declspec(dllexport)
    #define BOLD_VISION_IMPORT __declspec(dllimport)
  #endif
  #ifdef BOLD_VISION_BUILDING_LIBRARY
    #define BOLD_VISION_PUBLIC BOLD_VISION_EXPORT
  #else
    #define BOLD_VISION_PUBLIC BOLD_VISION_IMPORT
  #endif
  #define BOLD_VISION_PUBLIC_TYPE BOLD_VISION_PUBLIC
  #define BOLD_VISION_LOCAL
#else
  #define BOLD_VISION_EXPORT __attribute__ ((visibility("default")))
  #define BOLD_VISION_IMPORT
  #if __GNUC__ >= 4
    #define BOLD_VISION_PUBLIC __attribute__ ((visibility("default")))
    #define BOLD_VISION_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define BOLD_VISION_PUBLIC
    #define BOLD_VISION_LOCAL
  #endif
  #define BOLD_VISION_PUBLIC_TYPE
#endif

#endif  // BOLD_VISION__VISIBILITY_CONTROL_H_
