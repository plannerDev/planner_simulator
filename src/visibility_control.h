#ifndef RRT_PLUGIN__VISIBILITY_CONTROL_H_
#define RRT_PLUGIN__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define RRT_PLUGIN_EXPORT __attribute__((dllexport))
#define RRT_PLUGIN_IMPORT __attribute__((dllimport))
#else
#define RRT_PLUGIN_EXPORT __declspec(dllexport)
#define RRT_PLUGIN_IMPORT __declspec(dllimport)
#endif
#ifdef RRT_PLUGIN_BUILDING_LIBRARY
#define RRT_PLUGIN_PUBLIC RRT_PLUGIN_EXPORT
#else
#define RRT_PLUGIN_PUBLIC RRT_PLUGIN_IMPORT
#endif
#define RRT_PLUGIN_PUBLIC_TYPE RRT_PLUGIN_PUBLIC
#define RRT_PLUGIN_LOCAL
#else
#define RRT_PLUGIN_EXPORT __attribute__((visibility("default")))
#define RRT_PLUGIN_IMPORT
#if __GNUC__ >= 4
#define RRT_PLUGIN_PUBLIC __attribute__((visibility("default")))
#define RRT_PLUGIN_LOCAL __attribute__((visibility("hidden")))
#else
#define RRT_PLUGIN_PUBLIC
#define RRT_PLUGIN_LOCAL
#endif
#define RRT_PLUGIN_PUBLIC_TYPE
#endif

#endif // RRT_PLUGIN__VISIBILITY_CONTROL_H_
