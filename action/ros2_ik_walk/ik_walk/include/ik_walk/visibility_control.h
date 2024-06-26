// Copyright 2019 Bold Hearts
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef IK_WALK__VISIBILITY_CONTROL_H_
#define IK_WALK__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define IK_WALK_EXPORT __attribute__ ((dllexport))
    #define IK_WALK_IMPORT __attribute__ ((dllimport))
  #else
    #define IK_WALK_EXPORT __declspec(dllexport)
    #define IK_WALK_IMPORT __declspec(dllimport)
  #endif
  #ifdef IK_WALK_BUILDING_LIBRARY
    #define IK_WALK_PUBLIC IK_WALK_EXPORT
  #else
    #define IK_WALK_PUBLIC IK_WALK_IMPORT
  #endif
  #define IK_WALK_PUBLIC_TYPE IK_WALK_PUBLIC
  #define IK_WALK_LOCAL
#else
  #define IK_WALK_EXPORT __attribute__ ((visibility("default")))
  #define IK_WALK_IMPORT
  #if __GNUC__ >= 4
    #define IK_WALK_PUBLIC __attribute__ ((visibility("default")))
    #define IK_WALK_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define IK_WALK_PUBLIC
    #define IK_WALK_LOCAL
  #endif
  #define IK_WALK_PUBLIC_TYPE
#endif

#endif  // IK_WALK__VISIBILITY_CONTROL_H_
