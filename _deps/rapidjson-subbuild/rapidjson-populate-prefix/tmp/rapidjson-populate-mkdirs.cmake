# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/Users/tristcrocker/Documents/HYPED/hyped-2023/_deps/rapidjson-src"
  "/Users/tristcrocker/Documents/HYPED/hyped-2023/_deps/rapidjson-build"
  "/Users/tristcrocker/Documents/HYPED/hyped-2023/_deps/rapidjson-subbuild/rapidjson-populate-prefix"
  "/Users/tristcrocker/Documents/HYPED/hyped-2023/_deps/rapidjson-subbuild/rapidjson-populate-prefix/tmp"
  "/Users/tristcrocker/Documents/HYPED/hyped-2023/_deps/rapidjson-subbuild/rapidjson-populate-prefix/src/rapidjson-populate-stamp"
  "/Users/tristcrocker/Documents/HYPED/hyped-2023/_deps/rapidjson-subbuild/rapidjson-populate-prefix/src"
  "/Users/tristcrocker/Documents/HYPED/hyped-2023/_deps/rapidjson-subbuild/rapidjson-populate-prefix/src/rapidjson-populate-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/Users/tristcrocker/Documents/HYPED/hyped-2023/_deps/rapidjson-subbuild/rapidjson-populate-prefix/src/rapidjson-populate-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/Users/tristcrocker/Documents/HYPED/hyped-2023/_deps/rapidjson-subbuild/rapidjson-populate-prefix/src/rapidjson-populate-stamp${cfgdir}") # cfgdir has leading slash
endif()
