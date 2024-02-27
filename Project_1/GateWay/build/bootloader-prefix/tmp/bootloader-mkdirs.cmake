# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/dongkhoa/esp/esp-idf/components/bootloader/subproject"
  "/home/dongkhoa/Documents/ssd1963/Project_1/GateWay/build/bootloader"
  "/home/dongkhoa/Documents/ssd1963/Project_1/GateWay/build/bootloader-prefix"
  "/home/dongkhoa/Documents/ssd1963/Project_1/GateWay/build/bootloader-prefix/tmp"
  "/home/dongkhoa/Documents/ssd1963/Project_1/GateWay/build/bootloader-prefix/src/bootloader-stamp"
  "/home/dongkhoa/Documents/ssd1963/Project_1/GateWay/build/bootloader-prefix/src"
  "/home/dongkhoa/Documents/ssd1963/Project_1/GateWay/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/dongkhoa/Documents/ssd1963/Project_1/GateWay/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/dongkhoa/Documents/ssd1963/Project_1/GateWay/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
