# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/karl/esp/esp-idf/components/bootloader/subproject"
  "/home/karl/Documents/dev/redshift/node0_btReceiver/build/bootloader"
  "/home/karl/Documents/dev/redshift/node0_btReceiver/build/bootloader-prefix"
  "/home/karl/Documents/dev/redshift/node0_btReceiver/build/bootloader-prefix/tmp"
  "/home/karl/Documents/dev/redshift/node0_btReceiver/build/bootloader-prefix/src/bootloader-stamp"
  "/home/karl/Documents/dev/redshift/node0_btReceiver/build/bootloader-prefix/src"
  "/home/karl/Documents/dev/redshift/node0_btReceiver/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/karl/Documents/dev/redshift/node0_btReceiver/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
