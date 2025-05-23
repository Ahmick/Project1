# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

# If CMAKE_DISABLE_SOURCE_CHANGES is set to true and the source directory is an
# existing directory in our source tree, calling file(MAKE_DIRECTORY) on it
# would cause a fatal error, even though it would be a no-op.
if(NOT EXISTS "E:/Projects/Programming/esp-idf/v5.4/esp-idf/components/bootloader/subproject")
  file(MAKE_DIRECTORY "E:/Projects/Programming/esp-idf/v5.4/esp-idf/components/bootloader/subproject")
endif()
file(MAKE_DIRECTORY
  "E:/Projects/Programming/Projects/Project1/ESP32_C6/Example Projects/i2c_eeprom/build/bootloader"
  "E:/Projects/Programming/Projects/Project1/ESP32_C6/Example Projects/i2c_eeprom/build/bootloader-prefix"
  "E:/Projects/Programming/Projects/Project1/ESP32_C6/Example Projects/i2c_eeprom/build/bootloader-prefix/tmp"
  "E:/Projects/Programming/Projects/Project1/ESP32_C6/Example Projects/i2c_eeprom/build/bootloader-prefix/src/bootloader-stamp"
  "E:/Projects/Programming/Projects/Project1/ESP32_C6/Example Projects/i2c_eeprom/build/bootloader-prefix/src"
  "E:/Projects/Programming/Projects/Project1/ESP32_C6/Example Projects/i2c_eeprom/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "E:/Projects/Programming/Projects/Project1/ESP32_C6/Example Projects/i2c_eeprom/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "E:/Projects/Programming/Projects/Project1/ESP32_C6/Example Projects/i2c_eeprom/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
