# Install script for directory: E:/xingshan/bearpi-pico_h2821e-master/drivers/drivers/driver

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "C:/Program Files (x86)/bs21e_CFBB")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "TRUE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "E:/xingshan/bearpi-pico_h2821e-master/tools/bin/compiler/riscv/cc_riscv32_musl_b010/cc_riscv32_musl_fp_win/bin/riscv32-linux-musl-objdump.exe")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("E:/xingshan/bearpi-pico_h2821e-master/output/bs21e/acore/standard-bs21e-1100e/driver/adc/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("E:/xingshan/bearpi-pico_h2821e-master/output/bs21e/acore/standard-bs21e-1100e/driver/cpu_trace/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("E:/xingshan/bearpi-pico_h2821e-master/output/bs21e/acore/standard-bs21e-1100e/driver/dma/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("E:/xingshan/bearpi-pico_h2821e-master/output/bs21e/acore/standard-bs21e-1100e/driver/flash/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("E:/xingshan/bearpi-pico_h2821e-master/output/bs21e/acore/standard-bs21e-1100e/driver/gpio/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("E:/xingshan/bearpi-pico_h2821e-master/output/bs21e/acore/standard-bs21e-1100e/driver/i2c/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("E:/xingshan/bearpi-pico_h2821e-master/output/bs21e/acore/standard-bs21e-1100e/driver/i2s/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("E:/xingshan/bearpi-pico_h2821e-master/output/bs21e/acore/standard-bs21e-1100e/driver/ir/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("E:/xingshan/bearpi-pico_h2821e-master/output/bs21e/acore/standard-bs21e-1100e/driver/keyscan/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("E:/xingshan/bearpi-pico_h2821e-master/output/bs21e/acore/standard-bs21e-1100e/driver/lpc/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("E:/xingshan/bearpi-pico_h2821e-master/output/bs21e/acore/standard-bs21e-1100e/driver/lpm/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("E:/xingshan/bearpi-pico_h2821e-master/output/bs21e/acore/standard-bs21e-1100e/driver/memory_core/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("E:/xingshan/bearpi-pico_h2821e-master/output/bs21e/acore/standard-bs21e-1100e/driver/pinmux/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("E:/xingshan/bearpi-pico_h2821e-master/output/bs21e/acore/standard-bs21e-1100e/driver/pmp/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("E:/xingshan/bearpi-pico_h2821e-master/output/bs21e/acore/standard-bs21e-1100e/driver/pwm/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("E:/xingshan/bearpi-pico_h2821e-master/output/bs21e/acore/standard-bs21e-1100e/driver/qdec/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("E:/xingshan/bearpi-pico_h2821e-master/output/bs21e/acore/standard-bs21e-1100e/driver/security/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("E:/xingshan/bearpi-pico_h2821e-master/output/bs21e/acore/standard-bs21e-1100e/driver/spi/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("E:/xingshan/bearpi-pico_h2821e-master/output/bs21e/acore/standard-bs21e-1100e/driver/systick/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("E:/xingshan/bearpi-pico_h2821e-master/output/bs21e/acore/standard-bs21e-1100e/driver/tcxo/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("E:/xingshan/bearpi-pico_h2821e-master/output/bs21e/acore/standard-bs21e-1100e/driver/timer/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("E:/xingshan/bearpi-pico_h2821e-master/output/bs21e/acore/standard-bs21e-1100e/driver/touch/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("E:/xingshan/bearpi-pico_h2821e-master/output/bs21e/acore/standard-bs21e-1100e/driver/uart/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("E:/xingshan/bearpi-pico_h2821e-master/output/bs21e/acore/standard-bs21e-1100e/driver/ulp_aon/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("E:/xingshan/bearpi-pico_h2821e-master/output/bs21e/acore/standard-bs21e-1100e/driver/watchdog/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("E:/xingshan/bearpi-pico_h2821e-master/output/bs21e/acore/standard-bs21e-1100e/driver/pdm/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("E:/xingshan/bearpi-pico_h2821e-master/output/bs21e/acore/standard-bs21e-1100e/driver/sfc/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("E:/xingshan/bearpi-pico_h2821e-master/output/bs21e/acore/standard-bs21e-1100e/driver/pm/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("E:/xingshan/bearpi-pico_h2821e-master/output/bs21e/acore/standard-bs21e-1100e/driver/usb_unified/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("E:/xingshan/bearpi-pico_h2821e-master/output/bs21e/acore/standard-bs21e-1100e/driver/rtc_unified/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("E:/xingshan/bearpi-pico_h2821e-master/output/bs21e/acore/standard-bs21e-1100e/driver/efuse/cmake_install.cmake")
endif()

