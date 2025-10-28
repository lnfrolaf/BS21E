# Install script for directory: E:/xingshan/bearpi-pico_h2821e-master/drivers/drivers/hal

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
  include("E:/xingshan/bearpi-pico_h2821e-master/output/bs21e/acore/standard-bs21e-1100e/hal/adc/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("E:/xingshan/bearpi-pico_h2821e-master/output/bs21e/acore/standard-bs21e-1100e/hal/cpu_core/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("E:/xingshan/bearpi-pico_h2821e-master/output/bs21e/acore/standard-bs21e-1100e/hal/cpu_trace/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("E:/xingshan/bearpi-pico_h2821e-master/output/bs21e/acore/standard-bs21e-1100e/hal/dma/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("E:/xingshan/bearpi-pico_h2821e-master/output/bs21e/acore/standard-bs21e-1100e/hal/efuse/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("E:/xingshan/bearpi-pico_h2821e-master/output/bs21e/acore/standard-bs21e-1100e/hal/gpio/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("E:/xingshan/bearpi-pico_h2821e-master/output/bs21e/acore/standard-bs21e-1100e/hal/i2c/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("E:/xingshan/bearpi-pico_h2821e-master/output/bs21e/acore/standard-bs21e-1100e/hal/keyscan/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("E:/xingshan/bearpi-pico_h2821e-master/output/bs21e/acore/standard-bs21e-1100e/hal/lpc/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("E:/xingshan/bearpi-pico_h2821e-master/output/bs21e/acore/standard-bs21e-1100e/hal/mips/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("E:/xingshan/bearpi-pico_h2821e-master/output/bs21e/acore/standard-bs21e-1100e/hal/pinmux/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("E:/xingshan/bearpi-pico_h2821e-master/output/bs21e/acore/standard-bs21e-1100e/hal/pmp/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("E:/xingshan/bearpi-pico_h2821e-master/output/bs21e/acore/standard-bs21e-1100e/hal/pwm/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("E:/xingshan/bearpi-pico_h2821e-master/output/bs21e/acore/standard-bs21e-1100e/hal/qdec/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("E:/xingshan/bearpi-pico_h2821e-master/output/bs21e/acore/standard-bs21e-1100e/hal/reboot/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("E:/xingshan/bearpi-pico_h2821e-master/output/bs21e/acore/standard-bs21e-1100e/hal/reg_config/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("E:/xingshan/bearpi-pico_h2821e-master/output/bs21e/acore/standard-bs21e-1100e/hal/security/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("E:/xingshan/bearpi-pico_h2821e-master/output/bs21e/acore/standard-bs21e-1100e/hal/spi/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("E:/xingshan/bearpi-pico_h2821e-master/output/bs21e/acore/standard-bs21e-1100e/hal/systick/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("E:/xingshan/bearpi-pico_h2821e-master/output/bs21e/acore/standard-bs21e-1100e/hal/tcxo/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("E:/xingshan/bearpi-pico_h2821e-master/output/bs21e/acore/standard-bs21e-1100e/hal/timer/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("E:/xingshan/bearpi-pico_h2821e-master/output/bs21e/acore/standard-bs21e-1100e/hal/uart/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("E:/xingshan/bearpi-pico_h2821e-master/output/bs21e/acore/standard-bs21e-1100e/hal/watchdog/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("E:/xingshan/bearpi-pico_h2821e-master/output/bs21e/acore/standard-bs21e-1100e/hal/pdm/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("E:/xingshan/bearpi-pico_h2821e-master/output/bs21e/acore/standard-bs21e-1100e/hal/sfc/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("E:/xingshan/bearpi-pico_h2821e-master/output/bs21e/acore/standard-bs21e-1100e/hal/sio/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("E:/xingshan/bearpi-pico_h2821e-master/output/bs21e/acore/standard-bs21e-1100e/hal/rtc_unified/cmake_install.cmake")
endif()

