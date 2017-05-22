project(MPC)

cmake_minimum_required (VERSION 3.5)

# -g allows for gdb debugging
# turn on -03 for best performance
add_definitions(-std=gnu++11 -O3)
#add_definitions(-std=gnu++11 -g)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

add_definitions(-DUSE_ASIO -D_GNU_SOURCE)

set(sources src/MPC.cpp src/main.cpp src/twiddle.cpp)

include_directories(/usr/local/include)
link_directories(/usr/local/lib)
include_directories(src/Eigen-3.3)
include_directories(/home/uidg5371/CoinIpopt/build/include/coin)
include_directories(/usr/include)
# Include directory for uws
include_directories(/usr/include/uWS) 

link_directories(/home/uidg5371/CoinIpopt/build/lib)
link_directories(/usr/lib/gcc/x86_64-pc-cygwin/5.4.0)
link_directories(/usr/lib/gcc/x86_64-pc-cygwin/5.4.0/../../../../x86_64-pc-cygwin/lib/../lib)
link_directories(/usr/lib/gcc/x86_64-pc-cygwin/5.4.0/../../../../lib)
link_directories(/lib/../lib)
link_directories(/usr/lib/../lib)
link_directories(/usr/lib/gcc/x86_64-pc-cygwin/5.4.0/../../../../x86_64-pc-cygwin/lib)
link_directories(/usr/lib/gcc/x86_64-pc-cygwin/5.4.0/../../..)
link_directories(/home/uidg5371/CoinIpopt/build/lib)
link_directories(/usr/lib/gcc/x86_64-pc-cygwin/5.4.0)
link_directories(/usr/lib/gcc/x86_64-pc-cygwin/5.4.0/../../../../x86_64-pc-cygwin/lib/../lib)
link_directories(/usr/lib/gcc/x86_64-pc-cygwin/5.4.0/../../../../lib)
link_directories(/lib/../lib)
link_directories(/usr/lib/../lib)
link_directories(/usr/lib/gcc/x86_64-pc-cygwin/5.4.0/../../../../x86_64-pc-cygwin/lib)
link_directories(/usr/lib/gcc/x86_64-pc-cygwin/5.4.0/../../..)
link_directories(home/uidg5371/CoinIpopt/build/lib)
link_directories(/usr/lib/gcc/x86_64-pc-cygwin/5.4.0)
link_directories(/usr/lib/gcc/x86_64-pc-cygwin/5.4.0/../../../../x86_64-pc-cygwin/lib/../lib)
link_directories(/usr/lib/gcc/x86_64-pc-cygwin/5.4.0/../../../../lib)
link_directories(/lib/../lib)
link_directories(/usr/lib/../lib)
link_directories(/usr/lib/gcc/x86_64-pc-cygwin/5.4.0/../../../../x86_64-pc-cygwin/lib)
link_directories(/usr/lib/gcc/x86_64-pc-cygwin/5.4.0/../../..)
link_directories(home/uidg5371/CoinIpopt/build/lib)
link_directories(/usr/lib/gcc/x86_64-pc-cygwin/5.4.0)
link_directories(/usr/lib/gcc/x86_64-pc-cygwin/5.4.0/../../../../x86_64-pc-cygwin/lib/../lib)
link_directories(/usr/lib/gcc/x86_64-pc-cygwin/5.4.0/../../../../lib)
link_directories(/usr/lib/../lib)
link_directories(/usr/lib/gcc/x86_64-pc-cygwin/5.4.0/../../../../x86_64-pc-cygwin/lib)
link_directories(/usr/lib/gcc/x86_64-pc-cygwin/5.4.0/../../..)

# Include path for uws
link_directories(/lib)

add_executable(MPC ${sources})

target_link_libraries(MPC ipopt m dl coinmumps gfortran quadmath m advapi32 shell32 user32 kernel32 coinmetis coinhsl gfortran quadmath m advapi32 shell32 user32 kernel32 coinlapack gfortran quadmath m advapi32 shell32 user32 kernel32 coinblas gfortran quadmath m advapi32 shell32 user32 kernel32 python2.7 z ssl crypto boost_system libuWS)
