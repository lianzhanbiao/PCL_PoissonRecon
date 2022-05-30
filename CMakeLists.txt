# CMakeList.txt: PRecon 的 CMake 项目，在此处包括源代码并定义
# 项目特定的逻辑。
#
cmake_minimum_required (VERSION 3.8)

project ("PRecon")

# 将源代码添加到此项目的可执行文件。

file(GLOB POISSON_HEADER_FILES
	"./PoissonRecon/*.h"
	"./PoissonRecon/*.inl"
)

file(GLOB POISSON_CPP_FILES
	"./PoissonRecon/*.cpp"
)

add_executable(recon 
	${POISSON_HEADER_FILES}
	${POISSON_CPP_FILES}
	Precon.h
	Precon.cpp
)

find_package(PCL REQUIRED)
target_link_libraries(recon ${PCL_LIBRARIES})

target_include_directories(recon PRIVATE "./libs")

if (CMAKE_VERSION VERSION_GREATER 3.12)
  set_property(TARGET recon PROPERTY CXX_STANDARD 20)
endif()

# TODO: 如有需要，请添加测试并安装目标。
