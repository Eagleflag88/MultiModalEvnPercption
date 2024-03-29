cmake_minimum_required(VERSION 2.8.3)
project(ndt_gpu)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O3 -mtune=native ")
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
	velodyne_pointcloud
)
find_package(PCL REQUIRED)
find_package(CUDA)
find_package(Eigen3 REQUIRED)

if (CUDA_FOUND)
  set_directory_properties(PROPERTIES COMPILE_DEFINITIONS "")

	if(NOT DEFINED CUDA_CAPABILITY_VERSION_CHECKER)
		set(CUDA_CAPABILITY_VERSION_CHECKER
			"${CATKIN_DEVEL_PREFIX}/lib/capability_version_checker")
  endif()
  
  execute_process(
	    COMMAND ${CUDA_CAPABILITY_VERSION_CHECKER}
	    OUTPUT_VARIABLE CUDA_CAPABILITY_VERSION
	    OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    
	if("${CUDA_CAPABILITY_VERSION}" MATCHES "^[1-9][0-9]+$")
	  set(CUDA_ARCH "sm_75")
	else()
	  set(CUDA_ARCH "sm_75")
	endif()
	
	set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-arch=${CUDA_ARCH};-std=c++14;--ptxas-options=-v)
	
	set(SUBSYS_NAME ndt_gpu)
	set(SUBSYS_DESC "Point cloud ndt gpu library")
	set(SUBSYS_DEPS common)
	set(LIB_NAME "fast_pcl_ndt_gpu")
	
#	catkin_package(
#  	DEPENDS ${SUBSYS_DEPS}
#  	INCLUDE_DIRS include
#   LIBRARIES ${LIB_NAME}
#  )

  include_directories(
    ${PCL_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${CUDA_INCLUDE_DIRS}
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
  )

	set(srcs
	  src/MatrixDevice.cu
		src/MatrixHost.cu
		src/NormalDistributionsTransform.cu
		src/Registration.cu
		src/VoxelGrid.cu
		src/SymmetricEigenSolver.cu
	)
	
	set(incs
	  include/fast_pcl/ndt_gpu/common.h
	  include/fast_pcl/ndt_gpu/debug.h
	  include/fast_pcl/ndt_gpu/Matrix.h
	  include/fast_pcl/ndt_gpu/MatrixDevice.h
	  include/fast_pcl/ndt_gpu/MatrixHost.h
	  include/fast_pcl/ndt_gpu/NormalDistributionsTransform.h
	  include/fast_pcl/ndt_gpu/Registration.h
	  include/fast_pcl/ndt_gpu/SymmetricEigenSolver.h
	  include/fast_pcl/ndt_gpu/VoxelGrid.h
	)
	
	cuda_add_library("${LIB_NAME}" ${srcs} ${incs})
	message("GPU ndt Library is " ${LIB_NAME})
	target_include_directories("${LIB_NAME}" PRIVATE
		${CUDA_INCLUDE_DIRS}
	)
	
	target_link_libraries("${LIB_NAME}" 
		${CUDA_LIBRARIES}
		${CUDA_CUBLAS_LIBRARIES}
		${CUDA_curand_LIBRARY})

	#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 ")
	SET(CMAKE_CXX_FLAGS "-std=c++14 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")

	add_executable(test_node src/test_node.cpp)
	target_link_libraries(test_node
		${LIB_NAME}
		#${catkin_LIBRARIES}
  		${PCL_LIBRARIES}
	)
else()
	message("fast_pcl ndt_gpu requires CUDA")
endif()
