cmake_minimum_required(VERSION 3.6)
project(ORB_SLAM_AR)

set(OpenCV_INCLUDE_DIRS F:/OpenCV310/build/mingw32/install/include)
include_directories(${OpenCV_INCLUDE_DIRS})

set(OpenCV_LIBS
        F:/OpenCV310/build/mingw32/install/x86/mingw/bin/libopencv_world310.dll
        )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

INCLUDE_DIRECTORIES(
        ${g2o_SOURCE_DIR}/core
        ${g2o_SOURCE_DIR}/types
        ${g2o_SOURCE_DIR}/stuff)

ADD_LIBRARY(g2o STATIC
        #types
        ./Thirdparty/g2o/g2o/types/types_sba.h
        ./Thirdparty/g2o/g2o/types/types_six_dof_expmap.h
        ./Thirdparty/g2o/g2o/types/types_sba.cpp
        ./Thirdparty/g2o/g2o/types/types_six_dof_expmap.cpp
        ./Thirdparty/g2o/g2o/types/types_seven_dof_expmap.cpp
        ./Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h
        ./Thirdparty/g2o/g2o/types/se3quat.h
        ./Thirdparty/g2o/g2o/types/se3_ops.h
        ./Thirdparty/g2o/g2o/types/se3_ops.hpp
        #core
        ./Thirdparty/g2o/g2o/core/base_edge.h
        ./Thirdparty/g2o/g2o/core/base_binary_edge.h
        ./Thirdparty/g2o/g2o/core/hyper_graph_action.cpp
        ./Thirdparty/g2o/g2o/core/base_binary_edge.hpp
        ./Thirdparty/g2o/g2o/core/hyper_graph_action.h
        ./Thirdparty/g2o/g2o/core/base_multi_edge.h
        ./Thirdparty/g2o/g2o/core/hyper_graph.cpp
        ./Thirdparty/g2o/g2o/core/base_multi_edge.hpp
        ./Thirdparty/g2o/g2o/core/hyper_graph.h
        ./Thirdparty/g2o/g2o/core/base_unary_edge.h
        ./Thirdparty/g2o/g2o/core/linear_solver.h
        ./Thirdparty/g2o/g2o/core/base_unary_edge.hpp
        ./Thirdparty/g2o/g2o/core/marginal_covariance_cholesky.cpp
        ./Thirdparty/g2o/g2o/core/base_vertex.h
        ./Thirdparty/g2o/g2o/core/marginal_covariance_cholesky.h
        ./Thirdparty/g2o/g2o/core/base_vertex.hpp
        ./Thirdparty/g2o/g2o/core/matrix_structure.cpp
        ./Thirdparty/g2o/g2o/core/batch_stats.cpp
        ./Thirdparty/g2o/g2o/core/matrix_structure.h
        ./Thirdparty/g2o/g2o/core/batch_stats.h
        ./Thirdparty/g2o/g2o/core/openmp_mutex.h
        ./Thirdparty/g2o/g2o/core/block_solver.h
        ./Thirdparty/g2o/g2o/core/block_solver.hpp
        ./Thirdparty/g2o/g2o/core/parameter.cpp
        ./Thirdparty/g2o/g2o/core/parameter.h
        ./Thirdparty/g2o/g2o/core/cache.cpp
        ./Thirdparty/g2o/g2o/core/cache.h
        ./Thirdparty/g2o/g2o/core/optimizable_graph.cpp
        ./Thirdparty/g2o/g2o/core/optimizable_graph.h
        ./Thirdparty/g2o/g2o/core/solver.cpp
        ./Thirdparty/g2o/g2o/core/solver.h
        ./Thirdparty/g2o/g2o/core/creators.h
        ./Thirdparty/g2o/g2o/core/optimization_algorithm_factory.cpp
        ./Thirdparty/g2o/g2o/core/estimate_propagator.cpp
        ./Thirdparty/g2o/g2o/core/optimization_algorithm_factory.h
        ./Thirdparty/g2o/g2o/core/estimate_propagator.h
        ./Thirdparty/g2o/g2o/core/factory.cpp
        ./Thirdparty/g2o/g2o/core/optimization_algorithm_property.h
        ./Thirdparty/g2o/g2o/core/factory.h
        ./Thirdparty/g2o/g2o/core/sparse_block_matrix.h
        ./Thirdparty/g2o/g2o/core/sparse_optimizer.cpp
        ./Thirdparty/g2o/g2o/core/sparse_block_matrix.hpp
        ./Thirdparty/g2o/g2o/core/sparse_optimizer.h
        ./Thirdparty/g2o/g2o/core/hyper_dijkstra.cpp
        ./Thirdparty/g2o/g2o/core/hyper_dijkstra.h
        ./Thirdparty/g2o/g2o/core/parameter_container.cpp
        ./Thirdparty/g2o/g2o/core/parameter_container.h
        ./Thirdparty/g2o/g2o/core/optimization_algorithm.cpp
        ./Thirdparty/g2o/g2o/core/optimization_algorithm.h
        ./Thirdparty/g2o/g2o/core/optimization_algorithm_with_hessian.cpp
        ./Thirdparty/g2o/g2o/core/optimization_algorithm_with_hessian.h
        ./Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.cpp
        ./Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h
        ./Thirdparty/g2o/g2o/core/jacobian_workspace.cpp
        ./Thirdparty/g2o/g2o/core/jacobian_workspace.h
        ./Thirdparty/g2o/g2o/core/robust_kernel.cpp
        ./Thirdparty/g2o/g2o/core/robust_kernel.h
        ./Thirdparty/g2o/g2o/core/robust_kernel_factory.cpp
        ./Thirdparty/g2o/g2o/core/robust_kernel_factory.h
        ./Thirdparty/g2o/g2o/core/robust_kernel_impl.cpp
        ./Thirdparty/g2o/g2o/core/robust_kernel_impl.h
        #stuff
        ./Thirdparty/g2o/g2o/stuff/string_tools.h
        ./Thirdparty/g2o/g2o/stuff/color_macros.h
        ./Thirdparty/g2o/g2o/stuff/macros.h
        ./Thirdparty/g2o/g2o/stuff/timeutil.cpp
        ./Thirdparty/g2o/g2o/stuff/misc.h
        ./Thirdparty/g2o/g2o/stuff/timeutil.h
        ./Thirdparty/g2o/g2o/stuff/os_specific.c
        ./Thirdparty/g2o/g2o/stuff/os_specific.h
        ./Thirdparty/g2o/g2o/stuff/string_tools.cpp
        ./Thirdparty/g2o/g2o/stuff/property.cpp
        ./Thirdparty/g2o/g2o/stuff/property.h
        )

include_directories(
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/include
        ./Thirdparty
        ./Thirdparty/g2o
        ./Thirdparty/g2o/g2o/core
        ./Thirdparty/freeglut/include
)
add_library(${PROJECT_NAME} SHARED
        include/System.h
        include/Tracking.h
        include/LocalMapping.h
        include/LoopClosing.h
        include/ORBextractor.h
        include/ORBmatcher.h
        include/FrameDrawer.h
        include/Converter.h
        include/MapPoint.h
        include/KeyFrame.h
        include/Map.h
        include/Optimizer.h
        include/PnPsolver.h
        include/Frame.h
        include/KeyFrameDatabase.h
        include/Sim3Solver.h
        include/Initializer.h
        include/Common.h

        src/System.cc
        src/Tracking.cc
        src/LocalMapping.cc
        src/LoopClosing.cc
        src/ORBextractor.cc
        src/ORBmatcher.cc
        src/FrameDrawer.cc
        src/Converter.cc
        src/MapPoint.cc
        src/KeyFrame.cc
        src/Map.cc
        src/Optimizer.cc
        src/PnPsolver.cc
        src/Frame.cc
        src/KeyFrameDatabase.cc
        src/Sim3Solver.cc
        src/Initializer.cc

        ############DBow2############
        Thirdparty/DBoW2/DBoW2/BowVector.h
        Thirdparty/DBoW2/DBoW2/FORB.h
        Thirdparty/DBoW2/DBoW2/FClass.h
        Thirdparty/DBoW2/DBoW2/FeatureVector.h
        Thirdparty/DBoW2/DBoW2/ScoringObject.h
        Thirdparty/DBoW2/DBoW2/TemplatedVocabulary.h
        Thirdparty/DBoW2/DUtils/Random.h

        Thirdparty/DBoW2/DBoW2/BowVector.cpp
        Thirdparty/DBoW2/DBoW2/FORB.cpp
        Thirdparty/DBoW2/DBoW2/FeatureVector.cpp
        Thirdparty/DBoW2/DBoW2/ScoringObject.cpp
        Thirdparty/DBoW2/DUtils/Random.cpp )

target_link_libraries(${PROJECT_NAME}
        ${OpenCV_LIBS}
        g2o
        #${PROJECT_SOURCE_DIR}/Thirdparty/g2o/libwin/libg2o.so
        )

set(FREE_GLUT
        ${PROJECT_SOURCE_DIR}/Thirdparty/freeglut/lib/libfreeglut.a
        ${PROJECT_SOURCE_DIR}/Thirdparty/freeglut/lib/libfreeglut_static.a
        ${PROJECT_SOURCE_DIR}/Thirdparty/freeglut/bin/freeglut.dll
        opengl32
        glu32)

add_executable(mono_ar
        Examples/Monocular/ar/ros_mono_ar.cc
        Examples/Monocular/ar/ViewerAR.h
        Examples/Monocular/ar/ViewerAR.cc)
target_link_libraries(mono_ar
        ${PROJECT_NAME}
        ${FREE_GLUT}
        )

add_executable(mono_ar_tum
        Examples/Monocular/ar/ros_mono_ar_tum.cc
        Examples/Monocular/ar/ViewerAR.h
        Examples/Monocular/ar/ViewerAR.cc
        )

target_link_libraries(mono_ar_tum ${PROJECT_NAME}
        ${OpenCV_LIBS}
        ${FREE_GLUT}
        )

