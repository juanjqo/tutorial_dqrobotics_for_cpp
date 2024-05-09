
if (WIN32)
    include(C:/Tools/vcpkg/scripts/buildsystems/vcpkg.cmake)
    set(CMAKE_TOOLCHAIN_FILE C:/Tools/vcpkg/scripts/buildsystems/vcpkg.cmake)
    ADD_DEFINITIONS(-D_USE_MATH_DEFINES)
       FIND_PACKAGE(Eigen3 CONFIG REQUIRED)
       INCLUDE_DIRECTORIES(
               ${EIGEN3_INCLUDE_DIR}
               #${GLFW3_INCLUDE_DIRS}
               # Default path when using cmake .., cmake --build ., cmake --install .
               # "C:/Program Files (x86)/dqrobotics/include"
               #"C:/Tools/vcpkg/installed/x64-windows/include/"
               )
endif()

# DQ Robotics

set(DQROBOTICS_HEADERS
        ${DQROBOTICS_DIR}/include/dqrobotics/DQ.h

        ${DQROBOTICS_DIR}/include/dqrobotics/internal/_dq_linesegment.h

        ${DQROBOTICS_DIR}/include/dqrobotics/utils/DQ_Geometry.h
        ${DQROBOTICS_DIR}/include/dqrobotics/utils/DQ_LinearAlgebra.h
        ${DQROBOTICS_DIR}/include/dqrobotics/utils/DQ_Math.h

        ${DQROBOTICS_DIR}/include/dqrobotics/robot_modeling/DQ_CooperativeDualTaskSpace.h
        ${DQROBOTICS_DIR}/include/dqrobotics/robot_modeling/DQ_Kinematics.h
        ${DQROBOTICS_DIR}/include/dqrobotics/robot_modeling/DQ_SerialManipulator.h
        ${DQROBOTICS_DIR}/include/dqrobotics/robot_modeling/DQ_SerialManipulatorDH.h
        ${DQROBOTICS_DIR}/include/dqrobotics/robot_modeling/DQ_SerialManipulatorMDH.h
        ${DQROBOTICS_DIR}/include/dqrobotics/robot_modeling/DQ_SerialManipulatorDenso.h
        ${DQROBOTICS_DIR}/include/dqrobotics/robot_modeling/DQ_MobileBase.h
        ${DQROBOTICS_DIR}/include/dqrobotics/robot_modeling/DQ_HolonomicBase.h
        ${DQROBOTICS_DIR}/include/dqrobotics/robot_modeling/DQ_DifferentialDriveRobot.h
        ${DQROBOTICS_DIR}/include/dqrobotics/robot_modeling/DQ_WholeBody.h
        ${DQROBOTICS_DIR}/include/dqrobotics/robot_modeling/DQ_SerialWholeBody.h

        ${DQROBOTICS_DIR}/include/dqrobotics//robot_control/DQ_KinematicController.h
        ${DQROBOTICS_DIR}/include/dqrobotics/robot_control/DQ_PseudoinverseController.h
        ${DQROBOTICS_DIR}/include/dqrobotics/robot_control/DQ_NumericalFilteredPseudoInverseController.h
        ${DQROBOTICS_DIR}/include/dqrobotics/robot_control/DQ_KinematicConstrainedController.h
        ${DQROBOTICS_DIR}/include/dqrobotics/robot_control/DQ_QuadraticProgrammingController.h
        ${DQROBOTICS_DIR}/include/dqrobotics/robot_control/DQ_ClassicQPController.h

        ${DQROBOTICS_DIR}/include/dqrobotics/robots/Ax18ManipulatorRobot.h
        ${DQROBOTICS_DIR}/include/dqrobotics/robots/BarrettWamArmRobot.h
        ${DQROBOTICS_DIR}/include/dqrobotics/robots/ComauSmartSixRobot.h
        ${DQROBOTICS_DIR}/include/dqrobotics/robots/KukaLw4Robot.h
        ${DQROBOTICS_DIR}/include/dqrobotics/robots/KukaYoubotRobot.h
        ${DQROBOTICS_DIR}/include/dqrobotics/robots/FrankaEmikaPandaRobot.h
)


set(DQROBOTICS_SOURCES
        ${DQROBOTICS_DIR}/src/DQ.cpp

        ${DQROBOTICS_DIR}/src/internal/_dq_linesegment.cpp

        ${DQROBOTICS_DIR}/src/utils/DQ_Geometry.cpp
        ${DQROBOTICS_DIR}/src/utils/DQ_LinearAlgebra.cpp
        ${DQROBOTICS_DIR}/src/utils/DQ_Math.cpp

        ${DQROBOTICS_DIR}/src/robot_modeling/DQ_CooperativeDualTaskSpace.cpp
        ${DQROBOTICS_DIR}/src/robot_modeling/DQ_Kinematics.cpp
        ${DQROBOTICS_DIR}/src/robot_modeling/DQ_SerialManipulator.cpp
        ${DQROBOTICS_DIR}/src/robot_modeling/DQ_SerialManipulatorDH.cpp
        ${DQROBOTICS_DIR}/src/robot_modeling/DQ_SerialManipulatorMDH.cpp
        ${DQROBOTICS_DIR}/src/robot_modeling/DQ_SerialManipulatorDenso.cpp
        ${DQROBOTICS_DIR}/src/robot_modeling/DQ_MobileBase.cpp
        ${DQROBOTICS_DIR}/src/robot_modeling/DQ_HolonomicBase.cpp
        ${DQROBOTICS_DIR}/src/robot_modeling/DQ_DifferentialDriveRobot.cpp
        ${DQROBOTICS_DIR}/src/robot_modeling/DQ_WholeBody.cpp
        ${DQROBOTICS_DIR}/src/robot_modeling/DQ_SerialWholeBody.cpp

        ${DQROBOTICS_DIR}/src/robot_control/DQ_KinematicController.cpp
        ${DQROBOTICS_DIR}/src/robot_control/DQ_PseudoinverseController.cpp
        ${DQROBOTICS_DIR}/src/robot_control/DQ_NumericalFilteredPseudoInverseController.cpp
        ${DQROBOTICS_DIR}/src/robot_control/DQ_KinematicConstrainedController.cpp
        ${DQROBOTICS_DIR}/src/robot_control/DQ_QuadraticProgrammingController.cpp
        ${DQROBOTICS_DIR}/src/robot_control/DQ_ClassicQPController.cpp

        ${DQROBOTICS_DIR}/src/robots/Ax18ManipulatorRobot.cpp
        ${DQROBOTICS_DIR}/src/robots/BarrettWamArmRobot.cpp
        ${DQROBOTICS_DIR}/src/robots/ComauSmartSixRobot.cpp
        ${DQROBOTICS_DIR}/src/robots/KukaLw4Robot.cpp
        ${DQROBOTICS_DIR}/src/robots/KukaYoubotRobot.cpp
        ${DQROBOTICS_DIR}/src/robots/FrankaEmikaPandaRobot.cpp
)

add_library(dqrobotics ${DQROBOTICS_HEADERS} ${DQROBOTICS_SOURCES})
include_directories(${DQROBOTICS_DIR}/include)
