cmake_minimum_required(VERSION 2.8.3)
project(nao_matador)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        rospy
        naoqi_bridge_msgs
        std_srvs
        tf
        message_generation
        sensor_msgs
        genmsg
        actionlib
        actionlib_msgs
        geometry_msgs
        cv_bridge
        image_transport
        aruco_ros
        )

find_package(Boost REQUIRED COMPONENTS system thread)
include_directories(${Boost_INCLUDE_DIRS})

add_action_files(
        FILES
        Blink.action
        SetSpeechVocabulary.action
        SpeechWithFeedback.action
)

add_message_files(
  FILES
  Detection_msg.msg
)

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

add_service_files(
        DIRECTORY srv
        FILES
        Walking_msgs.srv
        Led_msgs.srv
        Motion_msgs.srv
        Speaking_msgs.srv
        Audio_msgs.srv
)

generate_messages(
        DEPENDENCIES
        actionlib_msgs
        std_msgs
        )

catkin_package(CATKIN_DEPENDS message_runtime actionlib_msgs actionlib)
include_directories(${catkin_INCLUDE_DIRS})

catkin_install_python(PROGRAMS script/motion_server.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

catkin_install_python(PROGRAMS ./script/corazon.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

add_executable(module_vision src/vision.cpp)
add_dependencies(module_vision ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(module_vision
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES})

add_executable(module_walking src/walking.cpp)
add_dependencies(module_walking ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(module_walking
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES})

add_executable(module_leds src/leds.cpp)
add_dependencies(module_leds ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(module_leds
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES})

add_executable(module_bumpers src/bumpers.cpp)
add_dependencies(module_bumpers ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(module_bumpers
        ${catkin_LIBRARIES})

add_executable(module_speech src/speech.cpp)
add_dependencies(module_speech ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(module_speech
        ${catkin_LIBRARIES})

add_executable(test_cam src/test_cam.cpp)
add_dependencies(test_cam ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_cam
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES})