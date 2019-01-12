cmake_minimum_required(VERSION 2.8.0 FATAL_ERROR)

project(VATNAO)

INCLUDE_DIRECTORIES("${CMAKE_CURRENT_BINARY_DIR}")

INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${CTC_DIR}/libnaoqi/include)
INCLUDE_DIRECTORIES(${CTC_DIR}/zlib/include)
INCLUDE_DIRECTORIES(${CTC_DIR}/../sysroot_legacy/usr/include)

SET(CPP_FILES
  main.cpp
  options.cpp
  ui/ui.cpp
  ui/vatnaoManager.cpp
  ui/frameTools/readImage.cpp
  ui/frameTools/readBoundingBoxes.cpp
  ui/uiElements/imageView.cpp
  ui/uiElements/debugFrame.cpp
  ui/uiElements/option.cpp
  app/appAdaptor.cpp
  app/infoGeneration/generateFrameInfo.cpp
  app/world/dumpParser.cpp
  app/world/world.cpp
  app/ImageMethods/RgbImg.cpp
  app/VatnaoDebugMiddleware/VatnaoDebugMiddleware.cpp
  app/VatnaoDebugMiddleware/OverlayPainter.cpp
)

SET(QT4_MOC
    ui/vatnaoManager.hpp
    ui/uiElements/imageView.hpp
    ui/uiElements/option.hpp
    ui/uiElements/debugFrame.hpp
)

QT4_WRAP_CPP(QT4_WRAPPED_MOC ${QT4_MOC})

add_executable(vatnao.bin ${CPP_FILES} ${QT4_WRAPPED_MOC})

set_source_files_properties(
    ${CPP_FILES}
    PROPERTIES COMPILE_FLAGS "-I${QWT_INCLUDE_DIR}"
)

find_library ( QGLVIEWER_LIBRARY NAMES QGLViewer qglviewer-qt4 )
find_package ( OpenGL  REQUIRED )
find_package ( PNG     REQUIRED )

TARGET_LINK_LIBRARIES(
  vatnao.bin
  ${QT_LIBRARIES}
  ${QWT_LIBRARY}
  ${QGLVIEWER_LIBRARY}
  ${Boost_IOSTREAMS_LIBRARY}
  soccer
  ${OPENGL_gl_LIBRARY}
  ${OPENGL_glu_LIBRARY}
  ${PNG_LIBRARIES}
  ${CTC_DIR}/bzip2/lib/libbz2.so
  ${CTC_DIR}/../libuuid.so.1.3.0
)

set_target_properties(
  vatnao.bin
  PROPERTIES
  BUILD_WITH_INSTALL_RPATH FALSE
  INSTALL_RPATH ""
  INSTALL_RPATH_USE_LINK_PATH FALSE
  SKIP_BUILD_RPATH FALSE
)

configure_file(vatnao-wrapper.sh vatnao @ONLY)
configure_file(gdb-vatnao-wrapper.sh gdbvatnao @ONLY)
