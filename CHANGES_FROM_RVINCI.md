These are the changes made from original RVinci repository to compile their code:
* add this before project(...)
"
if (POLICY CMP0048)
  cmake_policy(SET CMP0048 NEW)
endif (POLICY CMP0048)
"

* comment out "include(${QT_USE_FILE})"

* edit find_packages for Qt to
"
find_package(Qt5 COMPONENTS Core Gui Widgets REQUIRED)
"

* for find_packages for Catkin, make sure it has
"
rvinci_input_msg
interaction_cursor_msgs
"

* change qt4_wrap_cpp(...) to qt5_wrap_cpp(...)

* make sure to have interaction_cursor_3d built