# This file was automatically generated for projects
# without default 'CMakeLists.txt' file.

set(srcs 
    main.cpp queue.cpp wifi.cpp)

idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS "../components/esp32-camera/target/private_include/")