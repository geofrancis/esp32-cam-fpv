# This file was automatically generated for projects
# without default 'CMakeLists.txt' file.

set(srcs 
    main.cpp queue.cpp wifi.cpp nvs_args.cpp)

idf_component_register(SRCS "file_server.cpp" "wifi.cpp" "main.cpp" "${srcs}"
                    INCLUDE_DIRS "../components/esp32-camera/target/private_include/"  EMBED_FILES "favicon.ico" "index.html")

