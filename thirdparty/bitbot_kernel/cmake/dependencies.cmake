include(${CMAKE_CURRENT_LIST_DIR}/bitbot.cmake)

BitbotAddDependency(
  NAME           pugixml
  GIT_REPO       https://github.com/zeux/pugixml.git
  GIT_TAG        v1.14
  GIT_SHALLOW    TRUE
  FILE_NAME      "pugixml-1.14.zip"
  FILE_PATH      ${BITBOT_DEPENDENCY_LOCAL_FILE_PATH}
  USE_PROXY      ${BITBOT_DEPENDENCY_USE_PROXY}
  USE_LOCAL_FILE ${BITBOT_DEPENDENCY_USE_LOCAL_FILE}
)

BitbotAddDependency(
  NAME           readerwriterqueue
  GIT_REPO       https://github.com/cameron314/readerwriterqueue.git
  GIT_TAG        v1.0.6
  GIT_SHALLOW    TRUE
  FILE_NAME      "readerwriterqueue-1.0.6.zip"
  FILE_PATH      ${BITBOT_DEPENDENCY_LOCAL_FILE_PATH}
  USE_PROXY      ${BITBOT_DEPENDENCY_USE_PROXY}
  USE_LOCAL_FILE ${BITBOT_DEPENDENCY_USE_LOCAL_FILE}
)

BitbotAddDependency(
  NAME           spdlog
  GIT_REPO       https://github.com/gabime/spdlog.git
  GIT_TAG        v1.12.0
  GIT_SHALLOW    TRUE
  FILE_NAME      "spdlog-1.12.0.zip"
  OPTIONS        "SPDLOG_BUILD_SHARED YES"
  FILE_PATH      ${BITBOT_DEPENDENCY_LOCAL_FILE_PATH}
  USE_PROXY      ${BITBOT_DEPENDENCY_USE_PROXY}
  USE_LOCAL_FILE ${BITBOT_DEPENDENCY_USE_LOCAL_FILE}
  OVERRIDE_FIND_PACKAGE TRUE
)

BitbotAddDependency(
  NAME           concurrentqueue
  GIT_REPO       https://github.com/cameron314/concurrentqueue.git
  GIT_TAG        v1.0.4
  GIT_SHALLOW    TRUE
  FILE_NAME      "concurrentqueue-1.0.4.zip"
  FILE_PATH      ${BITBOT_DEPENDENCY_LOCAL_FILE_PATH}
  USE_PROXY      ${BITBOT_DEPENDENCY_USE_PROXY}
  USE_LOCAL_FILE ${BITBOT_DEPENDENCY_USE_LOCAL_FILE}
)

BitbotAddDependency(
  NAME           glaze
  GIT_REPO       https://github.com/stephenberry/glaze.git
  GIT_TAG        v2.5.2
  GIT_SHALLOW    TRUE
  FILE_NAME      "glaze-2.5.2.zip"
  FILE_PATH      ${BITBOT_DEPENDENCY_LOCAL_FILE_PATH}
  USE_PROXY      ${BITBOT_DEPENDENCY_USE_PROXY}
  USE_LOCAL_FILE ${BITBOT_DEPENDENCY_USE_LOCAL_FILE}
)

BitbotAddDependency(
  NAME           libuv
  GIT_REPO       https://github.com/libuv/libuv.git
  GIT_TAG        v1.47.0
  GIT_SHALLOW    TRUE
  OPTIONS        "LIBUV_BUILD_SHARED OFF" "LIBUV_BUILD_TESTS OFF" "LIBUV_BUILD_BENCH OFF"
  FILE_NAME      "libuv-1.47.0.zip"
  FILE_PATH      ${BITBOT_DEPENDENCY_LOCAL_FILE_PATH}
  USE_PROXY      ${BITBOT_DEPENDENCY_USE_PROXY}
  USE_LOCAL_FILE ${BITBOT_DEPENDENCY_USE_LOCAL_FILE}
)

BitbotAddDependency(
  NAME           zlib
  GIT_REPO       https://github.com/limymy/zlib.git
  GIT_TAG        master
  GIT_SHALLOW    TRUE
  OPTIONS        "ZLIB_BUILD_TESTS OFF" "ZLIB_BUILD_EXAMPLES OFF"
  FILE_NAME      "zlib-master.zip"
  FILE_PATH      ${BITBOT_DEPENDENCY_LOCAL_FILE_PATH}
  USE_PROXY      ${BITBOT_DEPENDENCY_USE_PROXY}
  USE_LOCAL_FILE ${BITBOT_DEPENDENCY_USE_LOCAL_FILE}
)

BitbotAddDependency(
  NAME           usockets
  URL            https://github.com/uNetworking/uSockets/archive/refs/tags/v0.8.6.zip
  FILE_NAME      "uSockets-0.8.6.zip"
  FILE_PATH      ${BITBOT_DEPENDENCY_LOCAL_FILE_PATH}
  USE_PROXY      ${BITBOT_DEPENDENCY_USE_PROXY}
  USE_LOCAL_FILE ${BITBOT_DEPENDENCY_USE_LOCAL_FILE}
)

BitbotAddDependency(
  NAME           uwebsockets
  URL            https://github.com/uNetworking/uWebSockets/archive/refs/tags/v20.48.0.zip
  FILE_NAME      "uWebSockets-20.48.0.zip"
  FILE_PATH      ${BITBOT_DEPENDENCY_LOCAL_FILE_PATH}
  USE_PROXY      ${BITBOT_DEPENDENCY_USE_PROXY}
  USE_LOCAL_FILE ${BITBOT_DEPENDENCY_USE_LOCAL_FILE}
)

# find_package(OpenSSL REQUIRED)

aux_source_directory(${usockets_SOURCE_DIR}/src/ USOCKETS_SRC_MAIN)
aux_source_directory(${usockets_SOURCE_DIR}/src/crypto USOCKETS_SRC_CRYPTO)
aux_source_directory(${usockets_SOURCE_DIR}/src/eventing USOCKETS_SRC_EVENTING)
set(USOCKETS_SRC 
    ${USOCKETS_SRC_MAIN}
    # ${USOCKETS_SRC_CRYPTO}
    ${USOCKETS_SRC_EVENTING}
)
add_library(usockets STATIC ${USOCKETS_SRC})
target_include_directories(usockets PRIVATE
  ${usockets_SOURCE_DIR}/src
  ${usockets_SOURCE_DIR}/src/internal
)
target_compile_features(usockets PUBLIC cxx_std_17)
target_compile_definitions(usockets PRIVATE
                          #  LIBUS_USE_OPENSSL
                          LIBUS_NO_SSL
)
target_link_libraries(usockets PUBLIC
  # OpenSSL::SSL
  zlibstatic
  uv_a
)
add_library(uwebsockets INTERFACE)
target_include_directories(uwebsockets INTERFACE
  ${uwebsockets_SOURCE_DIR}/src
  ${usockets_SOURCE_DIR}/src
)
