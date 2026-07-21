cmake_minimum_required(VERSION 3.16)
project(kinova_lowlevel LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# =============================================================================
# Kortex SDK path
#
# Default matches Sahil's lab workstation layout.
# Override:  cmake .. -DKORTEX_DIR=/path/to/kortex_api
# =============================================================================
set(KORTEX_DIR "$ENV{HOME}/kinova_learning/kortex/api_cpp/examples/kortex_api"
    CACHE PATH "Root of the Kortex C++ API SDK")

if(NOT EXISTS "${KORTEX_DIR}/include")
    message(FATAL_ERROR
        "Kortex SDK not found at ${KORTEX_DIR}/include\n"
        "Set -DKORTEX_DIR=/your/path  (ls the directory to verify structure)")
endif()

# --- SDK includes (SYSTEM = suppress warnings from SDK headers) ---
# The Kortex SDK triggers deprecated-declarations and anonymous-struct
# warnings on GCC 13 — these are inside their code, not ours.

# --- SDK libraries ---
link_directories(${KORTEX_DIR}/lib/release)

# =============================================================================
# Targets
# =============================================================================

# NOTE: No USE_KORTEX_MOCK option here.
# This binary is hardware-only — the low-level cyclic API requires real UDP
# transport, which the mock doesn't have. If you need to compile-check
# on a machine without the SDK, add a mock guard later.

add_executable(low_level_feedback_loop
    src/low_level_feedback_loop.cpp
)

target_include_directories(low_level_feedback_loop SYSTEM PRIVATE
    ${KORTEX_DIR}/include
    ${KORTEX_DIR}/include/client
    ${KORTEX_DIR}/include/client_stubs
    ${KORTEX_DIR}/include/common
    ${KORTEX_DIR}/include/messages
    ${KORTEX_DIR}/include/google
)

target_link_libraries(low_level_feedback_loop
    KortexApiCpp
    pthread
)

target_compile_options(low_level_feedback_loop PRIVATE
    -Wall -Wextra
)
