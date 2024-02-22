# stb
add_library(stb INTERFACE)
target_include_directories(stb INTERFACE stb)

# hmm
add_library(hmm INTERFACE)
target_include_directories(hmm INTERFACE HandmadeMath)

# parallel-util
add_library(parallel-util INTERFACE)
target_include_directories(parallel-util INTERFACE parallel-util/include)

