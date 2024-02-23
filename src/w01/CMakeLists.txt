add_executable(w01 main.cpp)
target_link_libraries(w01 PRIVATE stb hmm)

add_executable(w01mt main-mt.cpp)
target_link_libraries(w01mt PRIVATE stb hmm parallel-util)
