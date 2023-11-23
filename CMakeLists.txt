cmake_minimum_required(VERSION 3.10)

project(index3d)


add_executable(compile_mesh src/compile_mesh.cxx)

set_property(TARGET compile_mesh PROPERTY CXX_STANDARD 23)
set_property(TARGET compile_mesh PROPERTY CXX_EXTENSIONS OFF)
set_property(TARGET compile_mesh PROPERTY CXX_STANDARD_REQUIRED ON)
