#
# Copyright 2022, University of New South Wales
#
# SPDX-
#

set(LibLwip ON CACHE BOOL "" FORCE)
set(LibEthdriverNumPreallocatedBuffers 32 CACHE STRING "" FORCE)

set(cpp_define -DKernelArchArm)

set(LibEthdriverRXDescCount 256 CACHE STRING "" FORCE)
set(LibEthdriverTXDescCount 256 CACHE STRING "" FORCE)
set(CAmkESNoFPUByDefault ON CACHE BOOL "" FORCE)
