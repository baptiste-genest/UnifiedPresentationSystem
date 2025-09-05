if (TARGET slope)
  return()
endif()

set(CMAKE_CXX_FLAGS_DEBUG_OLD "${CMAKE_CXX_FLAGS_DEBUG}")
set(CMAKE_CXX_FLAGS_DEBUG "-w")

CPMAddPackage(
  NAME slope
  GITHUB_REPOSITORY "baptiste-genest/UnifiedPresentationSystem"
)

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG_OLD}")

