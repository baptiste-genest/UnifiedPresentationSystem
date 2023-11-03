if (TARGET geometry-central)
  return()
endif()

CPMAddPackage(
  NAME geometry-central
  GIT_REPOSITORY https://github.com/nmwsharp/geometry-central.git
  GIT_SHALLOW    TRUE
)