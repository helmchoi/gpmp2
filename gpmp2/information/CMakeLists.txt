# Install headers
file(GLOB information_headers "*.h")
install(FILES ${information_headers} DESTINATION include/gpmp2/information)

# Build tests -> not implemented
# gtsamAddTestsGlob(obstacle "tests/*.cpp" "" ${PROJECT_NAME})
