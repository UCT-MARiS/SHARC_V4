# Add the Google Test directory

# Ensure no conflicting definitions
unset(GTEST_HAS_PTHREAD CACHE)
unset(GTEST_HAS_MUTEX_AND_THREAD_LOCAL CACHE)

# Add compiler definitions to disable std::mutex and pthread in googletest
add_compile_definitions(GTEST_HAS_PTHREAD=0 GTEST_HAS_MUTEX_AND_THREAD_LOCAL=0)

add_subdirectory(googletest)