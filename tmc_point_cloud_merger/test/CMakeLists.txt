# Tests
set(test_dependencies
  ${dependencies}
)

set(tests
  merge_trigger-test
  merger-test
  named_topic_map-test
  node-test
  subscriber-test
)

foreach(test ${tests})
  ament_add_gtest(${test}
    ${test}.cpp
  )
  target_include_directories(${test}
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
      $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(${test}
    ${test_dependencies}
  )
endforeach()
