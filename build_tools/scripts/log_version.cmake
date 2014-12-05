# Log current repository version/status
# Try Git
execute_process(COMMAND git rev-parse HEAD
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    OUTPUT_VARIABLE version_rev_string
    OUTPUT_STRIP_TRAILING_WHITESPACE
    RESULT_VARIABLE _not_git)
execute_process(COMMAND git status --porcelain
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    OUTPUT_VARIABLE version_status_string
    OUTPUT_STRIP_TRAILING_WHITESPACE)
if(_not_git)
    #Try SVN
    execute_process(COMMAND svnversion
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        OUTPUT_VARIABLE version_rev_string
        OUTPUT_STRIP_TRAILING_WHITEPSACE
        RESULT_VARIABLE _not_svn)
    execute_process(COMMAND svn status
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        OUTPUT_VARIABLE version_status_string)
endif(_not_git)
message(STATUS "Revision: ${version_rev_string}")
message(STATUS "Repository Status:\n${version_status_string}")
message(STATUS "End Version Info")
