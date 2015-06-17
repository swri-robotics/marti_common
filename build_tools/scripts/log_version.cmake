# Log current repository version/status
# Try Git
execute_process(COMMAND git rev-parse HEAD
    OUTPUT_VARIABLE version_rev_string
    OUTPUT_STRIP_TRAILING_WHITESPACE
    RESULT_VARIABLE _not_git)
execute_process(COMMAND git status --porcelain
    OUTPUT_VARIABLE version_status_string
    OUTPUT_STRIP_TRAILING_WHITESPACE)
if(_not_git)
    #Try SVN
    execute_process(COMMAND svnversion
        OUTPUT_VARIABLE version_rev_string
        OUTPUT_STRIP_TRAILING_WHITESPACE
        RESULT_VARIABLE _not_svn)
    execute_process(COMMAND svn status
        OUTPUT_VARIABLE version_status_string)
endif(_not_git)
message(STATUS "Revision: ${version_rev_string}")
message(STATUS "Repository Status:\n${version_status_string}")
message(STATUS "End Version Info")
