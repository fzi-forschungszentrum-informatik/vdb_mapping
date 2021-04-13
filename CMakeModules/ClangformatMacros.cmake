# ClangformatMacros
# -----------------
#
# Macros and functions to support clang format of the source tree.
#
# add_format_source( <scope>
#                    FILES <srcs>...
#                    PATTERN <patterns>... [RECURSIVE])
#
# ::
#
#   <srcs>  - list of source files to be formatted
#   <pattern> - list of patterns to file(GLOB_RECURSE / GLOB)
#   [RECURSIVE] - whenever a pattern should extend recursive through the filesystem tree
#
# Add a list of files or files selected by pattern GLOB to the list of formating
# source files for the scope <scope>.
# If RECURSIVE is specified all patterns are interpreted as GLOB_RECURSE statements.
# For example:
# add_format_source(tmpl PATTERN "include/tmpl/[a-zA-Z]*.h" RECURSIVE)
# Will add all header files in include/tmpl and its subdirectories.
#
# If a scope already exists the files will get appended. It is therefore legal to call
# add_fromat_sources several times with the same scope name.
#
# Note that the scope is an arbitraty string name which is uses as target name in the
# created make files. You could chooes a random name as long as there are no conflicts,
# But it is recommended to use the "project_subproject" approach.
# For example:
# In a project tmpl the main scope will be "tmpl".
# A subdirectory containing all plot executables may have the scope "tmpl_plots".
#
# To add a the actual pseude target, call ADD_CLANGFORMAT_TARGET( <scope> ) after
# all calles to add_fromat_source.
#
# ADD_CLANGFORMAT_TARGET( <scope> )
#
# ::
#
#   <scope> - the name of the scope to be added to the format target
#
# Adds the target "format" to the build system. This will apply clang format on all files
# added with the above mentioned functions.
#
# All scopes specified above in add_format_source will be available as additional targets.
# For example:
# In the project with scope "tmpl" and "tmpl_plots" there are three additional targets:
# "format" will call clangformat on all files specified by add_format_source()
# "format_tmpl" will call clangformat on all files specified by add_format_source(tmpl ...)
# "format_tmpl_plots" will call clangformat on all files specified by add_format_source(tmpl_plots ...)
#
# Note that there is no hierachical dependency between scopes, this emans the scope "tmpl" does
# NOT include the contents of scope "tmpl_plots". 
#

function(add_format_source scope)

  include(CMakeParseArguments)
  set(options RECURSIVE)
  set(multiValueArgs FILES PATTERN)
  cmake_parse_arguments(ARGS "${options}" "" "${multiValueArgs}" ${ARGN})

  if (ARGS_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "Unrecognized arguments: \"${ARGS_UNPARSED_ARGUMENTS}\"")
  endif()

  # resolve patterns if specified
  foreach(s IN LISTS ARGS_PATTERN)
    if (${ARGS_RECURSIVE})
      file(GLOB_RECURSE sources "${s}")
    else()
      file(GLOB sources "${s}")
    endif()
    list(APPEND ARGS_FILES ${sources})
  endforeach()

  get_property(is_defined GLOBAL PROPERTY SRCS_FRMT_LIST_${scope} DEFINED)
  if(NOT is_defined)
    define_property(GLOBAL PROPERTY SRCS_FRMT_LIST_${scope}
      BRIEF_DOCS "List of source files to be formated"
      FULL_DOCS "List of all source files to be formated for the scope of ${scope}")
  endif()
  # make absolute paths
  set(sources)
  foreach(s IN LISTS ARGS_FILES)
    if(NOT IS_ABSOLUTE "${s}")
      get_filename_component(s "${s}" ABSOLUTE)
    endif()
    list(APPEND sources "${s}")
  endforeach()
  # append to global list
  set_property(GLOBAL APPEND PROPERTY SRCS_FRMT_LIST_${scope} "${sources}")
endfunction(add_format_source)

macro (ADD_CLANGFORMAT_TARGET _SCOPE)
  # make sure the global target is available
  if (NOT TARGET format)
    add_custom_target(format
      COMMENT "Running clang-format"
      VERBATIM)
  endif()

  # add the scoped target as dependency to the global format target
  get_property(all_sources GLOBAL PROPERTY "SRCS_FRMT_LIST_${_SCOPE}")
  if (NOT all_sources)
    message(FATAL_ERROR "Empty source list to format for scope ${_SCOPE}!. Did you forget to add_format_source(${_SCOPE} ...)?")
  endif()

  # find clang-format
  # Note: Try to find clangformat v6.0 first - thus we pick a "stable"
  # version even if other versiona are available
  find_program(CLANG_FORMAT_EXE
    NAMES "clang-format-6.0" "clang-format"
    DOC "Path to clang-format executable, if found.")
  if(NOT  CLANG_FORMAT_EXE)
    message(WARNING "clang-format not found. Please install a current version of clang.")
  endif()

  if(CLANG_FORMAT_EXE)

    add_custom_target(format_${_SCOPE}
      COMMAND ${CLANG_FORMAT_EXE} --style=file -i ${all_sources}
      COMMENT "Running clang-format for ${_SCOPE}"
      VERBATIM)
    add_dependencies(format format_${_SCOPE})

  endif()
endmacro()
