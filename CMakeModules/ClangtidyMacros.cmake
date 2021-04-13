# ClangtidyMacros
# -----------------
#
# Macros and functions to support clangtidy of the source tree.
#
# ADD_CLANGTIDY_TARGET( <scope> )
#
# ::
#
#   <scope> - the name of the scope to be added to the format target
#
# Adds the target "tidy" and "tidy-fix" to the build system. This will apply clangtidy on all files
# added with the clang format macros.
#
# This is an additional macro to use the clang format marked files to generate targets for
# clangtidy.
# The same rules as for ADD_CLANGFORMAT_TARGET( <scope> ) appleis here: 
# All scopes specified in add_format_source will be available as additional targets.
# For example:
# In the project with scope "tmpl" and "tmpl_plots" there are six additional targets:
# "tidy" and "tidy-fix" will call clangtidy on all files specified by add_format_source()
# "tidy_tmpl" and "tidy-fix_tmpl" will call clangtidy on all files specified by add_format_source(tmpl ...)
# and so on ...
#
# Note that there is no hierachical dependency between scopes, this emans the scope "tmpl" does
# NOT include the contents of scope "tmpl_plots". 
#

macro (ADD_CLANGTIDY_TARGET _SCOPE)
  # make sure compile_commands are available
  # this may also be specified by ADD_USEFULL_PSEUDO_TARGETS()
  if (NOT TARGET compile_commands)
    add_custom_target(compile_commands
      COMMAND ${CMAKE_COMMAND} -DCMAKE_EXPORT_COMPILE_COMMANDS=1 ${CMAKE_SOURCE_DIR}
      COMMENT "Creating compile_commands.json"
      VERBATIM)
  endif()

  # make sure the global target is available
  if (NOT TARGET tidy)
    add_custom_target(tidy
      COMMENT "Running clang-tidy"
      VERBATIM)
    add_dependencies(tidy compile_commands)
  endif()
  
  if (NOT TARGET tidy-fix)
    add_custom_target(tidy-fix
      COMMENT "Running clang-tidy in fixit mode"
      VERBATIM)
    add_dependencies(tidy-fix compile_commands)
  endif()

  # find clang-tidy
  # Note: Try to find clangtidy v6.0 first - thus we pick a "stable"
  # version even if other versiona are available
  find_program(CLANG_TIDY_EXE
    NAMES "clang-tidy-6.0" "clang-tidy"
    DOC "Path to clang-tidy executable, if found.")
  if(NOT  CLANG_TIDY_EXE)
    message(WARNING "clang-tidy not found. Please install a current version of clang.")
  endif()

  if(CLANG_TIDY_EXE)

    add_custom_target(tidy_${_SCOPE}
      COMMENT "Running clang-tidy for ${_SCOPE}"
      VERBATIM)
    add_custom_target(tidy-fix_${_SCOPE}
      COMMENT "Running clang-tidy in fixit mode for ${_SCOPE}"
      VERBATIM)

    get_property(all_sources GLOBAL PROPERTY "SRCS_FRMT_LIST_${_SCOPE}")
    set(cpp_sources)
    foreach(s IN LISTS all_sources)
      if("${s}" MATCHES ".cpp$")
	get_filename_component(name "${s}" NAME)

	add_custom_target(tidy_${_SCOPE}_${name}
	  COMMAND ${CLANG_TIDY_EXE} -p ${CMAKE_BINARY_DIR} -header-filter=^${CMAKE_SOURCE_DIR}/.* ${s}
	  COMMENT "Running clang-tidy for ${s}"
	  VERBATIM)
	add_dependencies(tidy_${_SCOPE} tidy_${_SCOPE}_${name})
	add_dependencies(tidy_${_SCOPE}_${name} compile_commands)

	add_custom_target(tidy-fix_${_SCOPE}_${name}
	  COMMAND ${CLANG_TIDY_EXE} -p ${CMAKE_BINARY_DIR} -header-filter=^${CMAKE_SOURCE_DIR}/.* -fix ${s}
	  COMMENT "Running clang-tidy in fixit mode for ${s}"
	  VERBATIM)
	add_dependencies(tidy-fix_${_SCOPE} tidy-fix_${_SCOPE}_${name})
	add_dependencies(tidy-fix_${_SCOPE}_${name} compile_commands)
      endif()
    endforeach()
    
    add_dependencies(tidy tidy_${_SCOPE})
    add_dependencies(tidy-fix tidy-fix_${_SCOPE})
  endif()
endmacro()
