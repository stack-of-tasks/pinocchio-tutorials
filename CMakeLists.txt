CMAKE_MINIMUM_REQUIRED(VERSION 2.8 FATAL_ERROR)
PROJECT(pinocchio_tutos)

FIND_PROGRAM(ASCIIDOC NAMES asciidoc)
set(ASCIIDOC ${ASCIIDOC} --filter latex -a toc -a icons -o)
FIND_PROGRAM(ZIP NAMES zip)
#set(ZIP "${ZIP} -j")
MESSAGE(STATUS ZIP "--${ZIP}--")
# --- ASCIIDOC RULES ---------------------------------------------------------------
MACRO(ADD_ASCIIDOC_TARGET asciidocfile htmlfile)
  # Create a "CUSTOM_COMMAND" to create the htmlfile from the asciidoc file.
  add_custom_command(OUTPUT ${htmlfile}
    COMMAND ${ASCIIDOC}  ${htmlfile} ${asciidocfile} 
    DEPENDS ${asciidocfile}
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMENT "Compiling ASCIIDOC  ${asciidocfile}"
    VERBATIM)
  get_filename_component(targetname ${htmlfile} NAME_WE)
  add_custom_target(${targetname} DEPENDS ${htmlfile})
  MESSAGE(STATUS "Add target ${targetname}")
ENDMACRO(ADD_ASCIIDOC_TARGET)

MACRO(ASCIIDOC_FROM_PYTHON_TARGET pythonfile asciidocfile)
  # Create a "CUSTOM_COMMAND" to create a asciidoc file from a script file.
  get_filename_component(pythonshort ${pythonfile} NAME)
  set(SED_COMMAND     
    -e 1s@^@The\ script\ below\ can\ be\ found\ in\ link:${pythonfile}[this\ file].\\n[source,python]\\n----\\n@
    -e \$s@\$@\\n----\\n@)
  add_custom_command(OUTPUT ${asciidocfile}
    COMMAND sed  ${SED_COMMAND}  ${pythonfile} > ${asciidocfile}
    DEPENDS ${pythonfile}
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMENT "Building ASCIIDOC from ${pythonfile}"
    VERBATIM)
  MESSAGE(STATUS "Add target ${pythonfile}")
ENDMACRO(ASCIIDOC_FROM_PYTHON_TARGET)

MACRO(TUTO_NEW id name)
  # Set the variables to define a new tutorial.
  # <id> is typically a number, <name> a string, defining the directory in source_dir tp<id>_<name>
  set(fullid "tp${id}")
  set(dirtargets ${dirtargets} ${fullid})
  set(${fullid}_name ${name})
  set(${fullid}_dir "${fullid}_${name}")
  set(${fullid}_secundaries)
  set(${fullid}_pythons)
ENDMACRO(TUTO_NEW)

MACRO(TUTO_ADD_PYTHON id pythons)
  # Add a list of python to the tutorial <id>. <asciidocs> is a list of files with relative
  # path from source_dir/tp<id>/.
  set(fullid "tp${id}")
  LIST(APPEND ${fullid}_pythons ${pythons})
ENDMACRO(TUTO_ADD_PYTHON)

MACRO(TUTO_ADD_ASCIIDOC id asciidocs)
  # Add a list of asciidoc to the tutorial <id>. <asciidocs> is a list of files with relative
  # path from source_dir/tp<id>/.
  set(fullid "tp${id}")
  LIST(APPEND ${fullid}_secundaries ${asciidocs})
ENDMACRO(TUTO_ADD_ASCIIDOC)

# -----------------------------------------------------------------------------------
set(dirtargets)
OPTION (WITH_SOLUTION "Copy and install solution scripts" OFF)

tuto_new(0 install)
tuto_add_python(0 "viewer_unittest.py;install.sh")

tuto_new(1 directgeom)
if(WITH_SOLUTION)
  tuto_add_python(1 tp1.py)
endif(WITH_SOLUTION)

tuto_new(2 invgeom)
tuto_add_python(2 ur5x4.py)
if(WITH_SOLUTION)
  tuto_add_python(2
    "q10_parallelik.py;q1q2q3_optim_translation.py;q4q5_optim_redundancy.py;q6_optim_placement.py;q7q8_optim_quaternion.py")
endif(WITH_SOLUTION)

tuto_new(3 invkine)
tuto_add_python(3 "mobilerobot.py;example_mobilerobot.py")
if(WITH_SOLUTION)
  tuto_add_python(3 tp3.py)
endif(WITH_SOLUTION)

tuto_new(4 dyn)
tuto_add_python(4 "display.py;example_hand.py;example_qp.py;robot_hand.py")
if(WITH_SOLUTION)
  tuto_add_python(4 "q1q2q3_freesimu.py;q4q5_contactsimu.py")
endif(WITH_SOLUTION)


tuto_new(5 planner)
tuto_add_python(5 "example_collision.py;graph.py;prm_display.py")
if(WITH_SOLUTION)
  tuto_add_python(5 "planner.py;")
endif(WITH_SOLUTION)

tuto_new(6 wpg)
tuto_add_python(6 "factor.py;foot_steps.py;example_factor.py;example_foot_steps.py;example_matplotlib.py;example_romeo.py;example_scipy.py")

tuto_new(7 learn)
tuto_add_python(7 "display.py;dpendulum.py;pendulum.py")
if(WITH_SOLUTION)
  tuto_add_python(7 "continuous.py;ocp.py;qnet.py;qtable.py")
endif(WITH_SOLUTION)


# -----------------------------------------------------------------------------------
set(dest ${CMAKE_BINARY_DIR}/html)
file(MAKE_DIRECTORY ${dest})
set(destpy ${dest}/python)
file(MAKE_DIRECTORY ${destpy})

set(alldeps)
foreach(dirname ${dirtargets})
  # --- Compile main.asciidoc
  add_asciidoc_target(${CMAKE_SOURCE_DIR}/${${dirname}_dir}/main.asciidoc ${dest}/${dirname}.html)
  set(dirdeps ${dest}/${dirname}.html)
  # --- Compile secundary asciidoc files.
  foreach(secname ${${dirname}_secundaries})
    LIST(APPEND dirdeps ${dest}/${dirname}_${secname}.html)
    add_asciidoc_target(
      ${CMAKE_SOURCE_DIR}/${${dirname}_dir}/${secname}.asciidoc
      ${dest}/${dirname}_${secname}.html
      )
  endforeach(secname)
  # --- Add python files
  set(pydeps)
  foreach(pyname ${${dirname}_pythons})
    get_filename_component(basename ${pyname} NAME_WE)
    get_filename_component(extname ${pyname} EXT)
    string(SUBSTRING ${extname} 1 -1 extname)
    set(shortname ${dirname}_${basename}_${extname})
    file(MAKE_DIRECTORY ${destpy}/${dirname})
    add_custom_command(
      OUTPUT ${destpy}/${dirname}/${pyname}
      COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_SOURCE_DIR}/${${dirname}_dir}/${pyname} 
                                        ${destpy}/${dirname}/${pyname}
      DEPENDS ${CMAKE_SOURCE_DIR}/${${dirname}_dir}/${pyname}                                     
      COMMENT "Copy ${pyname}")
    LIST(APPEND dirdeps ${destpy}/${dirname}/${pyname})
    LIST(APPEND pydeps ${destpy}/${dirname}/${pyname})
    asciidoc_from_python_target(${destpy}/${dirname}/${pyname} ${shortname}.asciidoc )
    add_asciidoc_target(${shortname}.asciidoc ${dest}/${shortname}.html)
    LIST(APPEND dirdeps ${dest}/${shortname}.html)
  endforeach(pyname)
  # Add rule tpXX_ to compile all the directory.
  if(${dirname}_pythons)
    add_custom_command(
      OUTPUT ${dest}/${dirname}.zip
      COMMAND ${ZIP} -rj ${dest}/${dirname}.zip ${destpy}/${dirname}/
      DEPENDS ${dirdeps}
      COMMENT "Zip rule for ${dest}/${dirname}.zip ${ZIP} ${dest}/${dirname}.zip ${destpy}/${dirname}/")
    LIST(APPEND dirdeps ${dest}/${dirname}.zip)
  endif()
  add_custom_target(${dirname}_ DEPENDS ${dirdeps})
  LIST(APPEND alldeps ${dirname}_)
endforeach(dirname)

add_custom_target(tutos ALL DEPENDS ${alldeps})
