build_lib_example(
  NAME mpquic-scale-across
  SOURCE_FILES mpquic-scale-across.cc
  LIBRARIES_TO_LINK
    ${libquic}
    ${libapplications}
    ${libinternet}
    ${libpoint-to-point}
)
# Append this block to the existing src/quic/examples/CMakeLists.txt.
build_lib_example(
  NAME mpquic-dci
  SOURCE_FILES mpquic-dci.cc
  LIBRARIES_TO_LINK ${libquic} ${libapplications} ${libinternet} ${libpoint-to-point}
)
