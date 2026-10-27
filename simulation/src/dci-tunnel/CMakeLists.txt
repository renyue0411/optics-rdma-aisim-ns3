build_lib(
  LIBNAME dci-tunnel
  SOURCE_FILES model/dci-tunnel.cc model/dci-path-manager.cc
  HEADER_FILES model/dci-tunnel.h model/dci-path-manager.h
  LIBRARIES_TO_LINK ${libinternet} ${libpoint-to-point} ${libapplications}
)
