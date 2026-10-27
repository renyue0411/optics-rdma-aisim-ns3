build_lib_example(
  NAME rdma-dci-tunnel
  SOURCE_FILES rdma-dci-tunnel.cc
  LIBRARIES_TO_LINK ${libdci-tunnel} ${libinternet} ${libpoint-to-point} ${libapplications}
)
