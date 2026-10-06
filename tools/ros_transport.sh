#!/usr/bin/env bash
# Fast DDS 2.14 supports a UDP-only transport. Avoid shared-memory stream stalls
# observed when a real participant is suspended and resumed in fault tests.
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"
# 1280x960 RGB frames are 3.7 MB. The former 212992-byte default socket
# buffer lost large-message datagrams under the extra stress-image hop.
# Keep UDP and the existing freshness checks; enlarge per-participant sockets,
# within the verified 4 MiB kernel maxima, without changing host sysctls.
if [[ "$FASTDDS_BUILTIN_TRANSPORTS" == 'UDPv4' && -z "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" ]]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/fastdds_udp.xml"
fi
