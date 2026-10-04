#!/usr/bin/env bash
# Fast DDS 2.14 supports a UDP-only transport. Avoid shared-memory stream stalls
# observed when a real participant is suspended and resumed in fault tests.
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"
