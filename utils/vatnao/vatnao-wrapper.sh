#!/bin/bash
CTC_DIR=@CTC_DIR@
if [[ -n "$CTC_DIR" ]]; then
  export LD_LIBRARY_PATH="$CTC_DIR/../boost_libs"
fi
`dirname $0`/vatnao.bin "$@"
