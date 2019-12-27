#!/bin/bash

set -euo pipefail

case "${1:-}" in
  ("homebrew")
    brew install cmake gmp matplotlib numpy python scipy
    pip2 install ipython==5.4.1 jupyter nose
    ;;
  ("macports")
    port install cmake gmp py27-ipython py27-jupyter py27-matplotlib \
      py27-nose py27-numpy py27-scipy python27
    ;;
  ("ubuntu")
    apt-get install --no-install-recommends cmake g++ gcc git libgmp-dev make \
      python python-matplotlib python-nose python-numpy python-pip python-scipy
    pip install ipython==5.4.1 jupyter
    ;;
  (*)
    echo "Usage: $0 <package_manager>" 1>&2
    echo "where <package_manager> is one of the following:" 1>&2
    echo "  homebrew" 1>&2
    echo "  macports" 1>&2
    echo "  ubuntu" 1>&2
    exit 1
    ;;
esac
