#!/bin/sh

set -x

MYDIR=`dirname $0`
export ROOT=`readlink -f $MYDIR/../`

export WEVAL=$ROOT/../weval/target/release/weval
export WEVAL_SHELL=$ROOT/obj-weval-release/dist/bin/js
export WEVAL_PRECOMPILED=`mktemp`
export IC_PATH=`mktemp -d`

$WEVAL precompile -o $WEVAL_PRECOMPILED -i $WEVAL_SHELL


echo "Collecting ICs in $IC_PATH"

$ROOT/js/src/jit-test/jit_test.py --no-xdr -f $MYDIR/ic-collector-shell
