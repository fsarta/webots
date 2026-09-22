#!/bin/bash
# install open62541 (native OPC-UA stack) for the Webots OPC-UA module.
# Builds a static library plus the amalgamated header into:
#   $WEBOTS_HOME/include/open62541/open62541.h
#   $WEBOTS_HOME/lib/webots/libopen62541.a
# When this script is not run, Webots still builds and the OPC-UA module falls
# back to the in-memory mock backend.

set -e

WEBOTS_HOME=${WEBOTS_HOME:-$(cd "$(dirname "$0")/../.." && pwd)}
OPEN62541_VERSION=${OPEN62541_VERSION:-1.4.6}
BUILD_DIR="$WEBOTS_HOME/build/open62541-$OPEN62541_VERSION"
INSTALL_INCLUDE="$WEBOTS_HOME/include/open62541"
INSTALL_LIB="$WEBOTS_HOME/lib/webots"

echo "# installing open62541 $OPEN62541_VERSION"

mkdir -p "$WEBOTS_HOME/build"
if [ ! -d "$BUILD_DIR" ]; then
  git clone --depth 1 --branch v$OPEN62541_VERSION https://github.com/open62541/open62541.git "$BUILD_DIR"
fi

mkdir -p "$BUILD_DIR/build"
cd "$BUILD_DIR/build"
cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DUA_ENABLE_AMALGAMATION=ON \
  -DUA_ENABLE_SUBSCRIPTIONS=ON \
  -DUA_ENABLE_ENCRYPTION=OFF \
  -DBUILD_SHARED_LIBS=OFF \
  -DUA_LOGLEVEL=300
make -j"$(nproc 2>/dev/null || echo 2)"

mkdir -p "$INSTALL_INCLUDE" "$INSTALL_LIB"
cp open62541.h "$INSTALL_INCLUDE/"
cp open62541.c "$INSTALL_INCLUDE/" 2>/dev/null || true
cp libopen62541.a "$INSTALL_LIB/"

echo "# open62541 installed into $INSTALL_INCLUDE and $INSTALL_LIB"
