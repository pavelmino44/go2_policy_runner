#!/usr/bin/env bash
# Copyright (c) 2026, unitree-go2-rl contributors.
# SPDX-License-Identifier: Apache-2.0
# Download the ONNX Runtime CPU release for this machine (x64 or aarch64, e.g. Go2 EDU Jetson)
# into <package>/thirdparty/. CMake picks it up automatically.
#
#   ./scripts/fetch_onnxruntime.sh [version=1.22.0]
#
# Alternative without network: copy unitree_rl_lab/deploy/thirdparty/onnxruntime-linux-x64-1.22.0
# into thirdparty/ (x64 only), or build with -DONNXRUNTIME_ROOT=<dir>.
set -euo pipefail

VERSION="${1:-1.22.0}"
case "$(uname -m)" in
  x86_64) ARCH=x64 ;;
  aarch64|arm64) ARCH=aarch64 ;;
  *) echo "unsupported architecture $(uname -m)" >&2; exit 1 ;;
esac

PKG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEST="${PKG_DIR}/thirdparty"
NAME="onnxruntime-linux-${ARCH}-${VERSION}"
URL="https://github.com/microsoft/onnxruntime/releases/download/v${VERSION}/${NAME}.tgz"

mkdir -p "${DEST}"
if [[ -d "${DEST}/${NAME}" ]]; then
  echo "${DEST}/${NAME} already exists"
  exit 0
fi
echo "Downloading ${URL}"
curl -fL "${URL}" | tar -xz -C "${DEST}"
echo "Installed ${DEST}/${NAME}"
