#!/usr/bin/env python3
# Copyright (c) 2026, unitree-go2-rl contributors.
# SPDX-License-Identifier: Apache-2.0
"""Write a tiny linear ONNX policy (actions = obs @ W + b) without any third-party package.

Used for unit tests and as a pipeline smoke test (`--zero`: all actions zero -> the robot
simply holds default_joint_pos with the policy gains, which is safe in simulation).

    python3 make_linear_policy.py <out.onnx> --obs 45 --act 12 [--zero]

Weights (non-zero version): W[i][j] = 0.01 * ((i + j) % 5 - 2), b[j] = 0.001 * j
"""
import argparse
import struct


def varint(v):
    out = bytearray()
    v &= (1 << 64) - 1
    while True:
        b = v & 0x7F
        v >>= 7
        if v:
            out.append(b | 0x80)
        else:
            out.append(b)
            return bytes(out)


def field_varint(num, v):
    return varint(num << 3 | 0) + varint(v)


def field_bytes(num, data):
    if isinstance(data, str):
        data = data.encode()
    return varint(num << 3 | 2) + varint(len(data)) + data


def tensor_type(dims):
    shape = b''.join(field_bytes(1, field_varint(1, d)) for d in dims)  # Dimension.dim_value
    tensor = field_varint(1, 1) + field_bytes(2, shape)                  # elem_type FLOAT, shape
    return field_bytes(1, tensor)                                        # TypeProto.tensor_type


def value_info(name, dims):
    return field_bytes(1, name) + field_bytes(2, tensor_type(dims))


def initializer(name, dims, values):
    t = b''.join(field_varint(1, d) for d in dims)
    t += field_varint(2, 1)  # FLOAT
    t += field_bytes(8, name)
    t += field_bytes(9, struct.pack('<%df' % len(values), *values))
    return t


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('out')
    ap.add_argument('--obs', type=int, default=45)
    ap.add_argument('--act', type=int, default=12)
    ap.add_argument('--zero', action='store_true')
    a = ap.parse_args()

    n, m = a.obs, a.act
    if a.zero:
        w = [0.0] * (n * m)
        b = [0.0] * m
    else:
        w = [0.01 * ((i + j) % 5 - 2) for i in range(n) for j in range(m)]
        b = [0.001 * j for j in range(m)]

    node = (field_bytes(1, 'obs') + field_bytes(1, 'W') + field_bytes(1, 'b') +
            field_bytes(2, 'actions') + field_bytes(3, 'gemm') + field_bytes(4, 'Gemm'))
    graph = (field_bytes(1, node) + field_bytes(2, 'linear_policy') +
             field_bytes(5, initializer('W', [n, m], w)) +
             field_bytes(5, initializer('b', [m], b)) +
             field_bytes(11, value_info('obs', [1, n])) +
             field_bytes(12, value_info('actions', [1, m])))
    opset = field_bytes(1, '') + field_varint(2, 13)
    model = (field_varint(1, 8) + field_bytes(2, 'go2_policy_runner') +
             field_bytes(7, graph) + field_bytes(8, opset))
    with open(a.out, 'wb') as f:
        f.write(model)


if __name__ == '__main__':
    main()
