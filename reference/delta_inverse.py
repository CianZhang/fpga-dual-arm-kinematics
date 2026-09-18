#!/usr/bin/env python3
"""CLI added during repository organization, following the original Delta model.

Coordinates are millimetres. Returned angles are motor commands (130 - theta),
not a general robot joint convention. This is not a bit-accurate RTL model.
"""
import argparse
import json
import math


def solve(x, y, z):
    if not all(math.isfinite(v) for v in (x, y, z)):
        raise ValueError('Coordinates must be finite')
    m, n, radius_fixed, radius_moving = 130.0, 316.0, 74.6920, 34.6410
    d = radius_moving - radius_fixed
    branches = [(d + x, y),
                (d - 0.5*x + 0.8660*y, 0.8660*x + 0.5*y),
                (d - 0.5*x - 0.8660*y, 0.8660*x - 0.5*y)]
    angles = []
    for u, v in branches:
        a, b, c = 2*m*u, -2*z*m, u*u + v*v + z*z + m*m - n*n
        discriminant = a*a + b*b - c*c
        if discriminant < 0:
            raise ValueError('Negative discriminant: no real solution in this model')
        numerator = -a*b - c*math.sqrt(discriminant)
        denominator = a*a - c*c
        if denominator == 0:
            raise ValueError('Zero denominator: original formula is singular')
        quotient = numerator / denominator
        theta = (180 - math.degrees(math.atan(-quotient)) if quotient < 0
                 else math.degrees(math.atan(quotient)))
        angles.append(130 - theta)
    return angles


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--x', type=float, default=0)
    parser.add_argument('--y', type=float, default=0)
    parser.add_argument('--z', type=float, default=-237.5)
    args = parser.parse_args()
    try:
        angles = solve(args.x, args.y, args.z)
    except ValueError as exc:
        parser.error(str(exc))
    print(json.dumps({'position_mm': [args.x, args.y, args.z],
                      'motor_command_deg': angles,
                      'motor_command_x256_float': [v*256 for v in angles],
                      'note': 'Floating reference only; no mechanical-limit or collision validation'},
                     ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
