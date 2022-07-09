#! /usr/bin/env python3

import argparse
import random
from collections import namedtuple

Position = namedtuple("Position", ["x", "y"])
Velocity = namedtuple("Velocity", ["x", "y"])
Body = namedtuple("Body", ["position", "mass", "velocity"])


def parse_scenario_parameters() -> (Position, Position, int):
    """
    Parses the command-line arguments to obtain the bounding box's coordinates and the number of bodies to add
    :return: the bottom-left and top-right corners of the bounding box; and the number of bodies to add inside the
    bounding box
    """
    parser = argparse.ArgumentParser(description="Generate a random dataset of bodies contained some coordinates")

    parser.add_argument("-nw", "--north-west", type=float, nargs=2, required=True,
                        help="Upper-left coordinates of the bounding box", metavar="\b", dest="nw")
    parser.add_argument("-ne", "--north-east", type=float, nargs=2, required=True,
                        help="Upper-right coordinates of the bounding box", metavar="\b", dest="ne")
    parser.add_argument("-se", "--south-east", type=float, nargs=2, required=True,
                        help="Bottom-right coordinates of the bounding box", metavar="\b", dest="se")
    parser.add_argument("-sw", "--south-west", type=float, nargs=2, required=True,
                        help="Bottom-left coordinates of the bounding box", metavar="\b", dest="sw")
    parser.add_argument("-p", "--n-bodies", type=int, required=True,
                        help="Number of bodies to add inside the bounding box", metavar="\b", dest="p")
    parser.add_argument("-min-m", "--min-mass", type=int, required=True,
                        help="minimum mass body", metavar="\b", dest="min_m")
    parser.add_argument("-max-m", "--max-mass", type=int, required=True,
                        help="maximum mass body", metavar="\b", dest="max_m")

    args = parser.parse_args()

    (nw_x, nw_y), (ne_x, ne_y), (se_x, se_y), (sw_x, sw_y) = args.nw, args.ne, args.se, args.sw
    if nw_x != sw_x:
        parser.error(f"Upper-left x ({nw_x}) and bottom-left x ({sw_x}) are different ")
    if ne_x != se_x:
        parser.error(f"Upper-right x ({ne_x}) and bottom-right x ({se_x}) are different ")
    if nw_y != ne_y:
        parser.error(f"Upper-left y ({nw_y}) and upper-right y ({ne_y}) are different ")
    if sw_y != se_y:
        parser.error(f"Bottom-left y ({sw_y}) and bottom-right y {se_y}) are different")

    if ne_x <= nw_x:
        parser.error(f"Upper-right x {ne_x} is lower than upper-left x {nw_x}")
    if se_x <= sw_x:
        parser.error(f"Bottom-right x {se_x} is lower than bottom-left x {sw_x}")
    if nw_y <= sw_y:
        parser.error(f"Upper-left y {nw_y} is lower than bottom-left y {sw_y}")
    if ne_y <= se_y:
        parser.error(f"Upper-right y {ne_y} is lower than bottom-right y {se_y}")

    bottom_left, top_right = Position(sw_x, sw_y), Position(ne_x, ne_y)

    return bottom_left, top_right, args.p, args.min_m, args.max_m


def generate_bodies(bottom_left: Position, top_right: Position, n_bodies: int, min_m: int, max_m: int) -> [
    (float, float, float, float, float)]:
    bodies = []
    for _ in range(n_bodies):
        position = Position(random.uniform(bottom_left.x, top_right.x),
                            random.uniform(bottom_left.y, top_right.y))
        mass = random.uniform(min_m, max_m)
        velocity = Velocity(0, 0)

        body = Body(position, mass, velocity)
        bodies.append(body)
    return bodies


def write_bodies_to_file(bodies: list[Body]):
    with open("bodies.txt", "w") as f:
        f.write(f"{len(bodies)}\n")
        for body in bodies:
            f.write(f"{body.position.x} {body.position.y} {body.mass} {body.velocity.x} {body.velocity.y}\n")


def main():
    bottom_left, top_right, n_bodies, min_m, max_m = parse_scenario_parameters()
    bodies = generate_bodies(bottom_left, top_right, n_bodies, min_m, max_m)
    write_bodies_to_file(bodies)


if __name__ == "__main__":
    main()
