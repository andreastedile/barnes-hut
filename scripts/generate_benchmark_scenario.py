#! /usr/bin/env python3

import argparse
import random
import math
from collections import namedtuple

Position = namedtuple("Position", ["x", "y"])
Velocity = namedtuple("Velocity", ["x", "y"])
Body = namedtuple("Body", ["position", "mass", "velocity"])

min_mass = 13000000000
max_mass = 1898000000000000

def parse_scenario_parameters() -> (int):
    """
    Parses the command-line arguments
    """
    parser = argparse.ArgumentParser(description="Generate a benchmarking dataset containing n bodies")

    parser.add_argument("-p", "--n-bodies", type=int, required=True,
                        help="Number of bodies to add inside the bounding box", metavar="\b", dest="p")

    args = parser.parse_args()

    if(args.p < 0):
        parser.error("number of bodies must be positive")

    return args.p



# reference: for 10 bodies we have bbox length of 945024m and surface of 893070360576m^2
def generate_bodies(n_bodies: int) -> [
    (float, float, float, float, float)]:
    global min_mass, max_mass

    scenario_surface = (223267590144 * n_bodies) / 10
    bbox_length = math.sqrt(scenario_surface)

    bodies = []
    for _ in range(n_bodies):
        position = Position(random.uniform(0, bbox_length),
                            random.uniform(0, bbox_length))
        mass = random.uniform(min_mass, max_mass)
        velocity = Velocity(0, 0)

        body = Body(position, mass, velocity)
        bodies.append(body)
    return bodies


def write_bodies_to_file(bodies: list[Body]):
    with open(f"{len(bodies)}-bodies.txt", "w") as f:
        f.write(f"{len(bodies)}\n")
        for body in bodies:
            f.write(f"{body.position.x} {body.position.y} {body.mass} {body.velocity.x} {body.velocity.y}\n")


def main():
    n_bodies = parse_scenario_parameters()
    bodies = generate_bodies(n_bodies)
    write_bodies_to_file(bodies)


if __name__ == "__main__":
    main()
