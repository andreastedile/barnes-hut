#! /usr/bin/env python3

import argparse
import random
import math
from collections import namedtuple

Position = namedtuple("Position", ["x", "y"])
Velocity = namedtuple("Velocity", ["x", "y"])
Planet = namedtuple("Planet", ["position", "mass", "velocity"])

# Reference planetary system variables
MIN_PLANET_MASS = 13000000000
MAX_PLANET_MASS = 1898000000000000
N_PLANETS_IN_SOLAR_SYSTEM = 10
SOLAR_SYSTEM_SURFACE = 223267590144


def parse_scenario_parameters() -> int:
    """
    Parses the command-line arguments to obtain the number of planets to be added to the simulation scenario
    :return: number of planets
    """
    parser = argparse.ArgumentParser(description="Generate a benchmarking dataset containing n planets")

    parser.add_argument("-n", "--n-planets", type=int, required=True,
                        help="Number of planets to add inside the bounding box", metavar="\b", dest="p")

    args = parser.parse_args()

    if args.p < 0:
        parser.error("number of planets must non-negative")

    return args.p


def compute_scenario_surface(n_scenario_planets: int) -> float:
    """
    :return: the total surface where the planets are to be inserted
    """
    # N_PLANETS_IN_SOLAR_SYSTEM : SOLAR_SYSTEM_SURFACE = n_scenario_planets : scenario_surface
    return SOLAR_SYSTEM_SURFACE * n_scenario_planets / N_PLANETS_IN_SOLAR_SYSTEM


def generate_planet(pos_min: Position, pos_max: Position):
    """
    Randomly generates a planet.
    The planet's mass is between MIN_PLANET_MASS and MAX_PLANET_MASS.
    The planet's position coordinates are between 0 and bbox_length.
    :param bbox_length length of the bounding box in
    :return: a planet located in a bounding box of length bbox_length, with
    """
    position = Position(random.uniform(pos_min.x, pos_max.x),
                        random.uniform(pos_min.y, pos_max.y))
    mass = random.uniform(MIN_PLANET_MASS, MAX_PLANET_MASS)
    velocity = Velocity(0, 0)
    return Planet(position, mass, velocity)


def generate_scenario_planets(n_scenario_planets: int) -> [Planet]:
    scenario_surface = compute_scenario_surface(n_scenario_planets)
    bbox_length = math.sqrt(scenario_surface)

    planets = []
    for _ in range(n_scenario_planets):
        planet = generate_planet(Position(0, 0), Position(bbox_length, bbox_length))
        planets.append(planet)
    return planets


def write_planets_to_file(planets: [Planet]):
    with open(f"planets-{len(planets)}.txt", "w") as f:
        f.write(f"{len(planets)}\n")
        for planet in planets:
            f.write(f"{planet.position.x} {planet.position.y} {planet.mass} {planet.velocity.x} {planet.velocity.y}\n")


def main():
    n_scenario_planets = parse_scenario_parameters()
    planets = generate_scenario_planets(n_scenario_planets)
    write_planets_to_file(planets)


if __name__ == "__main__":
    main()
