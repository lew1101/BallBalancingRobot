from os import getenv
from argparse import ArgumentParser

from src.__main__ import main

# parser = ArgumentParser(description="Ball-balancing Robot")
# parser.add_argument('--setpoint',
#                     type=float,
#                     nargs=2,
#                     metavar='setPoint',
#                     help='Setpoint for ball position as two floats (x y)',
#                     default=(0.0, 0.0))

parser.add_argument("--debug",
                    action="store_true",
                    help="Enable debug mode",
                    default=getenv("DEBUG") == "true")

if __name__ == "__main__":
    args = parser.parse_args()
    main(args)
