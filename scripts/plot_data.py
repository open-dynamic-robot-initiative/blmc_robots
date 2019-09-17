#!/usr/bin/env python3
"""Plot previously recorded data."""

import sys
from blmc_robots.logger import Logger

def main():
    logger = Logger()
    print("load file %s" % sys.argv[1])
    logger.load(sys.argv[1])
    logger.check_integrity()
    logger.plot()


if __name__ == "__main__":
    main()
