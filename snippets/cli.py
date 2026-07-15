#!/usr/bin/python3
from argparse import ArgumentParser
import logging
import os
import sys

# Generator selectable via env: "es" (default), "random"
GENERATOR = os.environ.get("GENERATOR", "es")

if GENERATOR == "random":
    from random_generator import RandomGenerator as Generator
else:
    from es_generator import ESGenerator as Generator

import matplotlib
matplotlib.use("Agg")

logger = logging.getLogger(__name__)


def arg_parse():
    main_parser = ArgumentParser(description="UAV Test Generator")
    subparsers = main_parser.add_subparsers()
    parser = subparsers.add_parser(name="generate", description="generate tests")
    parser.add_argument("test", help="initial test description file address")
    parser.add_argument(
        "budget", type=int,
        help="total number of allowed simulations",
    )
    return main_parser.parse_args()


def config_loggers():
    # DEBUG log inside the run's folder (survives container crashes);
    # filemode='a' appends after a crash. Without TESTS_FOLDER, falls back to logs/.
    log_dir = os.environ.get("TESTS_FOLDER", "logs/")
    os.makedirs(log_dir, exist_ok=True)
    logging.basicConfig(
        level=logging.DEBUG,
        filename=os.path.join(log_dir, "debug.txt"),
        filemode="a",
        format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
    )
    root = logging.getLogger()
    c_handler = logging.StreamHandler()
    c_handler.setLevel(logging.INFO)
    c_handler.setFormatter(logging.Formatter("%(name)s - %(levelname)s - %(message)s"))
    root.addHandler(c_handler)
    logging.getLogger("rospy").setLevel(logging.WARNING)  # silences rospy's shutdown log line


if __name__ == "__main__":
    config_loggers()
    try:
        args = arg_parse()
        generator = Generator(case_study_file=args.test)
        os.makedirs(generator.output_dir, exist_ok=True)

        generator.generate(args.budget)

        # copies the selected subset into <output_dir>/consegna/ (rank01 =
        # best) and emits the final line required by the submission
        generator.write_final_selection()

    except Exception as e:
        logger.exception("program terminated: " + str(e))
        sys.exit(1)
