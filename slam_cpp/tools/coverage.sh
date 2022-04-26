#!/bin/bash

gcovr --html-details coverage.html -r .. -e ".*test*" -e ".*catch.hpp" -e ".*argparse.hpp" -e ".*matplotlibcpp.h" -e ".*main.cpp"
