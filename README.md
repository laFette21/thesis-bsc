# Slam algorithm implementation

## Simulator
Perception simulator for driverless cars

### Usage

```
python3 simulator.py [-h] [-d] [-n] [-p] <path/to/input/file>
```

#### Optional arguments
- -h, --help        - show the help message and exit
- -d, --directions  - show the directional vectors of the car
- -n, --noisy       - use noise for the output data
- -p, --perception  - show the vectors to the cones the car is currently detecting

## SLAM
SLAM algorithm implementation

### Compiling

```
mkdir build
cd build
cmake ..
make -j
```

### Usage
Executable file is created in the build/src folder.

```
./slam <path/to/input/file> <path/to/output/file>
```

### Testing
Executable files are created in their own folder. You can execute them separately or with the help of ```ctest``` from the build directory.

### Tools
With the help of ```plot.py``` found in the folder tools you can plot the data generated with the ```slam``` executable.
With the help of ```coverage.sh``` found in the folder tools you can generate a code coverage report for the dataenumerator and graph libraries and the misc headers.
