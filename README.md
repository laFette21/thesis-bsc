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
make
```

### Usage
Executable file is created in the bin folder.

```
./slam <path/to/input/file> <path/to/output/file>
```
