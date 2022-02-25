# Slam algorithm implementation

## Simulator
Perception simulator for driverless cars

### Usage

```
python3 simulator.py [-h] [-d] [-p]
```

#### Optional arguments
- -h, --help        - show the help message and exit
- -d, --directions  - show the directional vectors of the car
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
./slam
```

#### Optional arguments
- -h, --help        - show the help message and exit
- -d, --directions  - show the directional vectors of the car
- -p, --perception  - show the vectors to the cones the car is currently detecting
