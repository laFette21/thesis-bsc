# Slam algorithm implementation
For more substantial description please read the Szakdolgozat.pdf file.

## Simulator
Perception simulator for driverless cars

### Usage

```
python3 simulator.py [-h] [-d] [-n] [-p] input_file
```

#### Optional arguments
- -h, --help        - shows help message and exit
- -d, --directions  - shows directional vectors of the car
- -n, --noisy       - uses noise for the output data
- -p, --perception  - shows vectors to the cones the car is currently detecting

## SLAM
SLAM algorithm implementation

### Compiling
For normal use:

```
mkdir build
cd build
cmake ..
make -j
```

For checking the code coverage:

```
mkdir build
cd build
cmake -DCOVERAGE=1 ..
make -j
```

### Usage
Executable file is created in the build/src folder.

```
./slam [-h] [-v] [-l] [-n] [-p] [-s] input_file output_file
```

#### Optional arguments
- -h, --help          - shows help message and exits
- -v, --version       - prints version information and exits
- -l, --loop_closure  - enables loop closing for the optimization
- -n, --noise         - uses noise on the input data
- -p, --plot          - plots the results using matplotlib
- -s, --segmentation  - enables the segmentation of the data

### Testing
Executable files are created in their own folder. You can execute them separately or with the help of ```ctest``` from the build directory.

### Tools
With the help of ```plot.py``` found in the folder tools you can plot the data generated with the ```slam``` executable.
With the help of ```coverage.sh``` found in the folder tools you can generate a code coverage report for the dataenumerator and graph libraries and the misc headers.
