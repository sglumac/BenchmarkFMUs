# BenchmarkFMUs
Configurations of components compliant to FMI 2.0 specification

## Installation

Please, download and install Sundials (https://computation.llnl.gov/projects/sundials)
### Windows
```bat
mkdir X:\path\to\build\folder
cd X:\path\to\build\folder
cmake -D CMAKE_INSTALL_PREFIX="X:\target\folder" -D FMI2SPECIFICATION="X:\folder\with\fmi2\headers" -D SUNDIALS="X:\path\to\sundials" X:\path\to\repository\root
cmake --build . --target install
```
### Linux
```bash
mkdir /path/to/build/folder
cd /path/to/build/folder
cmake -D CMAKE_INSTALL_PREFIX="/target/folder" -D FMI2SPECIFICATION="/folder/with/fmi2/headers" -D SUNDIALS="/path/to/sundials" /path/to/repository/root
cmake --build . --target install
```

## Configurations
### Two-mass Oscillator
* TwoMassOscillatorD2D.xml
* TwoMassOscillatorF2D.xml
* TwoMassOscillatorReference.xml (CVODE solved reference solution)

### Control Loop PI-PT1
* Control10x.xml
* Control10xReference.xml (analytically solved reference)
* ControlI.xml
* ControlIReference.xml (analytically solved reference)

