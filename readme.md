## Project Setup
- JSON Library: https://github.com/nlohmann/json
    - Copied single .hpp file to include dir

- Triangle: https://www.cs.cmu.edu/~quake/triangle.html
    - ``` sudo apt-get install libtriangle-dev ```

- SPOT: https://spot.lrde.epita.fr/install.html
    - Followed the installation from tar.gz
    - Download and extract
    - ``` ./configure && make && sudo make install ```

- OMPL: https://ompl.kavrakilab.org/installation.html
    - Follow instructions on web for generic linux
    - Had to modify to c++ 17 in CompilerSettings.cmake and ompl.pc.in


## Project Objectives
- Implement LTL planner with SPOT
- Adapt environment for convex polygons
- Implement custom dynamics model
- Benchmark performance for various test cases
