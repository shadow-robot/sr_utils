# sr-utils
Contains generic useful libraries for our software.

## DTW
### Overview
This is a library for computing the Dynamic Time Warping distance of two chains. It can be used to compute the distance between 2 different kinematics chain (for remapping from one to the other for example).

The implementation is inspired by [the lbimproved library](https://code.google.com/p/lbimproved).

### Using
*@todo: Yi to add more comments to the library + a quick example in this readme (chain1, chain2, merge, DTW_distance)*

### Using this library in your package
Simply add the dependency to sr_utils to your `package.xml` and `CMakeLists.txt` and include the `<sr_utils/dtw/dtw.hpp>` in your file and you're good to go.
