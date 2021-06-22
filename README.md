# CI Statuses

Check | Status
---|---
Build|[<img src="https://codebuild.eu-west-2.amazonaws.com/badges?uuid=eyJlbmNyeXB0ZWREYXRhIjoiVzVHYmNHUytlVDV6NldybC9pK09Rcng4T0REVUUxZWNhc3BPcW9ud0Uxa3duU1RyaEQvYks2a3ZZTS9ubzNDdk42R2Q2VllBNldWcTFpREk3UkRuT1NrPSIsIml2UGFyYW1ldGVyU3BlYyI6ImgzcFlJM1BJRU9RUUhHRi8iLCJtYXRlcmlhbFNldFNlcmlhbCI6MX0%3D&branch=noetic-devel"/>](https://eu-west-2.console.aws.amazon.com/codesuite/codebuild/projects/auto_sr_utils_noetic-devel_install_check/)
Style|[<img src="https://codebuild.eu-west-2.amazonaws.com/badges?uuid=eyJlbmNyeXB0ZWREYXRhIjoiNUR1TzU1TGNJQkUxSW93UnJHWDMxK2hnbWdCUWI0VENlenJQNWxYQWtSOFkydVRONW9oVVlVSlZLU3VxMVN0OFQ0cHhjSDJYYmt2eTZ5YWtvVlVyY05VPSIsIml2UGFyYW1ldGVyU3BlYyI6IlNBM2h4S3JENldyQmVUYzQiLCJtYXRlcmlhbFNldFNlcmlhbCI6MX0%3D&branch=noetic-devel"/>](https://eu-west-2.console.aws.amazon.com/codesuite/codebuild/projects/auto_sr_utils_noetic-devel_style_check/)
Code Coverage|[<img src="https://codebuild.eu-west-2.amazonaws.com/badges?uuid=eyJlbmNyeXB0ZWREYXRhIjoiZHBncm5XV3RtYVBKaG5QOEhYK3ZLa0l1T2x0WHBsZlFBS1Jtb2xQZ2tTc2pCejhHL3ZtRmVVOEJwOVR5QjUwczFCVHVwRFdrY05PTWlJeFVPVmVnUXRnPSIsIml2UGFyYW1ldGVyU3BlYyI6Im9lTm1xbktCalF5UnorSG4iLCJtYXRlcmlhbFNldFNlcmlhbCI6MX0%3D&branch=noetic-devel"/>](https://eu-west-2.console.aws.amazon.com/codesuite/codebuild/projects/auto_sr_utils_noeticv-devel_code_coverage/)

# sr-utils
Contains generic useful libraries for our software.

## DTW
### Overview
This is a library for computing the Dynamic Time Warping distance of two chains. It can be used to compute the distance between 2 different kinematics chain (for remapping from one to the other for example).

The implementation is inspired by [the lbimproved library](https://code.google.com/p/lbimproved).

### Using
The vector-based DTW constructor takes two chains of nodes in 3D as its input arguments (the number of desired nodes is the optional third input argument). The chains may have different length. In fact, even overlapping nodes are allowed in the chains, as long as they are located next to each other.

Below is a small example:

```
#include <sr_utils/dtw/dtw.hpp>

std::vector<Eigen::Vector3d> chain_a;
chain_a.push_back( Eigen::Vector3d(-1, 1, 1) );
chain_a.push_back( Eigen::Vector3d(0, 0, 0) );
chain_a.push_back( Eigen::Vector3d(1, 1, 1) );

std::vector<Eigen::Vector3d> chain_b1;
chain_b1.push_back( Eigen::Vector3d(-1, 0, 1) );
chain_b1.push_back( Eigen::Vector3d(0, -1, 0) );

std::vector<Eigen::Vector3d> chain_b2;
chain_b2.push_back( Eigen::Vector3d(1, -1, 0) );
chain_b2.push_back( Eigen::Vector3d(1, -1, 0) ); // same as the previous node
chain_b2.push_back( Eigen::Vector3d(2, 0, 1) );

// Merge chain_b1 and chain_b2.
bool connect_at_tail = true;
std::vector<Eigen::Vector3d> chain_b = DTW::merge(chain_b1, connect_at_tail, chain_b2, !connect_at_tail);

// Compute the DTW distance between the two chains.
DTW dtw(chain_a, chain_b);
double dis = dtw.dtw_distance();
```

### Using this library in your package
Simply add the dependency to sr_utils to your `package.xml` and `CMakeLists.txt` and include the `<sr_utils/dtw/dtw.hpp>` in your file and you're good to go.

## sr_system_info

This is a library for collecting system and package information. See it's [README.md](sr_system_info/README.md).
