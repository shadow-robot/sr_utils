# sr_system_info

This package contains a utility to collect system and currently-sourced ROS package information, including:
* Binary package
    * Path
    * Version
* Source package
    * Path
    * Parent repository
* Source repository
    * Path
    * Commit SHA
    * Reference
    * URL
    * Diffs
* ROS
    * Package paths
    * Workspace paths
    * Installation path
* System
    * CPU specification
    * RAM
    * Kernel version

To collect and print system info to STDOUT, run:

```sh
rosrun sr_system_info system_info.py -v
```

The utility can also be used as a library, e.g.:

```python
from sr_system_info.system_info import SystemInfo

if __name__ == "__main__":
    benchmark_context = SystemInfo()
    benchmark_context.collect()
    benchmark_context.values  # A dictionary of all collected information
    print(benchmark_context.yaml())
```

Example output (package lists truncated):

```
bin_packages:
  actionlib: {path: /opt/ros/kinetic/share/actionlib, version: 1.11.13}
  actionlib_msgs: {path: /opt/ros/kinetic/share/actionlib_msgs, version: 1.12.7}
  ...
  xacro: {path: /opt/ros/kinetic/share/xacro, version: 1.11.3}
  xmlrpcpp: {path: /opt/ros/kinetic/share/xmlrpcpp, version: 1.12.14}
ros_package_paths: [/home/user/projects/shadow_robot/base/src, /home/user/projects/shadow_robot/base_deps/src,
  /opt/ros/kinetic/share]
ros_root_path: /opt/ros/kinetic/share/ros
src_package_paths: [/home/user/projects/shadow_robot/base/src, /home/user/projects/shadow_robot/base_deps/src]
src_packages:
  ackermann_steering_controller: {diff: '', path: /home/user/projects/shadow_robot/base_deps/src/ros_controllers/ackermann_steering_controller,
    ref: F_improve_latency, repo_name: ros_controllers, repo_path: /home/user/projects/shadow_robot/base_deps/src/ros_controllers,
    sha: 61bfc2565e9e81ca7dfa1dabab883cbb2c1a07e4, url: 'git@github.com:shadow-robot/ros_controllers.git'}
  bio_ik: {diff: '', path: /home/user/projects/shadow_robot/base_deps/src/bio_ik,
    ref: master, repo_name: bio_ik, repo_path: /home/user/projects/shadow_robot/base_deps/src/bio_ik,
    sha: 8424004c44f4e2aca939aba7798236b840b1471c, url: 'git@github.com:shadow-robot/bio_ik.git'}
  ...
  sr_description_common: {diff: 'diff --git a/sr_description_common/worlds/demo_space_large_folding_table.world
      b/sr_description_common/worlds/demo_space_large_folding_table.worldindex 85f9e44..062d2df
      100644--- a/sr_description_common/worlds/demo_space_large_folding_table.world+++
      b/sr_description_common/worlds/demo_space_large_folding_table.world@@ -89,8
      +89,8 @@           <contact_surface_layer>0.00000</contact_surface_layer>         </constraints>       </ode>-      <real_time_update_rate>0.000000</real_time_update_rate>-      <max_step_size>0.001000</max_step_size>+      <real_time_update_rate>2000.000000</real_time_update_rate>+      <max_step_size>0.000500</max_step_size>     </physics>     <scene>       <ambient>0.4
      0.4 0.4 1</ambient>', path: /home/user/projects/shadow_robot/base/src/common_resources/sr_description_common,
    ref: kinetic-devel, repo_name: common_resources, repo_path: /home/user/projects/shadow_robot/base/src/common_resources,
    sha: c99e8ee29d08de999f538073b3211cf35560c39b, url: 'git@github.com:shadow-robot/common_resources.git'}
  ...
  warehouse_ros: {diff: '', path: /home/user/projects/shadow_robot/base_deps/src/warehouse_ros,
    ref: kinetic-devel, sha: 9dccfce122eae78884da15d41e3923ee21f0be9a, url: 'https://github.com/ros-planning/warehouse_ros.git'}
  warehouse_ros_mongo: {diff: '', path: /home/user/projects/shadow_robot/base_deps/src/warehouse_ros_mongo,
    ref: B_working_jade_devel, sha: 5a5ecf659be760916d2e75a3aea269ac68e3a52f, url: 'https://github.com/shadow-robot/warehouse_ros_mongo.git'}
system:
  hardware: {cpu: 'Architecture:          x86_64

      CPU op-mode(s):        32-bit, 64-bit

      Byte Order:            Little Endian

      CPU(s):                12

      On-line CPU(s) list:   0-11

      Thread(s) per core:    2

      Core(s) per socket:    6

      Socket(s):             1

      NUMA node(s):          1

      Vendor ID:             GenuineIntel

      CPU family:            6

      Model:                 158

      Model name:            Intel(R) Core(TM) i7-8750H CPU @ 2.20GHz

      Stepping:              10

      CPU MHz:               2193.103

      CPU max MHz:           4100.0000

      CPU min MHz:           800.0000

      BogoMIPS:              4416.00

      Virtualization:        VT-x

      L1d cache:             32K

      L1i cache:             32K

      L2 cache:              256K

      L3 cache:              9216K

      NUMA node0 CPU(s):     0-11

      Flags:                 fpu vme de pse tsc msr pae mce cx8 apic sep mtrr pge
      mca cmov pat pse36 clflush dts acpi mmx fxsr sse sse2 ss ht tm pbe syscall nx
      pdpe1gb rdtscp lm constant_tsc art arch_perfmon pebs bts rep_good nopl xtopology
      nonstop_tsc cpuid aperfmperf tsc_known_freq pni pclmulqdq dtes64 monitor ds_cpl
      vmx est tm2 ssse3 sdbg fma cx16 xtpr pdcm pcid sse4_1 sse4_2 x2apic movbe popcnt
      tsc_deadline_timer aes xsave avx f16c rdrand lahf_lm abm 3dnowprefetch cpuid_fault
      epb invpcid_single pti ssbd ibrs ibpb stibp tpr_shadow vnmi flexpriority ept
      vpid ept_ad fsgsbase tsc_adjust bmi1 avx2 smep bmi2 erms invpcid mpx rdseed
      adx smap clflushopt intel_pt xsaveopt xsavec xgetbv1 xsaves dtherm ida arat
      pln pts hwp hwp_notify hwp_act_window hwp_epp md_clear flush_l1d', ram: 16.0
      GB}
  software:
    kernel: {release: 5.0.0-37-generic, version: '#40~18.04.1-Ubuntu SMP Thu Nov 14
        12:06:39 UTC 2019'}
```
