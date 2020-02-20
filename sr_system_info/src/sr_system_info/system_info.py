#!/usr/bin/env python
#
# Copyright (C) 2020 Shadow Robot Company Ltd - All Rights Reserved. Proprietary and Confidential.
# Unauthorized copying of the content in this file, via any medium is strictly prohibited.

import argparse
import psutil
import rospy
import rospkg
import subprocess
import yaml


class SystemInfo(object):

    def __init__(self):
        self._rospack = rospkg.RosPack()
        self._values = {}

    @property
    def values(self):
        return self._values

    def collect(self):
        self._values["ros_root_path"] = rospkg.get_ros_root()
        self._values["ros_package_paths"] = rospkg.get_ros_package_path().split(':')
        self._values["src_package_paths"] = [i for i in self._values["ros_package_paths"] if "src" in i]
        self.survey_packages()
        self.survey_source_repos()
        self.survey_system()

    def survey_packages(self):
        self._values["src_packages"] = {}
        self._values["bin_packages"] = {}
        self._values["src_repos"] = {}
        for package_name in self._rospack.list():
            for src_package_path in self._values["src_package_paths"]:
                path = self._rospack.get_path(package_name)
                if src_package_path in path:
                    self._values["src_packages"][package_name] = {}
                    self._values["src_packages"][package_name]["path"] = path
                    path_in_src = path.replace(src_package_path + "/", "")
                    if "/" in path_in_src:
                        repo_path = src_package_path + "/" + path_in_src[:path_in_src.find("/")]
                    else:
                        repo_path = src_package_path + "/" + path_in_src
                    repo_name = repo_path.replace(src_package_path + "/", "")
                    self._values["src_packages"][package_name]["repo_path"] = repo_path
                    self._values["src_packages"][package_name]["repo_name"] = repo_name
                    if repo_name not in self._values["src_repos"]:
                        self._values["src_repos"][repo_name] = {}
                        self._values["src_repos"][repo_name]["path"] = repo_path
            if package_name not in self._values["src_packages"]:
                self._values["bin_packages"][package_name] = {}
                self._values["bin_packages"][package_name]["path"] = self._rospack.get_path(package_name)
                self._values["bin_packages"][package_name]["version"] = self._rospack.get_manifest(package_name).version

    def survey_source_repos(self):
        git_diff_ignore_lists = {"repos": {
                                    "sr_teleop_internal": "':!sr_teleop_benchmarking/benchmarks'",
                                    "sr_interface": "':!sr_multi_moveit/sr_multi_moveit_config/launch/moveit.rviz'"},
                                 "packages": {
                                    "sr_teleop_benchmarking": "':!benchmarks'",
                                    "sr_multi_moveit_config": "':!launch/moveit.rviz'"}}
        for repo_name in self._values["src_repos"]:
            repo_path = self._values["src_repos"][repo_name]["path"]
            self._values["src_repos"][repo_name]["sha"] = subprocess.check_output(["git", "-C", repo_path, "rev-parse",
                                                                                   "HEAD"]).replace("\n", "")
            self._values["src_repos"][repo_name]["ref"] = subprocess.check_output(["git", "-C", repo_path, "rev-parse",
                                                                                   "--abbrev-ref",
                                                                                   "HEAD"]).replace("\n", "")
            self._values["src_repos"][repo_name]["url"] = subprocess.check_output(["git", "-C", repo_path, "remote",
                                                                                   "get-url",
                                                                                   "origin"]).replace("\n", "")
            git_diff_ignore = git_diff_ignore_lists["repos"].get(repo_name, "")
            self._values["src_repos"][repo_name]["diff"] = subprocess.check_output(
                "git --no-pager -C {} diff -- {} {}".format(repo_path, repo_path, git_diff_ignore),
                shell=True).replace("\n", "")
        for package_name in self._values["src_packages"]:
            src_repo_name = self._values["src_packages"][package_name]["repo_name"]
            self._values["src_packages"][package_name]["sha"] = self._values["src_repos"][src_repo_name]["sha"]
            self._values["src_packages"][package_name]["ref"] = self._values["src_repos"][src_repo_name]["ref"]
            self._values["src_packages"][package_name]["url"] = self._values["src_repos"][src_repo_name]["url"]
            git_diff_ignore = git_diff_ignore_lists["packages"].get(package_name, "")
            self._values["src_packages"][package_name]["diff"] = subprocess.check_output(
                "git --no-pager -C {} diff -- {} {}".format(self._values["src_packages"][package_name]["path"],
                                                            self._values["src_packages"][package_name]["path"],
                                                            git_diff_ignore),
                shell=True).replace("\n", "")

    def survey_system(self):
        self._values["system"] = {"hardware":
                                  {"cpu": self.stdout("lscpu"),
                                   "ram": str(round(psutil.virtual_memory().total / (1024.0 ** 3)))+" GB"},
                                  "software":
                                  {"kernel":
                                   {"release": self.stdout(["uname", "-r"]),
                                    "version": self.stdout(["uname", "-v"])}}}

    def yaml(self):
        return yaml.dump(self._values)

    def stdout(self, cmd):
        return subprocess.check_output(cmd).rstrip("\r\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Collect system and ROS package information.")
    parser.add_argument('-v', '--verbose', action='store_true')
    args = parser.parse_args()
    system_info = SystemInfo()
    system_info.collect()
    if args.verbose:
        print system_info.yaml()
