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
        self.rospack = rospkg.RosPack()

    def collect(self):
        self.values = {}
        self.values["ros_root_path"] = rospkg.get_ros_root()
        self.values["ros_package_paths"] = rospkg.get_ros_package_path().split(':')
        self.values["src_package_paths"] = [i for i in self.values["ros_package_paths"] if "src" in i]
        self.survey_packages()
        self.survey_source_repos()
        self.survey_system()
        # rospy.loginfo(self.yaml())

    def survey_packages(self):
        self.values["src_packages"] = {}
        self.values["bin_packages"] = {}
        self.values["src_repos"] = {}
        for package_name in self.rospack.list():
            for src_package_path in self.values["src_package_paths"]:
                path = self.rospack.get_path(package_name)
                if src_package_path in path:
                    self.values["src_packages"][package_name] = {}
                    self.values["src_packages"][package_name]["path"] = path
                    path_in_src = path.replace(src_package_path + "/", "")
                    if "/" in path_in_src:
                        repo_path = src_package_path + "/" + path_in_src[:path_in_src.find("/")]
                    else:
                        repo_path = src_package_path + "/" + path_in_src
                    repo_name = repo_path.replace(src_package_path + "/", "")
                    self.values["src_packages"][package_name]["repo_path"] = repo_path
                    self.values["src_packages"][package_name]["repo_name"] = repo_name
                    if repo_name not in self.values["src_repos"]:
                        self.values["src_repos"][repo_name] = {}
                        self.values["src_repos"][repo_name]["path"] = repo_path
            if package_name not in self.values["src_packages"]:
                self.values["bin_packages"][package_name] = {}
                self.values["bin_packages"][package_name]["path"] = self.rospack.get_path(package_name)
                self.values["bin_packages"][package_name]["version"] = self.rospack.get_manifest(package_name).version

    def survey_source_repos(self):
        for repo_name in self.values["src_repos"]:
            repo_path = self.values["src_repos"][repo_name]["path"]
            self.values["src_repos"][repo_name]["sha"] = subprocess.check_output(["git", "-C", repo_path, "rev-parse",
                                                                                  "HEAD"]).replace("\n", "")
            self.values["src_repos"][repo_name]["ref"] = subprocess.check_output(["git", "-C", repo_path, "rev-parse",
                                                                                  "--abbrev-ref",
                                                                                  "HEAD"]).replace("\n", "")
            self.values["src_repos"][repo_name]["url"] = subprocess.check_output(["git", "-C", repo_path, "remote",
                                                                                  "get-url",
                                                                                  "origin"]).replace("\n", "")
            self.values["src_repos"][repo_name]["diff"] = subprocess.check_output(
                "git --no-pager -C {} diff".format(repo_path), shell=True).replace("\n", "")
        for package_name in self.values["src_packages"]:
            src_repo_name = self.values["src_packages"][package_name]["repo_name"]
            self.values["src_packages"][package_name]["sha"] = self.values["src_repos"][src_repo_name]["sha"]
            self.values["src_packages"][package_name]["ref"] = self.values["src_repos"][src_repo_name]["ref"]
            self.values["src_packages"][package_name]["url"] = self.values["src_repos"][src_repo_name]["url"]
            self.values["src_packages"][package_name]["diff"] = subprocess.check_output(
                "git --no-pager -C {} diff".format(self.values["src_packages"][package_name]["path"]),
                shell=True).replace("\n", "")

    def survey_system(self):
        self.values["system"] = {"hardware":
                                 {"cpu": self.stdout("lscpu"),
                                  "ram": str(round(psutil.virtual_memory().total / (1024.0 ** 3)))+" GB"},
                                 "software":
                                 {"kernel":
                                  {"release": self.stdout(["uname", "-r"]),
                                   "version": self.stdout(["uname", "-v"])}}}

    def yaml(self):
        return yaml.dump(self.values)

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
