#!/usr/bin/env python3

# Copyright 2021 Shadow Robot Company Ltd.
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation version 2 of the License.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program. If not, see <http://www.gnu.org/licenses/>.

from __future__ import absolute_import
from builtins import round
import subprocess
import argparse
import dynamic_reconfigure
import dynamic_reconfigure.client
import psutil
import rospkg
import yaml


class SystemInfo:

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
                                                                                   "HEAD"], text=True).replace("\n", "")
            self._values["src_repos"][repo_name]["ref"] = subprocess.check_output(["git", "-C", repo_path, "rev-parse",
                                                                                   "--abbrev-ref",
                                                                                   "HEAD"], text=True).replace("\n", "")
            self._values["src_repos"][repo_name]["url"] = subprocess.check_output(["git", "-C", repo_path, "remote",
                                                                                   "get-url",
                                                                                   "origin"],
                                                                                  text=True).replace("\n", "")
            git_diff_ignore = git_diff_ignore_lists["repos"].get(repo_name, "")
            self._values["src_repos"][repo_name]["diff"] = subprocess.check_output(
                f"git --no-pager -C {repo_path} diff -- {repo_path} {git_diff_ignore}",
                shell=True, text=True).replace("\n", "")
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
                shell=True, text=True).replace("\n", "")

    def survey_system(self):
        self._values["system"] = {"hardware":
                                  {"cpu": stdout("lscpu"),
                                   "ram": str(round(psutil.virtual_memory().total / (1024.0 ** 3)))+" GB"},
                                  "software":
                                  {"kernel":
                                   {"release": stdout(["uname", "-r"]),
                                    "version": stdout(["uname", "-v"])}}}

    def yaml(self):
        return yaml.dump(self._values)

    def survey_dynamic_configuration(self):
        self._values["dynamic_reconfigure"] = {}
        # Find dynamically reconfigurable nodes
        servers = dynamic_reconfigure.find_reconfigure_services()
        # These keys in the dynamic reconfigure dictionaries do not specify parameters
        non_param_keys = ["groups", "id", "name", "parameters", "parent", "state", "type"]
        processed_keys = []
        # For each dynamically reconfigurable node:
        for server in servers:
            client = dynamic_reconfigure.client.Client(server)
            # Get the node parameters and current config
            config = client.get_configuration()
            self._values["dynamic_reconfigure"][server] = {}
            # Loop through the parameters groups, collecting config
            if "groups" in config.keys() and "groups" in config["groups"].keys():
                # For each parameter group in this node
                for group in config["groups"]["groups"].keys():
                    self._values["dynamic_reconfigure"][server][group] = {}
                    # For each parameter in this group
                    for key in config["groups"]["groups"][group].keys():
                        # Ignore non-parameter keys
                        if key in non_param_keys:
                            continue
                        self._values["dynamic_reconfigure"][server][group][key] = \
                            config["groups"]["groups"][group][key]
                        processed_keys.append(key)
            # Catch any non-grouped parameters in this node
            for key, value in config.items():
                if key != "groups" and key not in processed_keys:
                    self._values["dynamic_reconfigure"][server][key] = value

def stdout(cmd):
    return subprocess.check_output(cmd, text=True).rstrip("\r\n")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Collect system and ROS package information.")
    parser.add_argument('-v', '--verbose', action='store_true')
    args = parser.parse_args()
    system_info = SystemInfo()
    system_info.collect()
    if args.verbose:
        print(system_info.yaml())
