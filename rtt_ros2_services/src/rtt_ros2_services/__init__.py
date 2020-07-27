# Copyright 2020 Intermodalics BVBA
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import argparse
import pathlib
import sys

from rosidl_cmake import expand_template

from .resource import TEMPLATE_DIR


def generate_service_plugin(package, output_dir, services=[]):
    output_dir = pathlib.Path(output_dir)

    data = {
        'pkg_name': package,
        'services': services,
    }
    expand_template(
        'ros_service_plugin.cpp.em',
        data=data,
        output_file=(output_dir / "ros_service_plugin.cpp"),
        template_basepath=TEMPLATE_DIR)


def main(argv=sys.argv[1:]):
    parser = argparse.ArgumentParser(
        description='Generate an RTT plugin for services in a ROS interface package.',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument(
        '--package', required=True,
        help='The name of the package')
    parser.add_argument(
        '--services', metavar='MESSAGE', nargs='*',
        help='The service types')
    parser.add_argument(
        '--output-dir', required=True,
        help='The base directory for generated source files')
    args = parser.parse_args(argv)

    return generate_service_plugin(**vars(args))
