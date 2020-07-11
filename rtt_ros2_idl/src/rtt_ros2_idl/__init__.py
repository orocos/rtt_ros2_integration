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

from rosidl_cmake import convert_camel_case_to_lower_case_underscore
from rosidl_cmake import expand_template

from .resource import TEMPLATE_DIR


def generate_typekit(package, output_dir, messages=[], services=[], actions=[]):
    output_dir = pathlib.Path(output_dir)

    for message in messages:
        header_name = convert_camel_case_to_lower_case_underscore(message)
        data = {
            'header_name': header_name,
            'msg_header': f'{package}/msg/{header_name}.hpp',
            'msg_name': message,
            'pkg_name': package
        }
        expand_template(
            'msg_Types.hpp.em',
            data=data,
            output_file=(output_dir / "msg" / (header_name + "_Types.hpp")),
            template_basepath=TEMPLATE_DIR)
        expand_template(
            'msg_typekit.cpp.em',
            data=data,
            output_file=(output_dir / "msg" / (header_name + "_typekit.cpp")),
            template_basepath=TEMPLATE_DIR)

    for service in services:
        header_name = convert_camel_case_to_lower_case_underscore(service)
        data = {
            'header_name': header_name,
            'srv_header': f'{package}/srv/{header_name}.hpp',
            'srv_name': f'{service}',
            'pkg_name': package
        }
        expand_template(
            'srv_Types.hpp.em',
            data=data,
            output_file=(output_dir / "srv" / (header_name + "_Types.hpp")),
            template_basepath=TEMPLATE_DIR)
        expand_template(
            'srv_typekit.cpp.em',
            data=data,
            output_file=(output_dir / "srv" / (header_name + "_typekit.cpp")),
            template_basepath=TEMPLATE_DIR)

    for action in actions:
        header_name = convert_camel_case_to_lower_case_underscore(action)
        data = {
            'header_name': header_name,
            'action_header': f'{package}/action/{header_name}.hpp',
            'action_name': f'{action}',
            'pkg_name': package
        }
        expand_template(
            'action_Types.hpp.em',
            data=data,
            output_file=(output_dir / "action" / (header_name + "_Types.hpp")),
            template_basepath=TEMPLATE_DIR)
        expand_template(
            'action_typekit.cpp.em',
            data=data,
            output_file=(output_dir / "action" / (header_name + "_typekit.cpp")),
            template_basepath=TEMPLATE_DIR)

    data = {
        'pkg_name': package,
        'messages': messages,
        'services': services,
        'actions': actions
    }
    expand_template(
        'Types.hpp.em',
        data=data,
        output_file=(output_dir / "Types.hpp"),
        template_basepath=TEMPLATE_DIR)
    expand_template(
        'typekit_plugin.cpp.em',
        data=data,
        output_file=(output_dir / "typekit_plugin.cpp"),
        template_basepath=TEMPLATE_DIR)


def main(argv=sys.argv[1:]):
    parser = argparse.ArgumentParser(
        description='Generate an RTT typekit from message, service and action types in a ROS '
                    'interface package.',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument(
        '--package', required=True,
        help='The name of the package')
    parser.add_argument(
        '--messages', metavar='MESSAGE', nargs='*',
        help='The message types to generate typekit for')
    parser.add_argument(
        '--services', metavar='SERVICE', nargs='*',
        help='The service types to generate typekit for')
    parser.add_argument(
        '--actions', metavar='ACTION', nargs='*',
        help='The action types to generate typekit for')
    parser.add_argument(
        '--output-dir', required=True,
        help='The base directory to create typekit files in')
    args = parser.parse_args(argv)

    return generate_typekit(**vars(args))
