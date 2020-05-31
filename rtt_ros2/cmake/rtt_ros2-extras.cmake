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

# Find Boost with required components filesystem and system in downstream
# packages.
# See also https://github.com/ament/ament_cmake/issues/199 for why
# ament_export_dependencies() cannot be used here.
find_package(Boost REQUIRED COMPONENTS filesystem system)

# Find LibXml2 in downstream packages
find_package(LibXml2 REQUIRED)
