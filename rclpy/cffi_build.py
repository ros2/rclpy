# Copyright 2021 Open Source Robotics Foundation, Inc.
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
from cffi import FFI


def generate_cpython_extension(output_file, module_name, c_definitions, c_code):
    ffi_builder = FFI()
    ffi_builder.cdef(c_definitions)
    ffi_builder.set_source(module_name, c_code)
    ffi_builder.emit_c_code(output_file)


def main():
    parser = argparse.ArgumentParser(description='Generate Python Module')
    parser.add_argument('--output-module-name', metavar='STR', type=str, required=True,
                        help='The name of the generated python module.')
    parser.add_argument('--output-file', metavar='FILENAME', type=str, required=True,
                        help='The file of generated C code to create.')
    parser.add_argument('--c-def-file', metavar='FILENAME', type=str, required=True,
                        help='A file containing C definitions to generate Python code for.')
    parser.add_argument('--c-code-file', metavar='FILENAME', type=str, required=True,
                        help='A file containing extra C code to include in the output.')
    args = parser.parse_args()

    with open(args.c_def_file, 'r') as cdef_file:
        c_definitions = cdef_file.read()

    with open(args.c_code_file, 'r') as ccode_file:
        c_code = ccode_file.read()

    generate_cpython_extension(args.output_file, args.output_module_name, c_definitions, c_code)


if __name__ == "__main__":
    main()
