# Copyright 2016 Open Source Robotics Foundation, Inc.
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

"""Provides an Object proxy that allows the underlying class to change."""

import ast
import inspect

# Inspired from:
#   https://pypi.python.org/pypi/ProxyTypes (PSF/ZPL license)


class AbstractProxy(object):
    """Delegates all operations (except .__actual__) to another object."""

    __slots__ = ()

    def __call__(self, *args, **kw):
        return self.__actual__(*args, **kw)

    def __getattribute__(self, attr):
        subject = object.__getattribute__(self, '__actual__')
        if attr == '__actual__':
            return subject
        return getattr(subject, attr)

    def __setattr__(self, attr, val):
        if attr == '__actual__':
            object.__setattr__(self, attr, val)
        else:
            setattr(self.__actual__, attr, val)

    def __delattr__(self, attr):
        if attr == '__actual__':
            object.__delattr__(self, attr)
        else:
            delattr(self.__actual__, attr)

    def __nonzero__(self):
        return bool(self.__actual__)

    def __getitem__(self, arg):
        return self.__actual__[arg]

    def __setitem__(self, arg, val):
        self.__actual__[arg] = val

    def __delitem__(self, arg):
        del self.__actual__[arg]

    def __getslice__(self, i, j):
        return self.__actual__[i:j]

    def __setslice__(self, i, j, val):
        self.__actual__[i:j] = val

    def __delslice__(self, i, j):
        del self.__actual__[i:j]

    def __contains__(self, ob):
        return ob in self.__actual__

    for name in [
        'repr', 'str', 'hash', 'len', 'abs', 'complex', 'int', 'long', 'float',
        'iter', 'oct', 'hex'
    ]:
        code_object = compile(
            'def __{0}__(self): return {0}(self.__actual__)'.format(name),
            __file__,
            'exec',
            ast.PyCF_ONLY_AST,
        )
        ast.increment_lineno(code_object, inspect.getframeinfo(inspect.currentframe()).lineno - 6)
        exec(compile(code_object, __file__, 'exec'))

    for name in ['cmp', 'coerce', 'divmod']:
        code_object = compile(
            'def __{0}__(self, ob): return {0}(self.__actual__, ob)'.format(name),
            __file__,
            'exec',
            ast.PyCF_ONLY_AST,
        )
        ast.increment_lineno(code_object, inspect.getframeinfo(inspect.currentframe()).lineno - 6)
        exec(compile(code_object, __file__, 'exec'))

    for name, op in [
        ('lt', '<'), ('gt', '>'), ('le', '<='), ('ge', '>='), ('eq', '=='), ('ne', '!=')
    ]:
        code_object = compile(
            'def __{0}__(self, ob): return self.__actual__ {1} ob'.format(name, op),
            __file__,
            'exec',
            ast.PyCF_ONLY_AST,
        )
        ast.increment_lineno(code_object, inspect.getframeinfo(inspect.currentframe()).lineno - 6)
        exec(compile(code_object, __file__, 'exec'))

    for name, op in [('neg', '-'), ('pos', '+'), ('invert', '~')]:
        code_object = compile(
            'def __{0}__(self): return {1} self.__actual__'.format(name, op),
            __file__,
            'exec',
            ast.PyCF_ONLY_AST,
        )
        ast.increment_lineno(code_object, inspect.getframeinfo(inspect.currentframe()).lineno - 6)
        exec(compile(code_object, __file__, 'exec'))

    for name, op in [
        ('or', '|'), ('and', '&'), ('xor', '^'), ('lshift', '<<'), ('rshift', '>>'),
        ('add', '+'), ('sub', '-'), ('mul', '*'), ('div', '/'), ('mod', '%'),
        ('truediv', '/'), ('floordiv', '//')
    ]:
        code_object = compile(
            """
def __{name}__(self,ob):
    return self.__actual__ {op} ob

def __r{name}__(self,ob):
    return ob {op} self.__actual__

def __i{name}__(self,ob):
    self.__actual__ {op}=ob
    return self
""".format(name=name, op=op),
            __file__,
            'exec',
            ast.PyCF_ONLY_AST,
        )
        ast.increment_lineno(code_object, inspect.getframeinfo(inspect.currentframe()).lineno - 6)
        exec(compile(code_object, __file__, 'exec'))

    del name, op

    def __rdivmod__(self, ob):
        return divmod(ob, self.__actual__)

    def __pow__(self, *args):
        return pow(self.__actual__, *args)

    def __ipow__(self, ob):
        self.__actual__ **= ob
        return self

    def __rpow__(self, ob):
        return pow(ob, self.__actual__)


class ObjectProxy(AbstractProxy):
    """Proxy for an internally stored object."""

    __slots__ = '__actual__'

    def __init__(self, subject):
        self.__actual__ = subject
