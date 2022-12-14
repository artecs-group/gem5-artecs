# MIT License
#
# Copyright The SCons Foundation
#
# Permission is hereby granted, free of charge, to any person obtaining
# a copy of this software and associated documentation files (the
# "Software"), to deal in the Software without restriction, including
# without limitation the rights to use, copy, modify, merge, publish,
# distribute, sublicense, and/or sell copies of the Software, and to
# permit persons to whom the Software is furnished to do so, subject to
# the following conditions:
#
# The above copyright notice and this permission notice shall be included
# in all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY
# KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
# WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
# NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
# LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
# OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
# WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

"""conda_gcc

Tool-specific initialization for gcc (conda environment).

There normally shouldn't be any need to import this module directly.
It will usually be imported through the generic SCons.Tool.Tool()
selection method.

"""

import re
import subprocess

import SCons.Tool.cc as cc
import SCons.Util

prefix = 'x86_64-conda-linux-gnu-'
compilers = [prefix + 'gcc', prefix + 'cc']


def generate(env):
    """Add Builders and construction variables for gcc to an Environment."""

    env['CC'] = env.Detect(compilers) or compilers[0]

    cc.generate(env)

    if env['PLATFORM'] in ['cygwin', 'win32']:
        env['SHCCFLAGS'] = SCons.Util.CLVar('$CCFLAGS')
    else:
        env['SHCCFLAGS'] = SCons.Util.CLVar('$CCFLAGS -fPIC')
    # determine compiler version
    version = detect_version(env, env['CC'])
    if version:
        env['CCVERSION'] = version


def exists(env):
    # is executable, and is a GNU compiler (or accepts '--version' at least)
    return detect_version(env, env.Detect(compilers))


def detect_version(env, cc):
    """Return the version of the GNU compiler,
       or None if it is not a GNU compiler."""
    version = None
    if not cc:
        return version

    pipe=SCons.Action._subproc(env, SCons.Util.CLVar(cc) +
                               ['-dumpfullversion', '-dumpversion'],
                               stdin='devnull',
                               stderr='devnull',
                               stdout=subprocess.PIPE)
    if pipe.wait() != 0:
        return version

    with pipe.stdout:
        line = pipe.stdout.read().strip().decode()
        # Non-GNU compiler's output (like AIX xlc's) may exceed
        # the stdout buffer, so continue with reading to let
        # the child process actually terminate.
        while SCons.Util.to_str(pipe.stdout.readline()):
            pass

    if line:
        version = line

    return version

# Local Variables:
# tab-width:4
# indent-tabs-mode:nil
# End:
# vim: set expandtab tabstop=4 shiftwidth=4:
