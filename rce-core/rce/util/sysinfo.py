#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#     rce-core/rce/util/sysinfo.py
#
#     This file is part of the RoboEarth Cloud Engine framework.
#
#     Adapted from the psutil package and rewritten in pure Python with
#     relevant required functions. psutil is licensed under the BSD license
#
#         Copyright (c) 2009, Jay Loden, Dave Daeschler, Giampaolo Rodola
#         https://code.google.com/p/psutil/
#
#     This file was originally created for RoboEearth
#     http://www.roboearth.org/
#
#     The research leading to these results has received funding from
#     the European Union Seventh Framework Programme FP7/2007-2013 under
#     grant agreement no248942 RoboEarth.
#
#     Copyright 2013 RoboEarth
#
#     Licensed under the Apache License, Version 2.0 (the "License");
#     you may not use this file except in compliance with the License.
#     You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
#     Unless required by applicable law or agreed to in writing, software
#     distributed under the License is distributed on an "AS IS" BASIS,
#     WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#     See the License for the specific language governing permissions and
#     limitations under the License.
#
#     \author/s: Dhananjay Sathe : dhananjaysathe@gmail.com
#                Giampaolo Rodola : g.rodola@gmail.com
#

import os
import errno
import socket
import struct
import sys
import base64
import glob
import re
import stat
import time

from collections import namedtuple, defaultdict

# Necessary Object Declarations

class NoSuchProcess(Exception):
    """Exception raised when a process with a certain PID doesn't
    or no longer exists (zombie).
    """

    def __init__(self, pid, name=None, msg=None):
        self.pid = pid
        self.name = name
        self.msg = msg
        if msg is None:
            if name:
                details = "(pid=%s, name=%s)" % (self.pid, repr(self.name))
            else:
                details = "(pid=%s)" % self.pid
            self.msg = "process no longer exists " + details

    def __str__(self):
        return self.msg

class AccessDenied(Exception):
    """Exception raised when permission to perform an action is denied."""

    def __init__(self, pid=None, name=None, msg=None):
        self.pid = pid
        self.name = name
        self.msg = msg
        if msg is None:
            if (pid is not None) and (name is not None):
                self.msg = "(pid=%s, name=%s)" % (pid, repr(name))
            elif (pid is not None):
                self.msg = "(pid=%s)" % self.pid
            else:
                self.msg = ""

    def __str__(self):
        return self.msg


class constant(int):
    """A constant type; overrides base int to provide a useful name on str()."""

    def __new__(cls, value, name, doc=None):
        inst = super(constant, cls).__new__(cls, value)
        inst._name = name
        if doc is not None:
            inst.__doc__ = doc
        return inst

    def __str__(self):
        return self._name

    def __eq__(self, other):
        # Use both int or str values when comparing for equality
        # (useful for serialization):
        # >>> st = constant(0, "running")
        # >>> st == 0
        # True
        # >>> st == 'running'
        # True
        if isinstance(other, int):
            return int(self) == other
        if isinstance(other, long):
            return long(self) == other
        if isinstance(other, str):
            return self._name == other
        return False

    def __ne__(self, other):
        return not self.__eq__(other)

# Convenience functions

def _get_boot_time():
    """Return system boot time (epoch in seconds)"""
    f = open('/proc/stat', 'r')
    try:
        for line in f:
            if line.startswith('btime'):
                return float(line.strip().split()[1])
        raise RuntimeError("line not found")
    finally:
        f.close()


def _get_sys_totalmem():
    f = open('/proc/meminfo', 'r')
    total = None
    try:
        for line in f:
            if line.startswith('MemTotal:'):
                total = int(line.split()[1]) * 1024

            if  total is not None :
                break
        else:
            raise RuntimeError("line(s) not found")
    finally:
        f.close()
    return total


def _get_terminal_map():
    ret = {}
    ls = glob.glob('/dev/tty*') + glob.glob('/dev/pts/*')
    for name in ls:
        assert name not in ret
        ret[os.stat(name).st_rdev] = name
    return ret

_pmap = {}


def get_pid_list():
    """Returns a list of PIDs currently running on the system."""
    pids = [int(x) for x in os.listdir('/proc') if x.isdigit()]
    return pids


def pid_exists(pid):
    """Check whether pid exists in the current process table."""
    if not isinstance(pid, int):
        raise TypeError('an integer is required')
    if pid < 0:
        return False
    try:
        os.kill(pid, 0)
    except OSError:
        e = sys.exc_info()[1]
        return e.errno == errno.EPERM
    else:
        return True


def process_iter():
    """Return a generator yielding a Process class instance for all
    running processes on the local machine.

    Every new Process instance is only created once and then cached
    into an internal table which is updated every time this is used.

    The sorting order in which processes are yielded is based on
    their PIDs.
    """
    def add(pid):
        proc = Process(pid)
        _pmap[proc.pid] = proc
        return proc

    def remove(pid):
        _pmap.pop(pid, None)

    a = set(get_pid_list())
    b = set(_pmap.keys())
    new_pids = a - b
    gone_pids = b - a

    for pid in gone_pids:
        remove(pid)
    for pid, proc in sorted(list(_pmap.items()) + \
                            list(dict.fromkeys(new_pids).items())):
        try:
            if proc is None:  # new process
                yield add(pid)
            else:
                # use is_running() to check whether PID has been reused by
                # another process in which case yield a new Process instance
                if proc.is_running():
                    yield proc
                else:
                    yield add(pid)
        except NoSuchProcess:
            remove(pid)
        except AccessDenied:
            # Process creation time can't be determined hence there's
            # no way to tell whether the pid of the cached process
            # has been reused. Just return the cached version.
            yield proc


def _get_num_cpus():
    """Return the number of CPUs on the system"""
    # we try to determine num CPUs by using different approaches.
    # SC_NPROCESSORS_ONLN seems to be the safer and it is also
    # used by multiprocessing module
    try:
        return os.sysconf("SC_NPROCESSORS_ONLN")
    except ValueError:
        # as a second fallback we try to parse /proc/cpuinfo
        num = 0
        f = open('/proc/cpuinfo', 'r')
        try:
            lines = f.readlines()
        finally:
            f.close()
        for line in lines:
            if line.lower().startswith('processor'):
                num += 1

    # unknown format (e.g. amrel/sparc architectures), see:
    # http://code.google.com/p/psutil/issues/detail?id=200
    # try to parse /proc/stat as a last resort
    if num == 0:
        f = open('/proc/stat', 'r')
        try:
            lines = f.readlines()
        finally:
            f.close()
        search = re.compile('cpu\d')
        for line in lines:
            line = line.split(' ')[0]
            if search.match(line):
                num += 1

    if num == 0:
        raise RuntimeError("can't determine number of CPUs")
    return num


def isfile_strict(path):
    """Same as os.path.isfile() but does not swallow EACCES / EPERM
    exceptions, see:
    http://mail.python.org/pipermail/python-dev/2012-June/120787.html
    """
    try:
        st = os.stat(path)
    except OSError:
        err = sys.exc_info()[1]
        if err.errno in (errno.EPERM, errno.EACCES):
            raise
        return False
    else:
        return stat.S_ISREG(st.st_mode)


# --- decorators

def wrap_exceptions(callable):
    """Call callable into a try/except clause and translate ENOENT,
    EACCES and EPERM in NoSuchProcess or AccessDenied exceptions.
    """
    def wrapper(self, *args, **kwargs):
        try:
            return callable(self, *args, **kwargs)
        except EnvironmentError:
            # ENOENT (no such file or directory) gets raised on open().
            # ESRCH (no such process) can get raised on read() if
            # process is gone in meantime.
            err = sys.exc_info()[1]
            if err.errno in (errno.ENOENT, errno.ESRCH):
                raise NoSuchProcess(self.pid, self._process_name)
            if err.errno in (errno.EPERM, errno.EACCES):
                raise AccessDenied(self.pid, self._process_name)
            raise
    return wrapper


# DECLARATIONS AND CONSTANTS

_CLOCK_TICKS = os.sysconf(os.sysconf_names["SC_CLK_TCK"])
_PAGESIZE = os.sysconf("SC_PAGE_SIZE")
BOOT_TIME = _get_boot_time()
NUM_CPUS = _get_num_cpus()
TOTAL_PHYMEM = _get_sys_totalmem()

# --- constants

STATUS_RUNNING = constant(0, "running")
STATUS_SLEEPING = constant(1, "sleeping")
STATUS_DISK_SLEEP = constant(2, "disk sleep")
STATUS_STOPPED = constant(3, "stopped")
STATUS_TRACING_STOP = constant(4, "tracing stop")
STATUS_ZOMBIE = constant(5, "zombie")
STATUS_DEAD = constant(6, "dead")
STATUS_WAKE_KILL = constant(7, "wake kill")
STATUS_WAKING = constant(8, "waking")

# taken from /fs/proc/array.c
_status_map = {"R" : STATUS_RUNNING,
               "S" : STATUS_SLEEPING,
               "D" : STATUS_DISK_SLEEP,
               "T" : STATUS_STOPPED,
               "t" : STATUS_TRACING_STOP,
               "Z" : STATUS_ZOMBIE,
               "X" : STATUS_DEAD,
               "x" : STATUS_DEAD,
               "K" : STATUS_WAKE_KILL,
               "W" : STATUS_WAKING}


# http://students.mimuw.edu.pl/lxr/source/include/net/tcp_states.h
_TCP_STATES_TABLE = {"01" : "ESTABLISHED",
                     "02" : "SYN_SENT",
                     "03" : "SYN_RECV",
                     "04" : "FIN_WAIT1",
                     "05" : "FIN_WAIT2",
                     "06" : "TIME_WAIT",
                     "07" : "CLOSE",
                     "08" : "CLOSE_WAIT",
                     "09" : "LAST_ACK",
                     "0A" : "LISTEN",
                     "0B" : "CLOSING"
                     }

# named tuples for system functions
nt_sys_cputimes = namedtuple('cputimes', 'user nice system idle iowait irq softirq')
nt_virtmem_info = namedtuple('vmem', ' '.join([
    'total', 'available', 'percent', 'used', 'free',
    'active', 'inactive', 'buffers', 'cached']))
nt_net_iostat = namedtuple('iostat',
    'bytes_sent bytes_recv packets_sent packets_recv errin errout dropin dropout')
nt_disk_iostat = namedtuple('iostat', 'read_count write_count read_bytes write_bytes read_time write_time')


# named tuples for processes
nt_io = namedtuple('io', 'read_count write_count read_bytes write_bytes')
nt_cputimes = namedtuple('cputimes', 'user system')
nt_meminfo = namedtuple('meminfo', 'rss vms')
nt_ctxsw = namedtuple('amount', 'voluntary involuntary')
nt_thread = namedtuple('thread', 'id user_time system_time')
nt_openfile = namedtuple('openfile', 'path fd')
nt_connection = namedtuple('connection', 'fd family type local_address remote_address status')
nt_uids = namedtuple('user', 'real effective saved')
nt_gids = namedtuple('group', 'real effective saved')


# SYSTEM FUNCTIONS

# System - CPU

def _get_sys_cpu_times():
    """Return a named tuple representing the following CPU times:
    user, nice, system, idle, iowait, irq, softirq.
    """
    f = open('/proc/stat', 'r')
    try:
        values = f.readline().split()
    finally:
        f.close()

    values = values[1:8]
    values = tuple([float(x) / _CLOCK_TICKS for x in values])
    return nt_sys_cputimes(*values[:7])


def _get_sys_per_cpu_times():
    """Return a list of namedtuple representing the CPU times
    for every CPU available on the system.
    """
    cpus = []
    f = open('/proc/stat', 'r')
    # get rid of the first line who refers to system wide CPU stats
    try:
        f.readline()
        for line in f.readlines():
            if line.startswith('cpu'):
                values = line.split()[1:8]
                values = tuple([float(x) / _CLOCK_TICKS for x in values])
                entry = nt_sys_cputimes(*values[:7])
                cpus.append(entry)
        return cpus
    finally:
        f.close()

# top level wrapper functions for cpu
def cpu_times(percpu=False):
    """Return system-wide CPU times as a namedtuple object.
    Every CPU time represents the time CPU has spent in the given mode.
    The attributes availability varies depending on the platform.
    Here follows a list of all available attributes:
     - user
     - system
     - idle
     - nice (UNIX)
     - iowait (Linux)
     - irq (Linux, FreeBSD)
     - softirq (Linux)

    When percpu is True return a list of nameduples for each CPU.
    First element of the list refers to first CPU, second element
    to second CPU and so on.
    The order of the list is consistent across calls.
    """
    if not percpu:
        return _get_sys_cpu_times()
    else:
        return _get_sys_per_cpu_times()


_last_cpu_times = cpu_times()
_last_per_cpu_times = cpu_times(percpu=True)

def cpu_percent(interval=0.1, percpu=False):
    """Return a float representing the current system-wide CPU
    utilization as a percentage.

    When interval is > 0.0 compares system CPU times elapsed before
    and after the interval (blocking).

    When interval is 0.0 or None compares system CPU times elapsed
    since last call or module import, returning immediately.
    In this case is recommended for accuracy that this function be
    called with at least 0.1 seconds between calls.

    When percpu is True returns a list of floats representing the
    utilization as a percentage for each CPU.
    First element of the list refers to first CPU, second element
    to second CPU and so on.
    The order of the list is consistent across calls.
    """
    global _last_cpu_times
    global _last_per_cpu_times
    blocking = interval is not None and interval > 0.0

    def calculate(t1, t2):
        t1_all = sum(t1)
        t1_busy = t1_all - t1.idle

        t2_all = sum(t2)
        t2_busy = t2_all - t2.idle

        # this usually indicates a float precision issue
        if t2_busy <= t1_busy:
            return 0.0

        busy_delta = t2_busy - t1_busy
        all_delta = t2_all - t1_all
        busy_perc = (busy_delta / all_delta) * 100
        return round(busy_perc, 1)

    # system-wide usage
    if not percpu:
        if blocking:
            t1 = cpu_times()
            time.sleep(interval)
        else:
            t1 = _last_cpu_times
        _last_cpu_times = cpu_times()
        return calculate(t1, _last_cpu_times)
    # per-cpu usage
    else:
        ret = []
        if blocking:
            tot1 = cpu_times(percpu=True)
            time.sleep(interval)
        else:
            tot1 = _last_per_cpu_times
        _last_per_cpu_times = cpu_times(percpu=True)
        for t1, t2 in zip(tot1, _last_per_cpu_times):
            ret.append(calculate(t1, t2))
        return ret


# System - Memory

def usage_percent(used, total, _round=None):
    """Calculate percentage usage of 'used' against 'total'."""
    try:
        ret = (used / total) * 100
    except ZeroDivisionError:
        ret = 0
    if _round is not None:
        return round(ret, _round)
    else:
        return ret


def get_sys_meminfo():
    f = open('/proc/meminfo', 'r')
    total = free = buffers = cached = active = inactive = None
    try:
        for line in f:
            if line.startswith('MemTotal:'):
                total = int(line.split()[1]) * 1024
            elif line.startswith('MemFree:'):
                free = int(line.split()[1]) * 1024
            elif line.startswith('Buffers:'):
                buffers = int(line.split()[1]) * 1024
            elif line.startswith('Cached:'):
                cached = int(line.split()[1]) * 1024
            elif line.startswith('Active:'):
                active = int(line.split()[1]) * 1024
            elif line.startswith('Inactive:'):
                inactive = int(line.split()[1]) * 1024
            if  total is not None \
            and free is not None \
            and buffers is not None \
            and cached is not None \
            and active is not None \
            and inactive is not None:
                break
        else:
            raise RuntimeError("line(s) not found")
    finally:
        f.close()
    avail = free + buffers + cached
    used = total - free
    percent = usage_percent((total - avail), total, _round=1)
    return nt_virtmem_info(total, avail, percent, used, free,
                           active, inactive, buffers, cached)


# System - Network

def network_io_counters():
    """Return network I/O statistics for every network interface
    installed on the system as a dict of raw tuples.
    """
    f = open("/proc/net/dev", "r")
    try:
        lines = f.readlines()
    finally:
        f.close()

    retdict = dict()
    for line in lines[2:]:
        colon = line.find(':')
        assert colon > 0, line
        name = line[:colon].strip()
        fields = line[colon + 1:].strip().split()
        bytes_recv = int(fields[0])
        packets_recv = int(fields[1])
        errin = int(fields[2])
        dropin = int(fields[2])
        bytes_sent = int(fields[8])
        packets_sent = int(fields[9])
        errout = int(fields[10])
        dropout = int(fields[11])
        retdict[name] = nt_net_iostat(bytes_sent, bytes_recv, packets_sent, packets_recv,
                         errin, errout, dropin, dropout)
    return retdict


# System disk

def disk_io_counters():
    """Return disk I/O statistics for every disk installed on the
    system as a dict of raw tuples.
    """
    # man iostat states that sectors are equivalent with blocks and
    # have a size of 512 bytes since 2.4 kernels. This value is
    # needed to calculate the amount of disk I/O in bytes.
    SECTOR_SIZE = 512

    # determine partitions we want to look for
    partitions = []
    f = open("/proc/partitions", "r")
    try:
        lines = f.readlines()[2:]
    finally:
        f.close()
    for line in lines:
        _, _, _, name = line.split()
        if name[-1].isdigit():
            partitions.append(name)
    #
    retdict = {}
    f = open("/proc/diskstats", "r")
    try:
        lines = f.readlines()
    finally:
        f.close()
    for line in lines:
        _, _, name, reads, _, rbytes, rtime, writes, _, wbytes, wtime = \
            line.split()[:11]
        if name in partitions:
            rbytes = int(rbytes) * SECTOR_SIZE
            wbytes = int(wbytes) * SECTOR_SIZE
            reads = int(reads)
            writes = int(writes)
            rtime = int(rtime)
            wtime = int(wtime)
            retdict[name] = nt_disk_iostat(reads, writes, rbytes, wbytes, rtime, wtime)
    return retdict


# Representation and methods for a process

class Process(object):
    """Linux process implementation."""

    __slots__ = ["pid", "_process_name", "_last_sys_cpu_times",
                 "_last_proc_cpu_times", "_gone", "create_time",
                 "ppid"]

    def __init__(self, pid):
        if not isinstance(pid, int):
            raise TypeError('pid must be an integer')
        self.pid = pid
        self._process_name = None
        self._last_sys_cpu_times = None
        self._last_proc_cpu_times = None
        self._gone = False
        self.create_time = self.get_process_create_time()
        self.ppid = self.get_process_ppid()

    @wrap_exceptions
    def get_process_name(self):
        f = open("/proc/%s/stat" % self.pid)
        try:
            name = f.read().split(' ')[1].replace('(', '').replace(')', '')
        finally:
            f.close()
        # XXX - gets changed later and probably needs refactoring
        return name

    def is_running(self):
        """Return whether this process is running."""
        if self._gone:
            return False
        try:
            # Checking if pid is alive is not enough as the pid might
            # have been reused by another process.
            # pid + creation time, on the other hand, is supposed to
            # identify a process univocally.
            return self.create_time == \
                   self.get_process_create_time()
        except NoSuchProcess:
            self._gone = True
            return False

    def get_process_exe(self):
        try:
            exe = os.readlink("/proc/%s/exe" % self.pid)
        except (OSError, IOError):
            err = sys.exc_info()[1]
            if err.errno == errno.ENOENT:
                # no such file error; might be raised also if the
                # path actually exists for system processes with
                # low pids (about 0-20)
                if os.path.lexists("/proc/%s/exe" % self.pid):
                    return ""
                else:
                    # ok, it is a process which has gone away
                    raise NoSuchProcess(self.pid, self._process_name)
            if err.errno in (errno.EPERM, errno.EACCES):
                raise AccessDenied(self.pid, self._process_name)
            raise

        # readlink() might return paths containing null bytes causing
        # problems when used with other fs-related functions (os.*,
        # open(), ...)
        exe = exe.replace('\x00', '')
        # Certain names have ' (deleted)' appended. Usually this is
        # bogus as the file actually exists. Either way that's not
        # important as we don't want to discriminate executables which
        # have been deleted.
        if exe.endswith(" (deleted)") and not os.path.exists(exe):
            exe = exe[:-10]
        return exe

    @wrap_exceptions
    def get_process_cmdline(self):
        f = open("/proc/%s/cmdline" % self.pid)
        try:
            # return the args as a list
            return [x for x in f.read().split('\x00') if x]
        finally:
            f.close()

    @wrap_exceptions
    def get_process_terminal(self):
        f = open("/proc/%s/stat" % self.pid)
        try:
            tty_nr = int(f.read().split(' ')[6])
        finally:
            f.close()
        try:
            return _get_terminal_map()[tty_nr]
        except KeyError:
            return None

    @wrap_exceptions
    def get_process_io_counters(self):
        f = open("/proc/%s/io" % self.pid)
        try:
            for line in f:
                if line.startswith("rchar"):
                    read_count = int(line.split()[1])
                elif line.startswith("wchar"):
                    write_count = int(line.split()[1])
                elif line.startswith("read_bytes"):
                    read_bytes = int(line.split()[1])
                elif line.startswith("write_bytes"):
                    write_bytes = int(line.split()[1])
            return nt_io(read_count, write_count, read_bytes, write_bytes)
        finally:
            f.close()

    if not os.path.exists('/proc/%s/io' % os.getpid()):
        def get_process_io_counters(self):
            raise NotImplementedError('/proc/PID/io is not available')

    @wrap_exceptions
    def get_cpu_times(self):
        f = open("/proc/%s/stat" % self.pid)
        try:
            st = f.read().strip()
        finally:
            f.close()
        # ignore the first two values ("pid (exe)")
        st = st[st.find(')') + 2:]
        values = st.split(' ')
        utime = float(values[11]) / _CLOCK_TICKS
        stime = float(values[12]) / _CLOCK_TICKS
        return nt_cputimes(utime, stime)

    def get_cpu_percent(self, interval=0.1):
        """Return a float representing the current process CPU
        utilization as a percentage.

        When interval is > 0.0 compares process times to system CPU
        times elapsed before and after the interval (blocking).

        When interval is 0.0 or None compares process times to system CPU
        times elapsed since last call, returning immediately.
        In this case is recommended for accuracy that this function be
        called with at least 0.1 seconds between calls.
        """
        blocking = interval is not None and interval > 0.0
        if blocking:
            st1 = sum(cpu_times())
            pt1 = self.get_cpu_times()
            time.sleep(interval)
            st2 = sum(cpu_times())
            pt2 = self.get_cpu_times()
        else:
            st1 = self._last_sys_cpu_times
            pt1 = self._last_proc_cpu_times
            st2 = sum(cpu_times())
            pt2 = self.get_cpu_times()
            if st1 is None or pt1 is None:
                self._last_sys_cpu_times = st2
                self._last_proc_cpu_times = pt2
                return 0.0

        delta_proc = (pt2.user - pt1.user) + (pt2.system - pt1.system)
        delta_time = st2 - st1
        # reset values for next call in case of interval == None
        self._last_sys_cpu_times = st2
        self._last_proc_cpu_times = pt2

        try:
            # the utilization split between all CPUs
            overall_percent = (delta_proc / delta_time) * 100
        except ZeroDivisionError:
            # interval was too low
            return 0.0
        # the utilization of a single CPU
        single_cpu_percent = overall_percent * NUM_CPUS
        # on posix a percentage > 100 is legitimate
        # http://stackoverflow.com/questions/1032357/comprehending-top-cpu-usage
        # on windows we use this ugly hack to avoid troubles with float
        # precision issues
        if os.name != 'posix':
            if single_cpu_percent > 100.0:
                return 100.0
        return round(single_cpu_percent, 1)


    @wrap_exceptions
    def get_process_create_time(self):
        f = open("/proc/%s/stat" % self.pid)
        try:
            st = f.read().strip()
        finally:
            f.close()
        # ignore the first two values ("pid (exe)")
        st = st[st.rfind(')') + 2:]
        values = st.split(' ')
        # According to documentation, starttime is in field 21 and the
        # unit is jiffies (clock ticks).
        # We first divide it for clock ticks and then add uptime returning
        # seconds since the epoch, in UTC.
        starttime = (float(values[19]) / _CLOCK_TICKS) + BOOT_TIME
        return starttime


    def get_children(self, recursive=False):
        """Return the children of this process as a list of Process
        objects.
        If recursive is True return all the parent descendants.

        Example (A == this process):

         A ─┐
            │
            ├─ B (child) ─┐
            │             └─ X (grandchild) ─┐
            │                                └─ Y (great grandchild)
            ├─ C (child)
            └─ D (child)

        >>> p.get_children()
        B, C, D
        >>> p.get_children(recursive=True)
        B, X, Y, C, D

        Note that in the example above if process X disappears
        process Y won't be returned either as the reference to
        process A is lost.
        """
        if not self.is_running():
            name = self._process_name
            raise NoSuchProcess(self.pid, name)

        ret = []
        if not recursive:
            for p in process_iter():
                try:
                    if p.ppid == self.pid:
                        # if child happens to be older than its parent
                        # (self) it means child's PID has been reused
                        if self.create_time <= p.create_time:
                            ret.append(p)
                except NoSuchProcess:
                    pass
        else:
            # construct a dict where 'values' are all the processes
            # having 'key' as their parent
            table = defaultdict(list)
            for p in process_iter():
                try:
                    table[p.ppid].append(p)
                except NoSuchProcess:
                    pass
            # At this point we have a mapping table where table[self.pid]
            # are the current process's children.
            # Below, we look for all descendants recursively, similarly
            # to a recursive function call.
            checkpids = [self.pid]
            for pid in checkpids:
                for child in table[pid]:
                    try:
                        # if child happens to be older than its parent
                        # (self) it means child's PID has been reused
                        intime = self.create_time <= child.create_time
                    except NoSuchProcess:
                        pass
                    else:
                        if intime:
                            ret.append(child)
                            if child.pid not in checkpids:
                                checkpids.append(child.pid)
        return ret


    @wrap_exceptions
    def get_memory_info(self):
        f = open("/proc/%s/statm" % self.pid)
        try:
            vms, rss = f.readline().split()[:2]
            return nt_meminfo(int(rss) * _PAGESIZE,
                              int(vms) * _PAGESIZE)
        finally:
            f.close()

    _nt_ext_mem = namedtuple('meminfo', 'rss vms shared text lib data dirty')

    @wrap_exceptions
    def get_ext_memory_info(self):
        #  ============================================================
        # | FIELD  | DESCRIPTION                         | AKA  | TOP  |
        #  ============================================================
        # | rss    | resident set size                   |      | RES  |
        # | vms    | total program size                  | size | VIRT |
        # | shared | shared pages (from shared mappings) |      | SHR  |
        # | text   | text ('code')                       | trs  | CODE |
        # | lib    | library (unused in Linux 2.6)       | lrs  |      |
        # | data   | data + stack                        | drs  | DATA |
        # | dirty  | dirty pages (unused in Linux 2.6)   | dt   |      |
        #  ============================================================
        f = open("/proc/%s/statm" % self.pid)
        try:
            vms, rss, shared, text, lib, data, dirty = \
              [int(x) * _PAGESIZE for x in f.readline().split()[:7]]
        finally:
            f.close()
        return self._nt_ext_mem(rss, vms, shared, text, lib, data, dirty)

    _mmap_base_fields = ['path', 'rss', 'size', 'pss', 'shared_clean',
                         'shared_dirty', 'private_clean', 'private_dirty',
                         'referenced', 'anonymous', 'swap']
    nt_mmap_grouped = namedtuple('mmap', ' '.join(_mmap_base_fields))
    nt_mmap_ext = namedtuple('mmap', 'addr perms ' + ' '.join(_mmap_base_fields))

    def get_memory_percent(self):
        """Compare physical system memory to process resident memory and
        calculate process memory utilization as a percentage.
        """
        rss = self.get_memory_info()[0]
        try:
            return (rss / float(TOTAL_PHYMEM)) * 100
        except ZeroDivisionError:
            return 0.0

    def get_memory_maps(self):
        """Return process's mapped memory regions as a list of nameduples.
        Fields are explained in 'man proc'; here is an updated (Apr 2012)
        version: http://goo.gl/fmebo
        """
        f = None
        try:
            f = open("/proc/%s/smaps" % self.pid)
            first_line = f.readline()
            current_block = [first_line]

            def get_blocks():
                data = {}
                for line in f:
                    fields = line.split(None, 5)
                    if len(fields) >= 5:
                        yield (current_block.pop(), data)
                        current_block.append(line)
                    else:
                        data[fields[0]] = int(fields[1]) * 1024
                yield (current_block.pop(), data)

            if first_line:  # smaps file can be empty
                for header, data in get_blocks():
                    hfields = header.split(None, 5)
                    try:
                        addr, perms, offset, dev, inode, path = hfields
                    except ValueError:
                        addr, perms, offset, dev, inode, path = hfields + ['']
                    if not path:
                        path = '[anon]'
                    else:
                        path = path.strip()
                    yield (addr, perms, path,
                           data['Rss:'],
                           data['Size:'],
                           data.get('Pss:', 0),
                           data['Shared_Clean:'], data['Shared_Clean:'],
                           data['Private_Clean:'], data['Private_Dirty:'],
                           data['Referenced:'],
                           data['Anonymous:'],
                           data['Swap:'])
            f.close()
        except EnvironmentError:
            # XXX - Can't use wrap_exceptions decorator as we're
            # returning a generator;  this probably needs some
            # refactoring in order to avoid this code duplication.
            if f is not None:
                f.close()
            err = sys.exc_info()[1]
            if err.errno in (errno.ENOENT, errno.ESRCH):
                raise NoSuchProcess(self.pid, self._process_name)
            if err.errno in (errno.EPERM, errno.EACCES):
                raise AccessDenied(self.pid, self._process_name)
            raise
        except:
            if f is not None:
                f.close()
            raise

    if not os.path.exists('/proc/%s/smaps' % os.getpid()):
        def get_shared_libs(self, ext):
            msg = "this Linux version does not support /proc/PID/smaps " \
                  "(kernel < 2.6.14 or CONFIG_MMU kernel configuration " \
                  "option is not enabled)"
            raise NotImplementedError(msg)

    @wrap_exceptions
    def get_process_cwd(self):
        # readlink() might return paths containing null bytes causing
        # problems when used with other fs-related functions (os.*,
        # open(), ...)
        path = os.readlink("/proc/%s/cwd" % self.pid)
        return path.replace('\x00', '')

    @wrap_exceptions
    def get_num_ctx_switches(self):
        vol = unvol = None
        f = open("/proc/%s/status" % self.pid)
        try:
            for line in f:
                if line.startswith("voluntary_ctxt_switches"):
                    vol = int(line.split()[1])
                elif line.startswith("nonvoluntary_ctxt_switches"):
                    unvol = int(line.split()[1])
                if vol is not None and unvol is not None:
                    return nt_ctxsw(vol, unvol)
            raise RuntimeError("line not found")
        finally:
            f.close()

    @wrap_exceptions
    def get_process_num_threads(self):
        f = open("/proc/%s/status" % self.pid)
        try:
            for line in f:
                if line.startswith("Threads:"):
                    return int(line.split()[1])
            raise RuntimeError("line not found")
        finally:
            f.close()

    @wrap_exceptions
    def get_process_threads(self):
        thread_ids = os.listdir("/proc/%s/task" % self.pid)
        thread_ids.sort()
        retlist = []
        hit_enoent = False
        for thread_id in thread_ids:
            try:
                f = open("/proc/%s/task/%s/stat" % (self.pid, thread_id))
            except EnvironmentError:
                err = sys.exc_info()[1]
                if err.errno == errno.ENOENT:
                    # no such file or directory; it means thread
                    # disappeared on us
                    hit_enoent = True
                    continue
                raise
            try:
                st = f.read().strip()
            finally:
                f.close()
            # ignore the first two values ("pid (exe)")
            st = st[st.find(')') + 2:]
            values = st.split(' ')
            utime = float(values[11]) / _CLOCK_TICKS
            stime = float(values[12]) / _CLOCK_TICKS
            ntuple = nt_thread(int(thread_id), utime, stime)
            retlist.append(ntuple)
        if hit_enoent:
            # raise NSP if the process disappeared on us
            os.stat('/proc/%s' % self.pid)
        return retlist


    @wrap_exceptions
    def get_process_status(self):
        f = open("/proc/%s/status" % self.pid)
        try:
            for line in f:
                if line.startswith("State:"):
                    letter = line.split()[1]
                    if letter in _status_map:
                        return _status_map[letter]
                    return constant(-1, '?')
        finally:
            f.close()

    @wrap_exceptions
    def get_open_files(self):
        retlist = []
        files = os.listdir("/proc/%s/fd" % self.pid)
        hit_enoent = False
        for fd in files:
            fd_file = "/proc/%s/fd/%s" % (self.pid, fd)
            if os.path.islink(fd_file):
                try:
                    fd_file = os.readlink(fd_file)
                except OSError:
                    # ENOENT == fd_file which is gone in the meantime
                    err = sys.exc_info()[1]
                    if err.errno == errno.ENOENT:
                        hit_enoent = True
                        continue
                    raise
                else:
                    # If file is not an absolute path there's no way
                    # to tell whether it's a regular file or not,
                    # so we skip it. A regular file is always supposed
                    # to be absolutized though.
                    if fd_file.startswith('/') and isfile_strict(fd_file):
                        ntuple = nt_openfile(fd_file, int(fd))
                        retlist.append(ntuple)
        if hit_enoent:
            # raise NSP if the process disappeared on us
            os.stat('/proc/%s' % self.pid)
        return retlist

    @wrap_exceptions
    def get_connections(self, kind='inet'):
        """Return connections opened by process as a list of namedtuples.
        The kind parameter filters for connections that fit the following
        criteria:

        Kind Value      Number of connections using
        inet            IPv4 and IPv6
        inet4           IPv4
        inet6           IPv6
        tcp             TCP
        tcp4            TCP over IPv4
        tcp6            TCP over IPv6
        udp             UDP
        udp4            UDP over IPv4
        udp6            UDP over IPv6
        all             the sum of all the possible families and protocols
        """
        # Note: in case of UNIX sockets we're only able to determine the
        # local bound path while the remote endpoint is not retrievable:
        # http://goo.gl/R3GHM
        inodes = {}
        # os.listdir() is gonna raise a lot of access denied
        # exceptions in case of unprivileged user; that's fine:
        # lsof does the same so it's unlikely that we can to better.
        for fd in os.listdir("/proc/%s/fd" % self.pid):
            try:
                inode = os.readlink("/proc/%s/fd/%s" % (self.pid, fd))
            except OSError:
                continue
            if inode.startswith('socket:['):
                # the process is using a socket
                inode = inode[8:][:-1]
                inodes[inode] = fd

        if not inodes:
            # no connections for this process
            return []

        def process(fin, family, type_):
            retlist = []
            try:
                f = open(fin, 'r')
            except IOError:
                # IPv6 not supported on this platform
                err = sys.exc_info()[1]
                if err.errno == errno.ENOENT and fin.endswith('6'):
                    return []
                else:
                    raise
            try:
                f.readline()  # skip the first line
                for line in f:
                    # IPv4 / IPv6
                    if family in (socket.AF_INET, socket.AF_INET6):
                        _, laddr, raddr, status, _, _, _, _, _, inode = \
                                                                line.split()[:10]
                        if inode in inodes:
                            laddr = self._decode_address(laddr, family)
                            raddr = self._decode_address(raddr, family)
                            if type_ == socket.SOCK_STREAM:
                                status = _TCP_STATES_TABLE[status]
                            else:
                                status = ""
                            fd = int(inodes[inode])
                            conn = nt_connection(fd, family, type_, laddr,
                                                 raddr, status)
                            retlist.append(conn)
                    elif family == socket.AF_UNIX:
                        tokens = line.split()
                        _, _, _, _, type_, _, inode = tokens[0:7]
                        if inode in inodes:

                            if len(tokens) == 8:
                                path = tokens[-1]
                            else:
                                path = ""
                            fd = int(inodes[inode])
                            type_ = int(type_)
                            conn = nt_connection(fd, family, type_, path,
                                                 None, "")
                            retlist.append(conn)
                    else:
                        raise ValueError(family)
                return retlist
            finally:
                f.close()

        tcp4 = ("tcp" , socket.AF_INET , socket.SOCK_STREAM)
        tcp6 = ("tcp6", socket.AF_INET6, socket.SOCK_STREAM)
        udp4 = ("udp" , socket.AF_INET , socket.SOCK_DGRAM)
        udp6 = ("udp6", socket.AF_INET6, socket.SOCK_DGRAM)
        unix = ("unix", socket.AF_UNIX, None)

        tmap = {
            "all"  : (tcp4, tcp6, udp4, udp6, unix),
            "tcp"  : (tcp4, tcp6),
            "tcp4" : (tcp4,),
            "tcp6" : (tcp6,),
            "udp"  : (udp4, udp6),
            "udp4" : (udp4,),
            "udp6" : (udp6,),
            "unix" : (unix,),
            "inet" : (tcp4, tcp6, udp4, udp6),
            "inet4": (tcp4, udp4),
            "inet6": (tcp6, udp6),
        }
        if kind not in tmap:
            raise ValueError("invalid %r kind argument; choose between %s"
                             % (kind, ', '.join([repr(x) for x in tmap])))
        ret = []
        for f, family, type_ in tmap[kind]:
            ret += process("/proc/net/%s" % f, family, type_)
        # raise NSP if the process disappeared on us
        os.stat('/proc/%s' % self.pid)
        return ret

    @wrap_exceptions
    def get_num_fds(self):
        return len(os.listdir("/proc/%s/fd" % self.pid))

    @wrap_exceptions
    def get_process_ppid(self):
        f = open("/proc/%s/status" % self.pid)
        try:
            for line in f:
                if line.startswith("PPid:"):
                    # PPid: nnnn
                    return int(line.split()[1])
            raise RuntimeError("line not found")
        finally:
            f.close()

    @wrap_exceptions
    def get_process_uids(self):
        f = open("/proc/%s/status" % self.pid)
        try:
            for line in f:
                if line.startswith('Uid:'):
                    _, real, effective, saved, fs = line.split()
                    return nt_uids(int(real), int(effective), int(saved))
            raise RuntimeError("line not found")
        finally:
            f.close()

    @wrap_exceptions
    def get_process_gids(self):
        f = open("/proc/%s/status" % self.pid)
        try:
            for line in f:
                if line.startswith('Gid:'):
                    _, real, effective, saved, fs = line.split()
                    return nt_gids(int(real), int(effective), int(saved))
            raise RuntimeError("line not found")
        finally:
            f.close()

    @staticmethod
    def _decode_address(addr, family):
        """Accept an "ip:port" address as displayed in /proc/net/*
        and convert it into a human readable form, like:

        "0500000A:0016" -> ("10.0.0.5", 22)
        "0000000000000000FFFF00000100007F:9E49" -> ("::ffff:127.0.0.1", 40521)

        The IP address portion is a little or big endian four-byte
        hexadecimal number; that is, the least significant byte is listed
        first, so we need to reverse the order of the bytes to convert it
        to an IP address.
        The port is represented as a two-byte hexadecimal number.

        Reference:
        http://linuxdevcenter.com/pub/a/linux/2000/11/16/LinuxAdmin.html
        """
        ip, port = addr.split(':')
        port = int(port, 16)
        # this usually refers to a local socket in listen mode with
        # no end-points connected
        if not port:
            return ()
        if family == socket.AF_INET:
            # see: http://code.google.com/p/psutil/issues/detail?id=201
            if sys.byteorder == 'little':
                ip = socket.inet_ntop(family, base64.b16decode(ip)[::-1])
            else:
                ip = socket.inet_ntop(family, base64.b16decode(ip))
        else:  # IPv6
            # old version - let's keep it, just in case...
            # ip = ip.decode('hex')
            # return socket.inet_ntop(socket.AF_INET6,
            #          ''.join(ip[i:i+4][::-1] for i in xrange(0, 16, 4)))
            ip = base64.b16decode(ip)
            # see: http://code.google.com/p/psutil/issues/detail?id=201
            if sys.byteorder == 'little':
                ip = socket.inet_ntop(socket.AF_INET6,
                                struct.pack('>4I', *struct.unpack('<4I', ip)))
            else:
                ip = socket.inet_ntop(socket.AF_INET6,
                                struct.pack('<4I', *struct.unpack('<4I', ip)))
        return (ip, port)
