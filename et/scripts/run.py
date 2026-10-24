#!/usr/bin/env python3
"""Build and test the 9front RISC-V/Maxion kernels from a 9P-served container root.

  up     export the committed source, start the 9P container and the amd64 build guest
  build  run guest/build.rc on the guest (full; --kernel reuses a previous full build)
  cmd    run one rc command on the build guest console
  test   boot exports/9maxion diskless in RISC-V QEMU and run the smoke test
  down   stop the guest and container, restore workspace ownership

Everything mutable lives in $ET_WORK (default et/tmp/work). The guest has no
disk: its root is the immutable container export plus the writable /rw volume,
which is $ET_WORK/rw on the host, so logs and exports appear there directly.
"""
import argparse
import contextlib
import fcntl
import hashlib
import json
import math
import os
import pathlib
import re
import shlex
import shutil
import socket
import subprocess
import sys
import tempfile
import time
import uuid

IMAGE = 'ghcr.io/rvs/9front-amd64@sha256:131b06bdc301a87c1e861e548a79e08f81ce52afe3297b6258f08a63dab194d2'
KERNEL_SHA256 = '7d40d3216ef5f51ab0b980e1b7bc2e94f43a889a7f59ba9c57b7e6272da8019e'
ROOT = pathlib.Path(__file__).resolve().parents[1]
SOURCE = ROOT.parent
WORK = pathlib.Path(os.environ.get('ET_WORK', ROOT / 'tmp/work')).expanduser().resolve()
DOCKER = shlex.split(os.environ.get('ET_DOCKER', 'docker'))
STATE = WORK / 'state.json'
CHILDREN = {}


def sh(*cmd, **kw):
    return subprocess.run(cmd, check=True, text=True, capture_output=True, **kw).stdout


def docker(*args, **kw):
    return sh(*DOCKER, *args, **kw)


def state():
    return json.loads(STATE.read_text()) if STATE.exists() else {}


def save(**kw):
    s = state()
    s.update(kw)
    tmp = STATE.with_name(f'.state.{os.getpid()}.tmp')
    tmp.write_text(json.dumps(s, indent=1) + '\n')
    tmp.replace(STATE)


@contextlib.contextmanager
def operation(name):
    if name == 'cmd':
        yield
        return
    with (WORK / 'operation.lock').open('a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError('another workspace operation is running')
        yield


def freeport():
    with socket.socket() as s:
        s.bind(('127.0.0.1', 0))
        return s.getsockname()[1]


def container_name(suffix=''):
    return 'et-9p-' + hashlib.sha256(str(WORK).encode()).hexdigest()[:12] + suffix


def remove_container(name):
    r = subprocess.run(DOCKER + ['rm', '-f', name], text=True, capture_output=True)
    if r.returncode and 'No such container' not in r.stdout + r.stderr:
        raise RuntimeError('could not remove container ' + name + ': ' + r.stdout + r.stderr)


def serial_delay():
    delay = float(os.environ.get('P9_SERIAL_DELAY', '0.05'))
    if not math.isfinite(delay) or delay < 0:
        raise ValueError('serial delay must be nonnegative and timeout positive')
    return delay


@contextlib.contextmanager
def serial(vm):
    """Hold the guest's single serial-client slot and a connected socket."""
    with (WORK / vm / 'serial-client.lock').open('a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError('serial console busy; detach the other client first')
        with socket.socket(socket.AF_UNIX) as s:
            s.settimeout(1)
            s.connect(str(WORK / vm / 'serial.sock'))
            yield s


def command(vm, text, timeout=60, client=None):
    """Send one rc line to a guest console and wait for its completion marker."""
    delay = serial_delay()
    if not math.isfinite(timeout) or timeout <= 0:
        raise ValueError('serial delay must be nonnegative and timeout positive')
    if client is None:
        with serial(vm) as s:
            return command(vm, text, timeout, s)
    marker = 'DONE' + uuid.uuid4().hex[:16]
    # rc's ^ makes the echoed command differ from the marker it prints.
    line = (text + "; result=$status; sleep 1; echo; echo 'DONE'^'" + marker[4:]
            + "' $result\n").encode()
    data = b''
    for b in line:
        client.sendall(bytes([b]))
        if delay:
            time.sleep(delay)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            b = client.recv(65536)
        except socket.timeout:
            continue
        if not b:
            break
        data += b
        out = data.decode(errors='replace').replace('\r', '')
        m = re.search(r'^' + marker + r'([^\n]*)\n', out, re.M)
        if m:
            return out[:m.start()].split('\n', 1)[-1], m.group(1).strip()
    raise RuntimeError('timed out or disconnected; command may have executed; '
                       'inspect the console log before retrying: ' +
                       data.decode(errors='replace')[-2000:])


def wait_ready(vm, timeout=180):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if (WORK / vm / 'serial.sock').exists():
            try:
                if command(vm, 'echo ready', 5)[0].strip() == 'ready':
                    return
            except (OSError, RuntimeError):
                pass
        time.sleep(2)
    sys.exit(vm + ' guest did not become ready; see ' + str(WORK / vm / 'console.log'))


def qemu(vm, args):
    """Start a detached QEMU with its serial console on a UNIX socket."""
    d = WORK / vm
    d.mkdir(parents=True, exist_ok=True)
    if len(str(d / 'serial.sock').encode()) >= 100 or ',' in str(d):
        sys.exit('ET_WORK path is too long for UNIX sockets or contains a comma')
    for f in ('serial.sock', 'monitor.sock'):
        (d / f).unlink(missing_ok=True)
    args += ['-display', 'none',
             '-chardev', f'socket,id=ser,path={d / "serial.sock"},server=on,wait=off,logfile={d / "console.log"}',
             '-serial', 'chardev:ser',
             '-monitor', f'unix:{d / "monitor.sock"},server=on,wait=off']
    with (d / 'qemu.log').open('w') as log:
        child = subprocess.Popen(args, stdout=log, stderr=log, stdin=subprocess.DEVNULL,
                                 start_new_session=True)
    CHILDREN[vm] = child
    save(**{vm + '_pid': child.pid})
    return child


def qemu_pid(vm):
    """Return the recorded PID only if it still belongs to this workspace."""
    pid = state().get(vm + '_pid')
    if not isinstance(pid, int) or pid <= 0:
        return None
    try:
        cmdline = pathlib.Path(f'/proc/{pid}/cmdline').read_bytes()
    except OSError:
        return None
    # Match the whole chardev path so /x/tmp/w is not mistaken for /tmp/w.
    if b'path=' + str(WORK / vm / 'serial.sock').encode() + b',' not in cmdline:
        return None
    return pid


def kill(pid, sig):
    try:
        os.kill(pid, sig)
    except ProcessLookupError:
        pass


def stop_qemu(vm):
    child = CHILDREN.pop(vm, None)
    pid = child.pid if child is not None else qemu_pid(vm)
    if child is not None and child.poll() is not None:
        pid = None
    if pid is not None:
        kill(pid, 15)
    if child is not None:
        try:
            child.wait(timeout=10)
        except subprocess.TimeoutExpired:
            child.kill()
            child.wait()
    elif pid is not None:
        for _ in range(100):
            if qemu_pid(vm) is None:
                break
            time.sleep(0.1)
        pid = qemu_pid(vm)
        if pid is not None:
            kill(pid, 9)
            for _ in range(100):
                if qemu_pid(vm) is None:
                    break
                time.sleep(0.1)
            if qemu_pid(vm) is not None:
                raise RuntimeError('could not stop workspace QEMU PID ' + str(pid))
    save(**{vm + '_pid': None})


def serve(name, port, volumes):
    """Serve the immutable image plus writable volumes over 9P on loopback."""
    remove_container(name)
    args = ['run', '-d', '--name', name, '--platform', 'linux/amd64',
            '--read-only', '--cap-drop', 'ALL',
            '--cap-add', 'SYS_CHROOT', '--cap-add', 'DAC_OVERRIDE',
            '--security-opt', 'no-new-privileges', '-p', f'127.0.0.1:{port}:564',
            '-e', 'READ_ONLY=0', '-e', f'NINEFRONT_HOST_PORT={port}']
    for host, guest in volumes:
        args += ['-v', f'{host}:{guest}']
    docker(*args, IMAGE)
    for _ in range(100):
        try:
            with socket.create_connection(('127.0.0.1', port), timeout=2) as s:
                s.sendall(b'\x13\0\0\0\x64\xff\xff\0\x20\0\0\x06\x009P2000')
                if b'9P2000' in s.recv(64):
                    return
        except OSError:
            pass
        time.sleep(0.2)
    sys.exit('9P server did not answer: ' + docker('logs', name))


def nineboot(port, extra):
    return ['-append', 'console=0 user=glenda fs=10.0.2.100 auth=127.0.0.1 ' + extra
            + " nobootprompt='tcp ether /net/ether0 10.0.2.15 255.255.255.0'",
            '-chardev', f'socket,id=fs9p,host=127.0.0.1,port={port}',
            '-netdev', 'user,id=n0,guestfwd=tcp:10.0.2.100:564-chardev:fs9p',
            '-device', 'virtio-net-pci,netdev=n0,disable-legacy=on']


def need(*tools):
    for tool in tools:
        if shutil.which(tool) is None:
            sys.exit('missing host tool: ' + tool)


def export_source():
    dirty = sh('git', 'status', '--porcelain', '--', '.', ':!et', cwd=SOURCE).strip()
    if dirty:
        sys.exit('commit kernel source changes first (et/ may be dirty):\n' + dirty)
    commit = sh('git', 'rev-parse', 'HEAD', cwd=SOURCE).strip()
    src = WORK / 'src'
    tmp = pathlib.Path(tempfile.mkdtemp(prefix='.src-', dir=WORK))
    try:
        # Keep the exact committed bytes whatever the caller's Git settings.
        tar = subprocess.Popen(['git', '-c', 'core.autocrlf=false', '-c', 'core.eol=lf',
                                '-c', 'core.attributesFile=/dev/null', 'archive',
                                '--format=tar', commit, ':(exclude)et'],
                               cwd=SOURCE, stdout=subprocess.PIPE)
        try:
            subprocess.run(['tar', '-x', '-C', str(tmp)], stdin=tar.stdout, check=True)
        finally:
            tar.stdout.close()
            if tar.wait():
                raise RuntimeError('git archive failed')
        if src.exists():
            shutil.rmtree(src)
        tmp.rename(src)
    finally:
        if tmp.exists():
            shutil.rmtree(tmp)
    return commit


def support_id():
    h = hashlib.sha256()
    files = list((ROOT / 'guest').glob('*.rc')) + list((ROOT / 'tests').glob('*.c'))
    for path in sorted(files):
        h.update(path.relative_to(ROOT).as_posix().encode() + b'\0')
        data = path.read_bytes()
        h.update(str(len(data)).encode() + b'\0' + data)
    return h.hexdigest()


def foreign_owned(*roots):
    """Return the first directory not owned by the caller, if any."""
    uid = os.getuid()
    for root in roots:
        for d, _, _ in os.walk(root):
            if os.lstat(d).st_uid != uid:
                return d
    return None


def up(a):
    if not DOCKER:
        sys.exit('ET_DOCKER is empty')
    need(DOCKER[0], 'git', 'tar', 'qemu-system-x86_64')
    if qemu_pid('amd64') is not None or qemu_pid('riscv64') is not None:
        sys.exit('a workspace guest is already running; use build/cmd/test or down first')
    foreign = foreign_owned(WORK / 'src', WORK / 'rw')
    if foreign:
        sys.exit('guest-owned files remain (' + foreign + '); run.py down restores ownership')
    rw = WORK / 'rw'
    lock = rw / 'log/lock'
    if lock.is_dir():
        # No guest of this workspace is running, so no job can hold the lock.
        print('removing stale build lock; rw/log/build.log and build.status are kept')
        lock.rmdir()
    for d in ('log', 'exports', 'tests', 'host', 'top'):
        (rw / d).mkdir(parents=True, exist_ok=True)
    commit = export_source()
    (rw / 'source-commit').write_text(commit + '\n')
    for f in (ROOT / 'guest').glob('*.rc'):
        shutil.copy(f, rw / f.name)
    for f in (ROOT / 'tests').glob('*.c'):
        shutil.copy(f, rw / 'tests' / f.name)
    support = support_id()
    (rw / 'support-id').write_text(support + '\n')
    save(commit=commit, support=support, image=IMAGE)
    kernel = WORK / '9pc64'
    if not kernel.exists() or hashlib.sha256(kernel.read_bytes()).hexdigest() != KERNEL_SHA256:
        data = subprocess.run(DOCKER + ['run', '--rm', '--network', 'none', '--read-only',
                                       '--cap-drop', 'ALL', '--security-opt', 'no-new-privileges',
                                       '--platform', 'linux/amd64', IMAGE, 'kernel'],
                              check=True, capture_output=True).stdout
        if hashlib.sha256(data).hexdigest() != KERNEL_SHA256:
            sys.exit('container kernel does not match the pinned SHA-256')
        kernel.write_bytes(data)
    port = freeport()
    try:
        serve(container_name(), port, [(rw, '/9front/rw'), (WORK / 'src', '/9front/rw/src')])
        save(port=port)
        qemu('amd64', ['qemu-system-x86_64', '-nodefaults', '-machine', 'pc,accel=kvm:tcg',
                       '-cpu', 'max', '-m', '2G', '-smp', '4', '-kernel', str(kernel)]
             + nineboot(port, 'sysname=build'))
        wait_ready('amd64')
    except BaseException:
        try:
            stop_qemu('amd64')
        finally:
            remove_container(container_name())
        raise
    print('ready: source', commit, '\nworkspace', WORK)


def build(a):
    if qemu_pid('amd64') is None:
        sys.exit('start the build guest first: run.py up')
    rw = WORK / 'rw'
    if a.kernel and not (rw / 'full-build').is_file():
        sys.exit('kernel-only rebuild requires a successful full build in this workspace')
    log = rw / 'log'
    lock = log / 'lock'
    status_path = log / 'build.status'
    path = log / 'build.log'
    serial_delay()
    # Hold the console first: if it is busy or unreachable, nothing changes.
    with serial('amd64') as console:
        try:
            lock.mkdir()
        except FileExistsError:
            sys.exit('a build is running or crashed; inspect rw/log/build.log and rw/log/lock')
        status_path.unlink(missing_ok=True)
        # Truncation keeps the old file, so the host could stream a previous
        # build's log before the guest rewrites it. Start from a new file.
        path.unlink(missing_ok=True)
        exports = rw / 'exports'
        for name in ('source-commit', 'support-id'):
            (exports / name).unlink(missing_ok=True)
        if not a.kernel:
            (rw / 'full-build').unlink(missing_ok=True)
        arg = ' kernel' if a.kernel else ''
        # rc's & leaves $status unchanged; a job that cannot run still
        # writes build.status and removes the lock.
        command('amd64', '{rc /rw/build.rc' + arg + ' >/rw/log/build.log >[2=1]; '
                'echo $status >/rw/log/build.status; rm /rw/log/lock} &', client=console)
    pos = 0

    def pump():
        nonlocal pos
        if path.exists():
            with path.open('rb') as f:
                f.seek(pos)
                chunk = f.read()
            pos += len(chunk)
            sys.stdout.write(chunk.decode(errors='replace'))
            sys.stdout.flush()

    def done():
        return status_path.exists() and status_path.stat().st_size > 0

    while not done():
        pump()
        if qemu_pid('amd64') is None:
            sys.exit('build guest exited without a status; inspect rw/log/build.log')
        # The guest writes its status before removing the lock: look again.
        if not lock.exists() and not done():
            sys.exit('build ended without a status; inspect rw/log/build.log')
        time.sleep(1)
    pump()
    result = status_path.read_text().strip()
    # The guest writes status, then removes its empty directory lock. Avoid
    # racing that tiny window while still refusing a genuinely orphaned lock.
    for _ in range(50):
        if not lock.exists():
            break
        time.sleep(0.1)
    if lock.exists():
        sys.exit('build finished but left its lock; inspect rw/log/lock')
    if result or not re.search(r'^BUILD-PASS$', path.read_text(), re.M):
        sys.exit('BUILD-FAIL ' + result)
    exports = require_exports()
    built = (exports / 'source-commit').read_text().strip()
    if not a.kernel and not (rw / 'full-build').is_file():
        sys.exit('BUILD-FAIL missing full-build completion record')
    print((exports / 'SHA256SUMS').read_text(), end='')
    if a.kernel:
        print('kernel-only: userland and cross tools from full build',
              (rw / 'full-build').read_text().strip())
    print('BUILD-PASS', built)


def cmd(a):
    out, status = command('amd64', a.text, a.timeout)
    print(out, end='')
    if status:
        sys.exit('status: ' + status)


def checks(vm, cmds):
    for c, expect in cmds:
        print('+ ' + c)
        out, status = command(vm, c, 180)
        print(out, end='')
        if status:
            sys.exit('FAIL: ' + status)
        if expect is not None and expect not in out:
            sys.exit('FAIL missing marker: ' + expect)


def require_exports():
    exports = WORK / 'rw/exports'
    s = state()
    try:
        built = (exports / 'source-commit').read_text().strip()
        support = (exports / 'support-id').read_text().strip()
    except OSError:
        built = support = None
    if not s.get('commit') or built != s['commit'] or support != s.get('support'):
        sys.exit('exports do not match the workspace source/support; run.py build first')
    log = WORK / 'rw/log'
    try:
        failed = (log / 'lock').exists() or (log / 'build.status').read_text().strip()
    except OSError:
        failed = True
    if failed:
        sys.exit('the last build did not finish successfully; inspect rw/log')
    required = {'9rv64', '9rv64.elf.sym', '9maxion', '9maxion.elf.sym'}
    try:
        lines = (exports / 'SHA256SUMS').read_text().splitlines()
        sums = {name: sha for sha, name in (line.split() for line in lines)}
        if len(lines) != len(required) or set(sums) != required:
            raise ValueError('wrong or duplicate export file set')
        for name in required:
            if hashlib.sha256((exports / name).read_bytes()).hexdigest() != sums[name]:
                raise ValueError('hash mismatch: ' + name)
    except (OSError, ValueError) as e:
        sys.exit('invalid build exports: ' + str(e))
    return exports


def test(a):
    need('qemu-system-riscv64')
    if (WORK / 'rw/log/lock').exists():
        sys.exit('a build is active or left a lock; inspect rw/log before testing')
    exports = require_exports() if a.generic or not a.kernel else WORK / 'rw/exports'
    kernel = (a.kernel or exports / '9maxion').resolve()
    if not kernel.is_file():
        sys.exit('build first: ' + str(kernel))
    if a.generic:
        if not DOCKER:
            sys.exit('ET_DOCKER is empty')
        need(DOCKER[0])
        if not (exports / '9rv64').is_file():
            sys.exit('build the generic kernel first: ' + str(exports / '9rv64'))
    stop_qemu('riscv64')
    base = ['qemu-system-riscv64', '-nodefaults', '-machine', 'virt', '-bios', 'default',
            '-m', '1G', '-smp', '1']
    # The generic kernel relies on Sstc, Zicbom and Svpbmt; the ET-SOC1 Maxion
    # hart has none of them, so its emulation must not offer them either.
    generic_cpu = ['-cpu', 'rv64,svpbmt=true,zicbom=true,sstc=true']
    maxion_cpu = ['-cpu', 'rv64,svpbmt=false,zicbom=false,sstc=false']
    try:
        if a.generic:
            port = freeport()
            try:
                serve(container_name('-rv'), port,
                      [(WORK / 'rw', '/9front/rw'), (WORK / 'src', '/9front/rw/src'),
                       (WORK / 'rw/host/riscv64', '/9front/riscv64')])
                qemu('riscv64', base + generic_cpu + ['-kernel', str(exports / '9rv64')]
                     + nineboot(port, ''))
                wait_ready('riscv64')
                full = WORK / 'rw/full-build'
                if full.is_file():
                    print('userland from full build', full.read_text().strip())
                # A failing layer must report a status, not exit the console
                # shell, so try it in a child namespace first.
                checks('riscv64', [
                    ('@{rfork n; . /rw/layer.rc} && . /rw/layer.rc && echo $cputype && cat /dev/sysstat',
                     'riscv64'),
                    ('cd /rw/tests; jc -o /tmp/smoke.j smoke.c && jl -o /tmp/smoke /tmp/smoke.j && /tmp/smoke', 'PASS: native riscv64'),
                    ('cd /rw/tests; pcc -o /tmp/ape ape.c && /tmp/ape', 'PASS: APE on riscv64'),
                    ('cd /rw/tests; jc -o /tmp/atomic.j atomic.c && jl -o /tmp/atomic /tmp/atomic.j && /tmp/atomic && echo ATOMIC-PASS', 'ATOMIC-PASS')])
                print('GENERIC-CHECK-PASS')
            finally:
                try:
                    stop_qemu('riscv64')
                finally:
                    remove_container(container_name('-rv'))
        qemu('riscv64', base + maxion_cpu + ['-kernel', str(kernel), '-nic', 'none',
                                             '-append', 'console=0 user=glenda *maxion=1 *ncpu=1 *uartpoll=1'])
        wait_ready('riscv64')
        checks('riscv64', [
            ('echo $cputype; cat /dev/cpuid; cat /dev/sysstat', None),
            ('/bin/smoke && ps', 'MAXION-SMOKE-PASS')])
        print('CHECK-PASS', kernel)
    finally:
        stop_qemu('riscv64')


def down(a):
    if not DOCKER:
        sys.exit('ET_DOCKER is empty')
    need(DOCKER[0])
    if (WORK / 'rw/log/lock').exists() and qemu_pid('amd64') is not None:
        sys.exit('a build is active or left a lock; inspect it before down')
    stop_qemu('amd64')
    stop_qemu('riscv64')
    for name in (container_name(), container_name('-rv')):
        remove_container(name)
    # u9fs runs as container root; hand the workspace back to the caller.
    for d in ('rw', 'src'):
        if (WORK / d).exists():
            docker('run', '--rm', '--network', 'none', '--platform', 'linux/amd64',
                   '--entrypoint', '/bin/chown',
                   '-v', f'{WORK / d}:/w', IMAGE, '-R', f'{os.getuid()}:{os.getgid()}', '/w')
    print('stopped', WORK)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest='command', required=True)
    sub.add_parser('up').set_defaults(run=up)
    b = sub.add_parser('build')
    b.add_argument('--kernel', action='store_true', help='kernels only, on top of a previous full build')
    b.set_defaults(run=build)
    c = sub.add_parser('cmd')
    c.add_argument('text')
    c.add_argument('-t', '--timeout', type=float, default=60)
    c.set_defaults(run=cmd)
    t = sub.add_parser('test')
    t.add_argument('--kernel', type=pathlib.Path, help='9maxion to test (default: rw/exports/9maxion)')
    t.add_argument('--generic', action='store_true', help='also boot 9rv64 with the built userland over 9P')
    t.set_defaults(run=test)
    sub.add_parser('down').set_defaults(run=down)
    a = p.parse_args(argv)
    if any(c in str(WORK) for c in (',', ':')) or any(c.isspace() for c in str(WORK)):
        sys.exit('ET_WORK must not contain spaces, commas or colons')
    WORK.mkdir(parents=True, exist_ok=True)
    try:
        with operation(a.command):
            a.run(a)
    except subprocess.CalledProcessError as e:
        sys.exit(f'{" ".join(map(str, e.cmd))}: {e.stderr or e.stdout}')
    except (OSError, RuntimeError, ValueError) as e:
        sys.exit(str(e))
    except KeyboardInterrupt:
        sys.exit('interrupted; a submitted guest build may still be running')


if __name__ == '__main__':
    main()
