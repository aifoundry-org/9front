"""Host-only tests for the container/QEMU runner; no VM or Docker is opened."""
import contextlib
import fcntl
import hashlib
import importlib.util
import io
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

ET = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('et_run', ET / 'scripts/run.py')
run = importlib.util.module_from_spec(spec)
spec.loader.exec_module(run)
MARKER = 'DONE0123456789abcdef'
UUID = types.SimpleNamespace(hex='0123456789abcdef0000000000000000')


class RunnerTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='et-runner-test-')
        self.addCleanup(self.temp.cleanup)
        self.base = Path(self.temp.name)
        self.work = self.base / 'work'
        self.work.mkdir()
        (self.work / 'amd64').mkdir()
        self.addCleanup(patch.stopall)
        patch.object(run, 'WORK', self.work).start()
        patch.object(run, 'STATE', self.work / 'state.json').start()

    def serial(self, chunks, env=None):
        with patch.dict(os.environ, env or {'P9_SERIAL_DELAY': '0'}, clear=True), \
             patch.object(run.socket, 'socket') as socket, \
             patch.object(run.uuid, 'uuid4', return_value=UUID), \
             patch.object(run.time, 'sleep') as sleep:
            sock = socket.return_value.__enter__.return_value
            sock.recv.side_effect = chunks
            result = run.command('amd64', 'echo hello')
            sent = b''.join(call.args[0] for call in sock.sendall.call_args_list)
            delays = [call.args[0] for call in sleep.call_args_list]
        return result, sent, delays, socket

    def test_serial_status_fragmentation_and_pacing(self):
        result, sent, _, _ = self.serial([
            b'input\nfailure\nDONE01234567', b'89abcdef build\n'])
        self.assertEqual(result, ('failure\n', 'build'))
        self.assertEqual(sent.count(b'\n'), 1)
        self.assertIn(b'; result=$status; sleep 1; echo; echo ', sent)
        self.assertIn(b"'DONE'^'0123456789abcdef' $result", sent)
        result, sent, delays, _ = self.serial([
            ('input\nhello\n\n' + MARKER + ' \n').encode()],
            {'P9_SERIAL_DELAY': '0.01'})
        self.assertEqual(result, ('hello\n\n', ''))
        self.assertEqual(delays, [0.01] * len(sent))

    def test_serial_rejects_bad_pacing_and_never_retries(self):
        for value in ('-1', 'nan', 'inf', 'nonsense'):
            with self.subTest(value=value), \
                 patch.dict(os.environ, {'P9_SERIAL_DELAY': value}), \
                 patch.object(run.socket, 'socket') as socket:
                with self.assertRaises(ValueError):
                    run.command('amd64', 'echo hello')
                socket.assert_not_called()
        with self.assertRaisesRegex(RuntimeError, 'command may have executed'):
            self.serial([b'input\nhello\nDOE0123456789abcdef \n', b''])

    def test_serial_client_lock(self):
        with (self.work / 'amd64/serial-client.lock').open('a') as held:
            fcntl.flock(held, fcntl.LOCK_EX | fcntl.LOCK_NB)
            with patch.dict(os.environ, {'P9_SERIAL_DELAY': '0'}), \
                 patch.object(run.socket, 'socket') as socket:
                with self.assertRaisesRegex(RuntimeError, 'serial console busy'):
                    run.command('amd64', 'echo hello')
                socket.assert_not_called()

    def git(self, repo, *args):
        return subprocess.check_output(['git', '-C', str(repo), *args], text=True).strip()

    def test_source_export_is_exact_committed_snapshot(self):
        repo = self.base / 'repo'
        (repo / 'sys').mkdir(parents=True)
        (repo / 'et').mkdir()
        self.git(repo, 'init', '-q')
        self.git(repo, 'config', 'user.name', 'Test')
        self.git(repo, 'config', 'user.email', 'test@localhost')
        (repo / 'sys/old').write_text('old\n')
        (repo / 'sys/keep').write_text('one\n')
        (repo / 'et/excluded').write_text('not for guest\n')
        self.git(repo, 'add', 'sys/old', 'sys/keep', 'et/excluded')
        self.git(repo, 'commit', '-qm', 'first')
        patch.object(run, 'SOURCE', repo).start()
        first = run.export_source()
        self.assertEqual(first, self.git(repo, 'rev-parse', 'HEAD'))
        self.assertFalse((self.work / 'src/et').exists())
        (self.work / 'src/sys/generated.j').write_text('stale\n')
        (repo / 'sys/old').unlink()
        (repo / 'sys/keep').write_text('two\n')
        (repo / 'sys/new').write_text('new\n')
        self.git(repo, 'add', 'sys/old', 'sys/keep', 'sys/new')
        self.git(repo, 'commit', '-qm', 'second')
        second = self.git(repo, 'rev-parse', 'HEAD')
        self.assertEqual(second, run.export_source())
        self.assertFalse((self.work / 'src/sys/old').exists())
        self.assertFalse((self.work / 'src/sys/generated.j').exists())
        self.assertEqual((self.work / 'src/sys/keep').read_text(), 'two\n')
        self.assertTrue((self.work / 'src/sys/new').exists())
        (repo / 'et/dirty').write_text('workflow changes are separate\n')
        self.assertEqual(run.export_source(), second)
        (repo / 'sys/keep').write_text('uncommitted\n')
        with self.assertRaisesRegex(SystemExit, 'commit kernel source'):
            run.export_source()
        self.assertEqual((self.work / 'src/sys/keep').read_text(), 'two\n')
        # A caller's Git line-ending settings must not alter exported bytes.
        (repo / 'sys/keep').write_text('two\n')
        attrs = self.base / 'attributes'
        attrs.write_text('* text eol=crlf\n')
        self.git(repo, 'config', 'core.autocrlf', 'true')
        self.git(repo, 'config', 'core.attributesFile', str(attrs))
        plain = subprocess.run(['git', '-C', str(repo), 'archive', 'HEAD', 'sys/keep'],
                               capture_output=True, check=True).stdout
        self.assertIn(b'two\r\n', plain)
        self.assertEqual(run.export_source(), second)
        self.assertEqual((self.work / 'src/sys/keep').read_bytes(), b'two\n')

    def write_exports(self, support='support'):
        exports = self.work / 'rw/exports'
        exports.mkdir(parents=True, exist_ok=True)
        names = ('9rv64', '9rv64.elf.sym', '9maxion', '9maxion.elf.sym')
        lines = []
        for name in names:
            data = (name + '\n').encode()
            (exports / name).write_bytes(data)
            lines.append(hashlib.sha256(data).hexdigest() + '  ' + name)
        (exports / 'SHA256SUMS').write_text('\n'.join(lines) + '\n')
        (exports / 'source-commit').write_text('commit\n')
        (exports / 'support-id').write_text(support + '\n')
        (self.work / 'rw/log').mkdir(exist_ok=True)
        (self.work / 'rw/log/build.status').write_text('\n')
        return exports

    def test_export_identity_and_hashes(self):
        exports = self.write_exports()
        run.save(commit='commit', support='support')
        self.assertEqual(run.require_exports(), exports)
        (exports / '9maxion').write_text('stale\n')
        with self.assertRaisesRegex(SystemExit, 'hash mismatch'):
            run.require_exports()
        self.write_exports()
        run.save(support='changed')
        with self.assertRaisesRegex(SystemExit, 'source/support'):
            run.require_exports()

    def test_exports_need_a_successful_last_build(self):
        self.write_exports()
        run.save(commit='commit', support='support')
        status = self.work / 'rw/log/build.status'
        status.write_text('build\n')
        with self.assertRaisesRegex(SystemExit, 'did not finish successfully'):
            run.require_exports()
        status.unlink()
        with self.assertRaisesRegex(SystemExit, 'did not finish successfully'):
            run.require_exports()
        status.write_text('\n')
        (self.work / 'rw/log/lock').mkdir()
        with self.assertRaisesRegex(SystemExit, 'did not finish successfully'):
            run.require_exports()
        (self.work / 'rw/log/lock').rmdir()
        run.require_exports()

    def test_build_lock_is_synchronous_and_full_record_is_required(self):
        rw = self.work / 'rw'
        (rw / 'log').mkdir(parents=True)
        (rw / 'exports').mkdir()
        run.save(commit='commit', support='support')
        args = types.SimpleNamespace(kernel=True)
        with patch.object(run, 'qemu_pid', return_value=123), \
             patch.object(run, 'serial', return_value=contextlib.nullcontext()):
            with self.assertRaisesRegex(SystemExit, 'requires a successful full build'):
                run.build(args)
            (rw / 'full-build').write_text('old\n')
            (rw / 'log/lock').mkdir()
            with patch.object(run, 'command') as command:
                with self.assertRaisesRegex(SystemExit, 'running or crashed'):
                    run.build(types.SimpleNamespace(kernel=False))
                command.assert_not_called()
            (rw / 'log/lock').rmdir()
            (rw / 'exports/source-commit').write_text('stale\n')
            (rw / 'exports/support-id').write_text('stale\n')

            def complete(vm, text, client=None):
                self.assertTrue((rw / 'log/lock').is_dir())
                self.assertFalse((rw / 'exports/source-commit').exists())
                self.assertFalse((rw / 'exports/support-id').exists())
                self.assertFalse((rw / 'full-build').exists())
                self.write_exports()
                (rw / 'full-build').write_text('commit\n')
                (rw / 'log/build.log').write_text('STAGE-kernel\nBUILD-PASS\n')
                (rw / 'log/build.status').write_text('\n')
                return '', ''

            with patch.object(run, 'command', side_effect=complete) as command, \
                 patch.object(run.time, 'sleep', side_effect=lambda _: (rw / 'log/lock').rmdir()), \
                 contextlib.redirect_stdout(io.StringIO()) as output:
                run.build(types.SimpleNamespace(kernel=False))
            command.assert_called_once()
            self.assertIn('BUILD-PASS commit', output.getvalue())

    def test_build_with_busy_console_changes_nothing(self):
        rw = self.work / 'rw'
        exports = self.write_exports()
        (rw / 'full-build').write_text('commit\n')
        (rw / 'log/build.log').write_text('BUILD-PASS\n')
        with (self.work / 'amd64/serial-client.lock').open('a') as held, \
             patch.dict(os.environ, {'P9_SERIAL_DELAY': '0'}), \
             patch.object(run, 'qemu_pid', return_value=123):
            fcntl.flock(held, fcntl.LOCK_EX | fcntl.LOCK_NB)
            with self.assertRaisesRegex(RuntimeError, 'serial console busy'):
                run.build(types.SimpleNamespace(kernel=False))
        self.assertFalse((rw / 'log/lock').exists())
        self.assertTrue((exports / 'source-commit').is_file())
        self.assertTrue((exports / 'support-id').is_file())
        self.assertTrue((rw / 'full-build').is_file())
        self.assertEqual((rw / 'log/build.status').read_text(), '\n')
        self.assertEqual((rw / 'log/build.log').read_text(), 'BUILD-PASS\n')

    def test_build_status_written_just_before_lock_removal(self):
        rw = self.work / 'rw'
        (rw / 'log').mkdir(parents=True)
        (rw / 'exports').mkdir()
        run.save(commit='commit', support='support')

        def submit(vm, text, client=None):
            self.write_exports()
            (rw / 'log/build.status').unlink()
            (rw / 'full-build').write_text('commit\n')
            (rw / 'log/build.log').write_text('BUILD-PASS\n')
            return '', ''

        calls = []

        def pid(vm):
            # The guest finishes between the host's status and lock checks.
            calls.append(vm)
            if len(calls) == 2:
                (rw / 'log/build.status').write_text('\n')
                (rw / 'log/lock').rmdir()
            return 123

        with patch.object(run, 'serial', return_value=contextlib.nullcontext()), \
             patch.object(run, 'command', side_effect=submit), \
             patch.object(run, 'qemu_pid', side_effect=pid), \
             patch.object(run.time, 'sleep'), \
             contextlib.redirect_stdout(io.StringIO()) as output:
            run.build(types.SimpleNamespace(kernel=False))
        self.assertIn('BUILD-PASS commit', output.getvalue())

    def test_build_streams_only_its_own_log(self):
        rw = self.work / 'rw'
        (rw / 'log').mkdir(parents=True)
        (rw / 'exports').mkdir()
        (rw / 'full-build').write_text('older\n')
        old = 'STAGE-source older\n' + 'previous full build\n' * 50 + 'BUILD-PASS\n'
        (rw / 'log/build.log').write_text(old)
        run.save(commit='commit', support='support')

        def submit(vm, text, client=None):
            # Like the guest's truncation, the new log is shorter than the old.
            self.assertFalse((rw / 'log/build.log').exists())
            self.write_exports()
            (rw / 'log/build.log').write_text('STAGE-source commit\nBUILD-PASS\n')
            (rw / 'log/lock').rmdir()
            return '', ''

        with patch.object(run, 'serial', return_value=contextlib.nullcontext()), \
             patch.object(run, 'command', side_effect=submit), \
             patch.object(run, 'qemu_pid', return_value=123), \
             contextlib.redirect_stdout(io.StringIO()) as output:
            run.build(types.SimpleNamespace(kernel=True))
        self.assertNotIn('previous full build', output.getvalue())
        self.assertIn('STAGE-source commit\nBUILD-PASS\n', output.getvalue())

    def test_kernel_build_names_the_full_build_it_reuses(self):
        rw = self.work / 'rw'
        (rw / 'log').mkdir(parents=True)
        (rw / 'exports').mkdir()
        (rw / 'full-build').write_text('older\n')
        run.save(commit='commit', support='support')

        def submit(vm, text, client=None):
            self.assertTrue(text.startswith('{rc /rw/build.rc kernel '))
            self.write_exports()
            (rw / 'log/build.log').write_text('BUILD-PASS\n')
            (rw / 'log/lock').rmdir()
            return '', ''

        with patch.object(run, 'serial', return_value=contextlib.nullcontext()), \
             patch.object(run, 'command', side_effect=submit), \
             patch.object(run, 'qemu_pid', return_value=123), \
             contextlib.redirect_stdout(io.StringIO()) as output:
            run.build(types.SimpleNamespace(kernel=True))
        self.assertIn('userland and cross tools from full build older', output.getvalue())
        self.assertIn('BUILD-PASS commit', output.getvalue())

    def test_up_refuses_guest_owned_files_then_clears_a_stale_lock(self):
        lock = self.work / 'rw/log/lock'
        lock.mkdir(parents=True)
        (self.work / 'src').mkdir()
        with patch.object(run, 'need'), patch.object(run, 'qemu_pid', return_value=None), \
             patch.object(run, 'export_source', side_effect=RuntimeError('exported')):
            with patch.object(run.os, 'getuid', return_value=os.getuid() + 1):
                with self.assertRaisesRegex(SystemExit, 'guest-owned files remain'):
                    run.up(types.SimpleNamespace())
            self.assertTrue(lock.is_dir())
            with contextlib.redirect_stdout(io.StringIO()) as output, \
                 self.assertRaisesRegex(RuntimeError, 'exported'):
                run.up(types.SimpleNamespace())
        self.assertFalse(lock.exists())
        self.assertIn('removing stale build lock', output.getvalue())

    def test_qemu_pid_requires_the_whole_socket_path(self):
        sock = str(self.work / 'amd64/serial.sock')
        for arg, owned in (('socket,id=ser,path=/x' + sock + ',server=on', False),
                           ('socket,id=ser,path=' + sock + ',server=on', True)):
            with self.subTest(owned=owned):
                child = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(60)', arg])
                try:
                    run.save(amd64_pid=child.pid)
                    self.assertEqual(run.qemu_pid('amd64'), child.pid if owned else None)
                finally:
                    child.kill()
                    child.wait()

    def test_stop_qemu_tolerates_a_process_that_already_exited(self):
        run.save(amd64_pid=123)
        with patch.object(run, 'qemu_pid', side_effect=[123, None, None]), \
             patch.object(run.os, 'kill', side_effect=ProcessLookupError):
            run.stop_qemu('amd64')
        self.assertIsNone(run.state()['amd64_pid'])

    def test_work_path_validation(self):
        for bad in ('/tmp/a b', '/tmp/a,b', '/tmp/a:b', '/tmp/a\tb'):
            with self.subTest(path=bad), patch.object(run, 'WORK', Path(bad)):
                with self.assertRaisesRegex(SystemExit, 'spaces, commas or colons'):
                    run.main(['down'])

    def test_operation_lock_and_support_fingerprint(self):
        with run.operation('build'):
            with self.assertRaisesRegex(RuntimeError, 'another workspace operation'):
                with run.operation('test'):
                    pass
        root = self.base / 'et'
        (root / 'guest').mkdir(parents=True)
        (root / 'tests').mkdir()
        (root / 'guest/build.rc').write_text('one\n')
        (root / 'tests/maxion-smoke.c').write_text('one\n')
        patch.object(run, 'ROOT', root).start()
        first = run.support_id()
        self.assertEqual(first, run.support_id())
        (root / 'tests/maxion-smoke.c').write_text('two\n')
        self.assertNotEqual(first, run.support_id())

    def test_layer_recreates_only_union_hidden_directories(self):
        text = (ET / 'guest/layer.rc').read_text()
        program = re.search(r"^hidden='([^']*)'$", text, re.M)
        self.assertIsNotNone(program)
        self.assertLess(text.index('awk -v'), text.index('must bind'))
        awk = shutil.which('awk')
        self.assertIsNotNone(awk, 'host awk is required')
        lines = ['S /rc', 'S /rc/bin', 'S /rc/lib',
                 'B /rc', 'B /rc/bin', 'B /rc/bin/aux', 'B /rc/bin/aux/sub',
                 'B /rc/bin/postscript', 'B /rc/lib', 'B /rc/ndb', 'B /rc/ndb/sub']
        out = subprocess.run([awk, '-v', 'top=/rc', program.group(1)],
                             input='\n'.join(lines) + '\n', text=True,
                             capture_output=True, check=True).stdout.split()
        self.assertEqual(out, ['/rw/src/rc/bin/aux', '/rw/src/rc/bin/aux/sub',
                               '/rw/src/rc/bin/postscript'])

    def test_generic_kernel_exported_before_maxion_clean(self):
        # portmkfile's clean runs $i.clean for every CONFLIST entry, which
        # deletes 9rv64 and 9rv64.elf.sym before the Maxion build.
        lines = (ET / 'guest/build.rc').read_text().splitlines()
        stage = lines.index('echo STAGE-kernel')
        index = lambda s: next(i for i in range(stage, len(lines)) if lines[i].strip() == s)
        generic = index('must cp 9rv64 9rv64.elf.sym /rw/exports')
        self.assertLess(index('must mk'), index('must mk install'))
        self.assertLess(index('must mk install'), generic)
        self.assertLess(generic, index("must mk 'CONF=maxion' clean"))
        self.assertLess(index("must mk 'CONF=maxion' 'loadaddr=0x8600200000'"),
                        index('must cp 9maxion 9maxion.elf.sym /rw/exports'))
        sums = index('must sha1sum -2 256 9rv64 9rv64.elf.sym 9maxion 9maxion.elf.sym > SHA256SUMS')
        self.assertLess(sums, index('must echo $commit > source-commit'))

    def test_generic_failure_stops_guest_and_removes_server(self):
        exports = self.write_exports()
        run.save(commit='commit', support='support')
        args = types.SimpleNamespace(kernel=None, generic=True)
        with patch.object(run, 'need'), \
             patch.object(run, 'stop_qemu') as stop, \
             patch.object(run, 'serve') as serve, \
             patch.object(run, 'qemu'), \
             patch.object(run, 'wait_ready', side_effect=RuntimeError('not ready')), \
             patch.object(run, 'remove_container') as remove:
            with self.assertRaisesRegex(RuntimeError, 'not ready'):
                run.test(args)
            volumes = serve.call_args.args[2]
            self.assertIn((self.work / 'src', '/9front/rw/src'), volumes)
            self.assertIn((self.work / 'rw/host/riscv64', '/9front/riscv64'), volumes)
            stop.assert_called_with('riscv64')
            remove.assert_called_once_with(run.container_name('-rv'))
            # A failure while stopping QEMU must not leave the 9P server up.
            stop.side_effect = [None, RuntimeError('could not stop'), RuntimeError('could not stop')]
            remove.reset_mock()
            with self.assertRaisesRegex(RuntimeError, 'could not stop'):
                run.test(args)
            remove.assert_called_once_with(run.container_name('-rv'))

    def test_generic_and_maxion_boots_use_their_own_cpu_models(self):
        exports = self.write_exports()
        run.save(commit='commit', support='support')
        args = types.SimpleNamespace(kernel=None, generic=True)
        with patch.object(run, 'need'), patch.object(run, 'stop_qemu'), \
             patch.object(run, 'serve'), patch.object(run, 'remove_container'), \
             patch.object(run, 'qemu') as qemu, patch.object(run, 'wait_ready'), \
             patch.object(run, 'checks') as checks, patch('builtins.print'):
            run.test(args)
        layer, expect = checks.call_args_list[0].args[1][0]
        self.assertTrue(layer.startswith('@{rfork n; . /rw/layer.rc} && . /rw/layer.rc && '))
        self.assertEqual(expect, 'riscv64')
        boots = [c.args[1] for c in qemu.call_args_list]
        self.assertEqual(len(boots), 2)
        cpu = lambda argv: argv[argv.index('-cpu') + 1]
        kernel = lambda argv: argv[argv.index('-kernel') + 1]
        self.assertEqual(kernel(boots[0]), str(exports / '9rv64'))
        self.assertEqual(cpu(boots[0]), 'rv64,svpbmt=true,zicbom=true,sstc=true')
        self.assertEqual(kernel(boots[1]), str((exports / '9maxion').resolve()))
        self.assertEqual(cpu(boots[1]), 'rv64,svpbmt=false,zicbom=false,sstc=false')
        self.assertTrue(all(b.count('-cpu') == 1 for b in boots))


if __name__ == '__main__':
    unittest.main()
