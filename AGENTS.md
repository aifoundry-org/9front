# AGENTS.md — 9front RISC-V / Maxion

## Scope and workflow

Kernel changes live in `sys/src/9/riscv64/`; shared kernel code is in
`sys/src/9/port/`. Read `et/README.md` for prerequisites, exact container and
bootstrap pins, and the build/test contract. The only host entry point is
`et/scripts/run.py`; no `~/p9` launcher, prepared VM or host cross-compiler
is required. Run host commands from the repository root.

Docker supplies the immutable amd64 Plan 9 root; host QEMU runs as a normal
user. Python/QEMU/Docker are external prerequisites, not rebuilt by this tree.
All runtime state belongs under ignored `et/tmp/`, default `et/tmp/work/`.
An explicit `ET_WORK` may select another short absolute workspace without
spaces, commas or colons; keep it identical across participating commands.
Do not move a running workspace. Keep generated binaries, images and logs out
of Git; retain source, recipes, tests and concise instructions only.

## Build and test

Commit intended source changes before `up`: it exports committed `HEAD`
excluding `et/`. Guest recipes and C tests are copied/fingerprinted separately.
For a clean reproduction choose a fresh workspace, then run sequentially:

```sh
python3 -m unittest discover -s et/tests -p 'test_*.py' -v
python3 et/scripts/run.py up
python3 et/scripts/run.py build
python3 et/scripts/run.py test --generic
python3 et/scripts/run.py down
```

The full build rebuilds cross-tools, libraries, APE, commands and both kernels.
Require terminal `BUILD-PASS` and successful host exit; submission is not
completion. Require `GENERIC-CHECK-PASS`, `MAXION-SMOKE-PASS` and `CHECK-PASS`
from `test --generic`. Plain `test` checks only Maxion. Tests use QEMU generic
firmware/virtual DT (`-bios default`), not hardware firmware; record QEMU's
version. A test pass is not hardware qualification or byte reproducibility.

`.github/workflows/kernel.yml` repeats the full build/test lifecycle on every
push to a fresh hosted runner, uploading validated kernels and diagnostic logs
as run artifacts. Keep action references pinned to full commits, permissions
read-only and build state uncached. CI artifacts are not releases or deployment.
Changing this workflow does not authorize pushing or triggering it remotely.

After a successful full build, `build --kernel` reuses its tools/libraries and
userland. After changing source/support: `down`, commit intended source, `up`,
then rebuild. Use a full `build` after compiler, shared-library, header or
userland changes, or whenever reuse is doubtful. Failed full builds invalidate
the full-build record; never force reuse of partial results.

Build log/status/lock: `$ET_WORK/rw/log/` (default `et/tmp/work/rw/log/`).
Exports/hashes/source and support identities: `$ET_WORK/rw/exports/`.
Each build replaces its log/status; preserve old results before resubmitting.
Verify actual completion, artifact identities and shutdown before reporting
success. Keep failed attempts separate; do not relabel previous build results
with new commit IDs after squashing or rewriting history.

## Locking and console

The runner serializes workspace operations and serial clients. `down` refuses
active builds. Host interruption or timeout does not prove a guest command
stopped: inspect host-side logs and guest/process state before resubmitting,
testing or dismantling. Never fabricate markers or bypass locks. Once a guest
and its job are known dead, use `down` then `up` for stale-lock recovery.

Use `run.py build` for recipes. For short commands at an idle shell, use
`python3 et/scripts/run.py cmd 'rc command'` (default timeout 60 seconds).
Monitor builds via the host log, not console input. Do not use tmux send-keys
or type into an interactive foreground command.

- Commands are parsed by **rc**, not sh. Quotes are single quotes; `"` is not
  special. No backslash escapes. Redirect stderr with `>[2=1]`; use `$status`.
- Quote regexes: `grep '^==' file` and `grep 'FAIL|PASS' file`. `^` concatenates
  and `=` assigns in rc; unquoted regexes can kill the completion command.
- From bash preserve rc quotes with a quoted heredoc, then pass the variable:

  ```sh
  cmd=$(cat <<'RC'
  grep 'STAGE-|BUILD-PASS|FAIL' /rw/log/build.log
  RC
  )
  python3 et/scripts/run.py cmd "$cmd"
  ```

- Never let `cat`, `grep`, etc. read console stdin: supply a file or explicit
  input/pipe. A timeout does not mean the command did not execute.
- Do not source `/rw/layer.rc` into the amd64 console shell. Use a child
  namespace: `@{rfork n; . /rw/layer.rc; ...}`. Guest `/tmp` is RAM-backed;
  persistent logs/exports stay in `/rw`.
- Plan 9 unions are not recursive: `layer.rc` recreates only hidden base
  directories. Do not copy a whole base skeleton over the source export.

## Safety and commits

Loopback 9P is unauthenticated and writable by local clients: use a trusted
single-user host/VM. Do not expose it publicly or run QEMU as root. Local QEMU
validation does not authorize hardware/card/UART access, flashing, resets,
driver changes or deployment.

Never push, publish releases or trigger remote workflows without explicit
approval for the action and target. Local commit preparation is not approval
to publish. Downstream commits use Roman V Shaposhnik <rvs@apache.org> as author
and committer, with `Co-authored-by: Shelley <shelley@exe.dev>`. Preserve
upstream history and authorship.

## C coding standard

All C written for 9front **must follow the Plan 9 C style** (`style(6)`,
`/sys/doc/comp.ms`, and the surrounding kernel sources):

- Tabs for indentation, never spaces; `/* */` comments only, no `//`.
- Function layout: return type on its own line, name at column 0, opening
  brace on its own line. `if`/`for`/`while` braces on the same line.
- All declarations at the top of the block; no mixed declarations and code,
  no `for(int i...)`.
- Plan 9 types and libs: `u8int`/`u32int`/`uintptr`/`uvlong`, `nil` not
  `NULL`, `nelem()`, `print`/`fprint`/`snprint`, `malloc`/`mallocz`,
  `USED()`, `SET()`. Includes: `<u.h>`, `<libc.h>` (kernel: `u.h`,
  `../port/lib.h`, `mem.h`, `dat.h`, `fns.h`, `io.h`).
- No `const` decoration, no `static inline`, no C99/C11-isms the Plan 9
  compilers (`?c`) don't accept. No `#ifdef` configuration — use `mkfile`s
  and the port/arch split instead.
- Short lowercase identifiers, no camelCase. Kernel errors via
  `error()`/`waserror()`/`poperror()`; user space via `werrstr()`/`sysfatal()`.
- Match the conventions of the file being edited.
