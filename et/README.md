# Build and test the Maxion kernel

Run `scripts/run.py` from this repository's root. It exports committed 9front
source and runs a Plan 9 amd64 build guest in host QEMU. A digest-pinned
container supplies the read-only base filesystem over 9P. No prepared VM
image, host RISC-V cross-compiler, external launcher or hardware is required.

## Prerequisites

- Linux with Python 3.11+ (standard library only), Git and `tar`.
- Docker CLI and a working daemon capable of running `linux/amd64` images.
- `qemu-system-x86_64` for building and `qemu-system-riscv64` for testing,
  with user-mode networking and generic RISC-V firmware installed.
  QEMU 8.2.2 is the tested version; the generic CPU must support the
  `svpbmt`, `zicbom` and `sstc` properties.
- POSIX `awk` for the host-only regression tests.

The development environment used Python 3.12.13 and rootful Docker 29.1.3.
Rootless Docker and Podman are untested. Run Python and QEMU as your normal
user, with access to Docker. KVM is optional; the builder falls back to TCG.

The runner pins this container, which Docker may fetch on first use:

```text
ghcr.io/rvs/9front-amd64@sha256:131b06bdc301a87c1e861e548a79e08f81ce52afe3297b6258f08a63dab194d2
```

It extracts the container's `9pc64` bootstrap kernel and checks SHA-256:

```text
7d40d3216ef5f51ab0b980e1b7bc2e94f43a889a7f59ba9c57b7e6272da8019e
```

The container supplies the initial Plan 9 tools and filesystem; the full
recipe builds this checkout's RISC-V cross-tools, libraries, APE and commands
before building both kernels. Once the image is cached, no network download
is needed. No Maxion hardware firmware or DTB is fetched.

## First build on a clean workspace

Commit intended source changes first. Run these commands sequentially from
this repository's root, proceeding only when each succeeds:

```sh
python3 -m unittest discover -s et/tests -p 'test_*.py' -v
python3 et/scripts/run.py up
python3 et/scripts/run.py build
python3 et/scripts/run.py test --generic
python3 et/scripts/run.py down
```

No environment overrides are required with normally installed tools.
By default all mutable state is under ignored `et/tmp/work/`. Optional
`ET_WORK` selects another workspace; use the same short absolute path for
all commands, without spaces, commas or colons. `ET_DOCKER` can select a
Docker wrapper or command with arguments. Neither override is a dependency.
Use a new workspace for an independent full build; do not move a running one.

`up` exports committed `HEAD` using `git archive`, excluding `et/`, and
refuses dirty/untracked source outside `et/`. It separately copies and
fingerprints `et/guest/*.rc` and `et/tests/*.c`. Source objects and outputs
stay in the workspace, never in the checkout or base image.

`build` waits for completion and streams `rw/log/build.log`. Require a zero
host exit and terminal **BUILD-PASS**, not merely submission of a job.
It builds `9rv64` and `9maxion` (`CONF=maxion`, `loadaddr=0x8600200000`).
Exports are in `et/tmp/work/rw/exports/` (or `$ET_WORK/rw/exports/`):

```text
9rv64                 9rv64.elf.sym
9maxion               9maxion.elf.sym
SHA256SUMS            source-commit          support-id
```

`test --generic` first boots the new generic kernel with the built userland
and runs native, APE and atomic tests. It then boots the Maxion RAM-root
kernel and its embedded smoke test. Require **GENERIC-CHECK-PASS**,
**MAXION-SMOKE-PASS** and **CHECK-PASS**. Plain `test` checks only Maxion.
Both use one hart and 1 GiB. Generic enables Sstc/Svpbmt/Zicbom; Maxion
explicitly disables them. Tests check build status, source/support identities
and export hashes before accepting the default kernels.

The RISC-V test guest stops automatically; `down` stops the amd64 builder
and containers and restores workspace ownership. RISC-V tests use QEMU's
`-bios default` firmware and virtual DT, not hardware Maxion OpenSBI/DTB.
Record the installed QEMU version with results. This is not hardware
qualification or a kernel byte-reproducibility claim.

## Kernel-only iteration

After a successful full build, the workspace's tools, libraries and userland
can be reused. If source or support files change, stop the guest, commit the
source, and run `up` again before rebuilding:

```sh
python3 et/scripts/run.py down
# Commit intended edits here.
python3 et/scripts/run.py up
python3 et/scripts/run.py build --kernel
python3 et/scripts/run.py test --generic
python3 et/scripts/run.py down
```

With unchanged inputs, omit `down`/`up` while the builder is still running.
Kernel-only mode reports the full-build commit it reuses. Use plain `build`
after compiler, header, shared-library or userland changes, or if reuse is
uncertain. A failed full build withdraws permission for kernel-only reuse.

## Logs, recovery and safety

Each build replaces its log/status; copy them first if retaining results.
Build log, status and lock live in `rw/log/`; console logs are in
`amd64/console.log` and `riscv64/console.log`. Starting a build invalidates
old export acceptance metadata. The generic console is replaced by the
subsequent Maxion boot; the test command's output retains both check results.

Workspace operations and serial clients are locked. `down` refuses an active
build. A timeout or Ctrl-C on the host does not prove the guest job stopped:
inspect the log, status, lock and process state before retrying or dismantling.
Never fake completion markers or bypass locks. If a guest died with its job,
`down` then `up` is the supported recovery; stale locks are cleared only when
no workspace guest runs. `down` also restores guest-owned workspace files.

For short commands at an idle shell use
`python3 et/scripts/run.py cmd 'rc command'`. This is Plan 9 rc, not sh; see root `AGENTS.md` for quoting.
Never source `/rw/layer.rc` into the amd64 console shell: use a child namespace
(`@{rfork n; . /rw/layer.rc; ...}`). RAM-backed guest `/tmp` handles scratch;
persistent output belongs in `/rw`.

The container's unauthenticated 9P listener is loopback-only but writable by
local clients; use a trusted single-user host/VM. Do not expose it to a network.
No host devices are passed through. Do not run QEMU as root. Local emulation
does not authorize card/UART access, flashing, resets or deployment.
