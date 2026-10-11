# AGENTS.md — 9front on Banana Pi F3 (riscv64)

## Build environment

Builds run inside a 9front amd64 VM (`~/p9/run9.sh`: qemu, tmux session `p9`,
serial console logged to `~/p9/console.log`). Commands are sent to it with:

    ~/p9/p9cmd [-t seconds] 'rc command'

`p9cmd` types the command into the console and polls `console.log` for a
`MARK<pid><rand> $status` line. Default timeout 600 s.

### p9cmd rules

- **The command is parsed by rc, not sh.** Use rc syntax only:
  - Quotes are single quotes: `'error|mk:'`. `"` is not special in rc.
  - No backslash escapes. Plan 9 `grep` alternation is plain `|`, not `\|`.
  - Redirect stderr with `>[2=1]`; the exit status is `$status`, not `$?`.
- **Never let a command block on stdin.** `grep`, `cat`, `sed`, etc. with no
  file argument read the console; the `MARK` line never prints and `p9cmd`
  sits there until the timeout. Always give a filename.
- Keep timeouts modest. Redirect long build output to a file in the VM
  (`>/tmp/x.log >[2=1]`) and inspect it with a second, short `p9cmd`.
- **Check progress non-disruptively** instead of waiting blindly:
  - `tail -n 30 ~/p9/console.log` or `tail -f ~/p9/console.log`
  - `tmux capture-pane -pt p9 | tail`
  - `tmux attach -t p9 -r` (read-only)
- If a `p9cmd` is stuck on stdin, `tmux send-keys -t p9 C-d` sends EOF and
  lets it finish. To interrupt a running command, send Delete:
  `tmux send-keys -t p9 DC`.
- Cross-compile for the target with `objtype=riscv64 mk ...` in `/sys/src`.

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
