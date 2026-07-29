/*
 * kexec — launch a Plan 9 RISC-V kernel from a running Linux via kexec_load(2).
 *
 * Usage:  kexec [-d <destaddr>] [-b "<extra args>"] [-l] [-r] <kernel.elf>
 *
 *   -d <destaddr>   physical address to load kernel at (must match the kernel's
 *                   link address as recorded in its ELF program headers).
 *                   If omitted, the lowest PT_LOAD's p_paddr is used.
 *   -b "<extra>"    append the given string to /chosen/bootargs in the DTB
 *                   we pass to the new kernel (e.g. -b "*maxmem=0x80000000").
 *                   May be given multiple times; entries are joined by spaces.
 *   -l              load only; do not launch (use 'systemctl kexec' afterwards).
 *   -r              raw launch with reboot(LINUX_REBOOT_CMD_KEXEC); skips
 *                   systemd's shutdown sequence — corrupts mounted filesystems.
 *
 * Default behaviour: load, then exec 'systemctl kexec' so init runs the normal
 * shutdown sequence (unmount, sync, remount-ro) before jumping into the new
 * kernel.  The staged kexec image survives the exec.
 *
 * The kernel must be passed as an ELF file (9qemu.elf, not the raw 9qemu).
 * Each PT_LOAD becomes one kexec segment with its true MemSiz, so the BSS
 * gets the memory the kernel's link map says it owns.  The DTB is read from
 * /sys/firmware/fdt at runtime and placed in its own segment immediately past
 * the kernel's BSS.
 *
 * All segment memsz values are rounded up to PAGE_SIZE; the kernel's
 * sanity_check_segment_list() silently returns EADDRNOTAVAIL otherwise.
 */

#define _GNU_SOURCE
#include <unistd.h>
#include <sys/syscall.h>
#include <sys/stat.h>
#include <linux/reboot.h>
#include <sys/reboot.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <elf.h>
#include <libfdt.h>

#ifndef KEXEC_ARCH_RISCV
#define KEXEC_ARCH_RISCV	(243 << 16)
#endif

#define PAGE_SIZE	4096UL
#define PAGE_ROUND(x)	(((x) + PAGE_SIZE - 1) & ~(PAGE_SIZE - 1))

char *FDT_PATH = 	"/sys/firmware/fdt";
#define MAX_SEGS	16

struct kexec_segment {
	const void	*buf;
	size_t		bufsz;
	unsigned long	mem;
	size_t		memsz;
};

/*
 * Read an entire file into a page-aligned, page-rounded, zero-padded
 * buffer.  *sz returns the actual file size.  Caller frees with free().
 */
static void *
slurp(const char *path, size_t *sz)
{
	int fd;
	struct stat st;
	void *buf;
	size_t pad;
	ssize_t n;

	fd = open(path, O_RDONLY);
	if(fd < 0){
		fprintf(stderr, "open %s: %s\n", path, strerror(errno));
		return NULL;
	}
	if(fstat(fd, &st) < 0){
		fprintf(stderr, "stat %s: %s\n", path, strerror(errno));
		close(fd);
		return NULL;
	}
	*sz = st.st_size;
	pad = PAGE_ROUND(*sz);
	buf = aligned_alloc(PAGE_SIZE, pad);
	if(buf == NULL){
		fprintf(stderr, "aligned_alloc(%zu): %s\n", pad, strerror(errno));
		close(fd);
		return NULL;
	}
	memset(buf, 0, pad);
	for(size_t got = 0; got < *sz; ){
		n = read(fd, (char *)buf + got, *sz - got);
		if(n < 0){
			if(errno == EINTR)
				continue;
			fprintf(stderr, "read %s: %s\n", path, strerror(errno));
			free(buf);
			close(fd);
			return NULL;
		}
		if(n == 0){
			fprintf(stderr, "read %s: EOF at %zu of %zu\n", path, got, *sz);
			free(buf);
			close(fd);
			return NULL;
		}
		got += (size_t)n;
	}
	close(fd);
	return buf;
}

/*
 * Return 1 if [base, base+size) lies entirely within one top-level
 * "System RAM" entry of /proc/iomem; 0 otherwise.
 */
static int
iomem_covers(unsigned long base, unsigned long size)
{
	FILE *fp;
	char line[256];
	unsigned long lo, hi, end;
	int ok;

	fp = fopen("/proc/iomem", "r");
	if(fp == NULL){
		fprintf(stderr, "open /proc/iomem: %s\n", strerror(errno));
		return 0;
	}
	end = base + size - 1;
	ok = 0;
	while(fgets(line, sizeof line, fp) != NULL){
		if(line[0] == ' ' || line[0] == '\t')
			continue;
		if(sscanf(line, "%lx-%lx :", &lo, &hi) != 2)
			continue;
		if(strstr(line, "System RAM") == NULL)
			continue;
		if(base >= lo && end <= hi){
			ok = 1;
			break;
		}
	}
	fclose(fp);
	return ok;
}

/*
 * Normalised PT_LOAD descriptor (always 64-bit), populated from either an
 * ELF32 or ELF64 file.  Plan 9's jl -H5 writes ELF32 with e_machine set to
 * EM_X86_64 even for RISC-V — we don't check e_machine.
 */
struct loadseg {
	uint64_t paddr;
	uint64_t vaddr;
	uint64_t filesz;
	uint64_t memsz;
	uint64_t offset;
};

/*
 * Parse an ELF executable, fill `out[]` with up to `max` PT_LOAD descriptors,
 * and return the count (or -1 on error).  *entry_out is set to e_entry.
 */
static int
parse_elf_loads(const void *base, size_t sz, struct loadseg *out, int max,
                uint64_t *entry_out)
{
	const unsigned char *eh = base;
	int class, n = 0, i;

	if(sz < EI_NIDENT || memcmp(eh, ELFMAG, SELFMAG) != 0){
		fprintf(stderr, "not an ELF file\n");
		return -1;
	}
	if(eh[EI_DATA] != ELFDATA2LSB){
		fprintf(stderr, "ELF is not little-endian\n");
		return -1;
	}
	class = eh[EI_CLASS];
	if(class != ELFCLASS32 && class != ELFCLASS64){
		fprintf(stderr, "ELF class %u not supported\n", class);
		return -1;
	}

	if(class == ELFCLASS32){
		const Elf32_Ehdr *e = base;
		const char *p;

		if(sz < sizeof(*e) || e->e_phoff + (uint64_t)e->e_phnum * e->e_phentsize > sz){
			fprintf(stderr, "ELF32 program headers run off end of file\n");
			return -1;
		}
		*entry_out = e->e_entry;
		p = (const char *)base + e->e_phoff;
		for(i = 0; i < e->e_phnum; i++, p += e->e_phentsize){
			const Elf32_Phdr *ph = (const Elf32_Phdr *)p;
			if(ph->p_type != PT_LOAD || ph->p_memsz == 0)
				continue;
			if(n >= max){
				fprintf(stderr, "too many PT_LOAD segments (>%d)\n", max);
				return -1;
			}
			out[n].paddr  = ph->p_paddr;
			out[n].vaddr  = ph->p_vaddr;
			out[n].filesz = ph->p_filesz;
			out[n].memsz  = ph->p_memsz;
			out[n].offset = ph->p_offset;
			n++;
		}
	} else {
		const Elf64_Ehdr *e = base;
		const char *p;

		if(sz < sizeof(*e) || e->e_phoff + (uint64_t)e->e_phnum * e->e_phentsize > sz){
			fprintf(stderr, "ELF64 program headers run off end of file\n");
			return -1;
		}
		*entry_out = e->e_entry;
		p = (const char *)base + e->e_phoff;
		for(i = 0; i < e->e_phnum; i++, p += e->e_phentsize){
			const Elf64_Phdr *ph = (const Elf64_Phdr *)p;
			if(ph->p_type != PT_LOAD || ph->p_memsz == 0)
				continue;
			if(n >= max){
				fprintf(stderr, "too many PT_LOAD segments (>%d)\n", max);
				return -1;
			}
			out[n].paddr  = ph->p_paddr;
			out[n].vaddr  = ph->p_vaddr;
			out[n].filesz = ph->p_filesz;
			out[n].memsz  = ph->p_memsz;
			out[n].offset = ph->p_offset;
			n++;
		}
	}
	return n;
}

/*
 * Return a new page-aligned buffer holding the original DTB with `extra`
 * appended to /chosen/bootargs (separated by a space).  *outsz is set to
 * the new DTB total size; the buffer is sized at least PAGE_ROUND(outsz).
 * On error, NULL is returned and the original buffer is unchanged.
 */
static void *
dtb_append_bootargs(const void *orig, size_t origsz, const char *extra, size_t *outsz)
{
	size_t cap;
	void *buf;
	int rc, choff;
	const char *oldargs;
	int oldlen;
	char *newargs;
	size_t newlen;

	/* Add generous slack for the new property + any string-table growth. */
	cap = PAGE_ROUND(origsz + strlen(extra) + 4096);
	buf = aligned_alloc(PAGE_SIZE, cap);
	if(buf == NULL){
		fprintf(stderr, "aligned_alloc(%zu): %s\n", cap, strerror(errno));
		return NULL;
	}
	rc = fdt_open_into(orig, buf, (int)cap);
	if(rc < 0){
		fprintf(stderr, "fdt_open_into: %s\n", fdt_strerror(rc));
		free(buf);
		return NULL;
	}
	choff = fdt_path_offset(buf, "/chosen");
	if(choff == -FDT_ERR_NOTFOUND){
		choff = fdt_add_subnode(buf, 0, "chosen");
		if(choff < 0){
			fprintf(stderr, "fdt_add_subnode(/chosen): %s\n", fdt_strerror(choff));
			free(buf);
			return NULL;
		}
	} else if(choff < 0){
		fprintf(stderr, "fdt_path_offset(/chosen): %s\n", fdt_strerror(choff));
		free(buf);
		return NULL;
	}

	oldargs = fdt_getprop(buf, choff, "bootargs", &oldlen);
	if(oldargs != NULL && oldlen > 0){
		/* oldlen includes the trailing NUL. */
		newlen = (size_t)oldlen + 1 + strlen(extra) + 1;
		newargs = malloc(newlen);
		if(newargs == NULL){
			fprintf(stderr, "malloc(%zu): %s\n", newlen, strerror(errno));
			free(buf);
			return NULL;
		}
		snprintf(newargs, newlen, "%s %s", oldargs, extra);
	} else {
		newargs = strdup(extra);
		if(newargs == NULL){
			fprintf(stderr, "strdup: %s\n", strerror(errno));
			free(buf);
			return NULL;
		}
	}

	rc = fdt_setprop_string(buf, choff, "bootargs", newargs);
	free(newargs);
	if(rc < 0){
		fprintf(stderr, "fdt_setprop_string(bootargs): %s\n", fdt_strerror(rc));
		free(buf);
		return NULL;
	}
	rc = fdt_pack(buf);
	if(rc < 0){
		fprintf(stderr, "fdt_pack: %s\n", fdt_strerror(rc));
		free(buf);
		return NULL;
	}
	*outsz = fdt_totalsize(buf);
	return buf;
}

static void
usage(const char *argv0)
{
	fprintf(stderr,
		"usage: %s [-d <destaddr>] [-b \"<extra args>\"] [-l] [-r] <kernel.elf>\n"
		"  -d <destaddr>   physical address to load kernel at\n"
		"                  (overrides the lowest PT_LOAD's p_paddr)\n"
		"  -b \"<extra>\"    append to /chosen/bootargs in the staged DTB\n"
		"                  (repeatable; entries joined by spaces)\n"
		"  -l              load only; launch later with 'systemctl kexec'\n"
		"  -r              raw reboot — corrupts filesystems, debug only\n"
		"\n"
		"Default behaviour: load, then exec 'systemctl kexec' so init unmounts\n"
		"and syncs before the new kernel takes over.\n",
		argv0);
	exit(2);
}

int
main(int argc, char **argv)
{
	const char *kpath = NULL;
	unsigned long dest_override = 0;
	int have_dest_override = 0;
	int loadonly = 0, raw = 0;
	char *extra = NULL;
	size_t extralen = 0;
	void *kbuf = NULL, *fbuf = NULL;
	size_t ksz = 0, fsz = 0;
	struct kexec_segment segs[MAX_SEGS];
	struct loadseg loads[MAX_SEGS];
	int nseg = 0, nload;
	uint64_t entry, base_paddr;
	unsigned long kernel_end;
	long ret;
	int i;

	for(i = 1; i < argc; i++){
		if(strcmp(argv[i], "-d") == 0 && i + 1 < argc){
			dest_override = strtoull(argv[++i], NULL, 0);
			have_dest_override = 1;
		} else if(strcmp(argv[i], "-f") == 0 && i + 1 < argc){
			FDT_PATH = argv[++i];
		} else if(strcmp(argv[i], "-b") == 0 && i + 1 < argc){
			const char *e = argv[++i];
			size_t newlen = extralen + (extralen ? 1 : 0) + strlen(e) + 1;
			char *grown = realloc(extra, newlen);
			if(grown == NULL){
				fprintf(stderr, "realloc: %s\n", strerror(errno));
				free(extra);
				return 1;
			}
			if(extralen){
				grown[extralen] = ' ';
				memcpy(grown + extralen + 1, e, strlen(e) + 1);
			} else {
				memcpy(grown, e, strlen(e) + 1);
			}
			extra = grown;
			extralen = newlen - 1;
		} else if(strcmp(argv[i], "-l") == 0){
			loadonly = 1;
		} else if(strcmp(argv[i], "-r") == 0){
			raw = 1;
		} else if(argv[i][0] == '-'){
			usage(argv[0]);
		} else if(kpath == NULL){
			kpath = argv[i];
		} else {
			usage(argv[0]);
		}
	}
	if(loadonly && raw){
		fprintf(stderr, "-l and -r are mutually exclusive\n");
		return 2;
	}
	if(kpath == NULL)
		usage(argv[0]);
	if(have_dest_override && (dest_override & (PAGE_SIZE - 1))){
		fprintf(stderr, "destaddr 0x%lx not page-aligned\n", dest_override);
		return 1;
	}

	kbuf = slurp(kpath, &ksz);
	if(kbuf == NULL)
		return 1;

	nload = parse_elf_loads(kbuf, ksz, loads, MAX_SEGS - 1, &entry);
	if(nload < 0){
		fprintf(stderr,
			"hint: pass the .elf companion (e.g. 9qemu.elf), not the raw -H6 binary.\n");
		free(kbuf);
		return 1;
	}
	if(nload == 0){
		fprintf(stderr, "%s: no PT_LOAD segments\n", kpath);
		free(kbuf);
		return 1;
	}

	/*
	 * Plan 9's linker abuts text and data at byte granularity, so PT_LOAD
	 * p_paddr values usually aren't page-aligned (a kexec_load requirement).
	 * Build one merged contiguous segment spanning [min(paddr), max(paddr+memsz))
	 * page-rounded, and memcpy each PT_LOAD into its slot.  Untouched bytes
	 * stay zero, which also covers the BSS tail.
	 */
	{
		unsigned long lo, hi, span, off, shift;
		void *merged;

		lo = loads[0].paddr;
		hi = loads[0].paddr + loads[0].memsz;
		for(i = 1; i < nload; i++){
			unsigned long s = loads[i].paddr;
			unsigned long e = loads[i].paddr + loads[i].memsz;
			if(s < lo) lo = s;
			if(e > hi) hi = e;
		}
		base_paddr = lo;
		if(have_dest_override){
			shift = dest_override - (lo & ~(PAGE_SIZE - 1));
		} else {
			shift = 0;
		}
		lo = (lo & ~(PAGE_SIZE - 1)) + shift;
		hi = PAGE_ROUND(hi) + shift;
		if(lo & (PAGE_SIZE - 1)){
			fprintf(stderr, "merged base 0x%lx not page-aligned\n", lo);
			free(kbuf);
			return 1;
		}
		span = hi - lo;
		merged = aligned_alloc(PAGE_SIZE, span);
		if(merged == NULL){
			fprintf(stderr, "aligned_alloc(%lu): %s\n", span, strerror(errno));
			free(kbuf);
			return 1;
		}
		memset(merged, 0, span);
		for(i = 0; i < nload; i++){
			off = (loads[i].paddr + shift) - lo;
			if(off + loads[i].filesz > span){
				fprintf(stderr, "load %d would overflow merged buffer\n", i);
				free(merged);
				free(kbuf);
				return 1;
			}
			memcpy((char *)merged + off,
				(char *)kbuf + loads[i].offset,
				loads[i].filesz);
		}
		segs[nseg].buf   = merged;
		segs[nseg].bufsz = span;
		segs[nseg].mem   = lo;
		segs[nseg].memsz = span;
		nseg++;
		kernel_end = hi;
		if(have_dest_override)
			entry = entry - base_paddr + dest_override;
	}

	/* kbuf is no longer needed; merged owns the kernel image. */
	free(kbuf);
	kbuf = NULL;

	/* DTB segment immediately past the highest kernel-end. */
	fbuf = slurp(FDT_PATH, &fsz);
	if(fbuf == NULL)
		return 1;
	{
		unsigned char *m = fbuf;
		if(!(m[0] == 0xd0 && m[1] == 0x0d && m[2] == 0xfe && m[3] == 0xed))
			fprintf(stderr,
				"warning: %s does not start with the DTB magic d00dfeed\n",
				FDT_PATH);
	}

	/* If -b was given, build an amended DTB with extra args appended. */
	if(extra != NULL){
		size_t newsz;
		void *newbuf = dtb_append_bootargs(fbuf, fsz, extra, &newsz);
		if(newbuf == NULL){
			free(fbuf);
			free(extra);
			return 1;
		}
		fprintf(stderr, "dtb: appended to /chosen/bootargs: %s\n", extra);
		free(fbuf);
		fbuf = newbuf;
		fsz = newsz;
	}
	free(extra);
	extra = NULL;

	segs[nseg].buf   = fbuf;
	segs[nseg].bufsz = fsz;
	segs[nseg].mem   = kernel_end;
	segs[nseg].memsz = PAGE_ROUND(fsz);
	nseg++;

	/* Validate the whole footprint against /proc/iomem. */
	{
		unsigned long lo = segs[0].mem, hi = segs[0].mem;
		for(i = 0; i < nseg; i++){
			if(segs[i].mem < lo)
				lo = segs[i].mem;
			if(segs[i].mem + segs[i].memsz > hi)
				hi = segs[i].mem + segs[i].memsz;
		}
		if(!iomem_covers(lo, hi - lo)){
			fprintf(stderr,
				"destination 0x%lx-0x%lx is not inside a single 'System RAM' range\n"
				"in /proc/iomem; pick another -d.\n", lo, hi);
			free(kbuf);
			free(fbuf);
			return 1;
		}
	}

	for(i = 0; i < nseg; i++)
		fprintf(stderr, "seg %d  %zu bytes -> 0x%lx (%zu padded)%s\n",
			i, segs[i].bufsz, segs[i].mem, segs[i].memsz,
			(unsigned long)segs[i].buf == (unsigned long)fbuf ? "  [dtb]" : "");
	fprintf(stderr, "entry 0x%lx\n", entry);

	/* Drop any previously-staged image so retries are idempotent. */
	syscall(__NR_kexec_load, 0UL, 0UL, NULL, (unsigned long)KEXEC_ARCH_RISCV);

	ret = syscall(__NR_kexec_load,
		entry,
		(unsigned long)nseg,
		segs,
		(unsigned long)KEXEC_ARCH_RISCV);
	if(ret != 0){
		fprintf(stderr, "kexec_load: %s\n", strerror(errno));
		free(kbuf);
		free(fbuf);
		return 1;
	}

	free(kbuf);
	free(fbuf);

	if(loadonly){
		fprintf(stderr, "kernel staged; run 'systemctl kexec' to launch.\n");
		return 0;
	}
	if(raw){
		syscall(__NR_reboot,
			LINUX_REBOOT_MAGIC1, LINUX_REBOOT_MAGIC2,
			LINUX_REBOOT_CMD_KEXEC, NULL);
		fprintf(stderr, "reboot: %s\n", strerror(errno));
		return 1;
	}

	fprintf(stderr, "exec systemctl kexec ...\n");
	execlp("systemctl", "systemctl", "kexec", (char *)NULL);
	fprintf(stderr, "execlp systemctl: %s\n", strerror(errno));
	fprintf(stderr,
		"image is staged but launch failed; try 'systemctl kexec' manually,\n"
		"or rerun with -r to skip clean shutdown (unsafe).\n");
	return 1;
}
