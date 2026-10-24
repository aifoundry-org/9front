#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int
main(void)
{
	char *p;
	double x;

	p = malloc(32);
	if(p == NULL)
		return 1;
	strcpy(p, "APE on riscv64");
	x = strtod("2.5", NULL);
	if(x*2.0 != 5.0 || sizeof(void*) != 8)
		return 1;
	printf("PASS: %s, malloc, string, floating point\n", p);
	free(p);
	return 0;
}
