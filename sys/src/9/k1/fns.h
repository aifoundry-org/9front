#include "../rva23/fns.h"

/* ethersmte */
extern void smtedtbprop(char*, char*, uchar*, int);

/* k1reset */
extern int k1pmicread(int reg);
extern int k1pmicwrite(int reg, int val);

/* k1thermal */
extern void k1thermalinit(void);
