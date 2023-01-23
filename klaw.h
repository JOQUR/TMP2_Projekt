#ifndef KLAW_H
#define KLAW_H

#include "MKL05Z4.h"
#define S2_MASK	(1<<10)		// Maska dla klawisza S2
#define S3_MASK	(1<<11)		// Maska dla klawisza S3
#define S2	10						// Numer bitu dla klawisza S2
#define S3	11						// Numer bitu dla klawisza S3

void Klaw_Init(void);
void Klaw_S1(void);

#endif  /* KLAW_H */
