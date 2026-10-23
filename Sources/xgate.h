/* XGATE vector table entry */
typedef void (*_NEAR XGATE_Function)(int);
typedef struct {
  XGATE_Function pc;        /* pointer to the handler */
  int dataptr;              /* pointer to the data of the handler */
} XGATE_TableEntry;

#pragma push                /* save current segment definitions */
#pragma CONST_SEG __GPAGE_SEG XGATE_VECTORS  /* for the HCS12X/XGATE shared setup, HCS12X needs global addressing to access the vector table. */

#define XGATE_VECTOR_OFFSET 9                /* the first elements are unused and need not to be allocated (to save space) */

extern const XGATE_TableEntry XGATE_VectorTable[];
#pragma pop                 /* restore previous segment definitions */
