#if OEM == OEM_PLD_FOTON
#undef  L3_SOC_LOW           // 没有绝缘三级报警
#define L3_SOC_LOW          0
#endif


#ifndef HAVE_L1_CHG_TVOL_HIGH
#define L1_CHG_TVOL_HIGH

#ifndef HAVE_L1_EP_LOW
#define HAVE_L1_EP_LOW      0
#endif
#ifndef HAVE_L2_EP_LOW
#define HAVE_L2_EP_LOW      0
#endif
#ifndef HAVE_L3_EP_LOW
#define HAVE_L3_EP_LOW      0
#endif
#ifndef HAVE_L4_EP_LOW
#define HAVE_L4_EP_LOW      0
#endif

#ifndef HAVE_L1_EP_UNBA
#define HAVE_L1_EP_UNBA     0
#endif
#ifndef HAVE_L2_EP_UNBA
#define HAVE_L2_EP_UNBA     0
#endif
#ifndef HAVE_L3_EP_UNBA
#define HAVE_L3_EP_UNBA     0
#endif
#ifndef HAVE_L4_EP_UNBA
#define HAVE_L4_EP_UNBA     0
#endif

#if !HAVE_L1_EP_LOW
#undef  L1_EP_TMP_LOW
#define L1_EP_TMP_LOW       0
#endif

#if !HAVE_L2_EP_LOW
#undef  L2_EP_TMP_LOW
#define L2_EP_TMP_LOW       0
#endif

#if !HAVE_L3_EP_LOW
#undef  L3_EP_TMP_LOW
#define L3_EP_TMP_LOW       0
#endif

#if !HAVE_L4_EP_LOW
#undef  L4_EP_TMP_LOW
#define L4_EP_TMP_LOW       0
#endif

#if !HAVE_L1_EP_UNBA
#undef  L1_EP_TMP_UNBA
#define L1_EP_TMP_UNBA      0
#endif

#if !HAVE_L2_EP_UNBA
#undef  L2_EP_TMP_UNBA
#define L2_EP_TMP_UNBA      0
#endif

#if !HAVE_L3_EP_UNBA
#undef  L3_EP_TMP_UNBA
#define L3_EP_TMP_UNBA      0
#endif

#if !HAVE_L4_EP_UNBA
#undef  L4_EP_TMP_UNBA
#define L4_EP_TMP_UNBA      0
#endif
