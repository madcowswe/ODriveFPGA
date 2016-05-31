#ifndef _ALTERA_NIOS_CUSTOM_INSTR_FLOATING_POINT_2_
#define _ALTERA_NIOS_CUSTOM_INSTR_FLOATING_POINT_2_

/***********************************************
 * altera_nios_custom_instr_floating_point_2.h *
 ***********************************************/

/* 
 * The fixed custom instruction 8-bit N values for the generation 2 FPCI
 * (floating-point custom instructions).
 */
#define ALT_FPCI2_FABSS_N     224
#define ALT_FPCI2_FNEGS_N     225
#define ALT_FPCI2_FCMPNES_N   226
#define ALT_FPCI2_FCMPEQS_N   227
#define ALT_FPCI2_FCMPGES_N   228
#define ALT_FPCI2_FCMPGTS_N   229
#define ALT_FPCI2_FCMPLES_N   230
#define ALT_FPCI2_FCMPLTS_N   231
#define ALT_FPCI2_FMAXS_N     232
#define ALT_FPCI2_FMINS_N     233
#define ALT_FPCI2_ROUND_N     248
#define ALT_FPCI2_FIXSI_N     249
#define ALT_FPCI2_FLOATIS_N   250
//#define ALT_FPCI2_FSQRTS_N    251
#define ALT_FPCI2_FMULS_N     252
#define ALT_FPCI2_FADDS_N     253
#define ALT_FPCI2_FSUBS_N     254
#define ALT_FPCI2_FDIVS_N     255 

/*
 * Macros that cause calls to Newlib math library routines
 * to be replaced with a custom instruction.
 * Required because GCC does not reliably convert these 
 * function calls to custom instructions.
 */
#define fminf(A, B) \
    __builtin_custom_fnff(ALT_FPCI2_FMINS_N, (A), (B))

#define fmaxf(A, B) \
    __builtin_custom_fnff(ALT_FPCI2_FMAXS_N, (A), (B))

//#define sqrtf(A) \
//    __builtin_custom_fnf(ALT_FPCI2_FSQRTS_N, (A))

#define lroundf(A) \
    __builtin_custom_inf(ALT_FPCI2_ROUND_N, (A))

#endif /* _ALTERA_NIOS_CUSTOM_INSTR_FLOATING_POINT_2_ */
