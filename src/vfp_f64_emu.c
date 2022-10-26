#include "vfp_vector.h"

#include <stdbool.h>

static inline void VMOV_F64(double *dd, double dn, double dm)
{
    *dd = dm;
}
static inline void VADD_F64(double *dd, double dn, double dm)
{
    *dd = dn + dm;
}
static inline void VSUB_F64(double *dd, double dn, double dm)
{
    *dd = dn - dm;
}
static inline void VDIV_F64(double *dd, double dn, double dm)
{
    *dd = dn / dm;
}
static inline void VMUL_F64(double *dd, double dn, double dm)
{
    *dd = dn * dm;
}
static inline void VNMUL_F64(double *dd, double dn, double dm)
{
    *dd = -(dn * dm);
}
static inline void VMLA_F64(double *dd, double dn, double dm)
{
    *dd = (dn * dm) + *dd;
}
static inline void VNMLA_F64(double *dd, double dn, double dm)
{
    *dd = -((dn * dm) + *dd);
}
static inline void VMLS_F64(double *dd, double dn, double dm)
{
    *dd = *dd - (dn * dm);
}
static inline void VNMLS_F64(double *dd, double dn, double dm)
{
    *dd = -(*dd - (dn * dm));
}
static inline void VABS_F64(double *dd, double dn, double dm)
{
    union
    {
        double f64;
        uint64_t u64;
    } tmp;

    tmp.f64 = dm;
    tmp.u64 &= ~0x8000000000000000;
    *dd = tmp.f64;
}
static inline void VNEG_F64(double *dd, double dn, double dm)
{
    *dd = -dm;
}
static inline void VSQRT_F64(double *dd, double dn, double dm)
{
    __asm__ volatile(
        "vldr d1, [%1]\n"
        "vsqrt.f64 d0, d1\n"
        "vstr d0, [%0]\n" ::"r"(dd),
        "r"(&dm));
}

void EmulateF64VFPOp(VFPInstruction *vfpInstr, KuKernelAbortContext *abortContext)
{
    double *registerBuffer = (double *)(&abortContext->vfpRegisters[0]);
    const double imm = vfpInstr->operands.imm.f64;

    int dReg = vfpInstr->dReg, nReg = vfpInstr->operands.regs.n, mReg = vfpInstr->operands.regs.m;
    int dRegBank = vfpInstr->dReg & 0x1C;
    int nRegBank = vfpInstr->operands.regs.n & 0x1C;
    int mRegBank = vfpInstr->operands.regs.m & 0x1C;
    for (int i = 0; i < vfpInstr->vectorLength; i++)
    {
        double *dd = &registerBuffer[dReg];
        double dn = registerBuffer[nReg];
        double dm = registerBuffer[mReg];
        switch (vfpInstr->op)
        {
        case VFP_OP_VMOV:
            VMOV_F64(dd, 0, dm);
            break;
        case VFP_OP_VMOV_IMM:
            VMOV_F64(dd, 0, imm);
            break;
        case VFP_OP_VADD:
            VADD_F64(dd, dn, dm);
            break;
        case VFP_OP_VSUB:
            VSUB_F64(dd, dn, dm);
            break;
        case VFP_OP_VDIV:
            VDIV_F64(dd, dn, dm);
            break;
        case VFP_OP_VMUL:
            VMUL_F64(dd, dn, dm);
            break;
        case VFP_OP_VNMUL:
            VNMUL_F64(dd, dn, dm);
            break;
        case VFP_OP_VMLA:
            VMLA_F64(dd, dn, dm);
            break;
        case VFP_OP_VNMLA:
            VNMLA_F64(dd, dn, dm);
            break;
        case VFP_OP_VMLS:
            VMLS_F64(dd, dn, dm);
            break;
        case VFP_OP_VNMLS:
            VNMLS_F64(dd, dn, dm);
            break;
        case VFP_OP_VABS:
            VABS_F64(dd, 0, dm);
            break;
        case VFP_OP_VNEG:
            VNEG_F64(dd, 0, dm);
            break;
        case VFP_OP_VSQRT:
            VSQRT_F64(dd, 0, dm);
            break;
        }

        dReg = ((dReg + vfpInstr->vectorStride) & 0x3) | dRegBank; // Ensure they wrap around in the regbank (not sure if this is correct).
        nReg = ((nReg + vfpInstr->vectorStride) & 0x3) | nRegBank;
        mReg = ((mReg + vfpInstr->vectorStride) & 0x3) | mRegBank;
    }
}