#include "vfp_vector.h"

#include <stdbool.h>

asm(
    R"(.text
.thumb
.fpu vfpv3

/**
 * VFP Op F32 emulation
 * Inputs:
 * r0 - Vector length
 * 
 * s8 - s15: destination vector
 * s16 - s23: source 1 vector
 * s24 - s31: source 2 vector
 */

/**
 * Moves sm to sd
 * 
 */
VMOV:
    vmov.f32 s8, s24
    sub r0, #0x1
    cbz r0, VMOV_exit
    vmov.f32 s9, s25
    sub r0, #0x1
    cbz r0, VMOV_exit
    vmov.f32 s10, s26
    sub r0, #0x1
    cbz r0, VMOV_exit
    vmov.f32 s11, s27
    sub r0, #0x1
    cbz r0, VMOV_exit
    vmov.f32 s12, s28
    sub r0, #0x1
    cbz r0, VMOV_exit
    vmov.f32 s13, s29
    sub r0, #0x1
    cbz r0, VMOV_exit
    vmov.f32 s14, s30
    sub r0, #0x1
    cbz r0, VMOV_exit
    vmov.f32 s15, s31
VMOV_exit:
    bx lr
/**
 * Inputs:
 * r0 - VFPInstruction pointer
 * r1 - Sd vector pointer
 * r2 - Sn vector pointer
 * r3 - Sm vector pointer
 */
ExecuteVFPOp:
    push {r4, lr}
    mov r4, r1
    vldmia r1, {d4 - d7}
    vldmia r2, {d8 - d11}
    vldmia r3, {d12 - d15}
    ldrb r1, [r0, #0x0]
    ldr r2, =vfpOpFns
    ldr r3, [r2, r1, lsl #0x2]
    ldrh r0, [r0, #0x10]
    orr r3, #0x1 // Add thumb bit
    blx r3
    vstmia r4, {d4 - d7}
    pop {r4, pc}
)");

void VMOV();

typedef void (*VfpOpFn)();
VfpOpFn vfpOpFns[VFP_OP_Count] = 
{
    VMOV,
    VMOV,
    VMOV,
    VMOV,
    VMOV,
    VMOV,
    VMOV,
    VMOV,
    VMOV,
    VMOV,
    VMOV,
    VMOV,
    VMOV,
    VMOV,
};

void ExecuteVFPOp(VFPInstruction *instr, float *sdBuffer, float *snBuffer, float *smBuffer);

static inline void CopySourceVector(VFPInstruction *vfpInstr, float *registerBuffer, int registerIndex, float *vectorBuffer)
{
    int regBufferOffset = registerIndex;
    int regBankBase = registerIndex & 0x18;
    for (int i = 0; i < vfpInstr->vectorLength; i++)
    {
        vectorBuffer[i] = registerBuffer[regBufferOffset];

        regBufferOffset += vfpInstr->vectorStride;

        regBufferOffset = (regBufferOffset & 0x7) | regBankBase; // Ensure it wraps around in the regbank (not sure if this is correct).
    }
}
static inline void FillVector(float *vectorBuffer, float scalar)
{
    for (int i = 0; i < 8; i++)
    {
        vectorBuffer[i] = scalar;
    }
}

static inline void CopyDestVector(VFPInstruction *vfpInstr, float *vectorBuffer, float *registerBuffer)
{
    int regBufferOffset = vfpInstr->destReg;
    int regBankBase = vfpInstr->destReg & 0x18;
    for (int i = 0; i < vfpInstr->vectorLength; i++)
    {
        registerBuffer[regBufferOffset] = vectorBuffer[i];

        regBufferOffset += vfpInstr->vectorStride;

        regBufferOffset = (regBufferOffset & 0x7) | regBankBase; // Ensure it wraps around in the regbank (not sure if this is correct).
    }
}


void EmulateF32VFPOp(VFPInstruction *vfpInstr, KuKernelAbortContext *abortContext)
{
    float snBuffer[8], smBuffer[8], sdBuffer[8];
    float *registerBuffer = (float *)(&abortContext->vfpRegisters[0]);
    uint32_t inputFlags = vfpOpInputFlags[vfpInstr->op];

    if (inputFlags & VFP_OP_INPUT_Sd)
        CopySourceVector(vfpInstr, registerBuffer, vfpInstr->destReg, sdBuffer);

    if ((inputFlags & VFP_OP_INPUT_IMM) == 0)
    {
        if (inputFlags & VFP_OP_INPUT_Sn)
            CopySourceVector(vfpInstr, registerBuffer, vfpInstr->operands.regs.sn, snBuffer);

        if (inputFlags & VFP_OP_INPUT_Sm)
        {
            if (vfpInstr->operands.regs.sm > 7)
                CopySourceVector(vfpInstr, registerBuffer, vfpInstr->operands.regs.sm, smBuffer);
            else
                FillVector(smBuffer, registerBuffer[vfpInstr->operands.regs.sm]);
        }
    }
    else
        FillVector(smBuffer, vfpInstr->operands.imm.f32);

    ExecuteVFPOp(vfpInstr, sdBuffer, snBuffer, smBuffer);

    CopyDestVector(vfpInstr, sdBuffer, registerBuffer);
}