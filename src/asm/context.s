/* ARM926EJ-S (ARMv5TE) context switch routines
 *
 * Stack frame layout (16 words = 64 bytes, low → high address):
 *   [0]  cpsr
 *   [4]  r0
 *   [8]  r1
 *   ...
 *   [52] r12
 *   [56] lr
 *   [60] pc
 *
 * NOTE: curly braces are doubled ({{ / }}) because this file is included
 * via global_asm!() which uses Rust's format-string escaping rules.
 */

/*
 * context_switch(from: *mut *mut u8, to: *const *mut u8)
 * r0 = address of from-thread sp field (offset 0 in Thread)
 * r1 = address of to-thread sp field
 * Called from SVC (thread) context.
 */
.global context_switch
context_switch:
    STMFD   SP!, {{LR}}               @ push LR as future PC
    STMFD   SP!, {{R0-R12, LR}}       @ push r0-r12, LR (14 regs)
    MRS     R4, CPSR
    STMFD   SP!, {{R4}}               @ push CPSR — frame base
    STR     SP, [R0]                  @ save SP → *from (thread.sp field)
    LDR     SP, [R1]                  @ load SP ← *to
    LDMFD   SP!, {{R4}}               @ pop CPSR into R4
    MSR     SPSR_cxsf, R4             @ set SPSR from frame
    LDMFD   SP!, {{R0-R12, LR, PC}}^  @ pop r0-r12, LR, PC; CPSR ← SPSR

/*
 * context_switch_to(to: *const *mut u8)
 * r0 = address of first thread's sp field
 * Used once to start the first thread.
 */
.global context_switch_to
context_switch_to:
    LDR     SP, [R0]
    LDMFD   SP!, {{R4}}
    MSR     SPSR_cxsf, R4
    LDMFD   SP!, {{R0-R12, LR, PC}}^

/*
 * context_switch_interrupt(from: *mut *mut u8, to: *const *mut u8)
 * r0 = &from_thread.sp, r1 = &to_thread.sp
 * Sets switch_interrupt_flag and from/to pointers for IRQ-exit switch.
 */
.section .data
.global switch_interrupt_flag
.global interrupt_from_thread
.global interrupt_to_thread
switch_interrupt_flag: .word 0
interrupt_from_thread: .word 0
interrupt_to_thread:   .word 0

.section .text
.global context_switch_interrupt
context_switch_interrupt:
    LDR     R2, =switch_interrupt_flag
    LDR     R3, [R2]
    CMP     R3, #1
    BEQ     _csi_reswitch
    MOV     R3, #1
    STR     R3, [R2]
    LDR     R2, =interrupt_from_thread
    STR     R0, [R2]
_csi_reswitch:
    LDR     R2, =interrupt_to_thread
    STR     R1, [R2]
    BX      LR
