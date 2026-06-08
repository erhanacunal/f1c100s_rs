/*
 * startup.s - Boot code for Allwinner F1C100s (ARM926EJ-S)
 *
 * Full xboot-style boot flow:
 *   1. BROM loads SPL from SPI flash to SRAM
 *   2. Save boot params, detect FEL mode
 *   3. Enter SVC mode, set up low vectors
 *   4. Init clock → DRAM
 *   5. Copy SPL from SRAM to DRAM (speedup)
 *   6. Relocate to DRAM link address
 *   7. Stack setup, BSS clear
 *   8. Jump to rust_main
 */

/* ARM CPU Mode constants */
.equ MODE_USR,        0x10
.equ MODE_FIQ,        0x11
.equ MODE_IRQ,        0x12
.equ MODE_SVC,        0x13
.equ MODE_ABT,        0x17
.equ MODE_UND,        0x1B
.equ MODE_SYS,        0x1F
.equ MODEMASK,        0x1F
.equ NOINT,           0xC0

.equ I_BIT,           0x80
.equ F_BIT,           0x40

/* Stack sizes for each mode */
.equ UND_STACK_SIZE,  0x00000100
.equ SVC_STACK_SIZE,  0x00002000  /* 8 KB — needed for Rust fmt + deep call chains */
.equ ABT_STACK_SIZE,  0x00000100
.equ FIQ_STACK_SIZE,  0x00000100
.equ IRQ_STACK_SIZE,  0x00000800  /* 2 KB — IRQ handler with context switch logic */
.equ SYS_STACK_SIZE,  0x00000100

/* Hardware registers */
.equ WDOG_BASE,       0x01C20CB8
.equ IRQ_MASK0,       0x01C20430
.equ IRQ_MASK1,       0x01C20434

/* Speedup target: top of DRAM for fast execution during init */
.equ SPEEDUP_ADDR,    0x81F80000

/*
 * ── reset ──  Entry point from BROM
 */
.section .text.spl
.arm
/*
 * Allwinner BROM boot header
 */
.global _start
_start:
    /*.long 0xea000016*/
    b _vector;
    .byte 'e', 'G', 'O', 'N', '.', 'B', 'T', '0'
    .long 0, __spl_size
    .byte 'S', 'P', 'L', 2
    .long 0, 0
    .long 0, 0, 0, 0, 0, 0, 0, 0
    .long 0, 0, 0, 0, 0, 0, 0, 0  /* 0x40 boot params, 0x58 fel type, 0x5c dram size */


_vector:
    b reset
    ldr pc, _undefined_instruction
    ldr pc, _software_interrupt
    ldr pc, _prefetch_abort
    ldr pc, _data_abort
    ldr pc, _not_used
    ldr pc, _irq
    ldr pc, _fiq

_undefined_instruction: .word undefined_instruction
_software_interrupt:     .word SVC_Handler
_prefetch_abort:         .word prefetch_abort
_data_abort:             .word data_abort
_not_used:               .word not_used
_irq:                    .word irq
_fiq:                    .word fiq

;.balignl 16, 0xdeadbeef

.global reset
reset:
    /* Save boot params to 0x00000040 */
    ldr r0, =0x00000040
    str sp, [r0, #0]
    str lr, [r0, #4]
    mrs lr, cpsr
    str lr, [r0, #8]
    mrc p15, 0, lr, c1, c0, 0
    str lr, [r0, #12]
    mrc p15, 0, lr, c1, c0, 0
    str lr, [r0, #16]

    /* Check FEL boot: magic 0x4c45462e at 0x00000008.
     * Save result to 0x58 before the vector copy overwrites 0x00000008. */
    mov r0, #0x0
    mov r1, #0
    str r1, [r0, #0x58]        /* Pre-clear FEL flag (non-FEL boot → 0) */
    ldr r1, [r0, #8]
    ldr r2, =0x4c45462e       /* ".FEL" in little-endian */
    cmp r1, r2
    bne 1f
    ldr r1, =0x1
    str r1, [r0, #0x58]        /* Mark FEL boot at 0x58 */
1:  nop

    /* Enter SVC mode and mask IRQ/FIQ interrupts */
    mrs r0, cpsr
    bic r0, r0, #0x1f
    orr r0, r0, #0xd3           /* MODE_SVC | NOINT */
    msr cpsr_cxsf, r0

    /* CPU critical init (cache invalidate, MMU disable) */
    bl cpu_init_crit

    /* Set vector base to low address (V=0 bit clear in CP15 c1) */
    mrc p15, 0, r0, c1, c0, 0
    bic r0, r0, #(1 << 13)
    mcr p15, 0, r0, c1, c0, 0

    /* Copy exception vectors to 0x00000000 */
    adr r0, _vector
    mrc p15, 0, r2, c1, c0, 0
    ands r2, r2, #(1 << 13)
    ldreq r1, =0x00000000
    ldrne r1, =0xffff0000
    ldmia r0!, {{r2-r8, r10}}
    stmia r1!, {{r2-r8, r10}}
    ldmia r0!, {{r2-r8, r10}}
    stmia r1!, {{r2-r8, r10}}

    /* Turn off watchdog */
    ldr r0, =WDOG_BASE
    mov r1, #0x0
    str r1, [r0]

    /* Mask all IRQ sources */
    ldr r1, =0xffffffff
    ldr r0, =IRQ_MASK0
    str r1, [r0], #0x04
    str r1, [r0]

    /* Set up temporary SVC stack in SRAM (DRAM not available yet).
     * All sections are linked at 0x80000000 (DRAM), so svc_stack_start
     * resolves to a DRAM address which is invalid before dram::init().
     * Use the top of F1C100s internal SRAM A (32 KB at 0x00000000). */
    ldr sp, =0x00008000
    ldr r0, =low_level_init
    blx r0
    
    /*
     * Speed up: if running from SRAM (adr != ldr _start),
     * copy SPL to DRAM and jump there for faster execution.
     */
    adr r0, _start
    ldr r1, =_start
    cmp r0, r1
    beq _relocate

    /* Copy SPL from SRAM (r0) to SPEEDUP_ADDR (r1) */
    ldr r1, =SPEEDUP_ADDR
    ldr r2, =__spl_size
    bl memcpy

    /* Jump to the speedup copy */
    ldr r0, =_relocate
    ldr r1, =_start
    sub r0, r0, r1
    ldr r1, =SPEEDUP_ADDR
    add r0, r0, r1
    mov pc, r0

_relocate:
    nop

    /* Detect xfel / FEL direct-to-DRAM boot.
     * Cannot re-read 0x00000008 here — the vector copy already overwrote it.
     * Instead check the flag we saved to 0x00000058 before the vector copy. */
    mov r0, #0
    ldr r1, [r0, #0x58]
    cmp r1, #0x1
    beq _dram_entry             /* FEL: skip flash copy */

    /* Normal SPI flash boot: copy full image from flash to DRAM */
    bl sys_copyself

    /* Jump to the freshly loaded DRAM image at the link address */
    ldr r0, =_dram_entry
    mov pc, r0

/*
 * cpu_init_crit - Critical CPU initialization
 */
cpu_init_crit:
    mov r0, #0
    mcr p15, 0, r0, c7, c7, 0    /* Invalidate both caches */
    mcr p15, 0, r0, c8, c7, 0    /* Invalidate TLB */

    mrc p15, 0, r0, c1, c0, 0
    bic r0, r0, #0x00002300       /* Clear V[13], I[12], Z[11] */
    bic r0, r0, #0x00000087       /* Clear B[7], C[2], M[0], A[1] */
    orr r0, r0, #0x00000002       /* Enable alignment fault checking */
    orr r0, r0, #0x00001000       /* Enable I-cache */
    mcr p15, 0, r0, c1, c0, 0
    bx lr

/*
 * memcpy - Copy n bytes from src (r0) to dst (r1), count in r2
 * Returns: r0 = dst
 */
.global memcpy
memcpy:
    cmp r2, #0
    bxeq lr
    mov r3, r0
1:  ldrb r12, [r0], #1
    strb r12, [r1], #1
    subs r2, r2, #1
    bne 1b
    mov r0, r3
    bx lr

.section .text
_dram_entry:
    /* ── Running from DRAM at 0x80000000 ── */
    
    /* Set up stacks for all CPU modes */
    mrs r0, cpsr
    bic r0, r0, #0x1f

    /* Undefined mode */
    orr r1, r0, #0x1b | NOINT
    msr cpsr_cxsf, r1
    ldr sp, =und_stack_start

    /* Abort mode */
    orr r1, r0, #0x17 | NOINT
    msr cpsr_cxsf, r1
    ldr sp, =abt_stack_start

    /* IRQ mode */
    orr r1, r0, #0x12 | NOINT
    msr cpsr_cxsf, r1
    ldr sp, =irq_stack_start

    /* FIQ mode */
    orr r1, r0, #0x11 | NOINT
    msr cpsr_cxsf, r1
    ldr sp, =fiq_stack_start

    /* System mode */
    orr r1, r0, #0x1F | NOINT
    msr cpsr_cxsf, r1
    ldr sp, =sys_stack_start

    /* Supervisor mode (default for main) */
    orr r1, r0, #0x13 | NOINT
    msr cpsr_cxsf, r1
    ldr sp, =svc_stack_start

    /* Clear BSS */
    ldr r0, =__bss_start
    mov r1, #0
    ldr r2, =__bss_end
    sub r2, r2, r0
    bl memset

    /* Jump to Rust entry point */
    ldr pc, _rust_main
_rust_main:
    .word rust_main

.section .text
/*
 * memset - Fill n bytes at dst (r0) with value (r1), count in r2
 * Returns: r0 = dst
 */
.global memset
memset:
    cmp r2, #0
    bxeq lr
    mov r3, r0
1:  strb r1, [r0], #1
    subs r2, r2, #1
    bne 1b
    mov r0, r3
    bx lr

/*
 * Stack space for each CPU mode
 */
.section .data
    .space UND_STACK_SIZE
    .align 3
    .global und_stack_start
und_stack_start:

    .space ABT_STACK_SIZE
    .align 3
    .global abt_stack_start
abt_stack_start:

    .space FIQ_STACK_SIZE
    .align 3
    .global fiq_stack_start
fiq_stack_start:

    .space IRQ_STACK_SIZE
    .align 3
    .global irq_stack_start
irq_stack_start:

    .skip SYS_STACK_SIZE
    .align 3
    .global sys_stack_start
sys_stack_start:

    .space SVC_STACK_SIZE
    .align 3
    .global svc_stack_start
svc_stack_start:    

/*
 * Exception handlers
 * All use: sub lr, lr, #4 (or #8 for data abort)
 */

.macro push_svc_reg
    sub sp, sp, #17 * 4
    stmia sp, {{r0 - r12}}
    mov r0, sp
    mrs r6, spsr
    str lr, [r0, #15*4]
    str r6, [r0, #16*4]
    str sp, [r0, #13*4]
    str lr, [r0, #14*4]
.endm

.align 5
undefined_instruction:
    sub lr, lr, #4
    push_svc_reg
    bl trap_undef
    b .

.align 5
.weak SVC_Handler
SVC_Handler:
software_interrupt:
    sub lr, lr, #4
    push_svc_reg
    bl trap_swi
    b .

.align 5
prefetch_abort:
    sub lr, lr, #4
    push_svc_reg
    bl trap_pabt
    b .

.align 5
data_abort:
    sub lr, lr, #8
    push_svc_reg
    bl trap_dabt
    b .

.align 5
not_used:
    b .

/*
 * IRQ handler with preemptive context switch support.
 *
 * On entry:  LR_irq = interrupted PC + 4,  SPSR_irq = interrupted CPSR
 * No sub-lr-4 before save — we compute PC = LR-4 during the switch.
 *
 * Normal exit:  SUBS PC, LR, #4  (= interrupted PC, restores CPSR via SPSR)
 * Switch exit:  rebuild SVC frame, swap stacks, restore new thread.
 *
 * SVC frame layout (matches context_switch frame):
 *   [sp+0]  cpsr   [sp+4..52] r0-r12   [sp+56] lr   [sp+60] pc
 */
.align 5
irq:
    stmfd   sp!, {{r0-r12,lr}}          @ save r0-r12, lr_irq (not pre-adjusted)
    bl      trap_irq                    @ dispatch interrupt
    ldr     r0, =switch_interrupt_flag
    ldr     r1, [r0]
    cmp     r1, #1
    beq     irq_context_switch
    ldmfd   sp!, {{r0-r12,lr}}
    subs    pc, lr, #4                  @ return: PC = LR_irq - 4 = interrupted PC

irq_context_switch:
    mov     r1, #0
    str     r1, [r0]                    @ clear switch flag

    mov     r1, sp                      @ r1 = irq_sp (points to saved r0)
    add     sp, sp, #4*4                @ advance past r0-r3 (16 bytes)
    ldmfd   sp!, {{r4-r12,lr}}          @ pop r4-r12 + lr_irq (10 regs)
    mrs     r0, spsr                    @ r0 = interrupted CPSR
    sub     r2, lr, #4                  @ r2 = interrupted PC = lr_irq - 4

    @ switch to SVC mode, IRQ+FIQ disabled (0x80|0x40|0x13 = 0xD3)
    msr     cpsr_c, #0xd3

    @ build context frame on SVC stack (same layout as context_switch)
    stmfd   sp!, {{r2}}                 @ push PC
    stmfd   sp!, {{r4-r12,lr}}          @ push r4-r12, lr_svc
    ldmfd   r1, {{r1-r4}}              @ load r0-r3 from IRQ stack (r1=irq_sp)
    stmfd   sp!, {{r1-r4}}             @ push r0-r3
    stmfd   sp!, {{r0}}                @ push cpsr

    @ save from-thread SP
    ldr     r4, =interrupt_from_thread
    ldr     r5, [r4]
    str     sp, [r5]                    @ from_thread->sp = svc_sp

    @ load to-thread SP
    ldr     r6, =interrupt_to_thread
    ldr     r6, [r6]
    ldr     sp, [r6]                    @ sp = to_thread->sp

    @ restore new thread context
    ldmfd   sp!, {{r4}}
    msr     spsr_cxsf, r4
    ldmfd   sp!, {{r0-r12,lr,pc}}^      @ restore r0-r12,lr,pc; CPSR←SPSR

.align 5
fiq:
    sub lr, lr, #4
    stmfd sp!, {{r0-r7,lr}}
    bl trap_fiq
    ldmfd sp!, {{r0-r7,lr}}
    subs pc, lr, #4

/*
 * Linker-defined symbol pointers (used by xboot-style relocation)
 */
    .align 4
_image_start:   .long __image_start
_image_end:     .long __image_end
_data_start:    .long __data_start
_data_end:      .long __data_end
_bss_start:     .long __bss_start
_bss_end:       .long __bss_end
