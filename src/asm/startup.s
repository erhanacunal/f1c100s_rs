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

    /* Route exceptions to SPL-resident debug handlers while we run from
     * SRAM: the normal handlers live past the BROM-loaded region and the
     * pointer slots hold DRAM link addresses that are not valid yet.
     * adr produces the current RUN address (SRAM or DRAM), so this works
     * in both boot paths. _dram_entry restores the normal handlers. */
    mov r1, #0
    adr r0, spl_exc_undef
    str r0, [r1, #0x20]         @ undefined instruction
    adr r0, spl_exc_swi
    str r0, [r1, #0x24]         @ software interrupt
    adr r0, spl_exc_pabt
    str r0, [r1, #0x28]         @ prefetch abort
    adr r0, spl_exc_dabt
    str r0, [r1, #0x2c]         @ data abort
    adr r0, spl_exc_undef
    str r0, [r1, #0x30]         @ not_used
    adr r0, spl_exc_irq
    str r0, [r1, #0x34]         @ irq
    adr r0, spl_exc_fiq
    str r0, [r1, #0x38]         @ fiq

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
    bl low_level_init

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

    /* Flush literal pool here: the .text.spl section also accumulates all
     * Rust #[link_section] functions after this asm, so without .ltorg the
     * pool lands at the section end — out of ldr's ±4KB range once the
     * Rust side grows. */
    .ltorg

/*
 * cpu_init_crit - Critical CPU initialization
 */
cpu_init_crit:
    /* Invalidate caches/TLB but do NOT touch the CP15 control register:
     * xboot's known-good boot leaves the BROM CPU state untouched (only
     * the V bit is cleared, by reset itself) all the way through clock
     * and DRAM init. I-cache + alignment checking are enabled later at
     * _dram_entry, once the SPL phase is over. */
    mov r0, #0
    mcr p15, 0, r0, c7, c7, 0    /* Invalidate both caches */
    mcr p15, 0, r0, c8, c7, 0    /* Invalidate TLB */
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

/*
 * ── Early UART2 Debug Initialization ──────────────────────────────────
 *
 * For debugging only.  Initializes UART2 (PE7=TX, PE8=RX) at 115200-8-N-1
 * so that boot progress can be monitored via serial before Rust starts.
 *
 * early_uart2_init   — GPIO mux + clock gate + reset + baud rate
 * early_uart2_putc   — blocking TX of a single byte (r0 = char)
 * early_uart2_puts   — blocking TX of a NUL-terminated string (r0 = ptr)
 *
 * Clobbers: r0-r2 (init), r0-r2 (putc), r0-r5 (puts)
 */
.section .text.spl
early_uart2_init:
    /* ── 1. GPIO mux: PE7 = TX, PE8 = RX (FUNC2 = 0x3) ──────────── */
    ldr r0, =0x01C20890         @ Port E CFG0 (pins 0-7)
    ldr r1, [r0]
    bic r1, r1, #(0xF << 28)    @ Clear PE7 config bits [31:28]
    orr r1, r1, #(0x3 << 28)    @ FUNC2
    str r1, [r0]

    ldr r0, =0x01C20894         @ Port E CFG1 (pins 8-15)
    ldr r1, [r0]
    bic r1, r1, #0xF            @ Clear PE8 config bits [3:0]
    orr r1, r1, #0x3            @ FUNC2
    str r1, [r0]

    /* ── 2. Enable UART2 clock gate (CCU BUS_GATE2, bit 22) ──────── */
    ldr r0, =0x01C20068         @ CCU_BASE + BUS_GATE2
    ldr r1, [r0]
    orr r1, r1, #(1 << 22)
    str r1, [r0]

    /* ── 3. De-assert UART2 reset (CCU BUS_SOFT_RST2, bit 22) ────── */
    ldr r0, =0x01C202D0         @ CCU_BASE + BUS_SOFT_RST2
    ldr r1, [r0]
    orr r1, r1, #(1 << 22)
    str r1, [r0]

    /* ── 4. Configure UART2: 115200-8-N-1 ────────────────────────── */
    ldr r0, =0x01C25800         @ UART2 base
    mov r1, #0
    str r1, [r0, #0x04]         @ IER = 0
    mov r1, #0x07
    str r1, [r0, #0x08]         @ FCR = 0x07 (enable FIFO, clear TX/RX)
    mov r1, #0
    str r1, [r0, #0x10]         @ MCR = 0

    mov r1, #0x80
    str r1, [r0, #0x0C]         @ LCR = DLAB
    mov r1, #54                 @ divisor = 100MHz / (115200*16) ≈ 54
    str r1, [r0, #0x00]         @ DLL
    mov r1, #0
    str r1, [r0, #0x04]         @ DLH

    mov r1, #0x03               @ 8-N-1, DLAB=0
    str r1, [r0, #0x0C]         @ LCR

    bx lr

/*
 * early_uart2_putc — Send one byte (blocking poll on TX FIFO)
 *   r0 = character
 */
early_uart2_putc:
    ldr r1, =0x01C25800
1:  ldr r2, [r1, #0x7C]        @ USR
    tst r2, #2                  @ TX_FIFO_NOT_FULL (bit 1)
    beq 1b
    str r0, [r1, #0x00]         @ THR
    bx lr

/*
 * ── SPL exception handlers ────────────────────────────────────────────
 *
 * Print the exception cause + banked LR (fault address) on UART2, then
 * halt. No stack use (banked SPs are uninitialized in exception modes),
 * no .rodata. LR meaning: data abort = fault PC + 8, prefetch abort /
 * undef = fault PC + 4.
 */
spl_exc_undef:
    mov r0, #'U'
    b spl_exc_common
spl_exc_swi:
    mov r0, #'W'
    b spl_exc_common
spl_exc_pabt:
    mov r0, #'P'
    b spl_exc_common
spl_exc_dabt:
    mov r0, #'A'
    b spl_exc_common
spl_exc_irq:
    mov r0, #'I'
    b spl_exc_common
spl_exc_fiq:
    mov r0, #'Q'
    b spl_exc_common

spl_exc_common:
    mov r4, lr                  @ save fault address (bl clobbers lr)
    bl early_uart2_putc
    mov r0, #':'
    bl early_uart2_putc
    mov r5, #8                  @ print r4 as 8 hex digits, MSB first
1:  mov r0, r4, lsr #28
    cmp r0, #9
    addls r0, r0, #'0'
    addhi r0, r0, #('A' - 10)
    bl early_uart2_putc
    mov r4, r4, lsl #4
    subs r5, r5, #1
    bne 1b
2:  b 2b                        @ halt

/*
 * early_uart2_puts — Send a NUL-terminated string
 *   r0 = pointer to string
 */
early_uart2_puts:
    mov r3, lr                  @ save return address
    mov r4, r0                  @ save string ptr
    ldr r5, =0x01C25800         @ UART2 base
1:  ldrb r0, [r4], #1
    cmp r0, #0
    beq 2f
    @ Wait for TX FIFO not full
0:  ldr r1, [r5, #0x7C]         @ USR
    tst r1, #2
    beq 0b
    str r0, [r5, #0x00]         @ THR
    b 1b
2:  bx r3

    .ltorg                      @ flush UART register-address literals

.section .rodata
/* Only valid once the full image is in DRAM (after sys_copyself / FEL load) —
 * .rodata is linked at a DRAM address and is NOT part of the BROM-loaded SPL. */
early_uart2_banner:
    .asciz "\r\n[UART2] early init OK\r\n"

.section .text
_dram_entry:
    /* ── Running from DRAM at 0x80000000 ── */

    /* Restore the normal exception handler pointers — the full image is
     * in DRAM now, so the link-time slots in _vector are valid again
     * (reset patched them to the SPL debug handlers). */
    ldr r0, =_vector
    ldr r1, =0x00000000
    ldmia r0!, {{r2-r8, r10}}
    stmia r1!, {{r2-r8, r10}}
    ldmia r0!, {{r2-r8, r10}}
    stmia r1!, {{r2-r8, r10}}

    /* Enable I-cache + alignment fault checking now that the SPL phase
     * is over (kept at BROM defaults during SPL to match xboot). This is
     * the CPU state the runtime has always used. */
    mrc p15, 0, r0, c1, c0, 0
    orr r0, r0, #0x00000002      /* A: alignment fault checking */
    orr r0, r0, #0x00001000      /* I: instruction cache */
    mcr p15, 0, r0, c1, c0, 0

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
    .ltorg                      @ flush _dram_entry's literals (.text grows too)

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
