//! MMU and Cache management for Allwinner F1C100s (ARM926EJ-S)
//!
//! Implements ARMv5TE-compatible MMU with:
//! - Level 1 page table (section descriptors, 1MB granularity)
//! - TLB management
//! - I-cache / D-cache control
//! - Cache maintenance operations by range

use crate::cpu;

/// ARM926EJ-S cache line size: 32 bytes (8 words)
pub const CACHE_LINE_SIZE: u32 = 32;

/// Number of level-1 page table entries (4096 sections × 1MB = 4GB)
pub const PAGE_TABLE_ENTRIES: usize = 4096;

/// Required alignment for the page table (16 KB)
pub const PAGE_TABLE_ALIGNMENT: usize = 16 * 1024;

// ── Section Descriptor Attributes ───────────────────────────────────────

/// Section descriptor base (bit 1 = section type, bit 4 = 1 for ARMv5)
pub const DESC_SEC: u32 = 0x2 | (1 << 4);

// C, B bits (Cacheable, Bufferable)
pub const CB: u32 = 3 << 2;   // cache on, write-back
pub const CNB: u32 = 2 << 2;  // cache on, write-through
pub const NCB: u32 = 1 << 2;  // cache off, write buffer on
pub const NCNB: u32 = 0 << 2; // cache off, write buffer off

// Access Permission bits (AP[1:0])
pub const AP_RW: u32 = 3 << 10; // supervisor=R/W, user=R/W
pub const AP_RO: u32 = 2 << 10; // supervisor=R/W, user=RO

// Domain access
pub const DOMAIN_FAULT: u32 = 0x0;
pub const DOMAIN_CHK: u32 = 0x1;
pub const DOMAIN_NOTCHK: u32 = 0x3;
pub const DOMAIN0: u32 = 0x0 << 5;
pub const DOMAIN1: u32 = 0x1 << 5;

pub const DOMAIN0_ATTR: u32 = DOMAIN_CHK << 0;
pub const DOMAIN1_ATTR: u32 = DOMAIN_FAULT << 2;

/// Pre-composed memory attributes
pub const RW_CB: u32 = AP_RW | DOMAIN0 | CB | DESC_SEC;       // cacheable write-back
pub const RW_CNB: u32 = AP_RW | DOMAIN0 | CNB | DESC_SEC;     // cacheable write-through
pub const RW_NCNB: u32 = AP_RW | DOMAIN0 | NCNB | DESC_SEC;   // non-cacheable
pub const RW_FAULT: u32 = AP_RW | DOMAIN1 | NCNB | DESC_SEC;  // access generates fault

// ── Memory Descriptor ───────────────────────────────────────────────────

/// Describes a memory region mapping for MMU setup
#[derive(Debug, Clone, Copy)]
pub struct MemDesc {
    pub vaddr_start: u32,
    pub vaddr_end: u32,
    pub paddr_start: u32,
    pub attr: u32,
}

impl MemDesc {
    /// Create a new memory descriptor
    pub const fn new(vaddr_start: u32, vaddr_end: u32, paddr_start: u32, attr: u32) -> Self {
        Self {
            vaddr_start,
            vaddr_end,
            paddr_start,
            attr,
        }
    }
}

// ── Page Table ──────────────────────────────────────────────────────────

/// Level 1 page table: 4096 entries, 16KB-aligned
#[repr(align(16384))]
pub struct PageTable {
    pub entries: [u32; PAGE_TABLE_ENTRIES],
}

impl PageTable {
    /// Create a new zero-initialized page table
    pub const fn new() -> Self {
        Self {
            entries: [0u32; PAGE_TABLE_ENTRIES],
        }
    }

    /// Fill a range of the page table with section descriptors.
    /// `vaddr_start` and `vaddr_end` define the virtual address range mapped.
    /// `paddr_start` is the physical start address.
    /// `attr` combines access permissions, domain, cacheability, etc.
    pub fn set_mapping(&mut self, vaddr_start: u32, vaddr_end: u32, paddr_start: u32, attr: u32) {
        let start_sec = (vaddr_start >> 20) as usize;
        let end_sec = (vaddr_end >> 20) as usize;
        let paddr_base = paddr_start >> 20;

        for (i, sec) in (start_sec..=end_sec).enumerate() {
            self.entries[sec] = attr | ((paddr_base + i as u32) << 20);
        }
    }

    /// Fill a range from a `MemDesc` descriptor
    pub fn set_mapping_from_desc(&mut self, desc: &MemDesc) {
        self.set_mapping(desc.vaddr_start, desc.vaddr_end, desc.paddr_start, desc.attr);
    }

    /// Fill multiple mappings from a slice of `MemDesc`
    pub fn set_mappings(&mut self, descs: &[MemDesc]) {
        for desc in descs {
            self.set_mapping_from_desc(desc);
        }
    }

    /// Get the physical base address of this page table
    pub fn base_address(&self) -> u32 {
        self as *const Self as u32
    }
}

// ── MMU Control ─────────────────────────────────────────────────────────

/// Set the Translation Table Base (TTB) register and initialize domains.
/// This invalidates the TLB, sets all domains to client mode,
/// then points TTBR0 to the page table.
pub fn set_ttb(page_table: &PageTable) {
    let base = page_table.base_address();
    cpu::invalidate_tlb();
    // Set domain access control: all 16 domains as client (01)
    cpu::cp15_write_dacr(0x5555_5555);
    // Set translation table base
    cpu::cp15_write_ttbr0(base);
}

/// Enable the MMU
pub fn enable_mmu() {
    let ctrl = cpu::cp15_read_ctrl();
    cpu::cp15_write_ctrl(ctrl | cpu::cp15_ctrl::MMU_ENABLE);
}

/// Disable the MMU
pub fn disable_mmu() {
    let ctrl = cpu::cp15_read_ctrl();
    cpu::cp15_write_ctrl(ctrl & !cpu::cp15_ctrl::MMU_ENABLE);
}

/// Enable I-cache
pub fn enable_icache() {
    let ctrl = cpu::cp15_read_ctrl();
    cpu::cp15_write_ctrl(ctrl | cpu::cp15_ctrl::ICACHE_ENABLE);
}

/// Disable I-cache
pub fn disable_icache() {
    let ctrl = cpu::cp15_read_ctrl();
    cpu::cp15_write_ctrl(ctrl & !cpu::cp15_ctrl::ICACHE_ENABLE);
}

/// Enable D-cache
pub fn enable_dcache() {
    let ctrl = cpu::cp15_read_ctrl();
    cpu::cp15_write_ctrl(ctrl | cpu::cp15_ctrl::DCACHE_ENABLE);
}

/// Disable D-cache
pub fn disable_dcache() {
    let ctrl = cpu::cp15_read_ctrl();
    cpu::cp15_write_ctrl(ctrl & !cpu::cp15_ctrl::DCACHE_ENABLE);
}

/// Enable alignment fault checking
pub fn enable_align_fault() {
    let ctrl = cpu::cp15_read_ctrl();
    cpu::cp15_write_ctrl(ctrl | cpu::cp15_ctrl::ALIGN_FAULT);
}

/// Disable alignment fault checking
pub fn disable_align_fault() {
    let ctrl = cpu::cp15_read_ctrl();
    cpu::cp15_write_ctrl(ctrl & !cpu::cp15_ctrl::ALIGN_FAULT);
}

/// Check if I-cache is enabled
pub fn icache_status() -> bool {
    cpu::cp15_read_ctrl() & cpu::cp15_ctrl::ICACHE_ENABLE != 0
}

/// Check if D-cache is enabled
pub fn dcache_status() -> bool {
    cpu::cp15_read_ctrl() & cpu::cp15_ctrl::DCACHE_ENABLE != 0
}

/// Check if MMU is enabled
pub fn mmu_status() -> bool {
    cpu::cp15_read_ctrl() & cpu::cp15_ctrl::MMU_ENABLE != 0
}

// ── Cache Maintenance (by range) ────────────────────────────────────────

/// Clean (write-back) D-cache for the given address range
pub fn clean_dcache(buffer: u32, size: u32) {
    let start = buffer & !(CACHE_LINE_SIZE - 1);
    let end = buffer + size;
    let mut ptr = start;

    while ptr < end {
        cpu::clean_dcache_mva(ptr);
        ptr += CACHE_LINE_SIZE;
    }
}

/// Invalidate D-cache for the given address range without write-back
pub fn invalidate_dcache(buffer: u32, size: u32) {
    let start = buffer & !(CACHE_LINE_SIZE - 1);
    let end = buffer + size;
    let mut ptr = start;

    while ptr < end {
        cpu::invalidate_dcache_mva(ptr);
        ptr += CACHE_LINE_SIZE;
    }
}

/// Clean and invalidate D-cache for the given address range
pub fn clean_invalidate_dcache(buffer: u32, size: u32) {
    let start = buffer & !(CACHE_LINE_SIZE - 1);
    let end = buffer + size;
    let mut ptr = start;

    while ptr < end {
        cpu::clean_invalidate_dcache_mva(ptr);
        ptr += CACHE_LINE_SIZE;
    }
}

// ── High-level MMU Initialization ───────────────────────────────────────

/// Initialize the MMU with the given memory descriptors.
///
/// This is the main entry point for MMU setup. It:
/// 1. Disables caches and MMU
/// 2. Invalidates TLB
/// 3. Fills the page table from descriptors
/// 4. Sets the TTB register
/// 5. Enables MMU, I-cache, D-cache
/// 6. Invalidates caches for clean state
pub fn init(mdesc: &[MemDesc]) {
    // Disable caches and MMU first
    disable_dcache();
    disable_icache();
    disable_mmu();
    cpu::invalidate_tlb();

    // Create page table and fill from descriptors
    let mut page_table = PageTable::new();
    page_table.set_mappings(mdesc);

    // Set TTB and domain access
    set_ttb(&page_table);

    // Enable MMU
    enable_mmu();

    // Enable caches
    enable_icache();
    enable_dcache();

    // Invalidate caches for clean state
    cpu::invalidate_icache();
    cpu::invalidate_dcache_all();
}
