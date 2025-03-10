use core::arch::naked_asm;
use core::sync::atomic::{Ordering, AtomicBool, AtomicU64, fence};

use fdt::Fdt;
use hermit_entry::Entry;
use hermit_entry::boot_info::RawBootInfo;

use super::{CPU_ONLINE, CURRENT_BOOT_ID, HART_MASK, NUM_CPUS, get_dtb_ptr};
use crate::arch::riscv64::kernel::CURRENT_STACK_ADDRESS;
use crate::arch::riscv64::kernel::processor;
use crate::{KERNEL_STACK_SIZE, env};

const MAX_CORES: usize = 32;

// Cache-line aligned CPU-lokale Daten
#[repr(align(64))]
struct PerCpuData {
    is_initialized: AtomicBool,
    local_counter: AtomicU64,
    padding: [u8; 48],  // Auffüllen auf volle Cache-Line
}

impl PerCpuData {
    const fn new() -> Self {
        Self {
            is_initialized: AtomicBool::new(false),
            local_counter: AtomicU64::new(0),
            padding: [0; 48],
        }
    }
}

static mut CPU_DATA: [PerCpuData; MAX_CORES] = {
    const CPU_LOCAL: PerCpuData = PerCpuData::new();
    [CPU_LOCAL; MAX_CORES]
};

/// Entrypoint - Initialize Stack pointer and Exception Table
#[unsafe(no_mangle)]
#[naked]
pub unsafe extern "C" fn _start(hart_id: usize, boot_info: Option<&'static RawBootInfo>) -> ! {
	// validate signatures
	// `_Start` is compatible to `Entry`
	{
		unsafe extern "C" fn _entry(_hart_id: usize, _boot_info: &'static RawBootInfo) -> ! {
			unreachable!()
		}
		pub type _Start =
			unsafe extern "C" fn(hart_id: usize, boot_info: Option<&'static RawBootInfo>) -> !;
		const _ENTRY: Entry = _entry;
		const _START: _Start = _start;
		const _PRE_INIT: _Start = pre_init;
	}

	unsafe {
		naked_asm!(
			// Use stack pointer from `CURRENT_STACK_ADDRESS` if set
			"ld      t0, {current_stack_pointer}",
			"beqz    t0, 2f",
			"li      t1, {top_offset}",
			"add     t0, t0, t1",
			"mv      sp, t0",
			"2:",

			"j       {pre_init}",
			current_stack_pointer = sym CURRENT_STACK_ADDRESS,
			top_offset = const KERNEL_STACK_SIZE,
			pre_init = sym pre_init,
		)
	}
}

unsafe extern "C" fn pre_init(hart_id: usize, boot_info: Option<&'static RawBootInfo>) -> ! {
    // Hart-ID Validierung optimiert
    if CPU_ONLINE.load(Ordering::Acquire) > 0 {
        // Schnellerer Check für Secondary-HARTs
        if (HART_MASK.load(Ordering::Relaxed) & (1 << hart_id)) == 0 {
            error!("Invalid hart ID: {}", hart_id);
            processor::halt();
        }
    }

    // Memory Fence vor ID-Speicherung
    fence(Ordering::Release);
    CURRENT_BOOT_ID.store(hart_id as u32, Ordering::Release);

    if CPU_ONLINE.load(Ordering::Acquire) == 0 {
        // Boot CPU Initialisierung
        unsafe {
            env::set_boot_info(*boot_info.unwrap());
            let fdt = Fdt::from_ptr(get_dtb_ptr()).expect("FDT is invalid");
            
            // Optimierte HART_MASK Berechnung
            let mut hart_mask = 0u64;
            for cpu in fdt.cpus() {
                if let Some(cpu_id) = cpu.property("reg").and_then(|p| p.as_usize()) {
                    if cpu.property("status")
                        .and_then(|p| p.as_str())
                        .map_or(false, |s| s != "disabled\u{0}") 
                    {
                        hart_mask |= 1 << cpu_id;
                    }
                }
            }

            NUM_CPUS.store(fdt.cpus().count().try_into().unwrap(), Ordering::Release);
            
            // Memory Fence vor HART_MASK Update
            fence(Ordering::Release);
            HART_MASK.store(hart_mask, Ordering::Release);
            
            CPU_DATA[hart_id].is_initialized.store(true, Ordering::Release);
            CPU_DATA[hart_id].local_counter.store(1, Ordering::Release);
        }
        crate::boot_processor_main()
    } else {
        #[cfg(not(feature = "smp"))]
        {
            error!("SMP support deactivated");
            loop {
                processor::halt();
            }
        }
        #[cfg(feature = "smp")] {
            unsafe {
                // Optimierte Secondary-HART Initialisierung
                fence(Ordering::Acquire);
                CPU_DATA[hart_id].is_initialized.store(true, Ordering::Release);
                CPU_DATA[hart_id].local_counter.fetch_add(1, Ordering::Relaxed);
            }
            crate::application_processor_main()
        }
    }
}
