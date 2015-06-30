#include "types.h"
#include "util/cassert.h"

#ifndef _SFI_H_
#define _SFI_H_

#define SFI_SIG_SYST "SYST"
#define SFI_SIG_FREQ "FREQ"
#define SFI_SIG_IDLE "IDLE"
#define SFI_SIG_CPUS "CPUS"
#define SFI_SIG_MTMR "MTMR"
#define SFI_SIG_MRTC "MRTC"
#define SFI_SIG_MMAP "MMAP"
#define SFI_SIG_APIC "APIC"
#define SFI_SIG_XSDT "XSDT"
#define SFI_SIG_WAKE "WAKE"
#define SFI_SIG_DEVS "DEVS"
#define SFI_SIG_GPIO "GPIO"
#define SFI_SIG_SIZE 4

#define SFI_SYST_SEARCH_BEGIN           0x000E0000
#define SFI_SYST_SEARCH_END             0x000FFFFF

typedef char sfi_signature[4];


#define EFI_RESERVED_MEMORY_TYPE        0
#define EFI_LOADER_CODE                 1
#define EFI_LOADER_DATA                 2
#define EFI_BOOT_SERVICES_CODE          3
#define EFI_BOOT_SERVICES_DATA          4
#define EFI_RUNTIME_SERVICES_CODE       5
#define EFI_RUNTIME_SERVICES_DATA       6
#define EFI_CONVENTIONAL_MEMORY         7
#define EFI_UNUSABLE_MEMORY             8
#define EFI_ACPI_RECLAIM_MEMORY         9
#define EFI_ACPI_MEMORY_NVS             10
#define EFI_MEMORY_MAPPED_IO            11
#define EFI_MEMORY_MAPPED_IO_PORT_SPACE 12
#define EFI_PAL_CODE                    13
#define EFI_MAX_MEMORY_TYPE             14

/* SFI Common Table Header Format */

typedef struct {
  sfi_signature signature;
  uint32 length;
  uint8 revision;
  uint8 checksum;
  uint64 oem_id : 48;
  char oem_table_id[8];
} PACKED sfi_common_table_header_t;

#define SFI_TABLE_HEADER_LEN 24

typedef struct {
  sfi_common_table_header_t common;
  uint64 entries[];
} PACKED sfi_system_table_t;

typedef struct {
  sfi_common_table_header_t common;
  uint32 lapic_ids[];
} PACKED sfi_cpus_table_t;

typedef struct {
  sfi_common_table_header_t common;
  uint64 ioapic_phys[];
} PACKED sfi_apic_table_t;

typedef struct {
  uint32 type;
  uint64 physical_start;
  uint64 virtual_start;
  uint64 num_of_pages;
  uint64 attribute;
} PACKED efi_memory_descriptor_t;

CASSERT(sizeof(efi_memory_descriptor_t) == 36, efi_memory_descriptor_size);

typedef struct {
  sfi_common_table_header_t common;
  efi_memory_descriptor_t memory_descriptors[];
} PACKED sfi_mmap_table_t;

typedef struct {
  uint32 frequency;
  uint32 transition_latency;
  uint32 perf_ctl_val;
} PACKED sfi_frequency_t;

CASSERT(sizeof(sfi_frequency_t) == 12, sfi_frequency_size);

typedef struct {
  sfi_common_table_header_t common;
  sfi_frequency_t frequencies[];
} PACKED sfi_frequency_table_t;

typedef struct {
  uint64 phys_addr;
  uint32 frequency;
  uint32 irq;
} PACKED sfi_mtimer_t;

CASSERT(sizeof(sfi_mtimer_t) == 16, sfi_mtimer_size);

typedef struct {
  sfi_common_table_header_t common;
  sfi_mtimer_t timers[];
} PACKED sfi_mtimer_table_t;


typedef struct {
  uint64 phys_addr;
  uint32 irq;
} PACKED sfi_mrtc_t;

CASSERT(sizeof(sfi_mrtc_t) == 12, sfi_mrtc_size);

typedef struct {
  sfi_common_table_header_t common;
  sfi_mrtc_t rtcs[];
} PACKED sfi_mrtc_table_t;

typedef struct {
  sfi_common_table_header_t common;
  uint64 phys_addr;
} PACKED sfi_wake_table_t;


typedef struct {
  uint8 host_type;
  uint8 host_number;
  uint16 address;
  uint8 irq;
  uint32 max_frequency;
  char name[16];
} PACKED sfi_device_t;

CASSERT(sizeof(sfi_device_t) == 25, sfi_device_size);

typedef struct {
  sfi_common_table_header_t common;
  sfi_device_t devices[];
} PACKED sfi_devs_table_t;



typedef struct {
  char controller_name[16];
  uint16 pin_number;
  char pin_name[16];
} PACKED sfi_gpio_t;

CASSERT(sizeof(sfi_gpio_t) == 34, sfi_gpio_size);

typedef struct {
  sfi_common_table_header_t common;
  sfi_gpio_t gpios[];
} PACKED sfi_gpio_table_t;

typedef struct {
  void* start_addr;
  phys_addr_t low_frame;
  size_t num_frames;
  sfi_system_table_t *system_table;
  sfi_cpus_table_t *cpus_table;
  sfi_apic_table_t *apic_table;
  sfi_mmap_table_t *mmap_table;
  sfi_frequency_table_t *freq_table;
  sfi_mtimer_table_t *mtmr_table;
  sfi_mrtc_table_t *mrtc_table;
  sfi_wake_table_t *wake_table;
  sfi_devs_table_t *devs_table;
  sfi_gpio_table_t *gpio_table;
} sfi_info_t;

CASSERT(sizeof(sfi_common_table_header_t) == SFI_TABLE_HEADER_LEN,
        sfi_common_table_header_size);

#define SFI_NUM_ENTRIES(_table, _entry_sz)                                    \
  (((_table)->common.length - SFI_TABLE_HEADER_LEN) / (_entry_sz))
#define SFI_NUM_SYST_ENTRIES(_table)                                          \
  SFI_NUM_ENTRIES(_table, sizeof(uint64))
#define SFI_NUM_CPUS_ENTRIES(_table)                                          \
  SFI_NUM_ENTRIES(_table, sizeof(uint32))
#define SFI_NUM_APIC_ENTRIES(_table)                                          \
  SFI_NUM_ENTRIES(_table, sizeof(uint64))
#define SFI_NUM_MMAP_ENTRIES(_table)                                          \
  SFI_NUM_ENTRIES(_table, sizeof(efi_memory_descriptor_t))
#define SFI_NUM_FREQ_ENTRIES(_table)                                          \
  SFI_NUM_ENTRIES(_table, sizeof(sfi_frequency_t))
#define SFI_NUM_MTMR_ENTRIES(_table)                                          \
  SFI_NUM_ENTRIES(_table, sizeof(sfi_mtimer_t))
#define SFI_NUM_MRTC_ENTRIES(_table)                                          \
  SFI_NUM_ENTRIES(_table, sizeof(sfi_mrtc_t))
#define SFI_NUM_DEVS_ENTRIES(_table)                                          \
  SFI_NUM_ENTRIES(_table, sizeof(sfi_device_t))
#define SFI_NUM_GPIO_ENTRIES(_table)                                          \
  SFI_NUM_ENTRIES(_table, sizeof(sfi_gpio_t))



int sfi_early_init(sfi_info_t *sfi_info);
int sfi_init(sfi_info_t *sfi_info);
void sfi_secondary_init(void);

#endif // _SFI_H_
