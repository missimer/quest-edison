#include "smp/sfi.h"
#include "arch/i386.h"
#include "kernel.h"
#include "mem/mem.h"
#include "smp/apic.h"
#include "util/cpuid.h"
#include <util/printf.h>

#define DEBUG_SFI
#define DEBUG_SFI_VERBOSE

#ifdef DEBUG_SFI
#define DLOG(fmt,...) DLOG_PREFIX("SFI",fmt,##__VA_ARGS__)
#define DLOG_NO_PRE(fmt,...) _DLOG_PREFIX(fmt,##__VA_ARGS__)
#else
#define DLOG(fmt,...) ;
#define DLOG_NO_PRE(fmt,...) ;
#endif

#ifdef DEBUG_SFI_VERBOSE
#define DLOGV(fmt,...) DLOG_PREFIX("SFI",fmt,##__VA_ARGS__)
#define DLOGV_NO_PRE(fmt,...) _DLOG_PREFIX(fmt,##__VA_ARGS__)
#else
#define DLOGV(fmt,...) ;
#define DLOGV_NO_PRE(fmt,...) ;
#endif

static phys_addr_t
sfi_syst_phys(void)
{
  phys_addr_t phys = SFI_SYST_SEARCH_BEGIN;

  do {
    char *virt = map_virtual_page((phys & 0xFFFFF000) | 3);

    if(virt == NULL) {
      panic("Failed to map virtual page in sfi_present");
    }
    int i;
    for(i = 0; i < 0x1000; i+=16) {
      if(strncmp(&virt[i], SFI_SIG_SYST, SFI_SIG_SIZE) == 0) {
        unmap_virtual_page(virt);
        return phys + i;
      }
    }
    unmap_virtual_page(virt);
    phys += 0x1000;

  } while(phys < SFI_SYST_SEARCH_END);

  return -1;
}

#define SFI_PHYS_TO_VIRT(sfi_info, phys) \
  (((char*)(sfi_info)->start_addr) + ((phys) - (sfi_info)->low_frame));

static void*
get_sfi_table(sfi_info_t *sfi_info, sfi_signature sig)
{
  size_t i;
  size_t num_tables = SFI_NUM_SYST_ENTRIES(sfi_info->system_table);

  for(i = 0; i < num_tables; i++) {
    sfi_common_table_header_t *table = (sfi_common_table_header_t*)
      SFI_PHYS_TO_VIRT(sfi_info, sfi_info->system_table->entries[i]);
    if(strncmp(sig, table->signature, SFI_SIG_SIZE) == 0) {
      return table;
    }
  }
  return NULL;
}

static void
print_syst_table(sfi_info_t *sfi_info)
{
  size_t i;
  DLOGV("Number of SFI table entries: %u", SFI_NUM_SYST_ENTRIES(sfi_info->system_table));
  for(i = 0; i < SFI_NUM_SYST_ENTRIES(sfi_info->system_table); i++) {
#ifdef DEBUG_SFI_VERBOSE
    sfi_common_table_header_t *table = (sfi_common_table_header_t*)
      SFI_PHYS_TO_VIRT(sfi_info, sfi_info->system_table->entries[i]);
#endif
     DLOGV("syst_table[%u] 0x%X %c%c%c%c", sfi_info->system_table->entries[i],
          table->signature[0], table->signature[1], table->signature[2],
          table->signature[3]);
  }
}

static void
print_cpus_table(sfi_info_t *sfi_info)
{
  if(sfi_info->cpus_table) {
    size_t i;
    DLOGV("Number of CPU table entries: %u",
         SFI_NUM_CPUS_ENTRIES(sfi_info->cpus_table));

    for(i = 0; i < SFI_NUM_CPUS_ENTRIES(sfi_info->cpus_table); i++) {
      DLOGV("sfi_info->cpus_table->lapic_ids[%d] = %u",
           i, sfi_info->cpus_table->lapic_ids[i]);
    }
  }
  else {
    DLOGV("No CPUS Table");
  }
}

static void
print_apic_table(sfi_info_t *sfi_info)
{
  if(sfi_info->cpus_table) {
    size_t i;
    DLOGV("Number of APIC table entries: %u",
         SFI_NUM_APIC_ENTRIES(sfi_info->cpus_table));

    for(i = 0; i < SFI_NUM_APIC_ENTRIES(sfi_info->cpus_table); i++) {
      DLOGV("ioapic_phys = 0x%X", (uint32)sfi_info->apic_table->ioapic_phys[i]);
    }
  }
  else {
    DLOGV("No APIC Table");
  }
}


static void
print_mmap_table(sfi_info_t *sfi_info)
{
  if(sfi_info->mmap_table) {
    size_t i;
    DLOGV("Number of MMAP table entries: %u",
         SFI_NUM_MMAP_ENTRIES(sfi_info->mmap_table));
    DLOGV("(type, physical_start, virtual_start, num_of_pages, attribute)");
    for(i = 0; i < SFI_NUM_MMAP_ENTRIES(sfi_info->mmap_table); i++) {
      DLOGV("(%u, 0x%X, 0x%X, 0x%X, 0x%X)",
           (uint32)sfi_info->mmap_table->memory_descriptors[i].type,
           (uint32)sfi_info->mmap_table->memory_descriptors[i].physical_start,
           (uint32)sfi_info->mmap_table->memory_descriptors[i].virtual_start,
           (uint32)sfi_info->mmap_table->memory_descriptors[i].num_of_pages,
           (uint32)sfi_info->mmap_table->memory_descriptors[i].attribute);
    }
  }
  else {
    DLOGV("No Memory Map Table");
  }

}

static void
print_freq_table(sfi_info_t *sfi_info)
{
  if(sfi_info->freq_table) {
    size_t i;
    DLOGV("Number of Frequency table entries: %u",
         SFI_NUM_FREQ_ENTRIES(sfi_info->freq_table));
    DLOGV("(frequency, transition_latency, perf_ctl_val)");
    for(i = 0; i < SFI_NUM_FREQ_ENTRIES(sfi_info->freq_table); i++) {
      DLOGV("(0x%X, 0x%X, 0x%X)",
           sfi_info->freq_table->frequencies[i].frequency,
           sfi_info->freq_table->frequencies[i].transition_latency,
           sfi_info->freq_table->frequencies[i].perf_ctl_val);
    }
  }
  else {
    DLOGV("No Frequency Table");
  }
}

static void
print_mtmr_table(sfi_info_t *sfi_info)
{
  if(sfi_info->mtmr_table) {
    size_t i;
    DLOGV("Number of M-Timer table entries: %u",
         SFI_NUM_MTMR_ENTRIES(sfi_info->mtmr_table));
    for(i = 0; i < SFI_NUM_MTMR_ENTRIES(sfi_info->mtmr_table); i++) {
      DLOGV("phys_addr = 0x%X\nfrequency = 0x%X\nirq = 0x%X",
           sfi_info->mtmr_table->timers[i].phys_addr,
           sfi_info->mtmr_table->timers[i].frequency,
           sfi_info->mtmr_table->timers[i].irq);
    }
  }
  else {
    DLOGV("No M-Timer Table");
  }
}

static void
print_mrtc_table(sfi_info_t *sfi_info)
{
  if(sfi_info->mrtc_table) {
    size_t i;
    DLOGV("Number of M-Real Time Clock table entries: %u",
         SFI_NUM_MRTC_ENTRIES(sfi_info->mrtc_table));
    for(i = 0; i < SFI_NUM_MRTC_ENTRIES(sfi_info->mtmr_table); i++) {
      DLOGV("phys_addr = 0x%X\nirq = 0x%X",
           sfi_info->mtmr_table->timers[i].phys_addr,
           sfi_info->mtmr_table->timers[i].irq);
    }
  }
  else {
    DLOGV("No M-Real Time Clock Table");
  }
}

static void
print_wake_table(sfi_info_t *sfi_info)
{
  if(sfi_info->wake_table) {
    DLOGV("Wake routine vector phys addr = 0x%X",
         (uint32)sfi_info->wake_table->phys_addr);
  }
  else {
    DLOGV("No Wake Table");
  }
}


static void
print_non_null_string_name(char* str, size_t len)
{
  size_t i;
  for(i = 0; i < len; i++) {
    if(str[i]) {
      DLOGV_NO_PRE("%c", str[i]);
    }
    else {
      break;
    }
  }
}

static void
print_gpio_table(sfi_info_t *sfi_info)
{
  if(sfi_info->gpio_table) {
    size_t i;
    DLOGV("Number of GPIO table entries: %u",
         SFI_NUM_GPIO_ENTRIES(sfi_info->gpio_table));
    DLOGV("(controller_name, pin_number, pin_name)");
    for(i = 0; i < SFI_NUM_GPIO_ENTRIES(sfi_info->gpio_table); i++) {
      DLOGV_NO_PRE("SFI: (");
      print_non_null_string_name(sfi_info->gpio_table->gpios[i].controller_name,
                                 sizeof(sfi_info->gpio_table->gpios[i].controller_name));

      DLOGV_NO_PRE(", 0x%X, ", (uint32)sfi_info->gpio_table->gpios[i].pin_number);
      print_non_null_string_name(sfi_info->gpio_table->gpios[i].pin_name,
                                 sizeof(sfi_info->gpio_table->gpios[i].pin_name));
      DLOGV_NO_PRE(")\n");
    }
  }
  else {
    DLOGV("No GPIO Table");
  }
}

static void
print_devs_table(sfi_info_t *sfi_info)
{
  if(sfi_info->devs_table) {
    size_t i;
    DLOGV("Number of device table entries: %u",
         SFI_NUM_DEVS_ENTRIES(sfi_info->devs_table));
    DLOGV("(host_type, host_number, address, irq, max_frequency, name)");
    for(i = 0; i < SFI_NUM_DEVS_ENTRIES(sfi_info->devs_table); i++) {
      DLOGV_NO_PRE("SFI: (0x%X, 0x%X, 0x%X, 0x%X, 0x%X, ",
           (uint32)sfi_info->devs_table->devices[i].host_type,
           (uint32)sfi_info->devs_table->devices[i].host_number,
           (uint32)sfi_info->devs_table->devices[i].address,
           (uint32)sfi_info->devs_table->devices[i].irq,
           (uint32)sfi_info->devs_table->devices[i].max_frequency);
      print_non_null_string_name(sfi_info->devs_table->devices[i].name,
                                 sizeof(sfi_info->devs_table->devices[i].name));
      DLOGV_NO_PRE(")\n");
    }
  }
  else {
    DLOGV("No Device Table");
  }
}

int
sfi_early_init(sfi_info_t *sfi_info)
{
  int i;
  size_t syst_num;
  phys_addr_t low_frame;
  phys_addr_t high_frame;
  phys_addr_t syst_phys;

  memset(sfi_info, 0, sizeof(*sfi_info));

  syst_phys = sfi_syst_phys();
  if(syst_phys == -1) {
    return 0;
  }

  low_frame = high_frame = syst_phys;

  sfi_info->system_table = (sfi_system_table_t*)
    (((char*)map_virtual_page((syst_phys & 0xFFFFF000) | 3))
    + (syst_phys & 0xFFF));

  syst_num = SFI_NUM_SYST_ENTRIES(sfi_info->system_table);

  for(i = 0; i < syst_num; i++) {
    if(sfi_info->system_table->entries[i] > 0xFFFFFFFF) {
      panic("SFI table above 4G region");
    }
    if(sfi_info->system_table->entries[i] < low_frame) {
      low_frame = sfi_info->system_table->entries[i];
    }
    if(sfi_info->system_table->entries[i] > high_frame) {
      high_frame = sfi_info->system_table->entries[i];
    }
  }

  sfi_info->low_frame = low_frame = low_frame & 0xFFFFF000;
  high_frame &= 0xFFFFF000;

  unmap_virtual_page(sfi_info->system_table);

  sfi_info->num_frames = ((high_frame - low_frame) / 0x1000) + 1;
  sfi_info->start_addr =
    map_contiguous_virtual_pages(low_frame | 3,
                                 sfi_info->num_frames);

  if(sfi_info->start_addr == NULL) {
    panic("Failed to map STI");
  }

  sfi_info->system_table = (sfi_system_table_t*)
    SFI_PHYS_TO_VIRT(sfi_info, syst_phys);

  print_syst_table(sfi_info);

  sfi_info->cpus_table = get_sfi_table(sfi_info, SFI_SIG_CPUS);
  print_cpus_table(sfi_info);

  sfi_info->apic_table = get_sfi_table(sfi_info, SFI_SIG_APIC);
  print_apic_table(sfi_info);

  sfi_info->mmap_table = get_sfi_table(sfi_info, SFI_SIG_MMAP);
  print_mmap_table(sfi_info);

  sfi_info->freq_table = get_sfi_table(sfi_info, SFI_SIG_FREQ);
  print_freq_table(sfi_info);

  sfi_info->mtmr_table = get_sfi_table(sfi_info, SFI_SIG_MTMR);
  print_mtmr_table(sfi_info);

  sfi_info->mrtc_table = get_sfi_table(sfi_info, SFI_SIG_MRTC);
  print_mrtc_table(sfi_info);

  sfi_info->wake_table = get_sfi_table(sfi_info, SFI_SIG_WAKE);
  print_wake_table(sfi_info);

  sfi_info->devs_table = get_sfi_table(sfi_info, SFI_SIG_DEVS);
  print_devs_table(sfi_info);

  sfi_info->gpio_table = get_sfi_table(sfi_info, SFI_SIG_GPIO);
  print_gpio_table(sfi_info);

  return 0;
}

static void
sfi_ap_init(sfi_info_t *sfi_info)
{
#ifndef NO_SMP
  u32 ebx;
  size_t i;
  cpuid (1, 0, NULL, &ebx, NULL, NULL);
  uint8 this_apic_id = ebx >> 24;

  for(i = 0; i < SFI_NUM_CPUS_ENTRIES(sfi_info->cpus_table); i++) {

    uint8 apic_id = sfi_info->cpus_table->lapic_ids[i];

    if (this_apic_id == apic_id) {
      com1_printf("Skipping apic_id = %u\n", (size_t)apic_id);
      continue;
    }

    if (smp_boot_cpu (apic_id, APIC_VER_NEW)) {
      CPU_to_APIC[mp_num_cpus] = apic_id;
      APIC_to_CPU[apic_id] = mp_num_cpus;
      mp_num_cpus++;
    }
    else {
      com1_printf("Failed to initialize AP core\n");
      panic("Failed to initialize AP core\n");
    }
  }
#else
#endif
}

int sfi_init(sfi_info_t *sfi_info)
{
  extern uint32 mp_LAPIC_addr;
  extern uint32 mp_num_IOAPICs;
  extern uint32 mp_IOAPIC_addr;
  extern mp_IOAPIC_info mp_IOAPICs[];
  extern uint32 mp_num_overrides;

  mp_ISA_bus_id = 0;

  mp_num_IOAPICs = SFI_NUM_APIC_ENTRIES(sfi_info->apic_table);

  if(mp_num_IOAPICs != 1) {
    panic("More than one IO APIC, don't know how to handle this");
  }

  mp_IOAPIC_addr = sfi_info->apic_table->ioapic_phys[0];

  /*
   * -- EM -- Hardcoding some of these values for the Intel Edison
   */
  mp_IOAPICs[0].id = 0;
  mp_IOAPICs[0].address = mp_IOAPIC_addr;
  mp_IOAPICs[0].startGSI = 0;
  mp_IOAPICs[0].numGSIs = 0x34;

  DLOG("mp_LAPIC_addr = 0x%X", mp_LAPIC_addr);
  DLOG("mp_num_IOAPICs = 0x%X", mp_num_IOAPICs);
  DLOG("mp_IOAPIC_addr = 0x%X", mp_IOAPIC_addr);

  mp_num_overrides = 0;

  sfi_ap_init(sfi_info);

  return 0;
}
