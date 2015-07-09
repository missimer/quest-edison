# Kernel configuration

# Compiler optimization level
OPT = 0

# Disable SMP
# CFG += -DNO_SMP

# Disable "logger" thread
# CFG += -DNO_LOGGER

# Disable ACPI support
# CFG += -DNO_ACPI

# Disable Intel Multiprocessor Specification parsing
# CFG += -DNO_INTEL_MPS

# Disable Simple Firmware Interface support
# CFG += -DNO_SFI

# Use VMX-based virtual machines for isolation
# CFG += -DUSE_VMX

# Use the memory mapped uart (used by the edison)
# CFG += -DSERIAL_MMIO32

# The physical address for serial_mmio32, specify if you want to boot print
# messages.  After PCI enumeration the mmio32 serial device is detected and
# initialized using the PCI information.
#CFG += -DMMIO32_MEMBASE=0xff010180

# Intel Mobile Internet Device (Not PC compatible) MID
# CFG += -DINTEL_MID
