# User space SMMU driver

- experimenting with moving the SMMU from seL4 kernel to user space on sel4cp
- sel4cp-sdk is compiled with KernelArmSMMU turned off 
    - set KernelArmSMMUset to false in build_sdk.py, delete set(KernelArmSMMU ON) from seL4/src/plat/tx2/config.cmake, and remove &{/iommu@12000000} from overlay-tx2.dts
    - built from https://github.com/Ivan-Velickovic/sel4cp/commit/a682a26432ed25d0258e32261b3bf84ff91742a1
- currently only tested for tx2

## Building the system

- in the jetson_tx2/hello folder

```sh
make BUILD_DIR=./build SEL4CP_SDK=../../sel4cp-sdk-1.2.6 SEL4CP_BOARD=jetson_tx2 SEL4CP_CONFIG=debug
```

