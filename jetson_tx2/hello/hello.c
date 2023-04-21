/*
 * Copyright 2021, Breakaway Consulting Pty. Ltd.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */
#include <stdint.h>
#include <sel4cp.h>

#include <stdio.h>
#include "smmu.h"

/* Memory regions. These all have to be here to keep compiler happy */
uintptr_t smmu_base;

#define NOPS ""


/*supported stages of translations*/
#define STAGE1_TRANS           (1 << 0)
#define STAGE2_TRANS           (1 << 1)
#define NESTED_TRANS           (1 << 2)
/*supported translation table formats*/
#define AARCH32S_FMT           (1 << 0)
#define AARCH32L_FMT           (1 << 1)
#define NO_AARCH32_FMT         (1 << 2)
#define TRANS_PAGES_4KB        (1 << 3)
#define TRANS_PAGES_16KB       (1 << 4)
#define TRANS_PAGES_64KB       (1 << 5)

/*the default vritual address bits for partition TTBR0 and TTBR1*/
#define SMMU_VA_DEFAULT_BITS      48

struct  smmu_feature {
    bool stream_match;              /*stream match register funtionality included*/
    bool trans_op;                  /*address translation operations supported*/
    bool cotable_walk;              /*coherent translation table walk*/
    bool broadcast_tlb;             /*broadcast TLB maintenance*/
    bool vmid16;                    /*16 bits VMIDs are supported*/
    uint32_t supported_trans;         /*supported translation stages*/
    uint32_t supported_fmt;           /*supported translation formats*/
    uint32_t num_cfault_ints;         /*supported number of context fault interrupts*/
    uint32_t num_stream_ids;          /*number of stream IDs*/
    uint32_t num_stream_map_groups;   /*num stream mapping register groups*/
    uint32_t smmu_page_size;          /*page size in SMMU register address space*/
    uint32_t smmu_num_pages;          /*number of pages in global or context bank address space*/
    uint32_t num_s2_cbanks;           /*cbanks that support stage 2 only*/
    uint32_t num_cbanks;              /*total number of context banks*/
    uint32_t va_bits;                 /*upstream address size*/
    uint32_t pa_bits;                 /*PA address size*/
    uint32_t ipa_bits;                /*IPA address size*/
    unsigned long cb_base;                   /*base of context bank address space*/
};

static struct smmu_feature smmu_dev_knowledge;

static inline uint32_t smmu_read_reg32(uintptr_t base, uint32_t index)
{
    return *(volatile uint32_t *)(base + index);
}

static inline void smmu_write_reg32(uintptr_t base, uint32_t index, uint32_t val)
{
    *(volatile uint32_t *)(base + index) = val;
}

static inline uint64_t smmu_read_reg64(uintptr_t base, uint32_t index)
{
    return *(volatile uint64_t *)(base + index);
}

static inline void smmu_write_reg64(unsigned long base, uint32_t index, uint64_t val)
{
    *(volatile uint64_t *)(base + index) = val;
}



static void smmu_config_prob(uintptr_t smmu_grs0)
{
    uint32_t reg, field;
    sel4cp_dbg_puts("In here\n");

    reg = smmu_read_reg32(smmu_grs0, SMMU_IDR0);

    /*ID0*/
    // sel4cp_dbg_puts("==> smmu_grs0: %lx\n", smmu_grs0);
    // sel4cp_dbg_puts("==> IDR0: %x\n", reg);
    /*stages supported*/
    if (reg & IDR0_S1TS) {
        sel4cp_dbg_puts("==> Supports stage 1 translation\n");
        smmu_dev_knowledge.supported_trans |= STAGE1_TRANS;
    }
    if (reg & IDR0_S2TS) {
        sel4cp_dbg_puts("==> Supports stage 2 translation\n");
        smmu_dev_knowledge.supported_trans |= STAGE2_TRANS;
    }
    if (reg & IDR0_NTS) {
        sel4cp_dbg_puts("==> Supports nested translation\n");
        smmu_dev_knowledge.supported_trans |= NESTED_TRANS;
    }
    /*stream matching register*/
    if (reg & IDR0_SMS) {
        smmu_dev_knowledge.stream_match = true;
    }
    /*address translation operation*/
    if ((reg & IDR0_ATOSNS) == 0) {
        sel4cp_dbg_puts("kernel| SMMU supports address translation operations\n");
        smmu_dev_knowledge.trans_op = true;
    } else {
        sel4cp_dbg_puts("kernel| SMMU doesn't support address translation operations\n");
    }
    /*AARCH32 translation format support*/
    field = IDR0_PTFS_VAL(reg & IDR0_PTFS);
    if (field == PTFS_AARCH32S_AARCH32L) {
        smmu_dev_knowledge.supported_fmt |= AARCH32L_FMT;
        smmu_dev_knowledge.supported_fmt |= AARCH32S_FMT;
    } else if (field == PTFS_AARCH32L_ONLY) {
        smmu_dev_knowledge.supported_fmt |= AARCH32L_FMT;
    } else {
        smmu_dev_knowledge.supported_fmt |= NO_AARCH32_FMT;
    }
    /*number of context fault intrrupts
    * However, in smmuv2, each context bank has dedicated interrupt pin
    * hence no requirement to specify implemented interrupts here.*/
    smmu_dev_knowledge.num_cfault_ints = IDR0_NUMIRPT_VAL(reg & IDR0_NUMIRPT);
    /*coherent translation table walk*/
    if (reg & IDR0_CTTW) {
        smmu_dev_knowledge.cotable_walk = true;
    }
    /*broadcast TLB maintenance*/
    if (reg & IDR0_BTM) {
        smmu_dev_knowledge.broadcast_tlb = true;
    }
    /*number of stream IDs*/
    smmu_dev_knowledge.num_stream_ids = (1 << IDR0_NUMSIDB_VAL(reg & IDR0_NUMSIDB)) - 1;
    /*number of stream mapping register groups*/
    smmu_dev_knowledge.num_stream_map_groups = reg & IDR0_NUMSMRG;

    /*ID1*/
    reg = smmu_read_reg32(smmu_grs0, SMMU_IDR1);
    // sel4cp_dbg_puts("==> IDR1: %x\n", reg);

    /*smmu page size*/
    if (reg & IDR1_PAGESIZE) {
        smmu_dev_knowledge.smmu_page_size = SMMU_PAGE_64KB;
    } else {
        smmu_dev_knowledge.smmu_page_size = SMMU_PAGE_4KB;
    }
    /*smmu num pages, 2^(numdxb + 1)*/
    field = IDR1_NUMPAGENDXB_VAL(reg & IDR1_NUMPAGENDXB);
    smmu_dev_knowledge.smmu_num_pages = 1 << (field + 1);
    /*num of stage 2 context banks*/
    smmu_dev_knowledge.num_s2_cbanks = IDR1_NUMS2CB_VAL(reg & IDR1_NUMS2CB);
    /*total num of context banks*/
    smmu_dev_knowledge.num_cbanks = reg & IDR1_NUMCB;
    /*calcuate the context bank base*/
    smmu_dev_knowledge.cb_base = SMMU_CB_BASE_PADDR(
                                     SMMU_GLOBAL_SIZE(smmu_dev_knowledge.smmu_num_pages, smmu_dev_knowledge.smmu_page_size));

    /*ID2*/
    reg = smmu_read_reg32(smmu_grs0, SMMU_IDR2);
    // sel4cp_dbg_puts("==> IDR2: %x\n", reg);

    /*VNID16S*/
    if (reg & IDR2_VMID16S) {
        smmu_dev_knowledge.vmid16 = true;
    }
    /*PTFSV8_64KB*/
    if (reg & IDR2_PTFSV8_64) {
        smmu_dev_knowledge.supported_fmt |= TRANS_PAGES_64KB;
    }
    /*PTFSV8_16KB*/
    if (reg & IDR2_PTFSV8_16) {
        smmu_dev_knowledge.supported_fmt |= TRANS_PAGES_16KB;
    }
    /*PTFSV8_64KB*/

    if (reg & IDR2_PTFSV8_4) {
        smmu_dev_knowledge.supported_fmt |= TRANS_PAGES_4KB;
    }
    // /*UBS virtual address size*/
    // smmu_dev_knowledge.va_bits = smmu_ubs_size_to_bits(IDR2_UBS_VAL(reg & IDR2_UBS));
    // /*OAS*/
    // smmu_dev_knowledge.pa_bits = smmu_obs_size_to_bits(IDR2_OAS_VAL(reg & IDR2_OAS));
    // /*IAS*/
    // smmu_dev_knowledge.ipa_bits = smmu_obs_size_to_bits(reg & IDR2_IAS);
}

void
init(void)
{
    sel4cp_dbg_puts("hello, world\n");
    smmu_config_prob(smmu_base);
}

void
notified(sel4cp_channel ch)
{
}