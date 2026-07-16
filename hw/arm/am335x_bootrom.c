/*
 * TI AM335x boot ROM, SD card file-system boot
 *
 * Copyright (C) 2026
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 */

/*
 * Emulation of the AM335x public boot ROM's MMC/SD "file system mode"
 * boot (TRM SPRUH73Q section 26.1.8.5, "MMC / SD Cards"): find a primary
 * FAT12/16/32 partition on the card, look up the booting file named "MLO"
 * in its root directory (26.1.8.5.6, "MMC/SD Read Sector Procedure in FAT
 * Mode"), and return its contents to the board code, which places the
 * contained SPL at its SRAM load address just as the ROM's image
 * downloader would.
 *
 * This is a from-documentation reimplementation of the behaviour the TRM
 * specifies (26.1.8.5.7, "FAT File system": MBR recognition, FAT boot
 * sector validation, root directory search, FAT cluster chain), not a
 * copy of TI's mask ROM binary. The card image is read through the QEMU
 * block layer at machine-init time rather than by bit-banging the MMCHS
 * controller; what is modelled faithfully is the algorithm -- which
 * partition the ROM picks, how the booting file is found, and how its
 * cluster chain is followed.
 *
 * Only SD card (MMC0) boot is modelled. The real ROM walks a
 * SYSBOOT-selected boot-device list (NAND, SPI, UART, USB, MMC0/1); none
 * of the other bootable peripherals exist on this machine, so SD boot is
 * hardcoded as the only supported boot mode.
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qemu/bswap.h"
#include "qemu/host-utils.h"
#include "qapi/error.h"
#include "system/block-backend.h"
#include "hw/arm/am335x_soc.h"

/* MBR partition-table LBAs are always in 512-byte units. */
#define MBR_SECTOR_SIZE     512
#define MBR_PART_TABLE      0x1BE
#define MBR_PART_ENTRY_LEN  16
#define BOOT_SIG_OFFSET     510     /* 55h AAh, both for MBR and FAT BPB */

/* FAT directory entries (TRM Table 26-24, "FAT Directory Entry"). */
#define FAT_DIRENT_LEN      32
#define FAT_ATTR_LFN        0x0F    /* long-filename metadata entry */
#define FAT_ATTR_VOLUME_ID  0x08
#define FAT_ATTR_DIRECTORY  0x10

/* The booting file's 8.3 name, "MLO", as an 11-byte packed short name. */
static const char fat_mlo_name[11] = "MLO        ";

typedef struct FatVolume {
    BlockBackend *blk;
    uint64_t part_off;      /* byte offset of the partition on the card */
    uint32_t bytes_per_sec;
    uint32_t sec_per_clus;
    uint32_t reserved_secs;
    uint32_t num_fats;
    uint32_t root_ent_cnt;  /* FAT12/16 only; 0 on FAT32 */
    uint32_t fat_secs;      /* sectors per FAT */
    uint32_t total_secs;
    uint32_t root_clus;     /* FAT32 only */
    uint32_t cluster_count; /* count of data clusters (determines FAT type) */
    uint32_t first_data_sec;
    int fat_type;           /* 12, 16 or 32 */
} FatVolume;

static bool bootrom_pread(BlockBackend *blk, uint64_t offset, void *buf,
                          size_t len, Error **errp)
{
    if (blk_pread(blk, offset, len, buf, 0) < 0) {
        error_setg(errp, "am335x-bootrom: failed to read %zu bytes at "
                   "offset %" PRIu64 " from the SD card", len, offset);
        return false;
    }
    return true;
}

/*
 * Validate a FAT12/16/32 boot sector / BPB (TRM 26.1.8.5.7.2 and Table
 * 26-23, "FAT Boot Sector") and fill in the volume geometry. Returns
 * false (without setting errp) if the sector is not a FAT boot sector.
 */
static bool fat_parse_boot_sector(FatVolume *v, const uint8_t *bs)
{
    uint32_t root_dir_secs, data_secs;

    if (lduw_le_p(bs + BOOT_SIG_OFFSET) != 0xAA55) {
        return false;
    }
    v->bytes_per_sec = lduw_le_p(bs + 0x0B);    /* BPB_BytsPerSec */
    v->sec_per_clus = bs[0x0D];                 /* BPB_SecPerClus */
    v->reserved_secs = lduw_le_p(bs + 0x0E);    /* BPB_RsvdSecCnt */
    v->num_fats = bs[0x10];                     /* BPB_NumFATs */
    v->root_ent_cnt = lduw_le_p(bs + 0x11);     /* BPB_RootEntCnt */
    v->total_secs = lduw_le_p(bs + 0x13);       /* BPB_TotSec16 */
    if (!v->total_secs) {
        v->total_secs = ldl_le_p(bs + 0x20);    /* BPB_TotSec32 */
    }
    v->fat_secs = lduw_le_p(bs + 0x16);         /* BPB_FATSz16 */
    if (!v->fat_secs) {
        v->fat_secs = ldl_le_p(bs + 0x24);      /* BPB_FATSz32 */
    }
    v->root_clus = ldl_le_p(bs + 0x2C);         /* BPB_RootClus (FAT32) */

    if ((v->bytes_per_sec != 512 && v->bytes_per_sec != 1024 &&
         v->bytes_per_sec != 2048 && v->bytes_per_sec != 4096) ||
        !v->sec_per_clus || !is_power_of_2(v->sec_per_clus) ||
        !v->reserved_secs || !v->num_fats || !v->fat_secs || !v->total_secs) {
        return false;
    }

    /*
     * The count of data clusters decides FAT12 vs FAT16 vs FAT32
     * (thresholds from Microsoft's FAT specification, which the TRM's FAT
     * description follows). The fixed root directory region only exists
     * on FAT12/16; on FAT32 the root directory is a normal cluster chain.
     */
    root_dir_secs = DIV_ROUND_UP(v->root_ent_cnt * FAT_DIRENT_LEN,
                                 v->bytes_per_sec);
    v->first_data_sec = v->reserved_secs + v->num_fats * v->fat_secs +
                        root_dir_secs;
    if (v->total_secs <= v->first_data_sec) {
        return false;
    }
    data_secs = v->total_secs - v->first_data_sec;
    v->cluster_count = data_secs / v->sec_per_clus;
    if (v->cluster_count < 4085) {
        v->fat_type = 12;
    } else if (v->cluster_count < 65525) {
        v->fat_type = 16;
    } else {
        v->fat_type = 32;
    }

    if (v->fat_type == 32 &&
        (v->root_ent_cnt || v->root_clus < 2 ||
         v->root_clus >= v->cluster_count + 2)) {
        return false;
    }
    if (v->fat_type != 32 && !v->root_ent_cnt) {
        return false;
    }
    return true;
}

/*
 * Find the FAT boot partition (TRM 26.1.8.5.7.1 and Figure 26-26, "MBR,
 * Get Partition"). The card is either "floppy-like" (the FAT filesystem
 * starts at sector 0, no MBR) or "hard-drive-like" (an MBR in sector 0
 * holds the partition table at 0x1BE, 4 entries of 16 bytes:
 * status/CHS-start/type/CHS-end/LBA-start/sector-count); both are
 * supported, as on the real ROM. Per the TRM the ROM wants exactly one
 * *active* (status 80h) primary FAT partition; we prefer the first active
 * FAT partition but fall back to the first inactive (status 00h) one so
 * images without the boot flag remain bootable.
 */
static bool fat_find_partition(FatVolume *v, Error **errp)
{
    uint8_t sec0[MBR_SECTOR_SIZE];
    int64_t fallback_lba = -1;
    int i;

    if (!bootrom_pread(v->blk, 0, sec0, sizeof(sec0), errp)) {
        return false;
    }

    /* Floppy-like: sector 0 is itself a valid FAT boot sector. */
    v->part_off = 0;
    if (fat_parse_boot_sector(v, sec0)) {
        return true;
    }

    if (lduw_le_p(sec0 + BOOT_SIG_OFFSET) != 0xAA55) {
        error_setg(errp, "am335x-bootrom: SD card has neither an MBR nor "
                   "a FAT boot sector in sector 0");
        return false;
    }

    for (i = 0; i < 4; i++) {
        const uint8_t *ent = sec0 + MBR_PART_TABLE + i * MBR_PART_ENTRY_LEN;
        uint8_t status = ent[0];
        uint8_t type = ent[4];
        uint32_t lba = ldl_le_p(ent + 8);
        uint32_t nsecs = ldl_le_p(ent + 12);

        switch (type) {
        case 0x01:  /* FAT12 */
        case 0x04:  /* FAT16 < 32M */
        case 0x06:  /* FAT16 */
        case 0x0B:  /* FAT32 (CHS) */
        case 0x0C:  /* FAT32 (LBA) */
        case 0x0E:  /* FAT16 (LBA) */
            break;
        default:
            continue;
        }
        if (!lba || !nsecs) {
            continue;
        }
        if (status == 0x80) {
            fallback_lba = lba;
            goto found;
        }
        if (status == 0x00 && fallback_lba < 0) {
            fallback_lba = lba;
        }
    }
    if (fallback_lba < 0) {
        error_setg(errp, "am335x-bootrom: no primary FAT12/16/32 partition "
                   "in the SD card's MBR");
        return false;
    }
found:
    v->part_off = fallback_lba * MBR_SECTOR_SIZE;
    if (!bootrom_pread(v->blk, v->part_off, sec0, sizeof(sec0), errp)) {
        return false;
    }
    if (!fat_parse_boot_sector(v, sec0)) {
        error_setg(errp, "am335x-bootrom: SD card partition at LBA %" PRId64
                   " has no valid FAT boot sector", fallback_lba);
        return false;
    }
    return true;
}

/*
 * Look up cluster's FAT entry (TRM Table 26-25, "FAT Entry Description":
 * 1.5 bytes on FAT12, 2 on FAT16, 4 -- 28 bits used -- on FAT32) and
 * return the next cluster in the chain, or 0 on end-of-chain / free /
 * bad-cluster markers.
 */
static bool fat_next_cluster(FatVolume *v, uint32_t clus, uint32_t *next,
                             Error **errp)
{
    uint64_t fat_off = v->part_off +
                       (uint64_t)v->reserved_secs * v->bytes_per_sec;
    uint8_t b[4];
    uint32_t val;

    switch (v->fat_type) {
    case 32:
        if (!bootrom_pread(v->blk, fat_off + (uint64_t)clus * 4, b, 4, errp)) {
            return false;
        }
        val = ldl_le_p(b) & 0x0FFFFFFF;
        val = (val >= 0x0FFFFFF7) ? 0 : val;
        break;
    case 16:
        if (!bootrom_pread(v->blk, fat_off + (uint64_t)clus * 2, b, 2, errp)) {
            return false;
        }
        val = lduw_le_p(b);
        val = (val >= 0xFFF7) ? 0 : val;
        break;
    default: /* FAT12: entry is 12 bits, packed 2 entries per 3 bytes */
        if (!bootrom_pread(v->blk, fat_off + clus + clus / 2, b, 2, errp)) {
            return false;
        }
        val = lduw_le_p(b);
        val = (clus & 1) ? (val >> 4) : (val & 0xFFF);
        val = (val >= 0xFF7) ? 0 : val;
        break;
    }
    *next = val;
    return true;
}

/* Byte offset of a data cluster on the card. */
static uint64_t fat_cluster_off(FatVolume *v, uint32_t clus)
{
    return v->part_off + ((uint64_t)v->first_data_sec +
                          (uint64_t)(clus - 2) * v->sec_per_clus) *
                         v->bytes_per_sec;
}

/*
 * Scan a buffer of 32-byte directory entries for the booting file's short
 * name. Long-filename metadata, volume labels, subdirectories and deleted
 * entries are skipped ("MLO" fits an 8.3 short name natively, so the
 * matching entry is always a plain short-name entry). Returns true when
 * the scan is over: either found (*start_clus and *file_size valid) or
 * the end-of-directory marker was hit.
 */
static bool fat_scan_dir_entries(FatVolume *v, const uint8_t *buf, size_t len,
                                 uint32_t *start_clus, uint32_t *file_size,
                                 bool *found)
{
    size_t off;

    for (off = 0; off + FAT_DIRENT_LEN <= len; off += FAT_DIRENT_LEN) {
        const uint8_t *ent = buf + off;
        uint8_t attr = ent[11];

        if (ent[0] == 0x00) {           /* end of directory */
            return true;
        }
        if (ent[0] == 0xE5 ||           /* deleted */
            attr == FAT_ATTR_LFN ||
            (attr & (FAT_ATTR_VOLUME_ID | FAT_ATTR_DIRECTORY))) {
            continue;
        }
        if (memcmp(ent, fat_mlo_name, sizeof(fat_mlo_name)) == 0) {
            /* FstClusHI is only meaningful on FAT32. */
            *start_clus = lduw_le_p(ent + 0x1A);
            if (v->fat_type == 32) {
                *start_clus |= (uint32_t)lduw_le_p(ent + 0x14) << 16;
            }
            *file_size = ldl_le_p(ent + 0x1C);
            *found = true;
            return true;
        }
    }
    return false;
}

/*
 * Find "MLO" in the root directory. On FAT12/16 the root directory is a
 * fixed region right after the FATs; on FAT32 it is a cluster chain
 * starting at BPB_RootClus.
 */
static bool fat_find_mlo(FatVolume *v, uint32_t *start_clus,
                         uint32_t *file_size, Error **errp)
{
    bool found = false;

    if (v->fat_type != 32) {
        size_t len = v->root_ent_cnt * FAT_DIRENT_LEN;
        g_autofree uint8_t *buf = g_malloc(len);
        uint64_t off = v->part_off +
                       (uint64_t)(v->reserved_secs +
                                  v->num_fats * v->fat_secs) *
                       v->bytes_per_sec;

        if (!bootrom_pread(v->blk, off, buf, len, errp)) {
            return false;
        }
        fat_scan_dir_entries(v, buf, len, start_clus, file_size, &found);
    } else {
        size_t clus_len = (size_t)v->sec_per_clus * v->bytes_per_sec;
        g_autofree uint8_t *buf = g_malloc(clus_len);
        uint32_t clus = v->root_clus;
        /* A sane boot partition's root directory holds < 64Ki entries. */
        uint32_t max_clus = DIV_ROUND_UP(65536 * FAT_DIRENT_LEN, clus_len);

        while (clus >= 2 && clus < v->cluster_count + 2 && max_clus--) {
            if (!bootrom_pread(v->blk, fat_cluster_off(v, clus), buf,
                               clus_len, errp)) {
                return false;
            }
            if (fat_scan_dir_entries(v, buf, clus_len, start_clus,
                                     file_size, &found)) {
                break;
            }
            if (!fat_next_cluster(v, clus, &clus, errp)) {
                return false;
            }
        }
    }

    if (!found) {
        error_setg(errp, "am335x-bootrom: no booting file \"MLO\" in the "
                   "root directory of the SD card's FAT partition");
        return false;
    }
    return true;
}

/*
 * Read the booting file "MLO" off the SD card the way the boot ROM's file
 * system mode does (see the file-head comment). Returns a g_malloc()ed
 * buffer holding the raw file contents (TI image headers included, as the
 * ROM sees them) and stores its size in *lenp, or NULL with errp set.
 */
uint8_t *am335x_bootrom_read_mlo(BlockBackend *blk, size_t *lenp,
                                 Error **errp)
{
    FatVolume v = { .blk = blk };
    g_autofree uint8_t *data = NULL;
    uint32_t start_clus, file_size, clus, nclus, i;
    size_t clus_len, pos;

    if (!fat_find_partition(&v, errp) ||
        !fat_find_mlo(&v, &start_clus, &file_size, errp)) {
        return NULL;
    }

    /* The SPL must fit in SRAM; anything bigger means a corrupt entry. */
    if (!file_size || file_size > 1 * MiB) {
        error_setg(errp, "am335x-bootrom: booting file \"MLO\" has an "
                   "implausible size (%u bytes)", file_size);
        return NULL;
    }

    /*
     * Follow the file's cluster chain (the ROM buffers this chain up
     * front as the "booting file map", TRM 26.1.8.5.6; reading as we walk
     * it is equivalent). The chain must supply exactly enough clusters to
     * cover the directory entry's file size.
     */
    clus_len = (size_t)v.sec_per_clus * v.bytes_per_sec;
    nclus = DIV_ROUND_UP(file_size, clus_len);
    data = g_malloc(file_size);
    clus = start_clus;
    pos = 0;
    for (i = 0; i < nclus; i++) {
        size_t chunk = MIN(clus_len, file_size - pos);

        if (clus < 2 || clus >= v.cluster_count + 2) {
            error_setg(errp, "am335x-bootrom: \"MLO\"'s FAT cluster chain "
                       "ends early (%u of %u clusters)", i, nclus);
            return NULL;
        }
        if (!bootrom_pread(v.blk, fat_cluster_off(&v, clus), data + pos,
                           chunk, errp)) {
            return NULL;
        }
        pos += chunk;
        if (!fat_next_cluster(&v, clus, &clus, errp)) {
            return NULL;
        }
    }

    *lenp = file_size;
    return g_steal_pointer(&data);
}
