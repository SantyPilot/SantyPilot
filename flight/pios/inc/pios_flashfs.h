/**
 ******************************************************************************
 * @file       pios_flashfs.h
 * @author     PhoenixPilot, http://github.com/PhoenixPilot, Copyright (C) 2012
 * @addtogroup PIOS PIOS Core hardware abstraction layer
 * @{
 * @addtogroup PIOS_FLASHFS Flash Filesystem API Definition
 * @{
 * @brief Flash Filesystem API Definition
 *****************************************************************************/
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#ifndef PIOS_FLASHFS_H
#define PIOS_FLASHFS_H

#include <stdint.h>

struct PIOS_FLASHFS_Stats {
    uint16_t num_free_slots; /* slots in free state */
    uint16_t num_active_slots; /* slots in active state */
};

// define logfs subdirectory of a yaffs flash device
#define PIOS_LOGFS_DIR "logfs"

int32_t PIOS_FLASHFS_Format(uintptr_t fs_id);
int32_t PIOS_FLASHFS_ObjSave(uintptr_t fs_id, uint32_t obj_id, uint16_t obj_inst_id, uint8_t *obj_data, uint16_t obj_size);
int32_t PIOS_FLASHFS_ObjLoad(uintptr_t fs_id, uint32_t obj_id, uint16_t obj_inst_id, uint8_t *obj_data, uint16_t obj_size);
int32_t PIOS_FLASHFS_ObjDelete(uintptr_t fs_id, uint32_t obj_id, uint16_t obj_inst_id);
int32_t PIOS_FLASHFS_GetStats(uintptr_t fs_id, struct PIOS_FLASHFS_Stats *stats);
#endif /* PIOS_FLASHFS_H */
