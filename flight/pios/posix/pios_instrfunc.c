/**
 ******************************************************************************
 * @addtogroup PIOS PIOS Core hardware abstraction layer
 * @{
 * @addtogroup PIOS_INSTRFUNC layer functions
 * @brief Hardware communication layer
 * @{
 *
 * @file       pios_instrfunc.c
 * @author     The SantyPilot Team, http://www.openpilot.org Copyright (C) 2010.
 *             Parts by Thorsten Klose (tk@midibox.org)
 * @brief      trace func functions
 * @see        The GNU Public License (GPL) Version 3
 *
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

#include <stdio.h>
#include <pios.h>

#define DUMP(func, call) printf("%s: func = %p, called by = %p\n", __FUNCTION__, func, call)

void __attribute__((no_instrument_function)) __cyg_profile_func_enter(void *this_func, void *call_site) /*DUMP(this_func, call_site);*/
{}
void __attribute__((no_instrument_function)) __cyg_profile_func_exit(void *this_func, void *call_site) /*DUMP(this_func, call_site);*/
{}


#ifdef __MINGW32__
/* Workaround windows freertos
 * simulator api */
void vPortYieldFromISR(void)
{
    portYIELD_WITHIN_API();
}
#endif
