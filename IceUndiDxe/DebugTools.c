/**************************************************************************

Copyright (c) 2018 - 2020, Intel Corporation. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of Intel Corporation nor the names of its contributors
      may be used to endorse or promote products derived from this software
      without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

***************************************************************************/

/** @file DebugTools.c Debug macros and utilities. */
#include <DebugTools.h>


#define dbg_printf(...) DEBUGDUMP(CRITICAL, (__VA_ARGS__))

typedef unsigned char uint8_t;

static inline int isprint ( int character ) {

	return ( ( character >= ' ' ) && ( character <= '~' ) );
}

static void dbg_hex_dump_da_row ( unsigned long dispaddr, const void *data,
				  unsigned long len, unsigned int offset ) {
	const uint8_t *bytes = data;
	unsigned int i;
	uint8_t byte;

	dbg_printf ( "%08lx :", ( dispaddr + offset ) );
	for ( i = offset ; i < ( offset + 16 ) ; i++ ) {
		if ( i >= len ) {
			dbg_printf ( "   " );
			continue;
		}
		dbg_printf ( "%c%02x",
			     ( ( ( i % 16 ) == 8 ) ? '-' : ' ' ), bytes[i] );
	}
	dbg_printf ( " : " );
	for ( i = offset ; i < ( offset + 16 ) ; i++ ) {
		if ( i >= len ) {
			dbg_printf ( " " );
			continue;
		}
		byte = bytes[i];
		dbg_printf ( "%c", ( isprint ( byte ) ? byte : '.' ) );
	}
	dbg_printf ( "\n" );
}

void dbg_hex_dump_da ( unsigned long dispaddr, const void *data,
		       unsigned long len ) {
	unsigned int offset;

	for ( offset = 0 ; offset < len ; offset += 16 ) {
		dbg_hex_dump_da_row ( dispaddr, data, len, offset );
	}
}
