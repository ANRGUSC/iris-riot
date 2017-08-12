/**
 * Copyright (c) 2017, Autonomous Networks Research Group. All rights reserved.
 * Developed by:
 * Autonomous Networks Research Group (ANRG)
 * University of Southern California
 * http://anrg.usc.edu/
 *
 * Contributors:
 * Jason A. Tran
 * Richard Kim
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy 
 * of this software and associated documentation files (the "Software"), to deal
 * with the Software without restriction, including without limitation the 
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or 
 * sell copies of the Software, and to permit persons to whom the Software is 
 * furnished to do so, subject to the following conditions:
 * - Redistributions of source code must retain the above copyright notice, this
 *     list of conditions and the following disclaimers.
 * - Redistributions in binary form must reproduce the above copyright notice, 
 *     this list of conditions and the following disclaimers in the 
 *     documentation and/or other materials provided with the distribution.
 * - Neither the names of Autonomous Networks Research Group, nor University of 
 *     Southern California, nor the names of its contributors may be used to 
 *     endorse or promote products derived from this Software without specific 
 *     prior written permission.
 * - A citation to the Autonomous Networks Research Group must be included in 
 *     any publications benefiting from the use of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
 * CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS WITH 
 * THE SOFTWARE.
 */

#include <inttypes.h>
#include "anchor_node_table.h"
#include "debug.h"
#include <stdio.h>

typedef struct {
    uint8_t l2_addr[8];
    int node_id;
} anchor_node_t;

/* Manually assign the node IDs using this table, but keep the openmote order in 
   alphabetical order. */
static const anchor_node_t anchor_node_table[2] = {
    // { {0x36, 0x32, 0x48, 0x33, 0x46, 0xda, 0x9e, 0x72}, 1}, //openmote_a
    // { {0x36, 0x32, 0x48, 0x33, 0x46, 0xda, 0x9e, 0x72}, 2}, //openmote_b
    { {0x00, 0x12, 0x4b, 0x00, 0x06, 0x13, 0x06, 0x22}, 0}, //openmote_c
    // { {0x00, 0x12, 0x4b, 0x00, 0x04, 0x33, 0xed, 0x19}, 1}  //openmote_d - 7f:ed
    // { {0x00, 0x12, 0x4b, 0x00, 0x04, 0x33, 0xed, 0x02}, 1},  //openmote_e
    // { {0x00, 0x12, 0x4b, 0x00, 0x04, 0x33, 0xed, 0x4c}, 2}  //openmote_f - 67:bd
    /* extend as needed */
};

int anchor_id_lookup(uint8_t *l2_addr, unsigned int l2_addr_len)
{
    if (l2_addr_len != 8) {
        DEBUG("Invalid l2 addr length\n");
        return -1;
    }

    for (int i = 0; i < sizeof(anchor_node_table); i++) {
        if (memcmp(anchor_node_table[i].l2_addr, l2_addr, 8) == 0) {
            /* return node ID */
            return anchor_node_table[i].node_id;
        }
    }

    // for (int i = 0; i < sizeof(temp_table); i++) {
    //     if (memcmp(temp_table[i].l2_addr, l2_addr, 8) == 0) {
    //         /* return node ID */
    //         return temp_table[i].node_id;
    //     }
    // }

    // for (int i = 0; i < sizeof(temp_table); i++) {
    //     if (memcmp(temp_table[i].l2_addr, l2_addr, 8) == 0) {
    //         /* return node ID */
    //         return temp_table[i].node_id;
    //     }
    // }

    /* error: entry does not exist */
    return -1;
}

// // assume l2_addr's length is 8
// void update_table(uint8_t *l2_addr)
// {
//     anchor_node_t new_node;
//     memcpy(new_node.l2_addr, l2_addr, sizeof(l2_addr));
//     new_node.node_id = table_counter + TABLE_SIZE;
//     memcpy(temp_table[table_counter], new_node, sizeof(anchor_node_t));
//     table_counter++;
// }

