/*
 * Authors: The code is authored by Carl Friess in the work
 * Friess, Carl, et al. "Fully Onboard SLAM for Distributed Mapping with a Swarm
 * of Nano-Drones." arXiv preprint arXiv:2309.03678 (2023).
 * https://arxiv.org/pdf/2309.03678.pdf
 */

#include <assert.h>
#include <string.h>

#include "crtp.h"

#define MIN(a, b) ((a) < (b) ? (a) : (b))

void send_burst_data(uint8_t type, void *data, size_t len) {
    assert(len < 0xff * 28);
    for (size_t i = 0, offset = 0; offset < len; i++, offset += 28) {
        CRTPPacket pk = {
            .header = CRTP_HEADER(1, 0),
            .size = MIN(len - offset, 28) + 2,
        };
        pk.data[0] = (type & 0x7f) | (offset + 28 >= len ? 0x80 : 0);
        pk.data[1] = i;
        memcpy(&(pk.data[2]), data + offset, pk.size - 2);
        crtpSendPacketBlock(&pk);
    }
}
