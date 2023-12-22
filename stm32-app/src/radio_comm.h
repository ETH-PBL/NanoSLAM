/*
 * Authors: The code is authored by Carl Friess in the work
 * Friess, Carl, et al. "Fully Onboard SLAM for Distributed Mapping with a Swarm
 * of Nano-Drones." arXiv preprint arXiv:2309.03678 (2023).
 * https://arxiv.org/pdf/2309.03678.pdf
 */

#ifndef RADIO_COMM_H
#define RADIO_COMM_H

void send_burst_data(uint8_t type, void *data, size_t len);

#endif