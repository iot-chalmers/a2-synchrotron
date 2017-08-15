# A2-Synchrotron

# Network-wide Consensus Utilizing the Capture Effect in Low-power Wireless Networks

## Intro

This repo hosts the source code of the A2 and Synchrotron protocols that we published in the SenSys 2017 conference.

Beshr Al Nahas, Simon Duquennoy and Olaf Landsiedel. 2017. 
"Network-wide Consensus Utilizing the Capture Effect in Low-power Wireless Networks". 
_In Proceedings of the Conference on Embedded Networked Sensor Systems (ACM SenSys)_.

## Abstract

In low-power wireless networking, new applications such as cooperative robots or industrial closed-loop control demand for network-wide agreement at low-latency and high reliability.
Distributed consensus protocols is a mature field of research in a wired context, but has received little attention in low-power wireless settings.
In this paper, we present A2: Agreement in the Air, a system that brings distributed consensus to low-power multi-hop networks.
We introduce Synchrotron, a synchronous transmissions kernel that builds a robust mesh by exploiting the capture effect, frequency hopping with parallel channels, and link-layer security.
We build A2 on top of this reliable base layer, and enable the two- and three-phase commit protocols, as well as network services such as joining, hopping sequence distribution and re-keying.

We evaluate A2 on four public testbeds with different deployment densities and sizes. 
A2 requires only 475 ms to complete a two-phase commit over 180 nodes.
The resulting duty cycle is 0.5 percent for 1-minute intervals.
We show that A2 achieves zero losses end-to-end over long experiments, representing millions of data points.

When further adding controlled failures, we show that two-phase commit ensures transaction consistency in A2 while three-phase commit provides liveness at the expense of inconsistency under specific failure scenarios.

## Implementation

We implement A2 in C for the Contiki OS targeting simple wireless nodes equipped with a low-power radio such as TelosB and Wsn430 platforms which feature a 16bit MSP430 CPU @ 4 MHz, 10 kB of RAM, 48 kB of firmware storage and CC2420 radio compatible with IEEE 802.15.4.

You can browse the main part of the implementation under [core/net/mac/chaos](./a2-synchrotron-contiki/core/net/mac/chaos/).

### Running in Cooja

To run the two-phase commit sample app in Cooja:
Go to [2PC](./a2-synchrotron-contiki/apps/chaos/2pc) and compile using:
```
make clean && make cooja log=0 printf=1 tx=31 mch=1 pch=0 sec=0 src=2 sync=0 failure=0 dynamic=1 initiator=3 interval=29 max_node_count=32
```
Then, start Cooja and choose one of the simulation files. 
For example, this one has [32 nodes](./a2-synchrotron-contiki/apps/chaos/2pc/2pc-app-32nodes.csc).

You can find the other sample applications in the [Apps folder](./a2-synchrotron-contiki/apps/chaos).

