ESP32 Wi-Fi reference model provenance
======================================

The initial original-ESP32 MAC/device code in ``hw/xtensa/esp32_wifi.c``,
``esp32_wifi_ap.c``, ``esp32_wlan_packet.c``, ``esp32_wlan.h``,
``esp32_wlan_packet.h``, and ``include/hw/xtensa/esp32_wifi.h`` was adapted
from the ESP32 Open MAC QEMU repository:

* Repository: https://github.com/esp32-open-mac/qemu
* Pinned repository commit: ``431466030220be88fb2df750abff6fcdcd13299a``
* MAC implementation provenance: ``4a5c772afec5eef22716a908e115ebd1062210fa``
  (2023-09-07, ``Implement memory logging``)
* License: ``hw/xtensa/esp32_wifi.c`` follows QEMU's GPL-2.0-or-later terms.
  ``esp32_wifi_ap.c``, ``esp32_wlan.h``, and the 802.11 frame helper files
  retain Clemens Kolbitsch's MIT license notice.

The model combines reverse-engineered ESP32 MAC registers with virtual
external Wi-Fi peers. It provides register access, DMA, interrupts,
clock/reset gating, SoftAP association/DHCP bridging, and station-mode
association with the configured peer. It is not a complete hardware
specification.

The MAC register block is mapped at ``0x3ff73000`` and its APB alias
``0x60033000``; its IRQ is interrupt-matrix source 0. Configure its network
backend with ``-nic user,model=misc.esp32_wifi``. ``open_eth`` is a separate
NIC choice, configured with its own ``-nic`` entry.

For guest SoftAP operation, a virtual station joins the guest's beacon,
obtains a lease from the guest DHCP server, and bridges packets through
802.11 DMA to the QEMU network backend. HTTP servers and other network
services execute in the guest firmware.

Configure the virtual station-mode AP with machine properties
``wifi-peer-ssid``, ``wifi-peer-password`` and ``wifi-peer-channel``::

  -machine esp32,wifi-peer-ssid=test-ap,wifi-peer-password=test-passphrase,wifi-peer-channel=1
  -nic user,model=misc.esp32_wifi

The peer channel defaults to 1. An 8-63 byte password enables WPA2-PSK/CCMP;
the peer validates RSN negotiation and EAPOL MICs, performs the four-message
handshake, and protects data with CCMP. Password derivation, MIC/replay
rejection, CCMP test vectors and RSN suite selection have unit coverage.

Guest MAC crypto is modeled separately from peer crypto. Key entry ``i``
has metadata at ``0x3ff74400 + 40*i``, key bytes at
``0x3ff74408 + 40*i``, and validity bit ``i`` at ``0x3ff73814``. These
locations follow ESP-IDF 6.1's ``libpp.a(hal_crypto.o)`` register accesses.
The MAC uses guest interface controls, valid key records and packet numbers
to encrypt TX and decrypt RX CCMP frames. Clear/reset invalidates key state.
This implements the observed programming path; it does not establish full
conformance to the undocumented silicon crypto engine.

The private RX-control, FE and RFPLL blocks implement selected register
protocols inferred from Espressif PHY libraries. RFPLL tuning at
``0x3ff508c4`` selects a channel from the programmed tune code when its
clock/reset gates permit completion. This replaces the imported model's
unknown-channel behavior. It does not reproduce analog RF lock behavior.

PHY measurement behavior has explicit approximations. TXDC residual signs
use an ideal midpoint of the programmed candidates. IQ estimation signals
readiness without calculating estimator data. PBUS, IQ and RFPLL completion
delays are fixed virtual-time approximations. Private register reset values
and analog transfer functions remain unverified. See
``docs/devel/esp32-apll.md`` for the analog-I2C/APLL subset. Register and
protocol tests do not establish complete PHY fidelity.

Receive buffers and retransmissions
----------------------------------

``RX_CTRL`` at ``0x3ff73084`` retains reception-enable bit 31. Descriptor
reload bit 0 self-clears. ``RX_DESCR_NEXT`` at ``0x3ff7308c`` reports
``0x3ff00000`` when the list is exhausted; ``RX_DESCR_LAST`` at
``0x3ff73090`` reports the last completed descriptor. Reloading an exhausted
list fetches the link appended to that descriptor. These operations follow
ESP-IDF 6.1's ``libpp.a(wdev.o)`` and ``libpp.a(hal_mac_rx.o)`` and the
hardware driver in ESP32 Open MAC, commit
``20ce43d595be914b4d3f553b28352bc07e003fa1``:

https://github.com/esp32-open-mac/esp32-open-mac/blob/20ce43d595be914b4d3f553b28352bc07e003fa1/main/hardware.c

That hardware reference states that automatic ACK transmission requires
placing the frame in an RX buffer. Each arriving frame therefore attempts
reception immediately. Disabled reception, an exhausted descriptor list,
or a descriptor unavailable to DMA prevents delivery and acknowledgement.
There is no receiver-side wait for the firmware to return a buffer. An
unsuccessful attempt does not advance the descriptor list or CCMP receive
packet number, and does not report a successful receive interrupt.

The virtual peer uses seven transmission attempts for unicast management
and data frames, the default short retry limit specified in RFC 5416,
section 6.7. It transmits at 6 Mb/s with ERP-OFDM airtime, ACK waiting and
DCF contention backoff. Retries keep the sequence number and encrypted
payload, set the Retry bit, and update FCS. Multicast/broadcast frames get
one attempt. Exhaustion drops the peer frame; TCP may subsequently
retransmit its data. These are explicit peer-fixture settings, not inferred
ESP32 transmitter configuration:

https://datatracker.ietf.org/doc/html/rfc5416#section-6.7

``esp32_wifi_peer_tx`` traces each attempt, its reception/ACK outcome, and
whether another attempt is scheduled. Host-backend queuing occurs before
these radio attempts and does not depend on available guest DMA buffers.

Overflow-specific silicon status and interrupt bits remain unverified;
the model does not invent them. RF interference, collisions with other
stations, rate adaptation, and aggregated/block-ack reception are not
covered by this peer's single-frame DCF model. A successful firmware
workload is not evidence for those unimplemented behaviors.
