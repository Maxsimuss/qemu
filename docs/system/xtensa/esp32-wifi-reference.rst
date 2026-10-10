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

The model is a reverse-engineering starting point. Register values and behavior
in this code were inferred from proprietary ESP32 Wi-Fi firmware and are not a
complete hardware specification. The local code has since added a virtual
external station for the guest SoftAP, including observed open-system
authentication and association, guest DHCP lease tracking, and a SLIRP packet
bridge. It also has a configurable virtual external AP peer for station-mode
protocol work. These paths are experimental and have not yet established the
complete product workflow or hardware conformance.

The key register locations are independently confirmed from the supplied
ESP-IDF 6.1 firmware's ``libpp.a(hal_crypto.o)`` disassembly. For key entry
``i``, ``hal_crypto_set_key_entry`` writes two metadata words at
``0x3ff74400 + 40*i``, copies key bytes beginning at ``0x3ff74408 + 40*i``,
and sets bit ``i`` in the validity word at ``0x3ff73814``. The clear helper
clears that bit and zeroes ten words beginning at the entry base. The model
backs these addresses as ordinary register storage and resets them with the
MAC window. This source establishes where the driver writes key records, but
not how the private crypto engine consumes them. Register storage therefore
does not claim guest-key CCMP support; the external peer's WPA2/CCMP
implementation is separate.

The first observed AP-start failure in the small IDF 6.2 fixture occurs in the
proprietary PHY routine ``disable_wifi_agc`` at a read from ``0x3ff5c01c``.
This address is inside the private RX-control bank identified by public
ESP32-Open-MAC reverse-engineering notes, but those sources do not document the
register's reset or AGC semantics. The model does not map this bank as generic
RAM or return fabricated values. A primary Espressif register specification,
reference implementation, or captured silicon behavior is still required to
model this part of PHY initialization and reach product-level Wi-Fi acceptance.

The local adaptation drops the source repository's optional stack-unwinding
MMIO logger and uses this tree's ESP32 register header. The machine now creates
the NIC-backed model when the matching QEMU NIC is configured, maps its register
block at ``0x3ff73000`` and the firmware APB alias at ``0x60033000``, and routes
its IRQ to original-ESP32 interrupt-matrix source 0. The TX DMA and interrupt
registers are exercised by ``tests/qtest/esp32-wifi-test.c``.

For guest SoftAP provisioning, QEMU supplies a virtual station that joins the
beacon advertised by the guest, obtains its address from the guest's DHCP
server, and bridges network packets through the guest's 802.11 DMA path. The
external endpoint is provided by QEMU's configured network backend. This is
intended to support host-forwarded HTTP requests after the lease is obtained.
The current implementation does not supply a guest HTTP server; that remains
part of the original firmware. The guest station's network receive path now
uses From-DS addressing, while guest-to-network frames are accepted only in
the observed To-DS/SNAP form.

The station-mode peer is configured with the machine properties
``wifi-peer-ssid``, ``wifi-peer-password``, and ``wifi-peer-channel``. For
example, start QEMU with
``-machine esp32,wifi-peer-ssid=test-ap,wifi-peer-password=test-passphrase,wifi-peer-channel=1``
and ``-nic user,model=misc.esp32_wifi``. The peer channel defaults to channel 1.
When configured with an 8–63 byte password, it advertises WPA2-PSK/CCMP,
validates the station RSN suite and EAPOL MIC, performs the four-message
handshake, and protects station data with CCMP. Password derivation and the
handshake/CCMP helpers have unit coverage, including rejection of a wrong
password, damaged MIC, and replayed packet number. CCMP encryption and
decryption also match the published P802.11i/D7.0 test MPDU used by FreeBSD's
net80211 regression suite, and RSN suite selection is tested with multiple
offered ciphers and AKMs. The end-to-end original ESP32 STA path remains
unverified because the ROM analog-I2C/PHY channel-selection path has not been
modeled; without a channel learned from the guest hardware, the configured peer
cannot yet be shown to participate in an actual scan or connection. ``open_eth``
remains a separate NIC choice; configure each model with its own ``-nic`` entry
when both are needed.

The Wi-Fi RF channel remains unset because this import does not model the ROM
analog-I2C/PHY channel-selection path. The peer helper therefore cannot perform
a working channel scan/association until that hardware behavior is implemented.

The pinned repository's optional ``hw/xtensa/esp32_ana.c`` does contain a
channel side effect, but its implementation is not supported by an independently
verified register mapping: it special-cases offset ``0xc4``, extracts the low
byte, checks ``value % 10 == 4``, and infers ``channel = value / 10 - 1``. The
implementation logs this as an analog register write and returns fabricated
constant reads for other addresses. Its last change is commit
``4a5d74b3ee28fdfceee35cdec0a66d58aaf872f0`` (2023-09-07). Because this is a
guess inside a generic unimplemented analog block, it was not ported into the
existing analog-I2C/APLL model. The Wi-Fi channel remains unknown until a
hardware-backed ROM analog-I2C/PHY sequence or an authoritative register
specification establishes it.
