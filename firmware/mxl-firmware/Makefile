# SPDX-License-Identifier: GPL-2.0-or-later

include $(TOPDIR)/rules.mk

PKG_NAME:=mxl-firmware
PKG_VERSION:=1.0.0
PKG_RELEASE:=1
# PKG_SOURCE_URL:=https://www.verizon.com/content/dam/verizon/support/consumer/documents/open-source/Verizon%20Router%20(CR1000A)%20source%20code%20release%204.0.22.40.zip

PKG_MAINTAINER:=Kenneth Kasilag <kenneth@kasilag.me>
PKG_LICENSE:=Proprietary
PKG_LICENSE_FILES:=

include $(INCLUDE_DIR)/package.mk

define Package/mxl-firmware
  SECTION:=firmware
  CATEGORY:=Firmware
  TITLE:=MaxLinear MXL371x MoCA PHY firmware (shared SoC ELF).
  URL:=
endef

define Package/mxl-firmware/description
  Shared, board-independent SoC firmware ELF for the MaxLinear MXL371x MoCA 2.5
  PHY.  The per-board MoCA config blobs ship in a separate device package.
endef

# Per-device MoCA config blob package.  The driver loads these by plain name
# from /lib/firmware and packs them into the boot-config payloads at runtime;
# which board's blobs are installed is selected at build time by adding the
# matching mxl-firmware-<board> package to the device's DEVICE_PACKAGES.
define Package/mxl-firmware-verizon-cr1000a
  SECTION:=firmware
  CATEGORY:=Firmware
  TITLE:=MoCA config blobs for the Verizon CR1000A.
  DEPENDS:=+mxl-firmware
  URL:=
endef

define Package/mxl-firmware-verizon-cr1000a/description
  Verizon CR1000A MoCA 2.5 config blobs (clink/mcast/rlapm/endet/sapm/rssi/
  platform), installed to /lib/firmware for the phy-mxl371x runtime packer.
endef

define Build/Compile
endef

define Package/mxl-firmware/install
	$(INSTALL_DIR) $(1)/lib/firmware
	$(INSTALL_DATA) ./ccpu.elf.leucadia $(1)/lib/firmware/
endef

define Package/mxl-firmware-verizon-cr1000a/install
	$(INSTALL_DIR) $(1)/lib/firmware
	$(INSTALL_DATA) ./verizon-cr1000a/*.bin $(1)/lib/firmware/
endef

$(eval $(call BuildPackage,mxl-firmware))
$(eval $(call BuildPackage,mxl-firmware-verizon-cr1000a))
