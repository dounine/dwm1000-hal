use crate::{DWM1000, Error, Ready, Uninitialized, device};
use core::num::Wrapping;
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;
use embedded_hal::spi::SpiDevice;

impl<SPI> DWM1000<SPI, Uninitialized>
where
    SPI: SpiDevice,
{
    /// Create a new instance of `DW1000`
    ///
    /// Requires the SPI peripheral and the chip select pin that are connected
    /// to the DW1000.
    pub fn new(spi: SPI) -> Self {
        DWM1000 {
            spi: device::DWM1000SpiDevice::new(spi),
            seq: Wrapping(0),
            state: Uninitialized,
        }
    }

    /// Initialize the DW1000
    ///
    /// The DW1000's default configuration is somewhat inconsistent, and the
    /// user manual (section 2.5.5) has a long list of default configuration
    /// values that should be changed to guarantee everything works correctly.
    /// This method does just that.
    ///
    /// Please note that this method assumes that you kept the default
    /// configuration. It is generally recommended not to change configuration
    /// before calling this method.
    pub fn init(
        mut self,
        // delay: &mut D,
    ) -> Result<DWM1000<SPI, Ready>, Error<SPI>> {
        // Set AGC_TUNE1. See user manual, section 2.5.5.1.
        self.spi.agc_tune1().write(|w| w.value(0x8870))?;

        // Set AGC_TUNE2. See user manual, section 2.5.5.2.
        self.spi.agc_tune2().write(|w| w.value(0x2502A907))?;

        // Set DRX_TUNE2. See user manual, section 2.5.5.3.
        self.spi.drx_tune2().write(|w| w.value(0x311A002D))?;

        // Set NTM. See user manual, section 2.5.5.4. This improves performance
        // in line-of-sight conditions, but might not be the best choice if non-
        // line-of-sight performance is important.
        self.spi.lde_cfg1().modify(|_, w| w.ntm(0xD))?;

        // Set LDE_CFG2. See user manual, section 2.5.5.5.
        self.spi.lde_cfg2().write(|w| w.value(0x1607))?;

        // Set TX_POWER. See user manual, section 2.5.5.6.
        self.spi.tx_power().write(|w| w.value(0x0E082848))?;

        // Set RF_TXCTRL. See user manual, section 2.5.5.7.
        self.spi
            .rf_txctrl()
            .modify(|_, w| w.txmtune(0b1111).txmq(0b111))?;

        // Set TC_PGDELAY. See user manual, section 2.5.5.8.
        self.spi.tc_pgdelay().write(|w| w.value(0xC0))?;

        // Set FS_PLLTUNE. See user manual, section 2.5.5.9.
        self.spi.fs_plltune().write(|w| w.value(0xBE))?;

        // Set LDOTUNE. See user manual, section 2.5.5.11.
        let ldotune_low = self.read_otp(0x004)?;
        if ldotune_low != 0 {
            let ldotune_high = self.read_otp(0x005)?;
            let ldotune = ldotune_low as u64 | (ldotune_high as u64) << 32;
            self.spi.ldotune().write(|w| w.value(ldotune))?;
        }

        // Set LDELOAD. See user manual, section 2.5.5.10.
        self.spi
            .pmsc_ctrl0()
            .modify(|r, w| w.raw_value(r.raw_value() | 0x0301))?;
        self.spi.otp_ctrl().write(|w| w.ldeload(0b1))?;
        // delay.delay_ms(5);
        self.spi
            .pmsc_ctrl0()
            .modify(|r, w| w.raw_value(r.raw_value() & !0x0101))?;

        Ok(DWM1000 {
            spi: self.spi,
            seq: self.seq,
            state: Ready,
        })
    }
}
