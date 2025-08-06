use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;
use embedded_hal::spi::SpiDevice;
use crate::{Error, Ready, Sleeping, DWM1000};

impl<SPI> DWM1000<SPI, Sleeping>
where
    SPI: SpiDevice,
{
    // Wakes the radio up.
    // pub fn wake_up<DELAY: DelayNs>(
    //     mut self,
    //     delay: &mut DELAY,
    // ) -> Result<DWM1000<SPI, Ready>, Error<SPI>> {
    //     // Wake up using the spi
    //     self.spi.assert_cs_low().map_err(|e| Error::Spi(e))?;
    //     delay.delay_us(850 * 2);
    //     self.spi.assert_cs_high().map_err(|e| Error::Spi(e))?;
    //
    //     // Now we must wait 4 ms so all the clocks start running.
    //     delay.delay_us(4000 * 2);
    //
    //     // Let's check that we're actually awake now
    //     if self.spi.dev_id().read()?.ridtag() != 0xDECA {
    //         // Oh dear... We have not woken up!
    //         return Err(Error::StillAsleep);
    //     }
    //
    //     // Reset the wakeupstatus
    //     self.spi.sys_status().write(|w| w.slp2init(1).cplock(1))?;
    //
    //     // Restore the tx antenna delay
    //     let delay = self.state.tx_antenna_delay;
    //     self.spi.tx_antd().write(|w| w.value(delay.value() as u16))?;
    //
    //     // All other values should be restored, so return the ready radio.
    //     Ok(DWM1000 {
    //         spi: self.spi,
    //         seq: self.seq,
    //         state: Ready,
    //     })
    // }
}
