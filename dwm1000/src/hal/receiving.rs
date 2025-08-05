use crate::{
    DWM1000, Error, Ready, RxConfig,
    configs::{BitRate, SfdSequence},
    mac,
    time::Instant,
};
use byte::BytesExt as _;
use core::convert::TryInto;
use embedded_hal::digital::OutputPin;
use embedded_hal::spi::SpiDevice;
use fixed::traits::LossyInto;
use ieee802154::mac::FooterMode;

use super::{AutoDoubleBufferReceiving, Receiving};

/// An incoming message
#[derive(Debug)]
pub struct Message<'l> {
    /// The time the message was received
    ///
    /// This time is based on the local system time, as defined in the SYS_TIME
    /// register.
    pub rx_time: Instant,

    /// The MAC frame
    pub frame: mac::Frame<'l>,
}

/// A struct representing the quality of the received message.
#[derive(Debug, serde::Serialize, serde::Deserialize)]
pub struct RxQuality {
    /// The confidence that there was Line Of Sight between the sender and the receiver.
    ///
    /// - 0 means it's very unlikely there was LOS.
    /// - 1 means it's very likely there was LOS.
    ///
    /// The number doesn't give a guarantee, but an indication.
    /// It is based on the APS006_Part-3-DW1000-Diagnostics-for-NLOS-Channels-v1.1 document.
    pub los_confidence_level: f32,
    /// The radio signal strength indicator in dBm.
    ///
    /// The value is an estimation that is quite accurate up to -85 dBm.
    /// Above -85 dBm, the estimation underestimates the actual value.
    pub rssi: f32,
}

impl<SPI, CS, RECEIVING> DWM1000<SPI, CS, RECEIVING>
where
    SPI: SpiDevice,
    CS: OutputPin,
    RECEIVING: Receiving,
{
    pub(super) fn start_receiving(&mut self, config: RxConfig) -> Result<(), Error<SPI, CS>> {
        // Really weird thing about double buffering I can't find anything about.
        // When a message is received in double buffer mode that should be filtered out,
        // the radio gives a really short fake interrupt.
        // This messes up all the logic, so unless a solution can be found we simply don't support it.
        if RECEIVING::DOUBLE_BUFFERED && config.frame_filtering {
            return Err(Error::RxConfigFrameFilteringUnsupported);
        }

        // For unknown reasons, the DW1000 gets stuck in RX mode without ever
        // receiving anything, after receiving one good frame. Reset the
        // receiver to make sure its in a valid state before attempting to
        // receive anything.
        self.spi.pmsc_ctrl0().modify(
            |_, w| w.softreset(0b1110), // reset receiver
        )?;
        self.spi.pmsc_ctrl0().modify(
            |_, w| w.softreset(0b1111), // clear reset
        )?;

        // We're already resetting the receiver in the previous step, and that's
        // good enough to make my example program that's both sending and
        // receiving work very reliably over many hours (that's not to say it
        // becomes unreliable after those hours, that's just when my test
        // stopped). However, I've seen problems with an example program that
        // only received, never sent, data. That got itself into some weird
        // state where it couldn't receive anymore.
        // I suspect that's because that example didn't have the following line
        // of code, while the send/receive example had that line of code, being
        // called from `send`.
        // While I haven't, as of this writing, run any hours-long tests to
        // confirm this does indeed fix the receive-only example, it seems
        // (based on my eyeball-only measurements) that the RX/TX example is
        // dropping fewer frames now.
        self.force_idle(false)?;

        self.spi.sys_cfg().modify(|_, w| {
            w.ffen(config.frame_filtering as u8) // enable or disable frame filtering
                .ffab(0b1) // receive beacon frames
                .ffad(0b1) // receive data frames
                .ffaa(0b1) // receive acknowledgement frames
                .ffam(0b1) // receive MAC command frames
                // Set the double buffering and auto re-enable
                .dis_drxb(!RECEIVING::DOUBLE_BUFFERED as u8)
                .rxautr(RECEIVING::AUTO_RX_REENABLE as u8)
                // Set whether the receiver should look for 110kbps or 850/6800kbps messages
                .rxm110k((config.bitrate == BitRate::Kbps110) as u8)
        })?;

        // Set PLLLDT bit in EC_CTRL. According to the documentation of the
        // CLKPLL_LL bit in SYS_STATUS, this bit needs to be set to ensure the
        // reliable operation of the CLKPLL_LL bit. Since I've seen that bit
        // being set, I want to make sure I'm not just seeing crap.
        self.spi.ec_ctrl().modify(|_, w| w.pllldt(0b1))?;

        // Now that PLLLDT is set, clear all bits in SYS_STATUS that depend on
        // it for reliable operation. After that is done, these bits should work
        // reliably.
        self.spi
            .sys_status()
            .write(|w| w.cplock(0b1).clkpll_ll(0b1))?;

        // Apply the config
        self.spi.chan_ctrl().modify(|_, w| {
            w.tx_chan(config.channel as u8)
                .rx_chan(config.channel as u8)
                .dwsfd(
                    (config.sfd_sequence == SfdSequence::Decawave
                        || config.sfd_sequence == SfdSequence::DecawaveAlt)
                        as u8,
                )
                .rxprf(config.pulse_repetition_frequency as u8)
                .tnssfd(
                    (config.sfd_sequence == SfdSequence::User
                        || config.sfd_sequence == SfdSequence::DecawaveAlt)
                        as u8,
                )
                .rnssfd(
                    (config.sfd_sequence == SfdSequence::User
                        || config.sfd_sequence == SfdSequence::DecawaveAlt)
                        as u8,
                )
                .tx_pcode(
                    config
                        .channel
                        .get_recommended_preamble_code(config.pulse_repetition_frequency),
                )
                .rx_pcode(
                    config
                        .channel
                        .get_recommended_preamble_code(config.pulse_repetition_frequency),
                )
        })?;

        match config.sfd_sequence {
            SfdSequence::IEEE => {} // IEEE has predefined sfd lengths and the register has no effect.
            SfdSequence::Decawave => self.spi.sfd_length().write(|w| w.value(8))?, // This isn't entirely necessary as the Decawave8 settings in chan_ctrl already force it to 8
            SfdSequence::DecawaveAlt => self.spi.sfd_length().write(|w| w.value(16))?, // Set to 16
            SfdSequence::User => {} // Users are responsible for setting the lengths themselves
        }

        // Set general tuning
        self.spi.drx_tune0b().write(|w| {
            w.value(
                config
                    .bitrate
                    .get_recommended_drx_tune0b(config.sfd_sequence),
            )
        })?;
        self.spi.drx_tune1a().write(|w| {
            w.value(
                config
                    .pulse_repetition_frequency
                    .get_recommended_drx_tune1a(),
            )
        })?;
        let drx_tune1b = config
            .expected_preamble_length
            .get_recommended_drx_tune1b(config.bitrate)?;
        self.spi.drx_tune1b().write(|w| w.value(drx_tune1b))?;
        let drx_tune2 = config
            .pulse_repetition_frequency
            .get_recommended_drx_tune2(
                config.expected_preamble_length.get_recommended_pac_size(),
            )?;
        self.spi.drx_tune2().write(|w| w.value(drx_tune2))?;
        self.spi
            .drx_tune4h()
            .write(|w| w.value(config.expected_preamble_length.get_recommended_dxr_tune4h()))?;

        // Set channel tuning
        self.spi
            .rf_rxctrlh()
            .write(|w| w.value(config.channel.get_recommended_rf_rxctrlh()))?;
        self.spi
            .fs_pllcfg()
            .write(|w| w.value(config.channel.get_recommended_fs_pllcfg()))?;
        self.spi
            .fs_plltune()
            .write(|w| w.value(config.channel.get_recommended_fs_plltune()))?;

        // Set the LDE registers
        self.spi
            .lde_cfg2()
            .write(|w| w.value(config.pulse_repetition_frequency.get_recommended_lde_cfg2()))?;
        self.spi.lde_repc().write(|w| {
            w.value(
                config.channel.get_recommended_lde_repc_value(
                    config.pulse_repetition_frequency,
                    config.bitrate,
                ),
            )
        })?;

        // Check if the rx buffer pointer is correct
        let status = self.spi.sys_status().read()?;
        if status.hsrbp() != status.icrbp() {
            // The RX Buffer Pointer of the host and the ic side don't point to the same one.
            // We need to switch over
            self.spi.sys_ctrl().modify(|_, w| w.hrbpt(1))?;
        }

        // Start receiving
        self.spi.sys_ctrl().modify(|_, w| w.rxenab(0b1))?;

        Ok(())
    }

    /// Wait for receive operation to finish
    ///
    /// This method returns an `nb::Result` to indicate whether the transmission
    /// has finished, or whether it is still ongoing. You can use this to busily
    /// wait for the transmission to finish, for example using `nb`'s `block!`
    /// macro, or you can use it in tandem with [`DWM1000::enable_rx_interrupts`]
    /// and the DW1000 IRQ output to wait in a more energy-efficient manner.
    ///
    /// Handling the DW1000's IRQ output line is out of the scope of this
    /// driver, but please note that if you're using the DWM1001 module or
    /// DWM1001-Dev board, that the `dwm1001` crate has explicit support for
    /// this.
    pub fn wait_receive<'b>(
        &mut self,
        buffer: &'b mut [u8],
    ) -> nb::Result<Message<'b>, Error<SPI, CS>> {
        // ATTENTION:
        // If you're changing anything about which SYS_STATUS flags are being
        // checked in this method, also make sure to update `enable_interrupts`.
        let sys_status = self
            .spi()
            .sys_status()
            .read()
            .map_err(|error| nb::Error::Other(Error::Spi(error)))?;

        // Is a frame ready?
        if sys_status.rxdfr() == 0b0 {
            // No frame ready. Check for errors.
            if sys_status.rxfce() == 0b1 {
                return Err(nb::Error::Other(Error::Fcs));
            }
            if sys_status.rxphe() == 0b1 {
                return Err(nb::Error::Other(Error::Phy));
            }
            if sys_status.rxrfsl() == 0b1 {
                return Err(nb::Error::Other(Error::ReedSolomon));
            }
            if sys_status.rxrfto() == 0b1 {
                return Err(nb::Error::Other(Error::FrameWaitTimeout));
            }
            if sys_status.rxovrr() == 0b1 {
                return Err(nb::Error::Other(Error::Overrun));
            }
            if sys_status.rxpto() == 0b1 {
                return Err(nb::Error::Other(Error::PreambleDetectionTimeout));
            }
            if sys_status.rxsfdto() == 0b1 {
                return Err(nb::Error::Other(Error::SfdTimeout));
            }
            if sys_status.affrej() == 0b1 {
                return Err(nb::Error::Other(Error::FrameFilteringRejection));
            }
            // Some error flags that sound like valid errors aren't checked here,
            // because experience has shown that they seem to occur spuriously
            // without preventing a good frame from being received. Those are:
            // - LDEERR: Leading Edge Detection Processing Error
            // - RXPREJ: Receiver Preamble Rejection

            // No errors detected. That must mean the frame is just not ready
            // yet.
            return Err(nb::Error::WouldBlock);
        }

        // Frame is ready. Continue.

        // Wait until LDE processing is done. Before this is finished, the RX
        // time stamp is not available.
        if sys_status.ldedone() == 0b0 {
            return Err(nb::Error::WouldBlock);
        }
        let rx_time = self
            .spi()
            .rx_time()
            .read()
            .map_err(|error| nb::Error::Other(Error::Spi(error)))?
            .rx_stamp();

        // `rx_time` comes directly from the register, which should always
        // contain a 40-bit timestamp. Unless the hardware or its documentation
        // are buggy, the following should never panic.
        let rx_time = unsafe { Instant::new_unchecked(rx_time) };

        // Read received frame
        let rx_finfo = self
            .spi()
            .rx_finfo()
            .read()
            .map_err(|error| nb::Error::Other(Error::Spi(error)))?;
        let rx_buffer = self
            .spi()
            .rx_buffer()
            .read()
            .map_err(|error| nb::Error::Other(Error::Spi(error)))?;

        let len = rx_finfo.rxflen() as usize;

        if buffer.len() < len {
            return Err(nb::Error::Other(Error::BufferTooSmall {
                required_len: len,
            }));
        }

        buffer[..len].copy_from_slice(&rx_buffer.data()[..len]);

        let frame = buffer[..len]
            .read_with(
                &mut 0,
                if self.state.get_rx_config().append_crc {
                    FooterMode::Explicit
                } else {
                    FooterMode::None
                },
            )
            .map_err(|error| nb::Error::Other(Error::Frame(error)))?;

        // Reset status bits. This is not strictly necessary in single buffered mode, but it helps, if
        // you have to inspect SYS_STATUS manually during debugging.
        self.clear_status()?;

        if RECEIVING::DOUBLE_BUFFERED {
            // Toggle to the other buffer. This will also signal the IC that the current buffer is free for use
            self.spi
                .sys_ctrl()
                .modify(|_, w| w.hrbpt(1))
                .map_err(|error| nb::Error::Other(Error::Spi(error)))?;
        }

        self.state.mark_finished();

        Ok(Message { rx_time, frame })
    }

    fn clear_status(&mut self) -> Result<(), Error<SPI, CS>> {
        let do_clear = |ll: &mut crate::spi::DWM1000<SPI, CS>| {
            ll.sys_status().write(|w| {
                w.rxprd(0b1) // Receiver Preamble Detected
                    .rxsfdd(0b1) // Receiver SFD Detected
                    .ldedone(0b1) // LDE Processing Done
                    .rxphd(0b1) // Receiver PHY Header Detected
                    .rxphe(0b1) // Receiver PHY Header Error
                    .rxdfr(0b1) // Receiver Data Frame Ready
                    .rxfcg(0b1) // Receiver FCS Good
                    .rxfce(0b1) // Receiver FCS Error
                    .rxrfsl(0b1) // Receiver Reed Solomon Frame Sync Loss
                    .rxrfto(0b1) // Receiver Frame Wait Timeout
                    .ldeerr(0b1) // Leading Edge Detection Processing Error
                    .rxovrr(0b1) // Receiver Overrun
                    .rxpto(0b1) // Preamble Detection Timeout
                    .rxsfdto(0b1) // Receiver SFD Timeout
                    .rxrscs(0b1) // Receiver Reed-Solomon Correction Status
                    .rxprej(0b1) // Receiver Preamble Rejection
            })
        };

        if RECEIVING::DOUBLE_BUFFERED {
            let status = self.spi.sys_status().read()?;

            if status.hsrbp() == status.icrbp() {
                let saved_sys_mask = self.spi.sys_mask().read()?.0;
                // Mask all status bits to prevent spurious interrupts
                self.spi.sys_mask().write(|w| w)?;

                do_clear(self.spi())?;

                // Restore the mask
                self.spi.sys_mask().write(|w| {
                    w.0.copy_from_slice(&saved_sys_mask);
                    w
                })?;
            }
        } else {
            do_clear(self.spi())?;
        }

        Ok(())
    }

    // fn calculate_luep(&mut self) -> Result<f32, Error<SPI, CS>> {
    //     #[allow(unused_imports)]
    //     use micromath::F32Ext;
    //
    //     let rx_time_register = self.spi().rx_time().read()?;
    //     let rx_fqual_register = self.spi().rx_fqual().read()?;
    //     let lde_cfg1_register = self.spi().lde_cfg1().read()?;
    //
    //     let path_position: f32 =
    //         fixed::types::U10F6::from_le_bytes(rx_time_register.fp_index().to_le_bytes())
    //             .lossy_into();
    //
    //     // Calculate a new low threshold by taking 0.6 times the reported noise threshold from the
    //     // diagnostics. This new threshold is shown in red in Figure 5. Get existing noise threshold as the
    //     // multiplication of STD_NOISE from Register 12:00 and NTM from Register 2E:0806.
    //     let noise_threshold: u16 = rx_fqual_register.std_noise() * lde_cfg1_register.ntm() as u16;
    //     let new_low_threshold = (noise_threshold as f32 * 0.6) as u16;
    //     // From the integer part of the first path position, pathPosition,
    //     // form an analysis window of 16 samples back tracked from that index.
    //     const WINDOW_SIZE: usize = 16;
    //     let window_start = (path_position as u16).saturating_sub(WINDOW_SIZE as u16);
    //
    //     let mut cir_buffer = [0u8; WINDOW_SIZE * 4 + 4];
    //     let cir = self.spi.cir(window_start * 4, &mut cir_buffer)?;
    //
    //     // To determine the number of peaks in the newly formed analysis window we take the difference of consecutive values.
    //     // We identify a peak when these differences change from positive to negative.
    //
    //     // Calculate the amplitudes in the cir buffer
    //     let mut amplitudes = [0.0; WINDOW_SIZE];
    //     let mut peak_count = 0;
    //     for index in 0..WINDOW_SIZE {
    //         let real = u16::from_le_bytes(cir[index * 4..index * 4 + 2].try_into().unwrap()) as f32;
    //         let imag =
    //             u16::from_le_bytes(cir[index * 4 + 2..index * 4 + 4].try_into().unwrap()) as f32;
    //
    //         amplitudes[index] = (real * real + imag * imag).sqrt();
    //
    //         if index >= 2 && amplitudes[index - 1] > new_low_threshold as f32 {
    //             let previous_difference = amplitudes[index - 1] - amplitudes[index - 2];
    //             let current_difference = amplitudes[index] - amplitudes[index - 1];
    //             peak_count += (previous_difference.is_sign_positive()
    //                 && current_difference.is_sign_negative()) as u8;
    //         }
    //     }
    //
    //     Ok(peak_count as f32 / (WINDOW_SIZE / 2) as f32)
    // }

    fn calculate_prnlos(&mut self) -> Result<f32, Error<SPI, CS>> {
        #[allow(unused_imports)]
        use micromath::F32Ext;

        let rx_time_register = self.spi().rx_time().read()?;

        let path_position: f32 =
            fixed::types::U10F6::from_le_bytes(rx_time_register.fp_index().to_le_bytes())
                .lossy_into();

        let peak_path_index: f32 = self.spi().lde_ppindx().read()?.value() as f32;

        let idiff = (path_position - peak_path_index).abs();
        if idiff <= 3.3 {
            Ok(0.0)
        } else if idiff < 6.0 {
            Ok(0.39178 * idiff - 1.31719)
        } else {
            Ok(1.0)
        }
    }

    fn calculate_mc(&mut self) -> Result<f32, Error<SPI, CS>> {
        let rx_time_register = self.spi().rx_time().read()?;
        let rx_fqual_register = self.spi().rx_fqual().read()?;

        let fp_ampl1: u16 = rx_time_register.fp_ampl1();
        let fp_ampl2: u16 = rx_fqual_register.fp_ampl2();
        let fp_ampl3: u16 = rx_fqual_register.fp_ampl3();
        let peak_path_amplitude: u16 = self.spi().lde_ppampl().read()?.value();

        Ok(fp_ampl1.max(fp_ampl2).max(fp_ampl3) as f32 / peak_path_amplitude as f32)
    }

    /// Calculate the rssi based on the info the chip provides.
    ///
    /// Algorithm was taken from `4.7.2 Estimating the receive signal power` of the user manual.
    fn calculate_rssi(&mut self) -> Result<f32, Error<SPI, CS>> {
        #[allow(unused_imports)]
        use micromath::F32Ext;

        let c = self.spi.rx_fqual().read()?.cir_pwr() as f32;
        let a = match self.state.get_rx_config().pulse_repetition_frequency {
            crate::configs::PulseRepetitionFrequency::Mhz16 => 113.77,
            crate::configs::PulseRepetitionFrequency::Mhz64 => 121.74,
        };

        let data_rate = self.state.get_rx_config().bitrate;
        let sfd_sequence = self.state.get_rx_config().sfd_sequence;

        let rxpacc = self.spi.rx_finfo().read()?.rxpacc();
        let rxpacc_nosat = self.spi.rxpacc_nosat().read()?.value();

        let n = if rxpacc == rxpacc_nosat {
            rxpacc as f32 + sfd_sequence.get_rxpacc_adjustment(data_rate) as f32
        } else {
            rxpacc as f32
        };

        let rssi = 10.0 * ((c * (1 << 17) as f32) / (n * n)).log10() - a;

        if rssi.is_finite() {
            Ok(rssi)
        } else {
            Err(Error::BadRssiCalculation)
        }
    }

    /// Reads the quality of the received message.
    ///
    /// This must be called after the [`DWM1000::wait_receive`] function has
    /// successfully returned.
    // pub fn read_rx_quality(&mut self) -> Result<RxQuality, Error<SPI, CS>> {
    //     assert!(
    //         self.state.is_finished(),
    //         "The function 'wait' must have successfully returned before this function can be called"
    //     );
    //
    //     let luep = self.calculate_luep()?;
    //     let prnlos = self.calculate_prnlos()?;
    //     let mc = self.calculate_mc()?;
    //
    //     let los_confidence_level = if luep > 0.0 {
    //         0.0
    //     } else if prnlos == 0.0 || mc >= 0.9 {
    //         1.0
    //     } else {
    //         1.0 - prnlos
    //     };
    //
    //     let rssi = self.calculate_rssi()?;
    //
    //     Ok(RxQuality {
    //         los_confidence_level: los_confidence_level.clamp(0.0, 1.0),
    //         rssi,
    //     })
    // }

    /// Gets the external sync values from the registers.
    ///
    /// The tuple contains (cycles_since_sync, nanos_until_tick, raw_timestamp).
    /// See the user manual at 6.1.3 to see how to calculate the actual time value.
    /// In the manual, the return values are named (N, T1, RX_RAWST)
    /// This is left to the user so the precision of the calculations are left to the user to decide.
    pub fn read_external_sync_time(&mut self) -> Result<(u32, u8, u64), Error<SPI, CS>> {
        assert!(
            self.state.is_finished(),
            "The function 'wait' must have successfully returned before this function can be called"
        );

        let cycles_since_sync = self.spi().ec_rxtc().read()?.rx_ts_est();
        let nanos_until_tick = self.spi().ec_golp().read()?.offset_ext();
        let raw_timestamp = self.spi().rx_time().read()?.rx_rawst();

        Ok((cycles_since_sync, nanos_until_tick, raw_timestamp))
    }

    /// Finishes receiving and returns to the `Ready` state
    ///
    /// If the receive operation has finished, as indicated by `wait`, this is a
    /// no-op. If the receive operation is still ongoing, it will be aborted.
    pub fn finish_receiving(mut self) -> Result<DWM1000<SPI, CS, Ready>, (Self, Error<SPI, CS>)> {
        if !self.state.is_finished() {
            // Can't use `map_err` and `?` here, as the compiler will complain
            // about `self` moving into the closure.
            match self.force_idle(RECEIVING::DOUBLE_BUFFERED) {
                Ok(()) => (),
                Err(error) => return Err((self, error)),
            }
        }

        Ok(DWM1000 {
            spi: self.spi,
            seq: self.seq,
            state: Ready,
        })
    }
}

impl<SPI, CS> DWM1000<SPI, CS, AutoDoubleBufferReceiving>
where
    SPI: SpiDevice,
    CS: OutputPin,
{
    /// Try to continue receiving
    pub fn continue_receiving(
        self,
    ) -> Result<
        DWM1000<SPI, CS, AutoDoubleBufferReceiving>,
        Result<DWM1000<SPI, CS, Ready>, (Self, Error<SPI, CS>)>,
    > {
        if !self.state.is_finished() {
            Err(self.finish_receiving())
        } else {
            Ok(self)
        }
    }
}
