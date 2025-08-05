//! High-level interface to the DW1000
//!
//! The entry point to this API is the [DWM1000] struct. Please refer to the
//! documentation there for more details.
//!
//! This module implements a high-level interface to the DW1000. This is the
//! recommended way to access the DW1000 using this crate, unless you need the
//! greater flexibility provided by the [register-level interface].
//!
//! [register-level interface]: ../ll/index.html

use crate::{hal, spi};
use core::{fmt, num::Wrapping};

pub use awake::*;
pub use error::*;
pub use ready::*;
pub use receiving::*;
pub use sending::*;
pub use sleeping::*;
pub use state_impls::*;
pub use uninitialized::*;

mod error;
mod ready;
mod state_impls;
mod receiving;
mod uninitialized;
mod sleeping;
mod sending;
mod awake;

/// Entry point to the DW1000 driver API
pub struct DWM1000<SPI, CS, State> {
    spi: spi::DWM1000<SPI, CS>,
    seq: Wrapping<u8>,
    state: State,
}

// Can't be derived without putting requirements on `SPI` and `CS`.
impl<SPI, CS, State> fmt::Debug for DWM1000<SPI, CS, State>
where
    State: fmt::Debug,
{
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "DW1000 {{ state: ")?;
        self.state.fmt(f)?;
        write!(f, ", .. }}")?;
        Ok(())
    }
}
