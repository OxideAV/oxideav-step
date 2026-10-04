//! Crate-local error type (std-only; the `registry` feature maps it onto
//! `oxideav_core::Error` at the decoder boundary).

use core::fmt;

/// Result alias for this crate.
pub type Result<T> = core::result::Result<T, Error>;

/// Why a STEP exchange structure could not be read.
#[derive(Debug, Clone, PartialEq)]
pub enum Error {
    /// The bytes are not an ISO 10303-21 exchange structure, or its
    /// syntax is malformed (wraps the physical-file parser's error).
    Parse(oxideav_ifc::Error),
    /// A DoS cap was crossed (input size, instance count, nesting,
    /// recursion depth, tessellation output size).
    LimitExceeded(String),
    /// The file parsed but carries nothing this reader can turn into
    /// geometry (no supported shape representation).
    NoGeometry(String),
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Parse(e) => write!(f, "STEP parse error: {e}"),
            Self::LimitExceeded(m) => write!(f, "STEP limit exceeded: {m}"),
            Self::NoGeometry(m) => write!(f, "STEP file has no usable geometry: {m}"),
        }
    }
}

impl std::error::Error for Error {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Parse(e) => Some(e),
            _ => None,
        }
    }
}

impl From<oxideav_ifc::Error> for Error {
    fn from(e: oxideav_ifc::Error) -> Self {
        match e {
            oxideav_ifc::Error::LimitExceeded(m) => Self::LimitExceeded(m),
            other => Self::Parse(other),
        }
    }
}
