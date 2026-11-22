/// Error returned when a variable-length MSP field exceeds its capacity.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CapacityError {
    pub capacity: usize,
    pub requested: usize,
}

impl core::fmt::Display for CapacityError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(
            f,
            "MSP fixed container capacity exceeded: capacity {}, requested {}",
            self.capacity, self.requested
        )
    }
}

impl core::error::Error for CapacityError {}
