/// Error returned when an item or slice exceeds the fixed capacity `N` of [`MspVec`] or [`MspString`].
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

/// A stack-allocated, `Copy`, fixed-capacity vector for bounded MSP arrays
/// (such as RC channels, motor telemetry, or byte buffers) with `N <= 255`.
///
/// Invariant: elements in `0..len` are initialized; elements in `len..N` are
/// always `T::default()` so that `Copy`, `PartialEq`, and `Eq` are sound and deterministic.
#[derive(Clone, Copy)]
pub struct MspVec<T, const N: usize> {
    data: [T; N],
    len: u8,
}

impl<T: Copy + Default, const N: usize> MspVec<T, N> {
    /// Creates an empty `MspVec`.
    ///
    /// Fails at compile time if `N > 255`.
    ///
    /// ```compile_fail,E0080
    /// use uf_msp::MspVec;
    ///
    /// let _ = MspVec::<u8, 256>::from_slice(&[]);
    /// ```
    #[must_use]
    pub fn new() -> Self {
        const {
            assert!(N <= 255, "MspVec capacity N must be at most 255");
        }
        Self {
            data: [T::default(); N],
            len: 0,
        }
    }

    /// Creates an `MspVec` by copying elements from `slice`.
    ///
    /// # Errors
    ///
    /// Returns [`CapacityError`] if `slice.len() > N`.
    pub fn from_slice(slice: &[T]) -> Result<Self, CapacityError> {
        let mut vec = Self::new();
        let dst = vec.data.get_mut(..slice.len()).ok_or(CapacityError {
            capacity: N,
            requested: slice.len(),
        })?;
        let Ok(len) = u8::try_from(slice.len()) else {
            return Err(CapacityError {
                capacity: N,
                requested: slice.len(),
            });
        };
        dst.copy_from_slice(slice);
        vec.len = len;
        Ok(vec)
    }

    /// Appends `value` to the end of the vector.
    ///
    /// # Errors
    ///
    /// Returns [`CapacityError`] if the vector is already at capacity `N`.
    pub fn push(&mut self, value: T) -> Result<(), CapacityError> {
        let idx = usize::from(self.len);
        let slot = self.data.get_mut(idx).ok_or(CapacityError {
            capacity: N,
            requested: idx.saturating_add(1),
        })?;
        let Ok(next_len) = u8::try_from(idx.saturating_add(1)) else {
            return Err(CapacityError {
                capacity: N,
                requested: idx.saturating_add(1),
            });
        };
        *slot = value;
        self.len = next_len;
        Ok(())
    }

    /// Clears the vector, resetting all elements to `T::default()`.
    pub fn clear(&mut self) {
        self.data.fill(T::default());
        self.len = 0;
    }
}

impl<T, const N: usize> MspVec<T, N> {
    /// Maximum number of elements this container can hold.
    #[must_use]
    pub const fn capacity(&self) -> usize {
        N
    }

    /// Number of active elements currently stored.
    #[must_use]
    pub const fn len(&self) -> usize {
        self.len as usize
    }

    /// Returns `true` if the vector contains no elements.
    #[must_use]
    pub const fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Returns a slice of the active elements (`0..self.len()`).
    #[must_use]
    pub fn as_slice(&self) -> &[T] {
        self.data.get(..usize::from(self.len)).unwrap_or(&[])
    }

    /// Returns a mutable slice of the active elements (`0..self.len()`).
    #[must_use]
    pub fn as_mut_slice(&mut self) -> &mut [T] {
        let len = usize::from(self.len);
        self.data.get_mut(..len).unwrap_or(&mut [])
    }
}

impl<T: Copy + Default, const N: usize> Default for MspVec<T, N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<T, const N: usize> core::ops::Deref for MspVec<T, N> {
    type Target = [T];

    fn deref(&self) -> &Self::Target {
        self.as_slice()
    }
}

impl<T, const N: usize> core::ops::DerefMut for MspVec<T, N> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.as_mut_slice()
    }
}

impl<T, const N: usize> AsRef<[T]> for MspVec<T, N> {
    fn as_ref(&self) -> &[T] {
        self.as_slice()
    }
}

impl<T: PartialEq, const N: usize> PartialEq for MspVec<T, N> {
    fn eq(&self, other: &Self) -> bool {
        self.as_slice() == other.as_slice()
    }
}

impl<T: Eq, const N: usize> Eq for MspVec<T, N> {}

impl<T: core::fmt::Debug, const N: usize> core::fmt::Debug for MspVec<T, N> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        core::fmt::Debug::fmt(self.as_slice(), f)
    }
}

#[cfg(feature = "defmt")]
impl<T: defmt::Format, const N: usize> defmt::Format for MspVec<T, N> {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(f, "{}", self.as_slice());
    }
}

impl<'a, T: Copy + Default, const N: usize> TryFrom<&'a [T]> for MspVec<T, N> {
    type Error = CapacityError;

    fn try_from(slice: &'a [T]) -> Result<Self, Self::Error> {
        Self::from_slice(slice)
    }
}

/// A stack-allocated, `Copy`, fixed-capacity UTF-8 string for bounded MSP text fields
/// (such as board names, pilot names, or build strings) with `N <= 255`.
#[derive(Clone, Copy, Default, PartialEq, Eq)]
pub struct MspString<const N: usize> {
    bytes: MspVec<u8, N>,
}

impl<const N: usize> MspString<N> {
    /// Creates an empty `MspString`.
    #[must_use]
    pub fn new() -> Self {
        Self {
            bytes: MspVec::new(),
        }
    }

    /// Creates an `MspString` from a string slice `s`.
    ///
    /// # Errors
    ///
    /// Returns [`CapacityError`] if `s.len() > N`.
    #[expect(
        clippy::should_implement_trait,
        reason = "inherent from_str returns CapacityError and mirrors core::str APIs alongside FromStr"
    )]
    pub fn from_str(s: &str) -> Result<Self, CapacityError> {
        Ok(Self {
            bytes: MspVec::from_slice(s.as_bytes())?,
        })
    }

    /// Creates an `MspString` by validating UTF-8 from `bytes`.
    ///
    /// Returns `None` if `bytes` is not valid UTF-8 or exceeds capacity `N`.
    #[must_use]
    pub fn from_utf8(bytes: &[u8]) -> Option<Self> {
        let s = core::str::from_utf8(bytes).ok()?;
        Self::from_str(s).ok()
    }

    /// Maximum number of UTF-8 bytes this string can hold.
    #[must_use]
    pub const fn capacity(&self) -> usize {
        N
    }

    /// Length of the string in bytes.
    #[must_use]
    pub const fn len(&self) -> usize {
        self.bytes.len()
    }

    /// Returns `true` if the string is empty.
    #[must_use]
    pub const fn is_empty(&self) -> bool {
        self.bytes.is_empty()
    }

    /// Returns the raw UTF-8 byte slice.
    #[must_use]
    pub fn as_bytes(&self) -> &[u8] {
        self.bytes.as_slice()
    }

    /// Returns the string slice `&str`.
    #[must_use]
    pub fn as_str(&self) -> &str {
        // All constructors preserve valid UTF-8 in `self.bytes`, and empty slice is valid UTF-8.
        core::str::from_utf8(self.bytes.as_slice()).unwrap_or("")
    }
}

impl<const N: usize> core::ops::Deref for MspString<N> {
    type Target = str;

    fn deref(&self) -> &Self::Target {
        self.as_str()
    }
}

impl<const N: usize> AsRef<str> for MspString<N> {
    fn as_ref(&self) -> &str {
        self.as_str()
    }
}

impl<const N: usize> AsRef<[u8]> for MspString<N> {
    fn as_ref(&self) -> &[u8] {
        self.as_bytes()
    }
}

impl<const N: usize> core::fmt::Debug for MspString<N> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        core::fmt::Debug::fmt(self.as_str(), f)
    }
}

impl<const N: usize> core::fmt::Display for MspString<N> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        core::fmt::Display::fmt(self.as_str(), f)
    }
}

#[cfg(feature = "defmt")]
impl<const N: usize> defmt::Format for MspString<N> {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(f, "{}", self.as_str());
    }
}

impl<'a, const N: usize> TryFrom<&'a str> for MspString<N> {
    type Error = CapacityError;

    fn try_from(s: &'a str) -> Result<Self, Self::Error> {
        Self::from_str(s)
    }
}

impl<const N: usize> core::str::FromStr for MspString<N> {
    type Err = CapacityError;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        Self::from_str(s)
    }
}
