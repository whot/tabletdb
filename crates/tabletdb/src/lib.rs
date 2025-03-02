// SPDX-License-Identifier: MIT
//! A database of information about graphics tablets
//!
//! It provides **static** information about tablets that cannot be obtained from
//! the kernel device itself and can help with:
//! - checking which axes are actually available on a given tool (the kernel exports all *possible*
//!   axes across all supported tools on a tablet)
//! - checking whether a tablet needs to be mapped to a screen, and helping decide which screen
//! - provides a detailed (SVG) and rough (top/bottom/left/right) layout of the button positions
//!
//! This crate does not affect the functionality of a tablet. It's a wrapper around a set of text
//! files that describe a tablet and never actually looks at the device itself beyond (maybe)
//! extracting the device's name and ids from `/sys`.
//!
//! ## Examples
//!
//! The typical entry point is via a [Cache] or the [Tablet] directly. Using the [Cache]
//! reduces the number of loads from the file system but keeps the database in memory. It is
//! useful for querying multiple tablets in quick order:
//! ```
//! # use tabletdb::{Error, TabletBuilder, Cache};
//! # use std::path::PathBuf;
//! # fn load()  -> Result<(), Error> {
//! // A cache with default include paths
//! let cache = Cache::new()?;
//! for entry in std::fs::read_dir("/dev/input").unwrap().flatten() {
//!     let builder = TabletBuilder::new_from_path(&entry.path())?;
//!     if let Some(tablet) = cache.find_tablet(builder) {
//!         println!("{:?}: {}", entry.path(), tablet.name());
//!     }
//! }
//! # Ok(())
//! # }
//! ```
//! See the [CacheBuilder] for cases where non-default include paths are needed.
//!
//! In most day-to-day use-cases a cache is unnecessary, tablets are unplugged rarely
//! and most users only have a single tablet (if any). Keeping the cache in memory is
//! thus unnecessary. Equivalent to the above is constructing via [TabletBuilder::build()]:
//! ```
//! # use tabletdb::{Error, TabletBuilder, Cache};
//! # use std::path::PathBuf;
//! # fn load()  -> Result<(), Error> {
//! let path = PathBuf::from("/dev/input/event0");
//! let builder = TabletBuilder::new_from_path(&path)?;
//! let tablet = builder.build()?.unwrap();
//! println!("Tablet is called {}", tablet.name());
//! # Ok(())
//! # }
//! ```
//! Once the [Tablet] is constructed, the [Cache] is discarded.
//!
//!
//! ## libwacom
//! This crate is equivalent to [libwacom](https://github.com/linuxwacom/libwacom).
//! Some of libwacom's deprecated APIs in libwacom are not present here, others
//! provide a different structure but the set of information is the same.
//!
//! <div class="warning">
//! This crate currently uses the libwacom data files as data source.  Note that these data files
//! are *not* stable API and updating libwacom may cause this crate to stop working.
//! </div>

use bitflags::bitflags;
use std::collections::HashMap;
use std::io::Read;
use std::path::{Path, PathBuf};
use thiserror::Error;

pub(crate) mod parser;

/// Crate-specific errors
#[derive(Error, Debug)]
pub enum Error {
    #[error("Invalid Argument: {message}")]
    InvalidArgument { message: String },

    #[error("Parsing failed: {message}")]
    ParserError { message: String },

    /// Failed to load required files for the cache.
    ///
    /// This error is raised if all files failed. The [Cache]'s include
    /// paths are optional and empty include directories are
    /// not a cause for failure.
    #[error("Failed to load DB: {message}")]
    IO { message: String },
}

impl From<std::io::Error> for Error {
    fn from(e: std::io::Error) -> Error {
        Error::IO {
            message: format!("{e}"),
        }
    }
}

pub(crate) type Result<T> = std::result::Result<T, Error>;

/// Builder struct for a tablet [Cache].
///
/// This builder is only required for adding non-default include paths,
/// use [Cache::new()] for a tablet cache that uses the defaults.
pub struct CacheBuilder {
    paths: Vec<PathBuf>,
}

impl CacheBuilder {
    /// Create a new, empty, [CacheBuilder] without any include paths.
    /// This builder is only needed where include paths need to be
    /// modified, for standard invocations use [Cache::new()].
    ///
    /// The typical invocation is
    /// ```
    /// # use tabletdb::*;
    /// # fn build() -> Result<(), tabletdb::Error> {
    /// let cache = CacheBuilder::new()
    ///                 .add_default_includes()
    ///                 .build()?;
    /// # Ok(())
    /// # }
    /// ```
    /// The above example is equivalent to [Cache::new()].
    pub fn new() -> Self {
        CacheBuilder { paths: Vec::new() }
    }

    /// Add the default set of include paths as compiled in, typically
    /// `/usr/share/libwacom`, `/etc/libwacom` and `$XDG_CONFIG_HOME/libwacom`.
    pub fn add_default_includes(mut self) -> Self {
        // FIXME: this should come from libwacom-data.pc or something
        // except libwacom explicitly has those tablet files as "not API"
        self.paths.push("/usr/share/libwacom".into());
        self.paths.push("/etc/libwacom".into());
        let xdg_dirs = xdg::BaseDirectories::with_prefix("libwacom").unwrap();
        self.paths.push(xdg_dirs.get_config_home());
        self
    }

    /// Add an include path at the current position. Include paths are
    /// processed in-order with a file in most recent path obscuring
    /// a file with the same name in an earlier include path.
    pub fn add_include(mut self, path: &Path) -> Self {
        self.paths.push(path.into());
        self
    }

    fn find_files(&self, suffix: &str) -> Result<Vec<PathBuf>> {
        let mut files: HashMap<String, PathBuf> = HashMap::new();
        for path in &self.paths {
            std::fs::read_dir(path)?
                .filter_map(|f| f.ok())
                .filter(|f| f.file_type().map(|f| f.is_file()).is_ok())
                .filter(|f| f.file_name().to_string_lossy().ends_with(suffix))
                .for_each(|f| {
                    files.insert(f.file_name().to_string_lossy().to_string(), f.path());
                });
        }

        let files: Vec<PathBuf> = files.into_values().collect();
        Ok(files)
    }

    fn build_tablets(&self) -> Result<Vec<parser::TabletEntry>> {
        let tablets = self
            .find_files(".tablet")?
            .iter()
            .map(|path| parser::TabletFile::new(path))
            .inspect(|res| match res {
                Ok(_) => {}
                Err(e) => log::warn!("{e}"),
            })
            .filter_map(|res| res.ok())
            .flat_map(|tf| tf.entries)
            .collect::<Vec<parser::TabletEntry>>();

        Ok(tablets)
    }

    fn build_styli(&self) -> Result<Vec<parser::StylusEntry>> {
        let styli = self
            .find_files(".stylus")?
            .iter()
            .map(|path| parser::StylusFile::new(path))
            .inspect(|res| match res {
                Ok(_) => {}
                Err(e) => log::warn!("{e}"),
            })
            .filter_map(|res| res.ok())
            .flat_map(|sf| sf.styli)
            .collect::<Vec<parser::StylusEntry>>();

        Ok(styli)
    }

    /// Create a tablet [Cache] from the current include paths.
    pub fn build(self) -> Result<Cache> {
        let mut tablet_entries = self.build_tablets()?;
        let stylus_entries = self.build_styli()?;

        let mut groups: HashMap<String, Vec<StylusId>> = HashMap::new();
        for stylus in &stylus_entries {
            if let Some(stylus_group) = &stylus.group {
                groups
                    .entry(stylus_group.clone())
                    .or_default()
                    .push(stylus.id);
            }
        }

        // Resolve the groups into actual ids
        for tablet in &mut tablet_entries {
            tablet.styli = tablet
                .styli
                .iter_mut()
                .flat_map(|r| match r {
                    StylusRef::ID(id) => vec![StylusRef::ID(*id)],
                    StylusRef::Group(group) => groups
                        .get(group)
                        .unwrap_or(&Vec::<StylusId>::new())
                        .iter()
                        .map(|id: &StylusId| StylusRef::ID(*id))
                        .collect(),
                })
                .collect();
        }

        let tablets = tablet_entries
            .into_iter()
            .enumerate()
            .map(|(idx, t)| Tablet {
                idx,
                name: t.name,
                model_name: t.model_name,
                kernel_name: t.device_match.name,
                fw_version: t.device_match.fw,
                layout: t.layout.map(PathBuf::from),

                width: Length { inches: t.width },
                height: Length { inches: t.height },

                integration_flags: t.integrated_in,

                paired_id: t.paired_id.map(|m| m.to_device_id()),

                bustype: t.device_match.bustype,
                vid: t.device_match.vid,
                pid: t.device_match.pid,

                is_reversible: t.reversible,
                has_touch: t.touch,
                has_touchswitch: t.touchswitch,
                has_stylus: t.stylus,

                buttons: t
                    .buttons
                    .iter()
                    .map(|b| crate::Button {
                        index: crate::ButtonIndex(b.index.0),
                        location: b.location,
                        evdev_code: crate::EvdevCode(b.evdev_code.0),
                    })
                    .collect(),
                rings: t.rings,
                strips: t.strips,
                dials: t.dials,

                styli: t
                    .styli
                    .into_iter()
                    .map(|r| match r {
                        StylusRef::ID(id) => id,
                        _ => panic!("Invalid stylus ref, should've been resolved"),
                    })
                    .collect::<Vec<StylusId>>(),

                owned_styli: Vec::new(),
                marker: std::marker::PhantomData,
            })
            .collect();

        let styli: Vec<Stylus> = stylus_entries
            .into_iter()
            .enumerate()
            .map(|(idx, s)| Stylus {
                idx,
                id: s.id,
                stylus_type: s.stylus_type,
                eraser_type: s.eraser_type,
                axes: s.axes,
                name: s.name,
                num_buttons: s.num_buttons,
                paired_id: s.paired_ids,
            })
            .collect();

        // FIXME:
        // Styli:
        // - resolve PairedIds and remove where the respective one doesnt exist

        Ok(Cache { tablets, styli })
    }
}

impl Default for CacheBuilder {
    fn default() -> Self {
        Self::new()
    }
}

/// The bustype of this device
#[derive(Debug, Copy, Clone, PartialEq, PartialOrd, Eq, Ord)]
pub enum BusType {
    /// A bustype currently unsupported by this crate
    Unknown {
        bustype: u16,
    },
    USB,
    Bluetooth,
    Serial,
    I2C,
}

impl std::fmt::Display for BusType {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(
            f,
            "{}",
            match self {
                BusType::USB => "usb",
                BusType::Bluetooth => "bluetooth",
                BusType::Serial => "serial",
                BusType::I2C => "i2c",
                BusType::Unknown { bustype: _ } => "unknown",
            }
        )
    }
}

impl From<u16> for BusType {
    fn from(b: u16) -> BusType {
        match b {
            0x3 => BusType::USB,
            0x5 => BusType::Bluetooth,
            0x11 => BusType::Serial,
            0x18 => BusType::I2C,
            _ => BusType::Unknown { bustype: b },
        }
    }
}

/// A 16-bit Vendor ID
#[derive(Debug, Copy, Clone, PartialEq, PartialOrd, Eq, Ord)]
pub struct VendorId(u16);

impl std::fmt::LowerHex for VendorId {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        let val = self.0;
        std::fmt::LowerHex::fmt(&val, f)
    }
}

impl std::fmt::UpperHex for VendorId {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        let val = self.0;
        std::fmt::LowerHex::fmt(&val, f)
    }
}

impl From<VendorId> for u16 {
    fn from(vid: VendorId) -> u16 {
        vid.0
    }
}

impl From<&VendorId> for u16 {
    fn from(vid: &VendorId) -> u16 {
        vid.0
    }
}

impl From<u16> for VendorId {
    fn from(vid: u16) -> VendorId {
        VendorId(vid)
    }
}

/// A 16-bit Product ID
#[derive(Debug, Copy, Clone, PartialEq, PartialOrd, Eq, Ord)]
pub struct ProductId(pub u16);

impl std::fmt::LowerHex for ProductId {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        let val = self.0;
        std::fmt::LowerHex::fmt(&val, f)
    }
}

impl std::fmt::UpperHex for ProductId {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        let val = self.0;
        std::fmt::LowerHex::fmt(&val, f)
    }
}

impl From<ProductId> for u16 {
    fn from(pid: ProductId) -> u16 {
        pid.0
    }
}

impl From<&ProductId> for u16 {
    fn from(pid: &ProductId) -> u16 {
        pid.0
    }
}

impl From<u16> for ProductId {
    fn from(pid: u16) -> ProductId {
        ProductId(pid)
    }
}

/// A 32-bit Stylus ID
#[derive(Debug, Copy, Clone, PartialEq, PartialOrd, Eq, Ord)]
pub struct ToolId(pub u32);

impl std::fmt::LowerHex for ToolId {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        let val = self.0;
        std::fmt::LowerHex::fmt(&val, f)
    }
}

impl std::fmt::UpperHex for ToolId {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        let val = self.0;
        std::fmt::LowerHex::fmt(&val, f)
    }
}

impl From<ToolId> for u32 {
    fn from(pid: ToolId) -> u32 {
        pid.0
    }
}

impl From<&ToolId> for u32 {
    fn from(pid: &ToolId) -> u32 {
        pid.0
    }
}

/// A USB device ID comprising a vendor and tool id
#[derive(Debug, Copy, Clone, PartialEq, PartialOrd, Eq, Ord)]
pub struct DeviceId {
    bustype: BusType,
    vid: VendorId,
    pid: ProductId,
}

impl DeviceId {
    pub fn bus_type(&self) -> BusType {
        self.bustype
    }

    pub fn vendor_id(&self) -> VendorId {
        self.vid
    }

    pub fn product_id(&self) -> ProductId {
        self.pid
    }
}

/// A stylus ID comprising a vendor and tool id
///
/// Unlike a [DeviceId] a stylus ID may have a 32-bit
/// [ToolId].
#[derive(Debug, Copy, Clone, PartialEq, PartialOrd, Eq, Ord)]
pub struct StylusId {
    vid: VendorId,
    pid: ToolId,
}

impl StylusId {
    pub fn vendor_id(&self) -> VendorId {
        self.vid
    }

    pub fn tool_id(&self) -> ToolId {
        self.pid
    }
}

/// Helper trait to convert between crazy units and
/// sensible ones.
pub trait Units {
    fn inches(&self) -> f32;

    fn cm(&self) -> f32 {
        self.inches() * 2.54
    }
    fn mm(&self) -> f32 {
        self.inches() * 25.4
    }
}

/// A physical length
#[derive(Debug, Copy, Clone, PartialEq, Eq, PartialOrd)]
pub struct Length {
    inches: usize,
}

impl Units for Length {
    fn inches(&self) -> f32 {
        self.inches as f32
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq, PartialOrd)]
pub enum IntegrationFlags {
    /// The tablet is integrated into a display, e.g.
    /// a Wacom Cintiq.
    Display,
    /// The tablet is integrated into a system, e.g.
    /// a built-in tablet in a laptop.
    ///
    /// This flag does not usually exist on its own
    /// and is set together with [Display](IntegrationFlags::Display).
    System,
    
    /// The tablet is an external remote like the
    /// Wacom ExpressKey Remote.
    Remote,
}

/// A cache of all tablets known to this crate at the
/// time of building the cache.
///
/// Since the overwhelmingly vast majority of users do
/// not have a tablet and of the remaining portion the
/// overwhelmingly vast majority of users only have one
/// tablet that is plugged in once and hardly ever removed
/// using a tablet cache is not always necessary. It may
/// be more efficient to simply create a [Tablet] when a
/// new device is detected.
#[derive(Debug, Clone)]
pub struct Cache {
    tablets: Vec<Tablet<BorrowedTablet>>,
    styli: Vec<Stylus>,
}

impl Cache {
    /// Create a new tablet cache with default include paths.
    /// This is a shortcut to avoid using the [CacheBuilder]
    /// for cases where modifying the include paths is not required.
    pub fn new() -> Result<Self> {
        CacheBuilder::new().add_default_includes().build()
    }

    /// Returns an iterator wrapper over the
    /// tablets available.
    ///
    /// This is equivalent to `libwacom_list_devices_from_database()`.
    pub fn tablets(&self) -> impl Iterator<Item = &Tablet<BorrowedTablet>> {
        self.tablets.iter()
    }

    pub fn styli(&self) -> Styli {
        Styli {
            styli: &self.styli,
            index: 0
        }
    }

    /// Find the first tablet matching all conditions in the builder.
    ///
    /// If multiple tablets match the conditions, the first one is returned. It
    /// is the responsibility of the caller to provide sufficient information
    /// in the [TabletBuilder] to identify the tablet uniquely.
    pub fn find_tablet(&self, builder: TabletBuilder) -> Option<&Tablet<BorrowedTablet>> {
        let matches: Vec<&Tablet<BorrowedTablet>> = self
            .tablets()
            .filter(|t| builder.bustype.is_none() || t.bustype == builder.bustype.unwrap())
            .filter(|t| builder.vid.is_none() || t.vid == builder.vid.unwrap())
            .filter(|t| builder.pid.is_none() || t.pid == builder.pid.unwrap())
            .filter(|t| {
                builder.package_name.is_none() || &t.name == builder.package_name.as_ref().unwrap()
            })
            .filter(|t| {
                builder.kernel_name.is_none()
                    || t.kernel_name.is_none()
                    || t.kernel_name == builder.kernel_name
            })
            .collect();

        matches.first().map(|t| &**t)
    }

    /// Same as [find_tablet()][Cache::find_tablet] but discards the
    /// cache and returns the [Tablet] to the caller.
    ///
    /// If the tablet cannot be found, the [Cache] is returned as error.
    pub fn take_tablet(
        self,
        builder: TabletBuilder,
    ) -> std::result::Result<Tablet<OwnedTablet>, Self> {
        let tablet = self.find_tablet(builder);
        if tablet.is_none() {
            return Err(self);
        }
        let tablet = tablet.unwrap();
        let tablet = self
            .tablets()
            .into_iter()
            .find(|&t| t.idx == tablet.idx)
            .unwrap()
            .clone();

        let styli = self
            .styli
            .into_iter()
            .filter(|s| tablet.styli().any(|&r| r == s.id))
            .collect::<Vec<Stylus>>();

        let tablet = tablet.take_styli(styli);

        Ok(tablet)
    }

}

/// Builder for a tablet
///
/// Used to specify the bits the tablet must match
///
/// The most convenient way to match an existing physical device is to use
/// [TabletBuilder::new_from_path()].
#[derive(Debug)]
pub struct TabletBuilder {
    bustype: Option<BusType>,
    vid: Option<VendorId>,
    pid: Option<ProductId>,
    package_name: Option<String>,
    kernel_name: Option<String>,
    uniq: Option<String>,
}

impl TabletBuilder {
    pub fn new() -> TabletBuilder {
        TabletBuilder {
            bustype: None,
            pid: None,
            vid: None,
            package_name: None,
            kernel_name: None,
            uniq: None,
        }
    }

    /// Build a [TabletBuilder] from the device at the given path. Supports:
    /// - `/dev/input/event0` evdev event nodes
    /// - `/sys/devices/.../input/input0` sysfs input paths
    pub fn new_from_path(path: &Path) -> Result<TabletBuilder> {
        let pathbuf = PathBuf::from(path);
        // FIXME: not the nicest approach...
        let sysfs = if pathbuf.to_string_lossy().starts_with("/dev/input/event") {
            let node = pathbuf.file_name().ok_or(Error::InvalidArgument {
                message: format!("Invalid path {pathbuf:?}"),
            })?;
            PathBuf::from("/sys/class/input")
                .join(node)
                .join(String::from("device"))
        } else if pathbuf.starts_with("/sys") && pathbuf.as_path().is_dir() {
            if pathbuf.join("id").as_path().is_dir() {
                pathbuf
            } else {
                pathbuf.join("device")
            }
        } else {
            return Err(Error::InvalidArgument {
                message: format!("Don't know how to handle {pathbuf:?}"),
            });
        };

        let mut bustype = String::new();
        std::fs::File::open(sysfs.join("id").join("bustype"))?.read_to_string(&mut bustype)?;
        let mut vid = String::new();
        std::fs::File::open(sysfs.join("id").join("vendor"))?.read_to_string(&mut vid)?;
        let mut pid = String::new();
        std::fs::File::open(sysfs.join("id").join("product"))?.read_to_string(&mut pid)?;
        let mut name = String::new();
        std::fs::File::open(sysfs.join("name"))?.read_to_string(&mut name)?;
        let mut uniq = String::new();
        std::fs::File::open(sysfs.join("name"))?.read_to_string(&mut uniq)?;

        let bustype = u16::from_str_radix(bustype.as_str().trim(), 16).map_err(|_| Error::IO {
            message: format!("Failed to parse bustype {bustype}"),
        })?;
        let vid = u16::from_str_radix(vid.as_str().trim(), 16).map_err(|_| Error::IO {
            message: format!("Failed to parse vendor {vid}"),
        })?;
        let pid = u16::from_str_radix(pid.as_str().trim(), 16).map_err(|_| Error::IO {
            message: format!("Failed to parse pid {pid}"),
        })?;

        Ok(TabletBuilder::new()
            .bustype(bustype.into())
            .vid(vid.into())
            .pid(pid.into())
            .kernel_name(name.trim().into())
            .uniq(uniq.trim().into()))
    }

    /// The tablet must match this bustype
    pub fn bustype(mut self, bustype: BusType) -> Self {
        self.bustype = Some(bustype);
        self
    }
    /// The tablet must match this VID
    pub fn vid(mut self, vid: VendorId) -> Self {
        self.vid = Some(vid);
        self
    }
    /// The tablet must match this PID
    pub fn pid(mut self, pid: ProductId) -> Self {
        self.pid = Some(pid);
        self
    }
    /// The tablet must match this name from its package.
    ///
    /// Many Huion, XP-Pen, etc. devices differ in the package name
    /// and the name announced by the firmware to the kernel, e.g.
    /// XP-Pen devices are typically sold as "XPPen Foo" but the
    /// firmware labels them as "UGEE Bar".
    ///
    /// The package name is the one specified in the tablet data
    /// files. There is rarely a need to specify this name unless you want
    /// to load a specific data file. Use [kernel_name()](Self::kernel_name)
    /// instead.
    ///
    /// Even where the device announces itself correctly,
    /// do not use this together with [kernel_name()](Self::kernel_name),
    /// the kernel's name usually has the type appended (e.g. "Pen")
    /// and specifying both the name and match name typically
    /// results in no match.
    pub fn package_name(mut self, name: String) -> Self {
        self.package_name = Some(name);
        self
    }
    /// The tablet must match this kernel name.
    ///
    /// Do not use this together with [package_name()](Self::package_name),
    /// the kernel's name usually has the type appended (e.g. "Pen")
    /// and specifying both the name and match name typically
    /// results in no match.
    pub fn kernel_name(mut self, name: String) -> Self {
        self.kernel_name = Some(name);
        self
    }
    /// The tablet must match the UNIQ string given
    pub fn uniq(mut self, uniq: String) -> Self {
        if !uniq.is_empty() {
            self.uniq = Some(uniq);
        }
        self
    }

    /// Return the tablet (if any) that matches the given
    /// requirements.
    ///
    /// This function constructs a [Cache] and discards it
    /// immediately, even if no tablet is found.
    pub fn build(self) -> Result<Option<Tablet<OwnedTablet>>> {
        let cache = Cache::new()?;
        Ok(cache.take_tablet(self).ok())
    }
}

impl Default for TabletBuilder {
    fn default() -> Self {
        Self::new()
    }
}

/// An evdev code of `EV_KEY` type (e.g. `BTN_0`, etc.)
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct EvdevCode(u16);

impl EvdevCode {
    pub fn code(&self) -> u16 {
        self.0
    }
}

impl From<&EvdevCode> for u16 {
    fn from(e: &EvdevCode) -> u16 {
        e.0
    }
}

/// A button index on a tablet
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct ButtonIndex(usize);

impl ButtonIndex {
    pub fn number(&self) -> usize {
        self.0
    }
}

/// A button on a tablet.
///
/// This is a button on the "pad" portion
/// of the tablet, not a button on a stylus.
#[derive(Debug, Clone, PartialEq)]
pub struct Button {
    index: ButtonIndex,
    location: Location,
    evdev_code: EvdevCode,
}

impl Button {
    pub fn index(&self) -> ButtonIndex {
        self.index
    }

    pub fn location(&self) -> Location {
        self.location
    }

    pub fn evdev_code(&self) -> EvdevCode {
        self.evdev_code
    }
}

/// The physical location of a button relative to the
/// input area.
///
/// This location is only an approximate guide and may be used to provide some
/// grouping of buttons in a UI.
#[derive(Debug, Copy, Clone, PartialEq)]
pub enum Location {
    Left,
    Right,
    Top,
    Bottom,
}

/// The type of an extra feature
#[derive(Debug, Clone, PartialEq)]
pub enum FeatureType {
    /// A ring providing absolute finger position, see e.g. the Wacom Intuos Pro series
    Ring,
    /// A touch strip, see e.g. the Wacom Intuos 3
    Strip,
    /// A relative dial, may physically be mouse-wheel-like or ring-like, see
    /// e.g. the Huion Inspiroy 2S or the Huion Inspiroy Dial 2
    Dial,
}

pub trait Feature {
    /// The number of hardware modes expected by this feature.
    ///
    /// This is usually tied to the number of LEDs that will cycle
    /// as the feature cycles through modes but some devices
    /// expect modes but do not have LEDs to represent the current mode.
    ///
    /// Devices that do not require modes return 0, it is up
    /// to the implementation to implement modes anyway.
    fn num_modes(&self) -> usize;

    fn feature_type(&self) -> FeatureType;

    /// 0 for the first ring/strip/dial, 1 for the second one, etc.
    fn index(&self) -> usize;

    /// `true` if this feature has a status LED indicating the current mode
    /// it is in. For devices where this returns false but [num_modes()](Self::num_modes)
    /// returns nonzero a virtual display of the current mode is required.
    fn has_status_led(&self) -> bool;

    /// The buttons associated with this feature, e.g. the button in
    /// the center of a ring. This button is the mode-switch button
    /// and pressing it should change into the next mode (if one button)
    /// or a specific mode (if multiple buttons). If this returns
    /// an empty iterator, no button is directly associated with
    /// this feature.
    fn buttons(&self) -> impl Iterator<Item = &ButtonIndex>;
}

/// A ring providing absolute finger position, see e.g. the Wacom Intuos Pro series
#[derive(Debug, Clone, PartialEq)]
pub struct Ring {
    index: usize,
    num_modes: usize,
    has_status_led: bool,
    buttons: Vec<ButtonIndex>,
}

impl Feature for Ring {
    fn feature_type(&self) -> FeatureType {
        FeatureType::Ring
    }

    fn num_modes(&self) -> usize {
        self.num_modes
    }

    fn index(&self) -> usize {
        self.index
    }

    fn has_status_led(&self) -> bool {
        self.has_status_led
    }

    fn buttons(&self) -> impl Iterator<Item = &ButtonIndex> {
        self.buttons.iter()
    }
}

/// A relative dial
///
/// A relative dial may physically be mouse-wheel-like or ring-like, see
/// e.g. the Huion Inspiroy 2S or the Huion Inspiroy Dial 2
#[derive(Debug, Clone, PartialEq)]
pub struct Dial {
    index: usize,
    num_modes: usize,
    has_status_led: bool,
    buttons: Vec<ButtonIndex>,
}

impl Feature for Dial {
    fn feature_type(&self) -> FeatureType {
        FeatureType::Dial
    }

    fn num_modes(&self) -> usize {
        self.num_modes
    }

    fn index(&self) -> usize {
        self.index
    }

    fn has_status_led(&self) -> bool {
        self.has_status_led
    }

    fn buttons(&self) -> impl Iterator<Item = &ButtonIndex> {
        self.buttons.iter()
    }
}

/// A touch strip, see e.g. the Wacom Intuos 3
#[derive(Debug, Clone, PartialEq)]
pub struct Strip {
    index: usize,
    num_modes: usize,
    has_status_led: bool,
    buttons: Vec<ButtonIndex>,
}

impl Feature for Strip {
    fn feature_type(&self) -> FeatureType {
        FeatureType::Strip
    }

    fn num_modes(&self) -> usize {
        self.num_modes
    }

    fn index(&self) -> usize {
        self.index
    }

    fn has_status_led(&self) -> bool {
        self.has_status_led
    }

    fn buttons(&self) -> impl Iterator<Item = &ButtonIndex> {
        self.buttons.iter()
    }
}

#[derive(Debug, Clone)]
enum StylusRef {
    Group(String),
    ID(StylusId),
}

/// Trait for denoting whether a [Tablet] is owned or borrowed
/// from the [Cache].
///
/// A borrowed tablet does not have access to the [Stylus] directly,
/// and the cache is required to retrieve the [Stylus].
pub trait TabletOwnership {}

#[doc(hidden)]
pub enum OwnedTablet {}

#[doc(hidden)]
#[derive(Debug, Clone)]
pub enum BorrowedTablet {}

impl TabletOwnership for OwnedTablet {}
impl TabletOwnership for BorrowedTablet {}

/// Static information about a tablet.
#[derive(Debug, Clone, PartialEq)]
pub struct Tablet<O: TabletOwnership> {
    idx: usize,
    bustype: BusType,
    vid: VendorId,
    pid: ProductId,
    name: String,
    model_name: Option<String>,

    // As used in the DeviceMatch
    kernel_name: Option<String>,
    fw_version: Option<String>,
    layout: Option<PathBuf>,

    paired_id: Option<DeviceId>,

    width: Length,
    height: Length,

    is_reversible: bool,
    has_stylus: bool,
    has_touch: bool,
    has_touchswitch: bool,

    integration_flags: Vec<IntegrationFlags>,

    buttons: Vec<Button>,
    rings: Vec<Ring>,
    dials: Vec<Dial>,
    strips: Vec<Strip>,

    styli: Vec<StylusId>,
    owned_styli: Vec<Stylus>,

    marker: std::marker::PhantomData<O>,
}

impl<O: TabletOwnership> Tablet<O> {
    /// The "official" name of the device.
    /// This is the name given to the device by the
    /// vendor's marketing material and is
    /// suitable for presentation. This name may not
    /// be the name that the tablet's firmware uses.
    ///
    /// For example, the "Huion Inspiroy 2S" has a [kernel_name()](Self::kernel_name()) of
    /// "HUION Huion Tablet_H641P"
    pub fn name(&self) -> &str {
        self.name.as_str()
    }
    /// A (typically) alpha-numeric model name that serves
    /// to identify a model where the [name()](Self::name) is ambiguous
    /// and used across multiple generations of devices.
    ///
    /// For example, the "Wacom Intuos Pro M" has a model name of
    /// "PTH-660".
    pub fn model_name(&self) -> Option<&str> {
        self.model_name.as_deref()
    }
    /// The kernel name used by this device, if any.
    /// This name may differ from [name()](Self::name) and
    /// is used to identify the device in the system.
    ///
    /// For example, the "Huion Inspiroy 2S" has a [kernel_name()](Self::kernel_name()) of
    /// "HUION Huion Tablet_H641P"
    pub fn kernel_name(&self) -> Option<&str> {
        self.kernel_name.as_deref()
    }

    /// The firmware version string, if any.
    pub fn firmware_version(&self) -> Option<&str> {
        self.fw_version.as_deref()
    }

    /// An optional path to an SVG file that represents the
    /// layout for this device.
    pub fn layout(&self) -> Option<&Path> {
        self.layout.as_deref()
    }
    pub fn bustype(&self) -> BusType {
        self.bustype
    }
    pub fn vendor_id(&self) -> VendorId {
        self.vid
    }
    pub fn product_id(&self) -> ProductId {
        self.pid
    }
    /// The physical width of the device as advertised on
    /// its marketing material.
    ///
    /// This may refer to the input
    /// area or the whole device, depending on the vendor.
    /// This value should only be used for presentation or
    /// identification, not calculation of input data.
    pub fn width(&self) -> Length {
        self.width
    }
    /// The physical height of the device as advertised on
    /// its marketing material.
    ///
    /// This may refer to the input
    /// area or the whole device, depending on the vendor.
    /// This value should only be used for presentation or
    /// identification, not calculation of input data.
    pub fn height(&self) -> Length {
        self.height
    }

    /// The [DeviceId] of the paired device, if any.
    ///
    /// The paired device is a device that shares the same physical
    /// device with the tablet, e.g. a touchscreen integrated into
    /// a graphics tablet.
    pub fn paired_id(&self) -> Option<DeviceId> {
        self.paired_id
    }

    /// `true` if the device is reversible, i.e. rotating the physical tablet
    /// by 180 degrees moves the button layout to the respective other side.
    /// This flag is typically used to determine if the tablet may be used
    /// in left-handed or right-handed mode.
    ///
    /// Tablets with the buttons at the top are typically **not** reversible,
    /// the button layout in the tablet's natural rotation is equally accessible for left- and for
    /// right-handed users.
    pub fn is_reversible(&self) -> bool {
        self.is_reversible
    }
    pub fn integration_flags(&self) -> &[IntegrationFlags] {
        &self.integration_flags
    }
    pub fn supports_stylus(&self) -> bool {
        self.has_stylus
    }
    pub fn supports_touch(&self) -> bool {
        self.has_touch
    }
    /// The tablet has a switch to disable touch.
    pub fn has_touchswitch(&self) -> bool {
        self.has_touchswitch
    }
    pub fn buttons(&self) -> impl Iterator<Item = &Button> {
        self.buttons.iter()
    }

    pub fn find_button(&self, index: &ButtonIndex) -> Option<&Button> {
        self.buttons.iter().find(|b| b.index == *index)
    }

    pub fn rings(&self) -> impl Iterator<Item = &Ring> {
        self.rings.iter()
    }

    pub fn dials(&self) -> impl Iterator<Item = &Dial> {
        self.dials.iter()
    }

    pub fn strips(&self) -> impl Iterator<Item = &Strip> {
        self.strips.iter()
    }

    // Create a fallback tablet device based on the information
    // provided in the builder.
    //
    // This fallback tablet device can be used in lieu of a
    // known tablet device, e.g. where the device is too new
    // and not yet known to the tablet database.
    //
    // Creating a fallback device does not construct a [Cache].
    pub fn new_fallback(builder: TabletBuilder) -> Tablet<O> {
        todo!();
    }
}

pub struct Styli<'a> {
    styli: &'a Vec<Stylus>,
    index: usize,
}

impl<'a> Iterator for Styli<'a> {
    type Item = &'a Stylus;

    fn next(&mut self) -> Option<Self::Item> {
        self.index += 1;
        return self.styli.get(self.index - 1);
    }
}

impl Tablet<BorrowedTablet> {
    /// The list of stylus ids supported by this device. Use
    /// [Cache::find_stylus()] to retrieve the [Stylus].
    pub fn styli(&self) -> impl Iterator<Item = &StylusId> {
        self.styli.iter()
    }

    fn take_styli(self, styli: Vec<Stylus>) -> Tablet<OwnedTablet> {
        Tablet {
            idx: self.idx,
            bustype: self.bustype,
            vid: self.vid,
            pid: self.pid,
            name: self.name,
            model_name: self.model_name,
            kernel_name: self.kernel_name,
            fw_version: self.fw_version,
            width: self.width,
            height: self.height,
            integration_flags: self.integration_flags,
            layout: self.layout,
            paired_id: self.paired_id,

            is_reversible: self.is_reversible,
            has_touch: self.has_touch,
            has_touchswitch: self.has_touchswitch,
            has_stylus: self.has_stylus,
            buttons: self.buttons,
            rings: self.rings,
            dials: self.dials,
            strips: self.strips,

            styli: Vec::new(),
            owned_styli: styli,

            marker: std::marker::PhantomData,
        }
    }
}

impl Tablet<OwnedTablet> {
    /// The list of styli supported by this device.
    pub fn styli(&self) -> impl Iterator<Item = &Stylus> {
        self.owned_styli.iter()
    }
}

#[derive(Debug, Copy, Clone, PartialEq)]
pub enum StylusType {
    Unknown,
    General,
    Inking,
    Airbrush,
    Classic,
    Marker,
    Stroke,
    Puck,
    Pen3D,
    Mobile,
}

/// Type of eraser on a stylus
#[derive(Debug, Copy, Clone, PartialEq)]
pub enum EraserType {
    Unknown,
    /// No eraser is present on the stylus
    None,
    /// Eraser is a separate tool on the opposite end of the stylus
    Invert,
    /// Eraser is a button alongside any other stylus buttons
    Button,
}

bitflags! {
    #[derive(Debug, Copy, Clone, PartialEq)]
    pub(crate) struct AxisTypes: u32 {
        const None = 0;
        const Pressure = 1 << 0;
        const Tilt = 1 << 1;
        const Distance = 1 << 2;
        const Slider = 1 << 3;
        const RotationZ = 1 << 4;
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct Stylus {
    idx: usize,
    name: String,
    id: StylusId,
    stylus_type: StylusType,
    eraser_type: EraserType,
    num_buttons: usize,
    axes: AxisTypes,
    paired_id: Option<StylusId>,
}

impl Stylus {
    pub fn name(&self) -> &str {
        &self.name
    }
    pub fn vendor_id(&self) -> VendorId {
        self.id.vid
    }
    pub fn tool_id(&self) -> ToolId {
        self.id.pid
    }
    pub fn stylus_type(&self) -> StylusType {
        self.stylus_type
    }
    pub fn eraser_type(&self) -> EraserType {
        self.eraser_type
    }
    pub fn num_buttons(&self) -> usize {
        self.num_buttons
    }
    pub fn has_tilt(&self) -> bool {
        self.axes.contains(AxisTypes::Tilt)
    }
    pub fn has_pressure(&self) -> bool {
        self.axes.contains(AxisTypes::Pressure)
    }
    pub fn has_distance(&self) -> bool {
        self.axes.contains(AxisTypes::Distance)
    }
    pub fn has_rotation(&self) -> bool {
        self.axes.contains(AxisTypes::RotationZ)
    }
    pub fn has_slider(&self) -> bool {
        self.axes.contains(AxisTypes::Slider)
    }
    pub fn paired_id(&self) -> Option<StylusId> {
        self.paired_id
    }
}
