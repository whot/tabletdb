// SPDX-License-Identifier: MIT
//! A database of information about graphics tablets.
//!
//! This crate provides **static** information about graphics tablets (Wacom, Huion, XP-Pen, Ugee,
//! etc.) that cannot be obtained from the kernel device itself such as:
//! - is the given tablet integrated into a display or a system (i.e. a laptop display)
//! - which axes and buttons are available on a given tool. The kernel exports *all possible*
//!   axes across *all possible* tools on a tablet.
//! - which tools are available on a given tablet
//! - obtaining a detailed (SVG) and rough (top/bottom/left/right) layout of the button positions
//!
//! A physical tablet is not required for querying information about tablet.
//! This crate does not affect the functionality of a tablet. It's a wrapper around a set of text
//! files that describe a tablet and never actually looks at the device itself beyond (maybe)
//! extracting the device's name and ids from `/sys`.
//!
//! This crate is tablet-vendor-agnostic and (in theory) platform-agnostic. Support for platforms
//! other than Linux is untested.
//!
//! ## Tablets, Styli, Buttons, Rings, Dials, Strips
//!
//! A tablet as seen by this crate refers to the physical tablet that is connected to the host via
//! one of the supported [BusTypes](BusType).
//!
//! Many tablets have [Buttons](Button) and almost all tablets support [Styli](Stylus), the notable
//! exception being those resembling a [Remote](IntegrationFlags::Remote), e.g. the Wacom
//! ExpressKey Remote.
//!
//! A tablet may also have additional [Features](Feature):
//! - a [Ring] is a ring-shaped feature providing absolute finger position, see e.g. the Wacom
//!   Intuos Pro series
//! - a [Dial] is a dial or wheel-shaped feature providing relative input data, see
//!   e.g. the Huion Inspiroy 2S or the Huion Inspiroy Dial 2
//! - a touch [Strip] strip providing absolute finger position, see e.g. the Wacom Intuos 3.
//!
//! A tablet may have more than one feature (e.g. two rings) and more than one feature type (e.g. a
//! ring and two strips).
//!
//! ## Modes and Mode Toggles
//!
//! Features may be logically associated with a [Button]. For example the Wacom Intuos Pro
//! series has one button inside the [Ring]. This button is physically independent
//! of the ring but often associated with the "modes" of the ring (and other buttons).
//! This button is referred to as "mode toggle button".
//!
//! Such a mode may allow assigning different actions to the feature depending on the current mode.
//! This crate only provides information about which button is logically associated
//! with the feature (see [Feature::buttons()]) and the number of modes expected on this tablet
//! ([Feature::num_modes()]).
//!
//! On Linux, the modes are typically tied to the LEDs on the device and pressing the
//! button associated with the feature will iterate and/or toggle the respective LED.
//!
//! Some tablets have more than one mode toggle button - on those tablets each button
//! is expected to switch to one specific mode. On tablets with one mode switch button
//! the button is typically expected to cycle modes.
//!
//! ## Examples
//!
//! The most common use is to query information about a tablet that exists locally:
//! ```
//! # use tabletdb::{Error, TabletInfo, Cache};
//! # use std::path::PathBuf;
//! # fn load()  -> Result<(), Error> {
//! // A cache with default include paths
//! let cache = Cache::new()?;
//! for entry in std::fs::read_dir("/dev/input").unwrap().flatten() {
//!     // Extract the information from a local tablet
//!     let info = TabletInfo::new_from_path(&entry.path())?;
//!     // Find that tablet in the cache
//!     for tablet in cache.iter().filter(|t| *t == &info) {
//!         println!("{:?}: {}", entry.path(), tablet.name());
//!     }
//! }
//! # Ok(())
//! # }
//! ```
//! But it's also possible to simply filter on other information if no device is present:
//! ```
//! # use tabletdb::{Error, BusType, TabletInfo, Cache};
//! # use std::path::PathBuf;
//! # fn load()  -> Result<(), Error> {
//! // A cache with default include paths
//! let cache = Cache::new()?;
//! for tablet in cache.iter().filter(|t| t.bustype() == BusType::Bluetooth) {
//!         println!("{} is a supported Bluetooth tablet", tablet.name());
//! }
//! # Ok(())
//! # }
//! ```
//!
//! See the [CacheBuilder] for cases where non-default include paths are needed.
//!
//! ## Relationship to libwacom
//! This crate aims to be equivalent to [libwacom](https://github.com/linuxwacom/libwacom).
//! Some of libwacom's deprecated APIs are not present here, others
//! provide a different structure but the set of information is the same.
//!
//! <div class="warning">
//! This crate currently uses the libwacom data files as data source.  Note that these data files
//! are <em>not</em> stable API and updating libwacom may cause this crate to stop working. We aim
//! to remove this dependency in future versions.
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
    /// This info is only needed where include paths need to be
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

    /// Add the default set of include paths as compiled in, typically:
    /// - `/usr/share/libwacom`,
    /// - `/etc/libwacom`
    /// - `$XDG_CONFIG_HOME/libwacom`.
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
    ///
    /// Include paths to not need to exist, empty or non-existing include
    /// paths are ignored.
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

        let tools: Vec<Tool> = stylus_entries
            .iter()
            .enumerate()
            // Filter out any tools with non-existing paired ids
            .filter(|(_, entry)| {
                entry
                    .paired_ids
                    .map(|id| stylus_entries.iter().any(|p| p.id == id))
                    .unwrap_or(true)
            })
            // Filter any invert erasers, we map those directl
            .filter(|(_, entry)| entry.eraser_type != Some(EraserType::Invert))
            .map(|(idx, s)| {
                if s.tool_type == parser::ToolType::Puck {
                    Ok(Tool::Mouse(Mouse {
                        idx,
                        id: s.id,
                        axes: s.axes,
                        name: s.name.clone(),
                        num_buttons: s.num_buttons,
                    }))
                } else {
                    let st = match s.tool_type {
                        parser::ToolType::General => StylusType::General,
                        parser::ToolType::Inking => StylusType::Inking,
                        parser::ToolType::Airbrush => StylusType::Airbrush,
                        parser::ToolType::Classic => StylusType::Classic,
                        parser::ToolType::Marker => StylusType::Marker,
                        parser::ToolType::Stroke => StylusType::Stroke,
                        parser::ToolType::Pen3D => StylusType::Pen3D,
                        parser::ToolType::Mobile => StylusType::Mobile,
                        _ => {
                            return Err(Error::ParserError {
                                message: format!("Unsupported tool type {:?}", s.tool_type),
                            })
                        }
                    };
                    let stylus = Stylus {
                        idx,
                        id: s.id,
                        stylus_type: st,
                        eraser_type: s.eraser_type,
                        axes: s.axes,
                        name: s.name.clone(),
                        num_buttons: s.num_buttons,
                    };
                    // This is really bad in the data files. A tool
                    // without a paired ID is always a stylus or an
                    // eraser but only the eraser has the EraserType
                    // set.
                    if s.eraser_type.is_none_or(|t| t == EraserType::Button) {
                        Ok(Tool::Stylus(stylus))
                    } else {
                        let paired_id = s.paired_ids.unwrap();
                        let eraser: Eraser = stylus_entries
                            .iter()
                            .filter(|e| e.id == paired_id)
                            .map(|e| Eraser {
                                idx,
                                id: e.id,
                                eraser_type: e.eraser_type.unwrap(),
                                axes: e.axes,
                                name: e.name.clone(),
                            })
                            .take(1)
                            .next()
                            .unwrap();
                        Ok(Tool::StylusWithEraser(stylus, eraser))
                    }
                }
            })
            .collect::<Result<Vec<Tool>>>()?;

        let tablets = tablet_entries
            .into_iter()
            .enumerate()
            .map(|(idx, t)| Tablet {
                idx,
                name: t.name,
                model_name: t.model_name,
                kernel_name: t.device_match.name,
                fw_version: t.device_match.fw,
                layout: t.layout,

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

                tools: tools
                    .iter()
                    .filter(|tool| {
                        t.styli.iter().any(|id| match id {
                            StylusRef::ID(id) => {
                                tool.vendor_id() == id.vendor_id() && tool.tool_id() == id.tool_id()
                            }
                            _ => false,
                        })
                    })
                    .cloned()
                    .collect::<Vec<Tool>>(),
            })
            .collect();

        Ok(Cache { tablets, tools })
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
    /// A bustype currently unsupported by this crate.
    ///
    /// This enum value must never be used for anything but debug printing.
    /// A future version of this crate will support the unknown bus type
    /// as separate enum value.
    Unknown {
        /// One of the `BUS_*` defines in `linux/input.h`.
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

/// Conversion into this BusType from one of the
/// `BUS_*` defines in `linux/input.h`.
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

impl std::ops::Deref for VendorId {
    type Target = u16;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl std::fmt::LowerHex for VendorId {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        let val = self.0;
        std::fmt::LowerHex::fmt(&val, f)
    }
}

impl std::fmt::UpperHex for VendorId {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        let val = self.0;
        std::fmt::UpperHex::fmt(&val, f)
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

impl std::ops::Deref for ProductId {
    type Target = u16;
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl std::fmt::LowerHex for ProductId {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        let val = self.0;
        std::fmt::LowerHex::fmt(&val, f)
    }
}

impl std::fmt::UpperHex for ProductId {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        let val = self.0;
        std::fmt::UpperHex::fmt(&val, f)
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

/// A 32-bit Stylus ID.
///
/// This ID can identify the type of a stylus but
/// is only supported by some tablets.
///
/// Typically styli that support a [ToolId] are also
/// uniquely identifiable at runtime via a unique serial number.
#[derive(Debug, Copy, Clone, PartialEq, PartialOrd, Eq, Ord)]
pub struct ToolId(pub u32);

impl std::ops::Deref for ToolId {
    type Target = u32;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl std::fmt::LowerHex for ToolId {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        let val = self.0;
        std::fmt::LowerHex::fmt(&val, f)
    }
}

impl std::fmt::UpperHex for ToolId {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        let val = self.0;
        std::fmt::UpperHex::fmt(&val, f)
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

/// A USB device ID comprising a bus type, vendor id and product id
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

/// A physical length, e.g. the Tablet's [width](Tablet::width) or [height](Tablet::height).
///
/// See the [Units] trait to access the length.
#[derive(Debug, Copy, Clone, PartialEq, Eq, PartialOrd)]
pub struct Length {
    inches: usize,
}

impl Units for Length {
    fn inches(&self) -> f32 {
        self.inches as f32
    }
}

/// Specifies how the tablet is integrated.
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
/// The overwhelmingly vast majority of users do
/// not have a tablet and of the remaining portion the
/// overwhelmingly vast majority of users only have one
/// tablet that is plugged in once and hardly ever removed.
/// Keeping the tablet cache around is not always necessary. It may
/// be more efficient to simply get the tablet and drop the
/// rest of the cache when a new device is detected.
///
/// ```
/// # use tabletdb::{Error, Tablet, TabletInfo, Cache};
/// # use std::path::Path;
/// # fn load(device_path: &Path)  -> Result<(), Error> {
/// // A cache with default include paths
/// let cache = Cache::new()?;
/// let info = TabletInfo::new_from_path(device_path)?;
/// let tablets: Vec<Tablet> = cache
///     .into_iter()
///     .filter(|t| t == &info)
///     .collect::<Vec<Tablet>>();
/// let tablet: &Tablet = tablets.first().unwrap();
/// # Ok(())
/// # }
/// ```
#[derive(Debug, Clone)]
pub struct Cache {
    tablets: Vec<Tablet>,
    tools: Vec<Tool>,
}

impl Cache {
    /// Create a new tablet cache with default include paths.
    /// This is a shortcut to avoid using the [CacheBuilder]
    /// for cases where modifying the include paths is not required.
    pub fn new() -> Result<Self> {
        CacheBuilder::new().add_default_includes().build()
    }

    /// Returns an iterator over all known tablets.
    ///
    /// Identical to [tablets()](Self::tablets).
    pub fn iter(&self) -> impl Iterator<Item = &Tablet> {
        self.tablets()
    }

    /// Returns an iterator over all known tablets.
    ///
    /// Identical to [iter()](Self::iter).
    pub fn tablets(&self) -> impl Iterator<Item = &Tablet> {
        self.tablets.iter()
    }

    /// Returns an iterator over all known styli.
    pub fn tools(&self) -> impl Iterator<Item = &Tool> {
        self.tools.iter()
    }
}

impl IntoIterator for Cache {
    type Item = Tablet;
    type IntoIter = std::vec::IntoIter<Self::Item>;

    fn into_iter(self) -> Self::IntoIter {
        self.tablets.into_iter()
    }
}

/// Info about a tablet.
///
/// This struct simplifies matching against a tablet in the [Cache].
/// The most convenient way to match an existing physical device is to use
/// [TabletInfo::new_from_path()].
#[derive(Debug)]
pub struct TabletInfo {
    bustype: Option<BusType>,
    vid: Option<VendorId>,
    pid: Option<ProductId>,
    name: Option<String>,
    kernel_name: Option<String>,
    uniq: Option<String>,
}

impl TabletInfo {
    pub fn new() -> TabletInfo {
        TabletInfo {
            bustype: None,
            pid: None,
            vid: None,
            name: None,
            kernel_name: None,
            uniq: None,
        }
    }

    /// Build a [TabletInfo] from the device at the given path. Supports:
    /// - `/dev/input/event0` evdev event nodes
    /// - `/sys/devices/.../input/input0` sysfs input paths
    pub fn new_from_path(path: &Path) -> Result<TabletInfo> {
        let sysfs = if path.starts_with("/dev/input")
            && path.is_file()
            && path
                .file_name()
                .unwrap()
                .to_string_lossy()
                .starts_with("event")
        {
            let node = path.file_name().ok_or(Error::InvalidArgument {
                message: format!("Invalid path {path:?}"),
            })?;
            PathBuf::from("/sys/class/input")
                .join(node)
                .join(String::from("device"))
        } else if path.starts_with("/sys") && path.is_dir() {
            let pathbuf = PathBuf::from(path);
            if pathbuf.join("id").as_path().is_dir() {
                pathbuf
            } else {
                pathbuf.join("device")
            }
        } else {
            return Err(Error::InvalidArgument {
                message: format!("Don't know how to handle {path:?}"),
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
        std::fs::File::open(sysfs.join("uniq"))?.read_to_string(&mut uniq)?;

        let bustype = u16::from_str_radix(bustype.as_str().trim(), 16).map_err(|_| Error::IO {
            message: format!("Failed to parse bustype {bustype}"),
        })?;
        let vid = u16::from_str_radix(vid.as_str().trim(), 16).map_err(|_| Error::IO {
            message: format!("Failed to parse vendor {vid}"),
        })?;
        let pid = u16::from_str_radix(pid.as_str().trim(), 16).map_err(|_| Error::IO {
            message: format!("Failed to parse pid {pid}"),
        })?;

        Ok(TabletInfo::new()
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
    /// The tablet must match this name given by the vendor.
    ///
    /// <div class="warning">
    /// This is almost never the name to use, <em>kernel_name</em> instead.
    /// </div>
    ///
    /// Many Huion, XP-Pen, etc. devices differ in the official name
    /// and the name announced by the firmware to the kernel, e.g.
    /// XP-Pen devices are typically sold as "XP-Pen Foo" but the
    /// firmware (and thus the kernel) labels them as "UGEE Bar".
    ///
    /// The [name()](Self::name) is the one specified in the tablet data
    /// files. There is rarely a need to specify this name unless you want
    /// to load a specific data file. Use [kernel_name()](Self::kernel_name)
    /// instead.
    ///
    /// Even where the device announces itself correctly,
    /// do not use this together with [kernel_name()](Self::kernel_name),
    /// the kernel's name usually has the type appended (e.g. "Pen")
    /// and specifying both the name and match name typically
    /// results in no match.
    pub fn name(mut self, name: String) -> Self {
        self.name = Some(name);
        self
    }
    /// The tablet must match this kernel name.
    ///
    /// Do not use this together with [name()](Self::name),
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
}

impl Default for TabletInfo {
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

/// A zero-based button index on a tablet
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
    /// The zero-based index of this button
    pub fn index(&self) -> ButtonIndex {
        self.index
    }

    /// An approximate location of this button. This location may be used to
    /// group buttons together in a UI or group buttons together
    /// with one button is the mode switch button.
    pub fn location(&self) -> Location {
        self.location
    }

    /// The code for this button
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

/// Static information about a tablet.
#[derive(Debug, Clone, PartialEq)]
// pub struct Tablet<O: TabletOwnership> {
pub struct Tablet {
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

    tools: Vec<Tool>,
}

impl Tablet {
    /// The "official" name of the device.
    /// This is the name given to the device by the
    /// vendor's marketing material and is
    /// suitable for presentation. This name may not
    /// be the name that the tablet's firmware uses and
    /// thus differ from the [kernel_name()](Self::kernel_name()).
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

    /// The firmware version string prefix, if any.
    ///
    /// This prefix is used to identify the a device further
    /// where [vendor_id()](Self::vendor_id), [product_id()](Self::product_id)
    /// and the device's [kernel_name()](Self::kernel_name) are not
    /// sufficient to identify the tablet. This is the case for
    /// e.g. many (most?) Huion devices. The firmware string is
    /// a prefix only, e.g. the real firmware version on Huion devices
    /// is typically suffixed with a date.
    ///
    /// This is **not** the tablet's current firmware version, this
    /// data is not queried from the device.
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

    /// Returns an iterator over the styli available on this tablet.
    pub fn tools(&self) -> impl Iterator<Item = &Tool> {
        self.tools.iter()
    }

    /// Match a tablet against the info.
    ///
    /// Any field set in the [TabletInfo] is matched against the
    /// respective field in the [Tablet] - fields unset are ignored.
    /// The caller is required to set sufficient fields to get a
    /// unique match, e.g. only setting the [TabletInfo::bustype()] will
    /// match against any device with that bustype.
    pub fn matches(&self, info: &TabletInfo) -> bool {
        (info.bustype.is_none() || self.bustype == info.bustype.unwrap())
            && (info.vid.is_none() || self.vid == info.vid.unwrap())
            && (info.pid.is_none() || self.pid == info.pid.unwrap())
            && (info.name.is_none() || &self.name == info.name.as_ref().unwrap())
            && (info.kernel_name.is_none()
                || self.kernel_name.is_none()
                || self.kernel_name == info.kernel_name)
    }
}

impl PartialEq<TabletInfo> for Tablet {
    fn eq(&self, other: &TabletInfo) -> bool {
        self.matches(other)
    }
}

/// An approximate description of the stylus type
///
/// This type may be used to e.g. present differnent icons
/// of a stylus depending on a type.
#[derive(Debug, Copy, Clone, PartialEq)]
pub enum StylusType {
    /// A standard pen-like stylus
    General,
    Inking,
    /// A tool that is used like an airbrush (i.e. it may
    /// never touch the surface). Airbrush tools may have a
    /// slider.
    Airbrush,
    Classic,
    Marker,
    Stroke,
    /// A pen that supports rotation axes in addition
    /// to the typical x/y and tilt
    Pen3D,
    Mobile,
}

/// Type of eraser on a stylus
#[derive(Debug, Copy, Clone, PartialEq)]
enum EraserType {
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
        const Lens = 1 << 5;
        const Wheel = 1 << 6;
    }
}

pub trait ToolFeatures {
    /// The name of this tool given by the manufacturer,
    /// e.g. "Grip Pen". This name is suitable for presentation.
    ///
    /// This name may not be unique.
    fn name(&self) -> &str;
    fn vendor_id(&self) -> VendorId;
    fn tool_id(&self) -> ToolId;

    fn has_tilt(&self) -> bool;
    fn has_pressure(&self) -> bool;
    fn has_distance(&self) -> bool;
    fn has_rotation(&self) -> bool;
    fn has_slider(&self) -> bool;
}

/// A physical tool that may be used on a device.
///
/// This is a rough grouping only, more information about each tool may be available
/// in the contained struct.
#[derive(Debug, Clone, PartialEq)]
pub enum Tool {
    /// A stylus-like tool without an eraser at the other entry.
    ///
    /// Such a stylus may have an eraser button, see [Stylus::has_eraser_button()],
    /// and *behave* like an eraser when that button is pressed. However, that
    /// eraser does not appear as separate tool in the system.
    Stylus(Stylus),
    /// A stylus-like tool with an eraser at the back.
    ///
    /// Such a stylus must not have an [eraser button](Stylus::has_eraser_button())
    /// and inverting the stylus will present the [Eraser] tool. This tool may
    /// have a different [VendorId] and [ToolId] to the [Stylus].
    StylusWithEraser(Stylus, Eraser),
    /// A mouse-like device
    ///
    /// These tools are of primarily historical interest, they are no longer
    /// for sale.
    Mouse(Mouse),
}

impl Tool {
    /// A convenience method wrapping [Stylus::name()] for this tool,
    /// ignoring the eraser (if any)
    pub fn name(&self) -> &str {
        match self {
            Tool::Stylus(s) => s.name(),
            Tool::StylusWithEraser(_, e) => e.name(),
            Tool::Mouse(m) => m.name(),
        }
    }
    /// A convenience method wrapping [Stylus::vendor_id()] for this tool,
    /// ignoring the eraser (if any)
    pub fn vendor_id(&self) -> VendorId {
        match self {
            Tool::Stylus(s) => s.vendor_id(),
            Tool::StylusWithEraser(_, e) => e.vendor_id(),
            Tool::Mouse(m) => m.vendor_id(),
        }
    }
    /// A convenience method wrapping [Stylus::tool_id()] for this tool,
    /// ignoring the eraser (if any)
    pub fn tool_id(&self) -> ToolId {
        match self {
            Tool::Stylus(s) => s.tool_id(),
            Tool::StylusWithEraser(_, e) => e.tool_id(),
            Tool::Mouse(m) => m.tool_id(),
        }
    }
}

/// A mouse-like device
///
/// These tools are of primarily historical interest, they are no longer
/// for sale.
#[derive(Debug, Clone, PartialEq)]
pub struct Mouse {
    idx: usize,
    name: String,
    id: StylusId,
    axes: AxisTypes,
    num_buttons: usize,
}

impl ToolFeatures for Mouse {
    /// The name of this eraser given by the manufacturer,
    /// e.g. "Grip Pen". This name is suitable for presentation.
    ///
    /// This name may not be unique. Using this name is not
    /// usually required, the eraser does not live without
    /// its corresponding stylus and the stylus name is a better
    /// option for presentation.
    fn name(&self) -> &str {
        &self.name
    }
    /// The vendor ID of the eraser. Theoretically this may
    /// be different to the [Stylus::vendor_id()] but no
    /// such devices have been observed in the wild yet.
    fn vendor_id(&self) -> VendorId {
        self.id.vid
    }
    /// The tool ID of the eraser.
    fn tool_id(&self) -> ToolId {
        self.id.pid
    }
    fn has_tilt(&self) -> bool {
        self.axes.contains(AxisTypes::Tilt)
    }
    fn has_pressure(&self) -> bool {
        self.axes.contains(AxisTypes::Pressure)
    }
    fn has_distance(&self) -> bool {
        self.axes.contains(AxisTypes::Distance)
    }
    fn has_rotation(&self) -> bool {
        self.axes.contains(AxisTypes::RotationZ)
    }
    fn has_slider(&self) -> bool {
        self.axes.contains(AxisTypes::Slider)
    }
}

impl Mouse {
    pub fn has_lens(&self) -> bool {
        self.axes.contains(AxisTypes::Lens)
    }
    pub fn has_wheel(&self) -> bool {
        self.axes.contains(AxisTypes::Wheel)
    }
    pub fn num_buttons(&self) -> usize {
        self.num_buttons
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct Eraser {
    idx: usize,
    name: String,
    id: StylusId,
    eraser_type: EraserType,
    axes: AxisTypes,
}

impl ToolFeatures for Eraser {
    /// The name of this eraser given by the manufacturer,
    /// e.g. "Grip Pen". This name is suitable for presentation.
    ///
    /// This name may not be unique. Using this name is not
    /// usually required, the eraser does not live without
    /// its corresponding stylus and the stylus name is a better
    /// option for presentation.
    fn name(&self) -> &str {
        &self.name
    }
    /// The vendor ID of the eraser. Theoretically this may
    /// be different to the [Stylus::vendor_id()] but no
    /// such devices have been observed in the wild yet.
    fn vendor_id(&self) -> VendorId {
        self.id.vid
    }
    /// The tool ID of the eraser.
    fn tool_id(&self) -> ToolId {
        self.id.pid
    }
    fn has_tilt(&self) -> bool {
        self.axes.contains(AxisTypes::Tilt)
    }
    fn has_pressure(&self) -> bool {
        self.axes.contains(AxisTypes::Pressure)
    }
    fn has_distance(&self) -> bool {
        self.axes.contains(AxisTypes::Distance)
    }
    fn has_rotation(&self) -> bool {
        self.axes.contains(AxisTypes::RotationZ)
    }
    fn has_slider(&self) -> bool {
        self.axes.contains(AxisTypes::Slider)
    }
}

/// A stylus describes one tool available on a tablet.
///
/// Not all [Stylus] tools are physically styli-like, e.g. the Wacom Lens Cursor
/// is a mouse-like device.
#[derive(Debug, Clone, PartialEq)]
pub struct Stylus {
    idx: usize,
    name: String,
    id: StylusId,
    stylus_type: StylusType,
    eraser_type: Option<EraserType>,
    num_buttons: usize,
    axes: AxisTypes,
}

impl ToolFeatures for Stylus {
    /// The name of this stylus given by the manufacturer,
    /// e.g. "Grip Pen". This name is suitable for presentation.
    ///
    /// This name may not be unique.
    fn name(&self) -> &str {
        &self.name
    }
    fn vendor_id(&self) -> VendorId {
        self.id.vid
    }
    fn tool_id(&self) -> ToolId {
        self.id.pid
    }
    fn has_tilt(&self) -> bool {
        self.axes.contains(AxisTypes::Tilt)
    }
    fn has_pressure(&self) -> bool {
        self.axes.contains(AxisTypes::Pressure)
    }
    fn has_distance(&self) -> bool {
        self.axes.contains(AxisTypes::Distance)
    }
    fn has_rotation(&self) -> bool {
        self.axes.contains(AxisTypes::RotationZ)
    }
    fn has_slider(&self) -> bool {
        self.axes.contains(AxisTypes::Slider)
    }
}

impl Stylus {
    /// Returns true if one of the buttons on the stylus triggers
    /// eraser behaviour. See the
    /// [Windows Pen States](https://learn.microsoft.com/en-us/windows-hardware/design/component-guidelines/windows-pen-states)
    /// for details.
    pub fn has_eraser_button(&self) -> bool {
        !self.eraser_type.is_none_or(|t| t != EraserType::Button)
    }
    /// The number of buttons on this Stylus
    ///
    /// Note that many styli have an [eraser button](Stylus::has_eraser_button)
    /// that the firmware enforces eraser-like behavior for,
    /// see the
    /// [Windows Pen States](https://learn.microsoft.com/en-us/windows-hardware/design/component-guidelines/windows-pen-states)
    /// for details.
    /// This button is excluded from the number of buttons.
    pub fn num_buttons(&self) -> usize {
        self.num_buttons
    }

    pub fn stylus_type(&self) -> StylusType {
        self.stylus_type
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hex_format() {
        let vid = VendorId(0x12ab);
        assert_eq!(format!("{vid:x}"), "12ab");
        assert_eq!(format!("{vid:X}"), "12AB");
        let pid = ProductId(0xbc12);
        assert_eq!(format!("{pid:x}"), "bc12");
        assert_eq!(format!("{pid:X}"), "BC12");
        let tid = ToolId(0xd12e);
        assert_eq!(format!("{tid:x}"), "d12e");
        assert_eq!(format!("{tid:X}"), "D12E");
    }
}
