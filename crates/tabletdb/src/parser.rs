// SPDX-License-Identifier: MIT
use crate::*;
use std::path::{Path, PathBuf};

#[derive(Debug)]
pub struct ParserError {
    pub file: Option<PathBuf>,
    pub key: Option<String>,
    pub message: String,
}

macro_rules! parser_error {
    ($msg:expr) => {
        ParserError {
            file: None,
            key: None,
            message: $msg.into(),
        }
    };
    ($key:expr, $msg:expr) => {
        ParserError {
            file: None,
            key: Some($key.into()),
            message: $msg.into(),
        }
    };
    ($file:expr, $key:expr, $msg:expr) => {
        ParserError {
            file: Some(PathBuf::from($file)),
            key: Some($key.into()),
            message: $msg.into(),
        }
    };
}

impl ParserError {
    fn file(self, path: &Path) -> Self {
        ParserError {
            file: Some(PathBuf::from(path)),
            key: self.key,
            message: self.message,
        }
    }
    fn key(self, key: &str) -> Self {
        ParserError {
            file: self.file,
            key: Some(String::from(key)),
            message: self.message,
        }
    }
}

impl std::fmt::Display for ParserError {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::result::Result<(), std::fmt::Error> {
        let msg = &self.message;
        match &self.key {
            Some(key) => write!(
                f,
                "{:?}: Field '{key}': {msg}",
                self.file.as_ref().unwrap_or(&PathBuf::from("<unknown>"))
            ),
            None => write!(f, "{:?}: {msg}", self.file),
        }
    }
}

type Result<T> = std::result::Result<T, ParserError>;

/// A simple implementation of anyhow's Context so
/// we can easily attach a file name to a [ParserError]
trait Context<T> {
    fn file_context(self, context: &Path) -> Result<T>;

    fn section_context(self, section: &str) -> Result<T>;
}

impl<T> Context<T> for Result<T> {
    fn file_context(self, context: &Path) -> Result<T> {
        match self {
            Ok(ok) => Ok(ok),
            Err(err) => Err(err.file(context)),
        }
    }

    fn section_context(self, section: &str) -> Result<T> {
        match self {
            Ok(ok) => Ok(ok),
            Err(err) => Err(err.key(section)),
        }
    }
}

impl From<std::num::ParseIntError> for ParserError {
    fn from(e: std::num::ParseIntError) -> ParserError {
        parser_error!(format!("{e}"))
    }
}

impl From<String> for ParserError {
    fn from(s: String) -> ParserError {
        parser_error!(format!("{s}"))
    }
}

impl From<ParserError> for crate::Error {
    fn from(e: ParserError) -> crate::Error {
        crate::Error::ParserError {
            message: format!("{e}"),
        }
    }
}

fn lookup_evdev_code(s: &str) -> Result<EvdevCode> {
    if s.starts_with("0x") {
        let v = u16::from_str_radix(s.strip_prefix("0x").unwrap_or(s), 16)
            .map_err(|e| parser_error!(format!("{e}")))?;
        Ok(EvdevCode(v))
    } else {
        Ok(EvdevCode(match s {
            "BTN_0" => 0x100,
            "BTN_1" => 0x101,
            "BTN_2" => 0x102,
            "BTN_3" => 0x103,
            "BTN_4" => 0x104,
            "BTN_5" => 0x105,
            "BTN_6" => 0x106,
            "BTN_7" => 0x107,
            "BTN_8" => 0x108,
            "BTN_9" => 0x109,
            "BTN_LEFT" => 0x110,
            "BTN_RIGHT" => 0x111,
            "BTN_MIDDLE" => 0x112,
            "BTN_SIDE" => 0x113,
            "BTN_EXTRA" => 0x114,
            "BTN_FORWARD" => 0x115,
            "BTN_BACK" => 0x116,
            "BTN_TASK" => 0x117,
            "BTN_SOUTH" | "BTN_A" => 0x130,
            "BTN_EAST" | "BTN_B" => 0x131,
            "BTN_C" => 0x132,
            "BTN_NORTH" | "BTN_X" => 0x133,
            "BTN_WEST" | "BTN_Y" => 0x134,
            "BTN_Z" => 0x135,
            "BTN_TL" => 0x136,
            "BTN_TR" => 0x137,
            "BTN_TL2" => 0x138,
            "BTN_TR2" => 0x139,
            "BTN_SELECT" => 0x13a,
            "BTN_START" => 0x13b,
            "BTN_MODE" => 0x13c,
            "BTN_THUMBL" => 0x13d,
            "BTN_THUMBR" => 0x13e,
            _ => {
                return Err(parser_error!(
                    "EvdevCode",
                    format!("Invalid button code {s}")
                ))
            }
        }))
    }
}

#[derive(Debug, Copy, Clone, PartialEq)]
pub enum ToolType {
    Unknown,
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
    /// A mouse-like device
    Puck,
    /// A pen that supports rotation axes in addition
    /// to the typical x/y and tilt
    Pen3D,
    Mobile,
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd)]
pub struct DeviceMatch {
    pub bustype: BusType,
    pub vid: VendorId,
    pub pid: ProductId,
    pub name: Option<String>,
    pub fw: Option<String>,
}

impl DeviceMatch {
    pub(crate) fn to_device_id(&self) -> DeviceId {
        DeviceId {
            bustype: self.bustype,
            vid: self.vid,
            pid: self.pid,
        }
    }
}

impl TryFrom<&str> for DeviceMatch {
    type Error = ParserError;

    fn try_from(s: &str) -> Result<DeviceMatch> {
        if s == "generic" {
            return Ok(DeviceMatch {
                bustype: BusType::Unknown { bustype: 0 },
                vid: VendorId::from(0x00),
                pid: ProductId::from(0x00),
                name: Some(String::from("Generic")),
                fw: None,
            });
        }
        let components: Vec<&str> = s.split("|").collect();
        if components.len() < 3 {
            return Err(parser_error!("DeviceMatch", s));
        }
        let bustype = match components[0] {
            "usb" => BusType::USB,
            "bluetooth" => BusType::Bluetooth,
            "serial" => BusType::Serial,
            "i2c" => BusType::I2C,
            _ => {
                return Err(parser_error!("DeviceMatch", s));
            }
        };

        let vid: VendorId = u16::from_str_radix(components[1], 16)
            .map_err(|_| parser_error!(s))?
            .into();
        let pid: ProductId = u16::from_str_radix(components[2], 16)
            .map_err(|_| parser_error!(s))?
            .into();

        let name: Option<String> = components.get(3).map(|name| String::from(*name));
        let fw: Option<String> = components.get(4).map(|fw| String::from(*fw));

        Ok(DeviceMatch {
            bustype,
            vid,
            pid,
            name,
            fw,
        })
    }
}

#[derive(Debug)]
pub struct TabletEntry {
    pub name: String,
    pub model_name: Option<String>,
    pub device_match: DeviceMatch,
    pub paired_id: Option<DeviceMatch>,
    pub width: usize,
    pub height: usize,
    pub layout: Option<PathBuf>,
    pub integrated_in: Vec<IntegrationFlags>,

    pub stylus: bool,
    pub reversible: bool,
    pub touch: bool,
    pub touchswitch: bool,

    pub buttons: Vec<Button>,
    pub rings: Vec<Ring>,
    pub dials: Vec<Dial>,
    pub strips: Vec<Strip>,

    pub styli: Vec<StylusRef>,
}

pub struct TabletFile {
    pub entries: Vec<TabletEntry>,
}

impl TabletFile {
    pub fn new(path: &Path) -> Result<TabletFile> {
        TabletFile::parse(path).file_context(path)
    }

    fn parse(path: &Path) -> Result<TabletFile> {
        let mut defaults = configparser::ini::IniDefault::default();
        defaults.case_sensitive = true;
        // Default includes ; which messes our data
        defaults.comment_symbols = vec!['#'];
        // Default includes ':' which may mess our data
        defaults.delimiters = vec!['='];
        let mut data = configparser::ini::Ini::new_from_defaults(defaults);
        data.load(path).map_err(|e| parser_error!(e))?;
        Self::parse_data(data, path.parent().unwrap()).map_err(|e| e.file(path))
    }

    fn parse_data(data: configparser::ini::Ini, base_path: &Path) -> Result<TabletFile> {
        // [Device]
        let name: String = data
            .get("Device", "Name")
            .ok_or(parser_error!("Name", "Field is missing"))?;
        let model_name: Option<String> = data.get("Device", "ModelName");
        let device_match: String = data
            .get("Device", "DeviceMatch")
            .ok_or(parser_error!("DeviceMatch", "Field is missing"))?;
        let paired_id: Option<DeviceMatch> = data
            .get("Device", "PairedID")
            .map(|s| DeviceMatch::try_from(s.as_str()).section_context("DeviceMatch"))
            .transpose()?;

        let width: usize = data.get("Device", "Width").map_or(Ok(0usize), |v| {
            str::parse(&v)
                .map_err(|e: std::num::ParseIntError| parser_error!("Width", format!("{e}")))
        })?;
        let height: usize = data.get("Device", "Height").map_or(Ok(0usize), |v| {
            str::parse(&v)
                .map_err(|e: std::num::ParseIntError| parser_error!("Height", format!("{e}")))
        })?;

        let layout: Option<PathBuf> = data.get("Device", "Layout").map(PathBuf::from);
        let layout: Option<PathBuf> = layout.map(|l| {
            vec![base_path, &PathBuf::from("layouts"), &l]
                .into_iter()
                .collect()
        });
        let integrated_in: Vec<IntegrationFlags> = data
            .get("Device", "IntegratedIn")
            .unwrap_or(String::from(""))
            .split(";")
            .filter(|s| !s.is_empty())
            .flat_map(|s| match s {
                "Display" => Some(IntegrationFlags::Display),
                "System" => Some(IntegrationFlags::System),
                "Remote" => Some(IntegrationFlags::Remote),
                _ => None,
            })
            .collect();

        let styli: Vec<StylusRef> = data
            .get("Device", "Styli")
            .map(|s| s.split(";").map(String::from).collect::<Vec<String>>())
            .iter()
            .flatten()
            .filter(|s| !s.is_empty())
            .map(|s| {
                if let Some(s) = s.strip_prefix("@") {
                    Ok(StylusRef::Group(String::from(s)))
                } else {
                    parse_stylus_id(s).map(StylusRef::ID)
                }
            })
            .collect::<Result<Vec<StylusRef>>>()?;

        // Note: Class intentionally omitted

        // [Features]
        let stylus: bool = data
            .getbool("Features", "Stylus")
            .map_err(|e| parser_error!("Stylus", &e))?
            .unwrap_or(false);
        let reversible: bool = data
            .getbool("Features", "Reversible")
            .map_err(|e| parser_error!("Reversible", &e))?
            .unwrap_or(false);
        let touch: bool = data
            .getbool("Features", "Touch")
            .map_err(|e| parser_error!("Touch", &e))?
            .unwrap_or(false);
        let touchswitch: bool = data
            .getbool("Features", "TouchSwitch")
            .map_err(|e| parser_error!("Touchswitch", &e))?
            .unwrap_or(false);

        let device_matches: Vec<DeviceMatch> = device_match
            .split(";")
            .filter(|s| !s.is_empty())
            .map(DeviceMatch::try_from)
            .collect::<Result<Vec<DeviceMatch>>>()?;

        let num_strips: usize = data
            .get("Features", "NumStrips")
            .map_or(Ok(0usize), |v| str::parse(&v))?;
        let num_rings: usize = data
            .get("Features", "NumRings")
            .map_or(Ok(0usize), |v| str::parse(&v))?;
        let num_dials: usize = data
            .get("Features", "NumDials")
            .map_or(Ok(0usize), |v| str::parse(&v))?;

        // [Buttons]

        // Note: OLEDs intentionally omitted

        let evdev_codes: Vec<EvdevCode> = data
            .get("Buttons", "EvdevCode")
            .map(|s| s.split(";").map(String::from).collect::<Vec<String>>())
            .iter()
            .flatten()
            .filter(|s| !s.is_empty())
            .map(|s| lookup_evdev_code(s))
            .collect::<Result<Vec<EvdevCode>>>()?;

        let evdev_codes = if evdev_codes.is_empty() {
            (0x100..=0x117)
                .collect::<Vec<u16>>()
                .iter()
                .map(|i| EvdevCode(*i))
                .collect()
        } else {
            evdev_codes
        };

        // Tablet files use 'A', 'B', etc. but here we have numeric indices because
        // that makes more sense in an API.
        let buttonindex =
            |c: char| -> ButtonIndex { ButtonIndex(std::cmp::min(26, (c as u8 - b'A') as usize)) };

        let mut buttons: Vec<Button> = Vec::new();
        for (key, location) in [
            ("Left", Location::Left),
            ("Right", Location::Right),
            ("Top", Location::Top),
            ("Bottom", Location::Bottom),
        ] {
            if let Some(s) = data.get("Buttons", key) {
                let indices: Vec<ButtonIndex> = s
                    .split(";")
                    .filter(|s| !s.is_empty())
                    .map(|s| buttonindex(s.chars().next().unwrap()))
                    .collect();

                for index in indices {
                    let evdev_code = evdev_codes.get(index.0).ok_or(parser_error!(
                        key,
                        format!("Missing evdev code(s) for buttons {}", index.0)
                    ))?;
                    if buttons.iter().any(|b| b.index == index) {
                        return Err(parser_error!(
                            key,
                            format!("Button {:?} listed in two locations", index)
                        ));
                    }
                    buttons.push(Button {
                        index,
                        location,
                        evdev_code: *evdev_code,
                    })
                }
            }
        }

        let statusleds: Vec<String> = data
            .get("Features", "StatusLEDs")
            .unwrap_or(String::from(""))
            .split(";")
            .filter(|s| !s.is_empty())
            .map(String::from)
            .collect();

        let mut rings: Vec<Ring> = Vec::new();
        let mut strips: Vec<Strip> = Vec::new();
        let mut dials: Vec<Dial> = Vec::new();
        for (count, feature, name, alt_name) in [
            (num_rings, FeatureType::Ring, "Ring", None),
            (num_strips, FeatureType::Strip, "Touchstrip", Some("Strips")),
            (num_dials, FeatureType::Dial, "Dial", None),
        ] {
            for num in 1..=count {
                let prefix = format!("{}{}", name, if num == 1 { "" } else { "2" });
                let key = &prefix;
                let button_idxs: Vec<ButtonIndex> = data
                    .get("Buttons", key)
                    .unwrap_or(String::from(""))
                    .split(";")
                    .filter(|s| !s.is_empty())
                    .map(|s| buttonindex(s.chars().next().unwrap()))
                    .collect();

                let key = format!("{}NumModes", alt_name.unwrap_or(&prefix));
                let num_modes = data
                    .get("Buttons", &key)
                    .map_or(Ok(0usize), |v| str::parse(&v))?;

                let buttons: Vec<ButtonIndex> = button_idxs
                    .into_iter()
                    .filter(|idx| buttons.iter().any(|b| b.index == *idx))
                    .collect();

                let has_status_led = statusleds.iter().any(|l| l == &prefix);
                match feature {
                    FeatureType::Ring => rings.push(Ring {
                        index: num - 1,
                        num_modes,
                        has_status_led,
                        buttons,
                    }),
                    FeatureType::Dial => dials.push(Dial {
                        index: num - 1,
                        num_modes,
                        has_status_led,
                        buttons,
                    }),
                    FeatureType::Strip => strips.push(Strip {
                        index: num - 1,
                        num_modes,
                        has_status_led,
                        buttons,
                    }),
                };
            }
        }

        let mut entries: Vec<TabletEntry> = device_matches
            .iter()
            .map(|m| TabletEntry {
                name: name.clone(),
                model_name: model_name.clone(),
                device_match: m.clone(),
                layout: layout.clone(),
                paired_id: paired_id.clone(),

                width,
                height,

                integrated_in: integrated_in.clone(),

                reversible,
                touch,
                touchswitch,
                stylus,

                buttons: buttons.clone(),
                strips: strips.clone(),
                dials: dials.clone(),
                rings: rings.clone(),

                styli: styli.clone(),
            })
            .collect();

        entries.sort_by_key(|t| {
            format!(
                "{}:{:04x}:{:04x}",
                t.device_match.bustype,
                u16::from(t.device_match.vid),
                u16::from(t.device_match.pid)
            )
        });

        Ok(TabletFile { entries })
    }
}

#[derive(Debug)]
pub struct StylusEntry {
    pub id: StylusId,
    pub name: String,
    pub num_buttons: usize,
    pub group: Option<String>,
    pub eraser_type: Option<EraserType>,
    pub axes: AxisTypes,
    pub tool_type: ToolType,
    pub paired_ids: Option<StylusId>,
}

pub struct StylusFile {
    pub styli: Vec<StylusEntry>,
}

fn parse_stylus_id(s: &str) -> Result<StylusId> {
    let mut ids = s.strip_suffix(";").unwrap_or(s).split(":");
    let pid = ids
        .next()
        .ok_or(parser_error!(s, "Invalid stylus id {section}"))?;
    let (vid, pid) = match ids.next() {
        None => ("0x56a", pid),
        Some(s) => (pid, s),
    };
    let vid = u16::from_str_radix(vid.strip_prefix("0x").unwrap_or(vid), 16)
        .map_err(|e| parser_error!(s, format!("Invalid VID {vid}: {e}").as_str()))?;
    let pid = u32::from_str_radix(pid.strip_prefix("0x").unwrap_or(pid), 16)
        .map_err(|e| parser_error!(s, format!("Invalid PID {pid}: {e}").as_str()))?;
    Ok(StylusId {
        vid: VendorId(vid),
        pid: ToolId(pid),
    })
}

impl StylusFile {
    pub fn new(path: &Path) -> Result<StylusFile> {
        StylusFile::parse(path).file_context(path)
    }

    pub fn parse(path: &Path) -> Result<StylusFile> {
        let mut defaults = configparser::ini::IniDefault::default();
        defaults.case_sensitive = true;
        // Default includes ; which messes our data
        defaults.comment_symbols = vec!['#'];
        // Default includes ':' which may mess our data
        defaults.delimiters = vec!['='];
        let mut data = configparser::ini::Ini::new_from_defaults(defaults);
        data.load(path)?;
        let data = data;

        let mut styli: Vec<StylusEntry> = Vec::new();

        for section in data.sections() {
            let stylus_id = parse_stylus_id(&section)?;
            let name =
                data.get(&section, "Name")
                    .ok_or(parser_error!(path, &section, "Missing Name"))?;
            let num_buttons: usize = data
                .get(&section, "Buttons")
                .map_or(Ok(0), |s| str::parse(&s))
                .map_err(|_| parser_error!(path, &section, format!("Invalid Buttons count")))?;
            let group = data.get(&section, "Group");
            let eraser_type: Option<EraserType> =
                data.get(&section, "EraserType")
                    .map_or(Ok(None), |s| match s.as_str() {
                        "Button" => Ok(Some(EraserType::Button)),
                        "Invert" => Ok(Some(EraserType::Invert)),
                        "None" => Ok(None),
                        _ => Err(parser_error!(
                            path,
                            &section,
                            format!("Invalid eraser type")
                        )),
                    })?;
            let paired_ids: Option<StylusId> = match data.get(&section, "PairedStylusIds") {
                None => None,
                Some(s) => Some(parse_stylus_id(&s)?),
            };
            let tool_type: ToolType = data
                .get(&section, "Type")
                .ok_or(parser_error!(path, &section, "Missing Type"))
                .map(|s| match s.as_str() {
                    "Unknown" => Ok(ToolType::Unknown),
                    "General" => Ok(ToolType::General),
                    "Inking" => Ok(ToolType::Inking),
                    "Airbrush" => Ok(ToolType::Airbrush),
                    "Classic" => Ok(ToolType::Classic),
                    "Marker" => Ok(ToolType::Marker),
                    "Stroke" => Ok(ToolType::Stroke),
                    "Puck" => Ok(ToolType::Puck),
                    "3D" => Ok(ToolType::Pen3D),
                    "Mobile" => Ok(ToolType::Mobile),
                    _ => Err(parser_error!(path, &section, format!("Invalid Type {s}"))),
                })??;
            let mut axes: AxisTypes = data
                .get(&section, "Axes")
                .unwrap_or(String::from(""))
                .split(";")
                .filter(|s| !s.is_empty())
                .collect::<Vec<&str>>()
                .iter()
                .try_fold(AxisTypes::None, |acc, a| match a {
                    &"Pressure" => Ok(acc | AxisTypes::Pressure),
                    &"Distance" => Ok(acc | AxisTypes::Distance),
                    &"Tilt" => Ok(acc | AxisTypes::Tilt),
                    &"Slider" => Ok(acc | AxisTypes::Slider),
                    &"RotationZ" => Ok(acc | AxisTypes::RotationZ),
                    n => Err(parser_error!(
                        path,
                        &section,
                        format!("Axis has invalid value: {n}")
                    )),
                })?;

            if data
                .getbool(&section, "HasLens")
                .map_err(|e| parser_error!(path, "HasLens", &e))?
                .unwrap_or(false)
            {
                axes |= AxisTypes::Lens;
            }
            if data
                .getbool(&section, "HasWheel")
                .map_err(|e| parser_error!(path, "HasWheel", &e))?
                .unwrap_or(false)
            {
                axes |= AxisTypes::Wheel;
            }

            styli.push(StylusEntry {
                id: stylus_id,
                name,
                num_buttons,
                group,
                eraser_type,
                axes,
                tool_type,
                paired_ids,
            });
        }

        Ok(StylusFile { styli })
    }
}
