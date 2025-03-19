use anyhow::Result;
use clap::{ColorChoice, Parser, Subcommand};
use std::collections::HashMap;
use std::path::PathBuf;
use std::process::ExitCode;
use tabletdb::{
    Axis, BusType, Button, Dial, Feature, Location, ProductId, Ring, Strip, Tablet, TabletInfo,
    Tool, ToolFeatures, Units, VendorId,
};

#[derive(Clone, Debug)]
struct Hex(u16);

impl std::str::FromStr for Hex {
    type Err = String;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        u16::from_str_radix(s.trim_start_matches("0x"), 16)
            .map_err(|e| format!("{e}"))
            .map(Hex)
    }
}

#[derive(Clone, Debug)]
struct DeviceMatch {
    bustype: String,
    vid: Hex,
    pid: Hex,
    name: Option<String>,
    uniq: Option<String>,
}

impl std::str::FromStr for DeviceMatch {
    type Err = String;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        let components: Vec<&str> = s.split("|").collect();
        if components.len() < 3 {
            return Err(format!("{s} is not a valid DeviceMatch"));
        }

        let bustype = components.first().map(|s| String::from(*s)).unwrap();
        if ["usb", "bluetooth", "i2c", "serial"]
            .iter()
            .all(|bt| *bt != bustype)
        {
            return Err(format!("Unsupported bus type {bustype}"));
        }

        let vid = components.get(1).map(|s| Hex::from_str(s)).unwrap()?;
        let pid = components.get(2).map(|s| Hex::from_str(s)).unwrap()?;
        let name = components
            .get(3)
            .filter(|s| !s.is_empty())
            .map(|s| String::from(*s));
        let uniq = components
            .get(4)
            .filter(|s| !s.is_empty())
            .map(|s| String::from(*s));

        Ok(DeviceMatch {
            bustype,
            vid,
            pid,
            name,
            uniq,
        })
    }
}

#[derive(Parser, Debug)]
#[command(version, about, long_about = None)]
struct Cli {
    // Print debugging information
    #[arg(short, long, default_value_t = false)]
    verbose: bool,

    // Use color
    #[arg(long, value_enum, default_value_t = ColorChoice::Auto)]
    color: ColorChoice,

    // Print debugging information
    #[arg(short, long, default_value_t = false)]
    debug: bool,

    #[command(subcommand)]
    command: Commands,
}

#[derive(Subcommand, Debug)]
enum Commands {
    /// List all supported tablets
    ListTablets,
    /// List all supported styli
    ListStyli,
    /// List supported local tablets
    ListLocalDevices,

    /// List information about one device
    Info {
        /// The path to a device
        #[arg(long)]
        device: Option<String>,

        /// The bus type
        #[arg(long,
             value_parser = clap::builder::PossibleValuesParser::new(
                ["usb", "bluetooth", "i2c", "serial"]
            ),
        )]
        bustype: Option<String>,

        /// The vendor id (overrides path and device-match)
        #[arg(long, value_parser = clap::value_parser!(Hex))]
        vendor_id: Option<Hex>,

        /// The product id (overrides path and device-match)
        #[arg(long, value_parser = clap::value_parser!(Hex))]
        product_id: Option<Hex>,

        /// The kernel name (overrides path and device-match)
        #[arg(long)]
        kernel_name: Option<String>,

        /// The uniq value (overrides path and device-match)
        #[arg(long)]
        uniq: Option<String>,

        /// A libwacom data-file compatible match
        #[arg(long, value_parser = clap::value_parser!(DeviceMatch))]
        device_match: Option<DeviceMatch>,
    },
}

fn cmd_list_tablets() -> Result<()> {
    let cache = tabletdb::Cache::new()?;
    println!("devices:");

    let mut tablets: Vec<&Tablet> = cache.tablets().collect();
    tablets.sort_by_key(|t| {
        format!(
            "{}:{:04x}:{:04x}",
            t.bustype(),
            t.vendor_id(),
            t.product_id()
        )
    });

    let namelen = tablets
        .iter()
        .max_by_key(|t| t.name().len())
        .map(|t| t.name().len())
        .unwrap_or(0);

    for tablet in tablets {
        println!(
            "- {{ bus: {:<11} vid: '0x{:04x}', pid: '0x{:04x}', name: '{}'{} }}",
            format!("'{}',", tablet.bustype()),
            *tablet.vendor_id(),
            tablet.product_id(),
            tablet.name(),
            tablet
                .firmware_version()
                .map(|fw| format!(
                    ", {:<width$}uniq: '{fw}'",
                    ' ',
                    width = namelen - tablet.name().len()
                ))
                .unwrap_or("".into())
        );
    }

    Ok(())
}

fn cmd_list_styli() -> Result<()> {
    let cache = tabletdb::Cache::new()?;

    println!("styli:");
    let mut tools: Vec<&Tool> = cache.tools().collect();
    tools.sort_by_key(|s| format!("{:04x}:{:08x}", s.vendor_id(), s.tool_id()));

    let namelen = tools
        .iter()
        .max_by_key(|t| t.name().len())
        .map(|t| t.name().len())
        .unwrap_or(0);

    for tool in tools.iter() {
        let name = format!(
            "name: '{}',{:width$}",
            tool.name(),
            ' ',
            width = 1 + namelen - tool.name().len()
        );

        let mut components = Vec::new();
        components.push(format!("vid: '0x{:04x}'", tool.vendor_id()));
        components.push(format!("pid: '0x{:08x}'", tool.tool_id()));
        let axes = tool
            .axes()
            .iter()
            .map(|a| match a {
                Axis::Pressure => "p",
                Axis::Distance => "d",
                Axis::Tilt => "t",
                Axis::RotationZ => "r",
                Axis::Wheel => "w",
                Axis::Slider => "s",
            })
            .collect::<Vec<&str>>();
        components.push(format!("axes: '{:width$}'", axes.join(""), width = 4));
        components.push(format!("buttons: {}", tool.buttons().len()));

        match tool {
            Tool::Mouse(m) => {
                if m.has_lens() {
                    components.push(format!("lens: {}", m.has_lens()))
                }
            }
            Tool::Stylus(s) => {
                components.push(format!("type: {}", s.stylus_type()));
                if s.has_eraser_button() {
                    components.push(format!("eraser-button: {}", s.has_eraser_button()));
                }
            }
            Tool::StylusWithEraser(s, e) => {
                components.push(format!("type: {}", s.stylus_type()));
                components.push(format!("eraser: {{ pid: 0x{:08x} }}", e.tool_id()));
            }
        }

        println!("- {{ {name} {} }}", components.join(", "));
    }

    Ok(())
}

fn cmd_info(
    path: Option<String>,
    bustype: Option<String>,
    vendor_id: Option<Hex>,
    product_id: Option<Hex>,
    kernel_name: Option<String>,
    uniq: Option<String>,
    device_match: Option<DeviceMatch>,
) -> Result<()> {
    let info: TabletInfo = if let Some(path) = &path {
        TabletInfo::new_from_path(&PathBuf::from(path))?
    } else {
        TabletInfo::new()
    };

    let bustype = bustype.or(device_match.clone().map(|m| m.bustype));
    let info = if let Some(bustype) = bustype {
        let bt = match bustype.as_str() {
            "usb" => BusType::USB,
            "bluetooth" => BusType::Bluetooth,
            "i2c" => BusType::I2C,
            "serial" => BusType::Serial,
            _ => panic!("Unsupported bus type {bustype}"),
        };
        info.bustype(bt)
    } else {
        info
    };

    let vendor_id = vendor_id.or(device_match.clone().map(|m| m.vid));
    let info = if let Some(vendor_id) = vendor_id {
        info.vid(VendorId::from(vendor_id.0))
    } else {
        info
    };

    let product_id = product_id.or(device_match.clone().map(|m| m.pid));
    let info = if let Some(product_id) = product_id.or(device_match.clone().map(|m| m.pid)) {
        info.pid(ProductId::from(product_id.0))
    } else {
        info
    };

    let kernel_name = kernel_name.or(device_match.clone().and_then(|m| m.name));
    let info = if let Some(name) = kernel_name {
        info.kernel_name(name)
    } else {
        info
    };

    let uniq = uniq.or(device_match.clone().and_then(|m| m.uniq));
    let info = if let Some(uniq) = uniq {
        info.uniq(uniq)
    } else {
        info
    };

    let cache = tabletdb::Cache::new()?;
    let mut tablets = cache.tablets().filter(|t| *t == &info).peekable();
    if tablets.peek().is_none() {
        println!("Not a known tablet device");
        return Ok(());
    }
    for tablet in tablets {
        println!("device:");
        println!("  name: \"{}\"", tablet.name());
        println!("  model_name: \"{}\"", tablet.model_name().unwrap_or(""));
        println!("  kernel_name: \"{}\"", tablet.kernel_name().unwrap_or(""));
        println!(
            "  firmware_version: \"{}\"",
            tablet.firmware_version().unwrap_or("")
        );
        println!(
            "  layout: \"{}\"",
            tablet
                .layout()
                .map(|l| String::from(l.to_string_lossy()))
                .unwrap_or(String::from(""))
        );
        println!(
            "  bus: \"{}\"",
            match tablet.bustype() {
                BusType::USB => "usb",
                BusType::Bluetooth => "bluetooth",
                BusType::Serial => "serial",
                BusType::I2C => "i2c",
            }
        );
        println!("  vid: \"0x{:04x}\"", tablet.vendor_id());
        println!("  pid: \"0x{:04x}\"", tablet.product_id());
        println!("  width: {}", tablet.width().inches());
        println!("  height: {}", tablet.height().inches());
        println!("  reversible: {}", tablet.is_reversible());
        println!("  stylus: {}", tablet.supports_stylus());
        println!("  touch: {}", tablet.supports_touch());
        println!("  touchswitch: {}", tablet.has_touchswitch());

        let buttons: Vec<&Button> = tablet.buttons().collect();
        if !buttons.is_empty() {
            println!("  buttons:");
            for button in buttons {
                println!(
                    "    - {{ idx: {}, location: \"{}\", evdev: \"0x{:x}\" }}",
                    button.index().number(),
                    match button.location() {
                        Location::Left => "left",
                        Location::Right => "right",
                        Location::Top => "top",
                        Location::Bottom => "bottom",
                    },
                    button.evdev_code().code()
                );
            }
        }

        fn print_feature(f: &impl Feature) {
            println!(
                "    - {{ idx: {}, modes: {}, has_status_led: {}, buttons: [{}] }}",
                f.index(),
                f.num_modes(),
                f.has_status_led(),
                f.buttons()
                    .map(|idx| format!("{}", idx.number()))
                    .collect::<Vec<String>>()
                    .join(", ")
            )
        }

        let rings: Vec<&Ring> = tablet.rings().collect();
        if !rings.is_empty() {
            println!("  rings:");
            for ring in rings {
                print_feature(ring);
            }
        }
        let dials: Vec<&Dial> = tablet.dials().collect();
        if !dials.is_empty() {
            println!("  dials:");
            for dial in dials {
                print_feature(dial);
            }
        }
        let strips: Vec<&Strip> = tablet.strips().collect();
        if !strips.is_empty() {
            println!("  strips:");
            for strip in strips {
                print_feature(strip);
            }
        }

        if tablet.supports_stylus() {
            println!("  styli:");
            for tool in tablet.tools() {
                println!(
                    "    - {{ vid: '0x{:04x}', pid: '0x{:08x}' }}",
                    tool.vendor_id(),
                    tool.tool_id()
                );
            }
        }
    }

    Ok(())
}

fn tablet_lookup_key(tablet: &Tablet) -> String {
    format!(
        "{}|{:04x}|{:04x}|{}|{}|",
        tablet.bustype(),
        tablet.vendor_id(),
        tablet.product_id(),
        tablet.name(),
        tablet.firmware_version().unwrap_or("")
    )
}

struct LocalTablet {
    tablet: Tablet,
    nodes: Vec<(PathBuf, String)>,
}

fn cmd_list_local() -> Result<()> {
    let mut locals: HashMap<String, LocalTablet> = HashMap::new();
    let cache = tabletdb::Cache::new()?;

    println!("device:");
    for file in std::fs::read_dir("/dev/input")?
        .flatten()
        .filter(|f| f.file_name().to_string_lossy().starts_with("event"))
    {
        log::debug!("Checking {:?}", file.path());

        let name: String = std::fs::read_to_string(format!(
            "/sys/class/input/{}/device/name",
            file.file_name().to_string_lossy()
        ))
        .unwrap_or(String::from(""))
        .trim()
        .to_string();

        if let Ok(info) = TabletInfo::new_from_path(&file.path()) {
            cache.tablets().filter(|t| *t == &info).for_each(|tablet| {
                let lookup = tablet_lookup_key(tablet);
                locals
                    .entry(lookup)
                    .and_modify(|t| t.nodes.push((file.path(), name.clone())))
                    .or_insert(LocalTablet {
                        tablet: tablet.clone(),
                        nodes: vec![(file.path(), name.clone())],
                    });
            })
        }
    }

    for local in locals.values() {
        let tablet = &local.tablet;
        println!("- name: '{}'", tablet.name());
        println!("  bus: '{}'", tablet.bustype());
        println!("  vid: '0x{:04x}'", tablet.vendor_id());
        println!("  pid: '0x{:04x}'", tablet.product_id());
        println!("  nodes:");
        for node in local.nodes.iter() {
            println!("  - {}: '{}'", node.0.to_string_lossy(), node.1);
        }
    }

    Ok(())
}

fn run() -> Result<()> {
    let cli = Cli::parse();

    unsafe {
        match cli.color {
            ColorChoice::Never => std::env::set_var("NO_COLOR", "1"),
            ColorChoice::Auto => {}
            ColorChoice::Always => std::env::set_var("FORCE_COLOR", "1"),
        }
    }

    let modules = vec![module_path!(), "tabletdb"];

    stderrlog::new()
        .modules(modules)
        .show_module_names(true)
        .verbosity(if cli.verbose {
            log::LevelFilter::Debug
        } else {
            log::LevelFilter::Warn
        })
        .color(match cli.color {
            ColorChoice::Never => stderrlog::ColorChoice::Never,
            ColorChoice::Auto => stderrlog::ColorChoice::Never,
            ColorChoice::Always => stderrlog::ColorChoice::Always,
        })
        .init()
        .unwrap();

    match cli.command {
        Commands::ListTablets => cmd_list_tablets(),
        Commands::ListStyli => cmd_list_styli(),
        Commands::ListLocalDevices => cmd_list_local(),
        Commands::Info {
            device,
            bustype,
            vendor_id,
            product_id,
            kernel_name,
            uniq,
            device_match,
        } => cmd_info(
            device,
            bustype,
            vendor_id,
            product_id,
            kernel_name,
            uniq,
            device_match,
        ),
    }
}

fn main() -> ExitCode {
    let rc = run();
    match rc {
        Ok(_) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("Error: {e:#}");
            ExitCode::FAILURE
        }
    }
}
