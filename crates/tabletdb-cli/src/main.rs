use anyhow::Result;
use clap::{ColorChoice, Parser, Subcommand};
use std::collections::HashMap;
use std::path::PathBuf;
use std::process::ExitCode;
use tabletdb::{
    BusType, Button, Dial, Feature, Location, Ring, Strip, Stylus, Tablet, TabletBuilder,
    TabletOwnership, Units,
};

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
    },
}

fn cmd_list_tablets() -> Result<()> {
    let cache = tabletdb::Cache::new()?;
    println!("devices:");

    let mut tablets: Vec<&Tablet<_>> = cache.tablets().collect();
    tablets.sort_by_key(|t| {
        format!(
            "{}:{:04x}:{:04x}",
            t.bustype(),
            u16::from(t.vendor_id()),
            u16::from(t.product_id())
        )
    });

    for tablet in tablets {
        println!(
            "- {{ bus: {:<11} vid: '0x{:04x}', pid: '0x{:04x}', name: '{}', uniq: '{}' }}",
            format!("'{}',", tablet.bustype()),
            u16::from(tablet.vendor_id()),
            u16::from(tablet.product_id()),
            tablet.name(),
            tablet.firmware_version().unwrap_or("")
        );
    }

    Ok(())
}

fn cmd_list_styli() -> Result<()> {
    let cache = tabletdb::Cache::new()?;

    println!("styli:");
    let mut styli: Vec<&Stylus> = cache.styli().collect();
    styli.sort_by_key(|s| {
        format!(
            "{:04x}:{:08x}",
            u16::from(s.vendor_id()),
            u32::from(s.tool_id())
        )
    });

    for stylus in styli {
        println!(
            "- {{ vid: '0x{:04x}', pid: '0x{:08x}', name: '{}' }}",
            u16::from(stylus.vendor_id()),
            u32::from(stylus.tool_id()),
            stylus.name()
        );
    }

    Ok(())
}

fn cmd_info(path: Option<String>) -> Result<()> {
    if let Some(path) = path {
        let cache = tabletdb::Cache::new()?;
        let builder = TabletBuilder::new_from_path(&PathBuf::from(&path))?;
        if let Ok(tablet) = cache.take_tablet(builder) {
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
                    BusType::Unknown { .. } => "unknown",
                }
            );
            println!("  vid: \"0x{:04x}\"", u16::from(tablet.vendor_id()));
            println!("  pid: \"0x{:04x}\"", u16::from(tablet.product_id()));
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
                for stylus in tablet.styli() {
                    println!(
                        "    - {{ vid: '0x{:04x}', pid: '0x{:08x}' }}",
                        stylus.vendor_id(),
                        stylus.tool_id(),
                    );
                }
            }
        } else {
            println!("{:?} is not a known tablet device", path);
        }
    }

    Ok(())
}

fn tablet_lookup_key<T: TabletOwnership>(tablet: &Tablet<T>) -> String {
    format!(
        "{}|{:04x}|{:04x}|{}|{}|",
        tablet.bustype(),
        u16::from(tablet.vendor_id()),
        u16::from(tablet.product_id()),
        tablet.name(),
        tablet.firmware_version().unwrap_or("")
    )
}

struct LocalTablet<T: TabletOwnership> {
    tablet: Tablet<T>,
    nodes: Vec<(PathBuf, String)>,
}

fn cmd_list_local() -> Result<()> {
    let mut locals: HashMap<String, LocalTablet<_>> = HashMap::new();
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

        let builder = TabletBuilder::new_from_path(&file.path())?;
        if let Some(tablet) = cache.find_tablet(builder) {
            let lookup = tablet_lookup_key(tablet);
            locals
                .entry(lookup)
                .and_modify(|t| t.nodes.push((file.path(), name.clone())))
                .or_insert(LocalTablet {
                    tablet: tablet.clone(),
                    nodes: vec![(file.path(), name)],
                });
        }
    }

    for local in locals.values() {
        let tablet = &local.tablet;
        println!("- name: '{}'", tablet.name());
        println!("  bus: '{}'", tablet.bustype());
        println!("  vid: '0x{:04x}'", u16::from(tablet.vendor_id()));
        println!("  pid: '0x{:04x}'", u16::from(tablet.product_id()));
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
        Commands::Info { device } => cmd_info(device),
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
