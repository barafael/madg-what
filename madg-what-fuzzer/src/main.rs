extern crate clap;
use clap::{App, Arg};

extern crate dlopen;
extern crate dlopen_derive;
use dlopen::symbor::Library;

extern crate libc;
use libc::c_float;

use dlopen::utils::{PLATFORM_FILE_EXTENSION, PLATFORM_FILE_PREFIX};
use std::env;
use std::path::PathBuf;

#[derive(Debug, Clone, Copy)]
#[repr(C)]
struct Quaternion {
    pub a: c_float,
    pub b: c_float,
    pub c: c_float,
    pub d: c_float,
}

#[derive(Debug, Clone, Copy)]
#[repr(C)]
struct Axis {
    pub x: c_float,
    pub y: c_float,
    pub z: c_float,
}

fn main() {
    let matches = App::new("madg-what-fuzzer")
        .version("0.1")
        .author("Rafael Bachmann bachmann@fim.uni-passau.de")
        .about("Fuzz data fusion algorithms")
        .arg(
            Arg::with_name("file")
                .takes_value(true)
                .min_values(1)
                .short("f"),
        ).get_matches();

    let files: Vec<_> = matches.values_of("file").unwrap().collect();

    println!("{:?}", files);

    let mut handles = Vec::new();

    for filename in files {
        match Library::open(filename) {
            Ok(h) => handles.push(h),
            Err(e) => println!("{}", e),
        }
    }
    let fun = unsafe {
        handles[0].symbol::<unsafe extern "C" fn(acc: Axis, gyro: Axis, mag: Axis) -> Quaternion>(
            "madgwick_filter",
        )
    }.unwrap();
    let acc = Axis {
        x: 1.0,
        y: 2.0,
        z: 3.0,
    };
    let gyro = Axis {
        x: 4.0,
        y: 5.0,
        z: 6.0,
    };
    let mag = Axis {
        x: 7.0,
        y: 8.0,
        z: 9.0,
    };
    println!("{:?}", unsafe { fun(acc, gyro, mag) });
}

//Rust when building dependencies adds some weird numbers to file names
// find the file using this pattern:
//const FILE_PATTERN: &str = concat!(PLATFORM_FILE_PREFIX, "example.*\\.", PLATFORM_FILE_EXTENSION);

pub fn example_lib_path() -> PathBuf {
    let file_pattern = format!(
        r"{}example.*\.{}",
        PLATFORM_FILE_PREFIX, PLATFORM_FILE_EXTENSION
    );
    let file_regex = regex::Regex::new(file_pattern.as_ref()).unwrap();
    //build path to the example library that covers most cases
    let mut lib_path = PathBuf::from(env::var_os("CARGO_MANIFEST_DIR").unwrap());
    println!("Library path: {}", lib_path.to_str().unwrap());
    lib_path.extend(["target", "debug", "deps"].iter());
    println!("Library path: {}", lib_path.to_str().unwrap());
    let entry = lib_path.read_dir().unwrap().find(|e| match *e {
        Ok(ref entry) => file_regex.is_match(entry.file_name().to_str().unwrap()),
        Err(ref err) => panic!("Could not read cargo debug directory: {}", err),
    });
    println!("Library path: {}", lib_path.to_str().unwrap());
    lib_path.push(entry.unwrap().unwrap().file_name());
    println!("Library path: {}", lib_path.to_str().unwrap());
    lib_path
}
