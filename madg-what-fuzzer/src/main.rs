extern crate clap;
use clap::{App, Arg};

extern crate dlopen;
extern crate dlopen_derive;
use dlopen::symbor::Library;

use std::fs;
use std::path::Path;

extern crate libc;
use libc::c_float;

use dlopen::utils::PLATFORM_FILE_EXTENSION;

use std::collections::HashSet;

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
        ).arg(
            Arg::with_name("dir")
                .takes_value(true)
                .min_values(1)
                .short("d"),
        ).get_matches();

    let mut files = HashSet::<String>::new();

    if let Some(f) = matches.values_of("file") {
        for path in f.collect::<Vec<&str>>().iter() {
            if Path::new(path).exists() {
                files.insert(path.to_string());
            }
        }
    }

    if let Some(d) = matches.values_of("dir") {
        for dir in d.collect::<Vec<&str>>().iter() {
            match fs::read_dir(dir) {
                Ok(entries) => {
                    for entry in entries {
                        match entry {
                            Ok(dir_entry) => {
                                let file_path = dir_entry.path();
                                if file_path.exists() && file_path.is_file() {
                                    file_path.extension().map(|ext| {
                                        if ext == PLATFORM_FILE_EXTENSION {
                                            if let Some(s) = file_path.to_str() {
                                                files.insert(String::from(s));
                                            }
                                        }
                                    });
                                }
                            },
                            Err(e) => eprintln!("{}", e),
                        }
                    }
                }
                Err(e) => eprintln!("{}", e),
            }
        }
    }

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

