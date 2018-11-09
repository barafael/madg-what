extern crate clap;
use clap::{App, Arg};

extern crate dlopen;
extern crate dlopen_derive;
use dlopen::symbor::Library;

use std::fs;
use std::path::Path;

use std::fmt;

extern crate libc;
use libc::c_float;

use dlopen::utils::PLATFORM_FILE_EXTENSION;

use std::collections::HashSet;

mod quaternion;
use crate::quaternion::{Measurement, Quaternion, Axis};
use crate::quaternion::{RandomGenerate};

struct FilterLib {
    filename: String,
    handle: Library,
}

#[derive(Debug)]
struct TestBench {
    libs: Vec<FilterLib>,
}

#[derive(Debug)]
struct TestRun {
    measurement: Measurement,
    results: Vec<(String, Option<Quaternion>)>,
}

trait MadgwickFilter {
    fn madgwick_filter(&self, measurement: Measurement) -> Option<Quaternion>;
    fn set_beta(&self, beta: f32);
    fn set_deltat(&self, deltat: f32);
}

impl MadgwickFilter for FilterLib {
    fn madgwick_filter(&self, measurement: Measurement) -> Option<Quaternion> {
        let sym = unsafe {
            self.handle
                .symbol::<unsafe extern "C" fn(acc: Axis, gyro: Axis, mag: Axis) -> Quaternion>(
                    "madgwick_filter",
                )
        };
        match sym {
            Ok(filter) => {
                Some(unsafe { filter(measurement.acc, measurement.gyro, measurement.mag) })
            }
            Err(e) => {
                eprintln!("{}", e);
                None
            }
        }
    }

    fn set_beta(&self, beta: f32) {
        let sym = unsafe {
            self.handle
                .symbol::<unsafe extern "C" fn(c_float)>("set_beta")
        };
        match sym {
            Ok(set) => {
                unsafe { set(beta as c_float) };
            }
            Err(e) => {
                eprintln!("{}", e);
            }
        }
    }

    fn set_deltat(&self, deltat: f32) {
        let sym = unsafe {
            self.handle
                .symbol::<unsafe extern "C" fn(c_float)>("set_deltat")
        };
        match sym {
            Ok(set) => {
                unsafe { set(deltat as c_float) };
            }
            Err(e) => {
                eprintln!("{}", e);
            }
        }
    }
}

impl fmt::Debug for FilterLib {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "Filename: {}", self.filename)
    }
}

impl TestBench {
    pub fn run(&self, measurement: Measurement) -> TestRun {
        let mut result = TestRun {
            measurement,
            results: Vec::new(),
        };
        for lib in self.libs.iter() {
            let quat = lib.madgwick_filter(measurement);
            result.results.push((lib.filename.clone(), quat));
        }
        result
    }
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
        )
        .arg(
            Arg::with_name("dir")
                .takes_value(true)
                .min_values(1)
                .short("d"),
        )
        .get_matches();

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
            let entries = match fs::read_dir(dir) {
                Ok(entries) => entries,
                Err(e) => {
                    eprintln!("{}", e);
                    continue;
                }
            };
            for entry in entries {
                let dir_entry = match entry {
                    Ok(entry) => entry,
                    Err(e) => {
                        eprintln!("{}", e);
                        continue;
                    }
                };
                let file_path = dir_entry.path();
                if !(file_path.exists() && file_path.is_file()) {
                    continue;
                }
                if let Some(ext) = file_path.extension() {
                    if ext == PLATFORM_FILE_EXTENSION {
                        if let Some(s) = file_path.to_str() {
                            files.insert(String::from(s));
                        }
                    }
                };
            }
        }
    }

    println!("{:#?}", files);

    let mut test_bench = TestBench { libs: Vec::new() };

    for filename in files {
        let lib = Library::open(&filename);
        match lib {
            Err(e) => eprintln!("{}", e),
            Ok(handle) => {
                test_bench.libs.push(FilterLib { filename, handle });
            }
        }
    }

    println!("{:#?}", test_bench);

    let measurement = Measurement::generate();

    let mut result = test_bench.run(measurement);

    result.results.sort_by(|a, b| {
        a.0.cmp(&b.0)
    });

    println!("{:#?}", result);
}
