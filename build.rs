use once_cell::sync::Lazy;
use std::{
    env, fs,
    path::{Path, PathBuf},
    process::Command,
    sync::RwLock,
};
use walkdir::WalkDir;

#[cfg(all(target_os = "windows", feature = "npcap-sdk-download"))]
use std::fs::File;

#[cfg(all(target_os = "windows", feature = "npcap-sdk-download"))]
use std::io::Write;

#[cfg(all(target_os = "windows", feature = "npcap-sdk-download"))]
use zip::ZipArchive;

static PROJECT_ROOT: Lazy<PathBuf> = Lazy::new(|| {
    PathBuf::from(
        env::var("CARGO_MANIFEST_DIR")
            .unwrap_or_else(|_| env::current_dir().unwrap().to_str().unwrap().to_string()),
    )
});

static BUILD_FOLDER_PATH: Lazy<PathBuf> = Lazy::new(|| PROJECT_ROOT.join("builds"));

static GEN_FOLDER_PATH: Lazy<PathBuf> = Lazy::new(|| PROJECT_ROOT.join("generated"));

#[cfg(all(target_os = "windows", feature = "npcap-sdk-download"))]
const NPCAP_SDK_DEFAULT_URL: &str = "https://npcap.com/dist/npcap-sdk-1.15.zip";

const RS_DRIVER_REPOSITORY_URL: &str = "https://github.com/RoboSense-LiDAR/rs_driver.git";
const RS_DRIVER_BRANCH_NAME: &str = "v1.5.18";
const RS_DRIVER_ROOT_ENV: &str = "RS_DRIVER_CORE_ROOT";
const RS_DRIVER_REPO_ENV: &str = "RS_DRIVER_REPOSITORY";
const RS_DRIVER_BRANCH_ENV: &str = "RS_DRIVER_BRANCH";

static RS_DRIVER_ROOT: Lazy<RwLock<PathBuf>> = Lazy::new(|| {
    let root = env::var(RS_DRIVER_ROOT_ENV)
        .map(PathBuf::from)
        .unwrap_or_else(|_| BUILD_FOLDER_PATH.join("rs-driver"));
    RwLock::new(root)
});

macro_rules! println_build {
    ($($tokens:tt)*) => {
        println!("cargo:warning=\r\x1b[32;1m   {}", format!($($tokens)*))
    };
}

fn main() {
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-env-changed={}", RS_DRIVER_ROOT_ENV);
    println!("cargo:rerun-if-env-changed={}", RS_DRIVER_REPO_ENV);
    println!("cargo:rerun-if-env-changed={}", RS_DRIVER_BRANCH_ENV);

    ensure_directory(&BUILD_FOLDER_PATH);
    ensure_directory(&GEN_FOLDER_PATH);

    let rs_driver_root = ensure_rs_driver_checkout();
    track_rs_driver_sources(&rs_driver_root);

    emit_link_directives(&rs_driver_root);
    build_bindings(&rs_driver_root);
}

fn ensure_directory(path: &Path) {
    if let Err(err) = fs::create_dir_all(path) {
        panic!("Failed to create directory {}: {err}", path.display());
    }
}

fn ensure_rs_driver_checkout() -> PathBuf {
    let repo_root = get_rs_driver_root();
    if repo_root.exists() && repo_root.join(".git").exists() {
        println_build!(
            "Using existing rs_driver checkout at {}",
            repo_root.display()
        );
        return repo_root;
    }

    let repo_url =
        env::var(RS_DRIVER_REPO_ENV).unwrap_or_else(|_| RS_DRIVER_REPOSITORY_URL.to_string());
    let branch =
        env::var(RS_DRIVER_BRANCH_ENV).unwrap_or_else(|_| RS_DRIVER_BRANCH_NAME.to_string());

    println_build!(
        "Cloning rs_driver ({}) into {}...",
        branch,
        repo_root.display()
    );

    clone_repository(&repo_url, &repo_root, Some(branch.as_str()))
        .unwrap_or_else(|err| panic!("Failed to clone rs_driver repository: {err}"));

    {
        let mut root_guard = RS_DRIVER_ROOT.write().unwrap();
        *root_guard = repo_root.clone();
    }

    repo_root
}

fn build_bindings(rs_driver_root: &Path) {
    let pcap_enabled = env::var("CARGO_FEATURE_PCAP").is_ok();

    let mut include_paths = vec![PROJECT_ROOT.join("src"), rs_driver_root.join("src")];
    include_paths.retain(|path| path.exists());

    if include_paths.len() < 2 {
        panic!(
            "Could not find rs_driver include directory at {}",
            rs_driver_root.join("src").display()
        );
    }

    // Get GCC system include paths to help clang find standard headers
    let gcc_include_output = Command::new("gcc")
        .args(["-E", "-Wp,-v", "-xc++", "/dev/null"])
        .output()
        .ok();

    let mut extra_args = vec!["-std=c++17".to_string()];

    // If PCAP support is not requested, skip the PCAP-related headers entirely.
    // rs_driver's `input_factory.hpp` uses this macro to avoid including `input_pcap*.hpp`.
    if !pcap_enabled {
        extra_args.push("-DDISABLE_PCAP_PARSE".to_string());
    }

    if let Some(output) = gcc_include_output {
        let stderr = String::from_utf8_lossy(&output.stderr);
        for line in stderr.lines() {
            let trimmed = line.trim();
            if trimmed.starts_with('/') && (trimmed.contains("include") || trimmed.contains("gcc"))
            {
                extra_args.push(format!("-I{}", trimmed));
            }
        }
    }

    // Add Npcap/WinPcap SDK paths for Windows (only when PCAP feature is enabled).
    #[cfg(target_os = "windows")]
    {
        if pcap_enabled {
            let sdk_root = find_or_fetch_npcap_sdk();
            if let Some(sdk_root) = sdk_root {
                let include_path = sdk_root.join("Include");
                if include_path.exists() {
                    println_build!("Using Npcap SDK at {}", sdk_root.display());
                    extra_args.push(format!("-I{}", include_path.display()));
                    include_paths.push(include_path);
                }
            }
        }
    }

    // IMPORTANT: only borrow include_paths after we are done mutating it (Windows SDK detection
    // may push additional include dirs).
    let include_refs: Vec<&Path> = include_paths.iter().map(|path| path.as_path()).collect();

    let extra_args_refs: Vec<&str> = extra_args.iter().map(|s| s.as_str()).collect();

    let builder = autocxx_build::Builder::new("src/bindings.rs", &include_refs)
        .extra_clang_args(&extra_args_refs);

    let mut cc_builder = builder
        .build()
        .expect("Unable to generate bindings for rs_driver");

    if !pcap_enabled {
        cc_builder.define("DISABLE_PCAP_PARSE", None);
    }

    for dir in &include_paths {
        cc_builder.include(dir);
    }

    cc_builder.file(PROJECT_ROOT.join("src/ffi/rs_driver_wrapper.cpp"));

    cc_builder
        .flag_if_supported("-std=c++17")
        .flag_if_supported("-Wno-unused-parameter")
        .flag_if_supported("-Wno-deprecated-copy")
        .flag_if_supported("-Wno-missing-field-initializers")
        .compile("carvi_rsd_binding");

    println!("cargo:rerun-if-changed=src/bindings.rs");
    println!("cargo:rerun-if-changed=src/ffi/rs_driver_wrapper.hpp");
    println!("cargo:rerun-if-changed=src/ffi/rs_driver_wrapper.cpp");
}

fn emit_link_directives(_rs_driver_root: &Path) {
    let pcap_enabled = env::var("CARGO_FEATURE_PCAP").is_ok();

    if cfg!(target_family = "unix") {
        if pcap_enabled {
            if let Err(err) = pkg_config::Config::new().probe("libpcap") {
                println_build!(
                    "pkg-config was unable to locate libpcap ({err}). Falling back to generic linkage."
                );
                println!("cargo:rustc-link-lib=dylib=pcap");
            }
        }
        println!("cargo:rustc-link-lib=dylib=pthread");
    } else if cfg!(target_family = "windows") {
        println!("cargo:rustc-link-lib=dylib=ws2_32");
        
        if pcap_enabled {
            if let Some(sdk_root) = find_or_fetch_npcap_sdk() {
                let lib_path_x64 = sdk_root.join("Lib/x64");
                let lib_path = sdk_root.join("Lib");

                if lib_path_x64.exists() {
                    println!("cargo:rustc-link-search=native={}", lib_path_x64.display());
                } else if lib_path.exists() {
                    println!("cargo:rustc-link-search=native={}", lib_path.display());
                }
            }

            // These import libraries come from the Npcap SDK.
            println!("cargo:rustc-link-lib=dylib=wpcap");
            println!("cargo:rustc-link-lib=dylib=Packet");
        }
    }
}

#[cfg(target_os = "windows")]
fn find_or_fetch_npcap_sdk() -> Option<PathBuf> {
    // 1) User-provided location(s)
    let candidates = [
        env::var("NPCAP_SDK").ok().map(PathBuf::from),
        Some(PathBuf::from("C:/Program Files/Npcap SDK")),
        Some(PathBuf::from("C:/npcap-sdk")),
        env::var("WINPCAP_SDK").ok().map(PathBuf::from),
        Some(BUILD_FOLDER_PATH.join("npcap-sdk")),
    ];

    for root in candidates.iter().flatten() {
        if root.join("Include/pcap.h").exists() {
            return Some(root.clone());
        }
    }

    // 2) Optional auto-download of the SDK ZIP (headers + .lib import libs)
    // NOTE: Npcap is not open source and has redistribution restrictions. We do NOT vendor it in
    // this repository. Auto-download is opt-in and fetches directly from the upstream site.
    // Default behavior:
    // - if 'npcap-sdk-download' feature is enabled: auto-download by default
    // - otherwise: do NOT auto-download unless explicitly requested
    let auto = env::var("NPCAP_SDK_AUTO_DOWNLOAD")
        .ok()
        .map(|v| v == "1" || v.eq_ignore_ascii_case("true"))
        .unwrap_or(cfg!(feature = "npcap-sdk-download"));

    if !auto {
        println_build!(
            "Npcap SDK not found. Set NPCAP_SDK to the SDK path, or enable the 'npcap-sdk-download' feature (or set NPCAP_SDK_AUTO_DOWNLOAD=1) to fetch it automatically."
        );
        return None;
    }

    // Auto-download requested.
    #[cfg(feature = "npcap-sdk-download")]
    {
        let url = env::var("NPCAP_SDK_URL").unwrap_or_else(|_| NPCAP_SDK_DEFAULT_URL.to_string());
        let sdk_root = BUILD_FOLDER_PATH.join("npcap-sdk");
        if sdk_root.join("Include/pcap.h").exists() {
            return Some(sdk_root);
        }

        if let Err(e) = download_and_extract_zip(&url, &sdk_root) {
            panic!("Failed to auto-download Npcap SDK from {url}: {e}");
        }

        return sdk_root
            .join("Include/pcap.h")
            .exists()
            .then_some(sdk_root);
    }

    #[cfg(not(feature = "npcap-sdk-download"))]
    {
        println_build!(
            "Npcap SDK auto-download requested, but this build was compiled without the 'npcap-sdk-download' feature. Rebuild with --features pcap,npcap-sdk-download, or set NPCAP_SDK to an existing SDK install."
        );
        None
    }
}

#[cfg(all(target_os = "windows", feature = "npcap-sdk-download"))]
fn download_and_extract_zip(url: &str, dest_dir: &Path) -> Result<(), String> {
    ensure_directory(dest_dir);

    let zip_path = dest_dir.join("npcap-sdk.zip");

    println_build!("Downloading Npcap SDK from {url} ...");
    let resp = reqwest::blocking::Client::new()
        .get(url)
        .header(reqwest::header::USER_AGENT, "carvi_rsd build.rs")
        .send()
        .map_err(|e| format!("HTTP request failed: {e}"))?;

    if !resp.status().is_success() {
        return Err(format!("Download failed with status {}", resp.status()));
    }

    let bytes = resp
        .bytes()
        .map_err(|e| format!("Failed reading response body: {e}"))?;
    let mut f = File::create(&zip_path).map_err(|e| format!("Create zip failed: {e}"))?;
    f.write_all(&bytes)
        .map_err(|e| format!("Write zip failed: {e}"))?;

    println_build!("Extracting {} ...", zip_path.display());
    let zip_file = File::open(&zip_path).map_err(|e| format!("Open zip failed: {e}"))?;
    let mut archive = ZipArchive::new(zip_file).map_err(|e| format!("Invalid zip: {e}"))?;

    for i in 0..archive.len() {
        let mut file = archive.by_index(i).map_err(|e| format!("Zip read failed: {e}"))?;
        let Some(relpath) = file.enclosed_name() else {
            // Skip paths that attempt traversal
            continue;
        };

        let relpath = relpath.to_owned();

        let outpath = dest_dir.join(relpath);
        if file.is_dir() {
            ensure_directory(&outpath);
            continue;
        }

        if let Some(parent) = outpath.parent() {
            ensure_directory(parent);
        }

        let mut outfile = File::create(&outpath).map_err(|e| format!("Create file failed: {e}"))?;
        std::io::copy(&mut file, &mut outfile).map_err(|e| format!("Extract failed: {e}"))?;
    }

    Ok(())
}

fn track_rs_driver_sources(rs_driver_root: &Path) {
    watch_path(&rs_driver_root.join("CMakeLists.txt"));
    watch_tree(&rs_driver_root.join("cmake"));
    watch_tree(&rs_driver_root.join("src"));
}

fn watch_path(path: &Path) {
    if path.exists() {
        println!("cargo:rerun-if-changed={}", path.display());
    }
}

fn watch_tree(root: &Path) {
    if !root.exists() {
        return;
    }

    for entry in WalkDir::new(root).into_iter().filter_map(|res| res.ok()) {
        let path = entry.path();
        if path
            .components()
            .any(|component| component.as_os_str() == ".git")
        {
            continue;
        }

        if entry.file_type().is_file() {
            println!("cargo:rerun-if-changed={}", path.display());
        }
    }
}

fn clone_repository(repo_url: &str, dest_path: &Path, branch: Option<&str>) -> Result<(), String> {
    if dest_path.exists() {
        if dest_path.join(".git").exists() {
            return Ok(());
        }
        return Err(format!(
            "Destination {} exists and is not a git repository",
            dest_path.display()
        ));
    }

    if let Some(parent) = dest_path.parent() {
        fs::create_dir_all(parent)
            .map_err(|err| format!("Failed to create {}: {err}", parent.display()))?;
    }

    let mut args = vec!["clone", "--recurse-submodules"];
    if let Some(branch_name) = branch {
        args.push("--branch");
        args.push(branch_name);
    }
    args.push(repo_url);
    args.push(
        dest_path
            .to_str()
            .ok_or_else(|| "Invalid destination for git clone".to_string())?,
    );

    println_build!("Running git {}", args.join(" "));

    let status = Command::new("git")
        .args(args)
        .status()
        .map_err(|e| format!("Failed to spawn git: {e}"))?;

    if !status.success() {
        return Err(format!("git clone failed with status {status}"));
    }

    Ok(())
}

fn get_rs_driver_root() -> PathBuf {
    RS_DRIVER_ROOT.read().unwrap().to_path_buf()
}
