use once_cell::sync::Lazy;
use std::{
    env, fs,
    path::{Path, PathBuf},
    process::Command,
    sync::RwLock,
};
use walkdir::WalkDir;

static PROJECT_ROOT: Lazy<PathBuf> = Lazy::new(|| {
    PathBuf::from(
        env::var("CARGO_MANIFEST_DIR")
            .unwrap_or_else(|_| env::current_dir().unwrap().to_str().unwrap().to_string()),
    )
});

static BUILD_FOLDER_PATH: Lazy<PathBuf> = Lazy::new(|| PROJECT_ROOT.join("builds"));

static GEN_FOLDER_PATH: Lazy<PathBuf> = Lazy::new(|| PROJECT_ROOT.join("generated"));

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
    let mut include_paths = vec![PROJECT_ROOT.join("src"), rs_driver_root.join("src")];
    include_paths.retain(|path| path.exists());

    if include_paths.len() < 2 {
        panic!(
            "Could not find rs_driver include directory at {}",
            rs_driver_root.join("src").display()
        );
    }

    let include_refs: Vec<&Path> = include_paths.iter().map(|path| path.as_path()).collect();

    // Get GCC system include paths to help clang find standard headers
    let gcc_include_output = Command::new("gcc")
        .args(&["-E", "-Wp,-v", "-xc++", "/dev/null"])
        .output()
        .ok();

    let mut extra_args = vec!["-std=c++17".to_string()];

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

    let extra_args_refs: Vec<&str> = extra_args.iter().map(|s| s.as_str()).collect();

    let builder = autocxx_build::Builder::new("src/bindings.rs", &include_refs)
        .extra_clang_args(&extra_args_refs);

    let mut cc_builder = builder
        .build()
        .expect("Unable to generate bindings for rs_driver");

    for dir in &include_paths {
        cc_builder.include(dir);
    }

    cc_builder.file(PROJECT_ROOT.join("src/ffi/rs_driver_wrapper.cpp"));

    cc_builder
        .flag_if_supported("-std=c++17")
        .compile("carvi_rsd_binding");

    println!("cargo:rerun-if-changed=src/bindings.rs");
    println!("cargo:rerun-if-changed=src/ffi/rs_driver_wrapper.hpp");
    println!("cargo:rerun-if-changed=src/ffi/rs_driver_wrapper.cpp");
}

fn emit_link_directives(_rs_driver_root: &Path) {
    if cfg!(target_family = "unix") {
        if let Err(err) = pkg_config::Config::new().probe("libpcap") {
            println_build!(
                "pkg-config was unable to locate libpcap ({err}). Falling back to generic linkage."
            );
            println!("cargo:rustc-link-lib=dylib=pcap");
        }
        println!("cargo:rustc-link-lib=dylib=pthread");
    } else if cfg!(target_family = "windows") {
        println!("cargo:rustc-link-lib=dylib=ws2_32");
    }
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
