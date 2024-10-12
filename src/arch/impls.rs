#[cfg(target_os = "linux")]
mod linux;

#[cfg(target_os = "windows")]
mod windows;

#[cfg(target_os = "linux")]
use linux as arch;

#[cfg(target_os = "windows")]
use windows as arch;
