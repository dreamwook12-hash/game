# Environment investigation — 2026-10-17 (Asia/Seoul)

Initial workspace contained only empty `outputs/` and `work/`. There was no source, asset, build system, CI, or linked project configuration. No existing content was overwritten. Delivery is `outputs/PROJECT_AETHER/`.

| Read-only command / check | Observed result |
|---|---|
| `Get-Location` | User-provided Codex workspace |
| `git status --short`, `git status --branch`, `git rev-parse --show-toplevel`, `git log -1 --oneline` | All failed: not a Git repository. No branch, HEAD or history invented. |
| `rg --files -g AGENTS.md` in workspace/Codex roots | No existing project instructions found |
| `Get-CimInstance Win32_OperatingSystem` | Windows 11 Home, 10.0.26200, x64 |
| `Get-CimInstance Win32_ComputerSystem` | T1LS TMB-771900; hypervisor present |
| `Get-CimInstance Win32_Processor` | Ryzen 7 PRO 7745; 8 cores / 16 threads; reported max clock 3801 MHz |
| `Get-CimInstance Win32_VideoController` | AMD Radeon(TM) Graphics; driver 32.0.21042.62; reported AdapterRAM 512 MiB; 1920×1080 at 60 Hz |
| `Get-PhysicalDisk` / logical disk query | NVMe SSD; volume about 452 GiB; about 377 GiB free at initial probe |
| PATH + standard folders + uninstall/SDK registry inspection | No UE/UBT, VS/MSVC, Windows SDK, CMake, Ninja, LLVM/GCC, Android SDK/NDK/Studio, JDK, .NET SDK or GPU profiler found |
| Bundled executables, actually version queried | Git 2.53.0.windows.3; Node 24.19.0; Python 3.12.14 |

Search is bounded to reasonable standard paths, registry and runtime directories; it is not proof that no custom installation exists anywhere on disk. A `python.exe` WindowsApps alias was present; the actual bundled Python was used when needed.

Execution environment classification: **UNKNOWN**. This is a directly accessible Windows desktop host, not a demonstrated container. Model information suggests a desktop; `HypervisorPresent=True` can reflect Windows virtualization security and does not establish whether this session is a VM. Identity with the intended play PC has not been independently confirmed. Hardware results apply only to this execution host.

After the read-only investigation, a bounded portable Zig toolchain was downloaded from its official distribution to workspace `work/tools/`, with published size and SHA-256 validation. No system install, PATH change, account, driver modification, or paid service was used. Detailed compiler execution/build results are recorded in BUILD.md and timestamped logs.

The reusable PowerShell probe stores selected non-identifying fields. It omits names of users/computers, IPs, serials, accounts and credentials. Fields that could not be measured remain UNKNOWN.
