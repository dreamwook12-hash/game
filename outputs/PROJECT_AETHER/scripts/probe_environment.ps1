param([string]$OutputPath)
$ErrorActionPreference = 'Continue'
# Read-only probe: deliberately excludes computer name, user, IP, IDs and serials.
$report = [ordered]@{
  observed_utc = [DateTime]::UtcNow.ToString('o')
  execution_environment = 'UNKNOWN: Windows host accessible; physical/virtual and intended play target not proven'
  os = @(Get-CimInstance Win32_OperatingSystem | Select-Object Caption,Version,OSArchitecture,TotalVisibleMemorySize,FreePhysicalMemory)
  host_kind = @(Get-CimInstance Win32_ComputerSystem | Select-Object Manufacturer,Model,HypervisorPresent,TotalPhysicalMemory)
  cpu = @(Get-CimInstance Win32_Processor | Select-Object Name,NumberOfCores,NumberOfLogicalProcessors,MaxClockSpeed,Architecture,L2CacheSize,L3CacheSize)
  gpu = @(Get-CimInstance Win32_VideoController | Select-Object Name,AdapterRAM,DriverVersion,CurrentHorizontalResolution,CurrentVerticalResolution,CurrentRefreshRate)
  disks = @(Get-CimInstance Win32_LogicalDisk -Filter 'DriveType=3' | Select-Object DeviceID,Size,FreeSpace)
  storage = @(Get-PhysicalDisk | Select-Object MediaType,BusType,Size)
  tools_on_path = @(Get-Command git,cmake,ninja,cl,clang++,g++,dotnet,python,py,node,nvidia-smi,java,adb -ErrorAction SilentlyContinue | Select-Object Name,Source)
  unknown = @('Dedicated/shared VRAM split','GPU temperature/power','HDR/VRR','Native display mode','Ray tracing API capability','Actual renderer performance','Target device identity')
}
$json = $report | ConvertTo-Json -Depth 5
if ($OutputPath) { $json | Set-Content -LiteralPath $OutputPath -Encoding utf8 } else { $json }
