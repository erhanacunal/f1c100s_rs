<#
.SYNOPSIS
    Build an F1C100s image and load it over USB FEL with xfel.

.DESCRIPTION
    Pipeline:  cargo build  ->  objcopy (ELF -> BIN)  ->  mksunxi (eGON header
    checksum, pad to 8 KiB)  ->  xfel.

    Modes:
      ram    (default)  xfel ddr f1c100s; xfel write 0x80000000; xfel exec 0x80000000
                        Nothing is written to flash. startup.s sees the FEL flag and
                        skips the flash copy; dram::init sees the 'X' marker left by
                        "xfel ddr" and skips DRAM re-init.
      flash             xfel spinor write 0 <bin>; xfel reset
                        Permanently programs SPI NOR from offset 0 (BROM boot).
      build             Only produce the .bin, do not touch the board.

    The board must be in FEL mode (hold BOOT / no bootable flash, then plug USB).

.EXAMPLE
    .\tools\flash.ps1                         # dma_demo, release, run from RAM
.EXAMPLE
    .\tools\flash.ps1 -Example dma_demo -Mode flash
.EXAMPLE
    .\tools\flash.ps1 -Elf ..\f1c100s_hello\target\armv5te-none-eabi\release\hello-f1c100s -Mode ram
.EXAMPLE
    .\tools\flash.ps1 -Bin .\some-image.bin -Mode flash -NoSunxi
#>
[CmdletBinding(DefaultParameterSetName = "Example")]
param(
    # Cargo example of this crate to build (see [[example]] in Cargo.toml)
    [Parameter(ParameterSetName = "Example")]
    [string]$Example = "dma_demo",

    # Use an already built ELF (e.g. from another crate) instead of building
    [Parameter(ParameterSetName = "Elf", Mandatory)]
    [string]$Elf,

    # Use a raw binary image instead of building
    [Parameter(ParameterSetName = "Bin", Mandatory)]
    [string]$Bin,

    [ValidateSet("ram", "flash", "build")]
    [string]$Mode = "ram",

    # Build the dev profile instead of release
    [switch]$DebugProfile,

    # Skip mksunxi (image already has a valid eGON header)
    [switch]$NoSunxi,

    # Erase the whole flash range covered by the image before writing (flash mode)
    [switch]$Erase
)

$ErrorActionPreference = "Stop"
$Root  = Split-Path -Parent $PSScriptRoot
$Tools = $PSScriptRoot

$DramBase = "0x80000000"

function Step($msg)  { Write-Host "[*] $msg" -ForegroundColor Cyan }
function Ok($msg)    { Write-Host "[+] $msg" -ForegroundColor Green }
function Fail($msg)  { Write-Host "[!] $msg" -ForegroundColor Red; exit 1 }

function Invoke-Native {
    param([string]$Exe, [string[]]$Arguments)
    & $Exe @Arguments
    if ($LASTEXITCODE -ne 0) { Fail "$(Split-Path -Leaf $Exe) $($Arguments -join ' ') failed (exit $LASTEXITCODE)" }
}

# ── Tools ──────────────────────────────────────────────────────────────────

$Xfel    = Join-Path $Tools "xfel.exe"
$Mksunxi = Join-Path $Tools "mksunxi.exe"
# xfel.exe needs libusb-1.0.dll, which lives in tools\xfel
$env:PATH = "$(Join-Path $Tools 'xfel');$env:PATH"

function Find-Objcopy {
    foreach ($name in "arm-none-eabi-objcopy", "rust-objcopy", "llvm-objcopy") {
        $cmd = Get-Command $name -ErrorAction SilentlyContinue
        if ($cmd) { return $cmd.Source }
    }
    $fallback = "D:\armeabi_none_toolchain\bin\arm-none-eabi-objcopy.exe"
    if (Test-Path $fallback) { return $fallback }
    Fail "objcopy not found (arm-none-eabi-objcopy / rust-objcopy / llvm-objcopy)"
}

# ── 1. Produce the ELF ─────────────────────────────────────────────────────

$BuildProfile = if ($DebugProfile) { "debug" } else { "release" }
$OutDir  = Join-Path $Root "target\armv5te-none-eabi\$BuildProfile"

switch ($PSCmdlet.ParameterSetName) {
    "Example" {
        Step "Building example '$Example' ($BuildProfile)"
        $cargoArgs = @("build", "-Zjson-target-spec", "--example", $Example)
        if (-not $DebugProfile) { $cargoArgs += "--release" }
        Push-Location $Root
        try { Invoke-Native "cargo" $cargoArgs } finally { Pop-Location }
        $Elf = Join-Path $OutDir "examples\$Example"
        if (-not (Test-Path $Elf)) { Fail "ELF not found: $Elf" }
    }
    "Elf" {
        $Elf = (Resolve-Path $Elf).Path
    }
    "Bin" {
        $Bin = (Resolve-Path $Bin).Path
    }
}

# ── 2. ELF -> BIN ──────────────────────────────────────────────────────────

if ($PSCmdlet.ParameterSetName -ne "Bin") {
    $Objcopy = Find-Objcopy
    $Bin = "$Elf.bin"
    Step "objcopy -> $Bin"
    Invoke-Native $Objcopy @("-O", "binary", $Elf, $Bin)

    $size = Get-Command "arm-none-eabi-size" -ErrorAction SilentlyContinue
    if ($size) { & $size.Source $Elf }
}

# ── 3. eGON header checksum ────────────────────────────────────────────────

if (-not $NoSunxi) {
    $head = [System.IO.File]::ReadAllBytes($Bin)[4..11]
    if ([System.Text.Encoding]::ASCII.GetString($head) -ne "eGON.BT0") {
        Fail "$Bin has no eGON.BT0 header at offset 4 - not an F1C100s boot image (use -NoSunxi to skip)"
    }
    Step "mksunxi (checksum + pad to 8 KiB)"
    $out = & $Mksunxi $Bin
    if ($LASTEXITCODE -ne 0) { Fail "mksunxi failed: $out" }
    Write-Host "    $out"
    if ("$out" -match "spl size is (\d+)" -and [int]$Matches[1] -gt 32768) {
        Write-Warning "SPL is $($Matches[1]) bytes - larger than the 32 KiB SRAM the BROM loads into; flash boot will fail."
    }
}

$binLen = (Get-Item $Bin).Length
Ok ("Image: {0} ({1:N0} bytes)" -f $Bin, $binLen)

if ($Mode -eq "build") { exit 0 }

# ── 4. Talk to the board ───────────────────────────────────────────────────

Step "Checking for F1C100s in FEL mode"
$ver = & $Xfel version 2>&1 | Out-String
if ($LASTEXITCODE -ne 0 -or $ver -notmatch "F1C100S") {
    Write-Host $ver.Trim()
    Fail "No F1C100s found in FEL mode. Hold BOOT (or boot without flash) and reconnect USB. On first use install the WinUSB driver from tools\xfel\Drivers (e.g. with Zadig)."
}
Write-Host "    $($ver.Trim())"

switch ($Mode) {
    "ram" {
        Step "Initialising DRAM (xfel ddr f1c100s)"
        Invoke-Native $Xfel @("ddr", "f1c100s")
        # startup.s takes the FEL path (skip the SPI-flash self-copy) only if
        # SRAM 0x8 holds ".FEL". Without it, sys_copyself would overwrite the
        # image we just loaded with whatever is in flash.
        $felMagic = & $Xfel read32 0x00000008 2>&1 | Out-String
        $dramMark = & $Xfel read32 0x0000005c 2>&1 | Out-String
        Write-Host ("    SRAM[0x08] = {0}   SRAM[0x5c] = {1}" -f $felMagic.Trim(), $dramMark.Trim())
        if ($felMagic -notmatch "4c45462e") {
            Step "FEL marker missing - writing '.FEL' to 0x00000008"
            Invoke-Native $Xfel @("write32", "0x00000008", "0x4c45462e")
        }
        if ($dramMark -notmatch "0x58[0-9a-f]{6}") {
            Write-Warning "SRAM[0x5c] has no 'X' DRAM marker - dram::init will re-init the controller while running from DRAM."
        }
        Step "Writing image to $DramBase"
        Invoke-Native $Xfel @("write", $DramBase, $Bin)
        Step "Executing at $DramBase"
        Invoke-Native $Xfel @("exec", $DramBase)
        Ok "Running from RAM (not persistent). Watch the console UART (examples use UART2: PE7 TX / PE8 RX, 115200 8N1)."
    }
    "flash" {
        Step "Detecting SPI NOR"
        Invoke-Native $Xfel @("spinor")
        if ($Erase) {
            $eraseLen = [math]::Ceiling($binLen / 65536) * 65536
            Step ("Erasing 0x0 .. 0x{0:X}" -f $eraseLen)
            Invoke-Native $Xfel @("spinor", "erase", "0", ("0x{0:X}" -f $eraseLen))
        }
        Step "Writing image to SPI NOR @ 0"
        Invoke-Native $Xfel @("spinor", "write", "0", $Bin)
        Step "Resetting board"
        Invoke-Native $Xfel @("reset")
        Ok "Flashed. The board now boots from SPI NOR."
    }
}
