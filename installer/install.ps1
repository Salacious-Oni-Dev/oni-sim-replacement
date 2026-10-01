<#
.SYNOPSIS
    Installs or removes the replacement simulation library (SimDLL.dll) for Oxygen Not Included.

.DESCRIPTION
    The game keeps its simulation library at
        <game>\OxygenNotIncluded_Data\Plugins\x86_64\SimDLL.dll
    Installing keeps the game's own copy beside it as SimDLL.dll.vanilla, checked byte for byte,
    and then puts the replacement in its place. Uninstalling puts the game's own copy back.

    The installer only proceeds on a game build this release was built against. It recognises
    the build by the SHA-256 of the game's own SimDLL.dll, listed in supported-builds.txt. On any
    other build it stops and changes nothing, because the library exchanges fixed-layout messages
    with the game and a build with different layouts would misread them.

    Files it writes, all in the Plugins\x86_64 folder:
        SimDLL.dll           the replacement (install) or the game's own (uninstall)
        SimDLL.dll.vanilla   the game's own library, kept while the replacement is installed
        SimDLL.dll.sdk       one line: the SHA-256 and version of the library it installed

.PARAMETER GamePath
    The game's folder, the one holding OxygenNotIncluded.exe. Found through Steam when omitted.

.PARAMETER Uninstall
    Put the game's own library back and remove the backup.

.PARAMETER Status
    Report what is installed and change nothing.

.PARAMETER Force
    Proceed on a game build that is not listed, or over a SimDLL.dll this installer did not put
    there. The backup is still taken and checked. Use only if you know why the check refused.

.EXAMPLE
    .\install.ps1
.EXAMPLE
    .\install.ps1 -Uninstall
.EXAMPLE
    .\install.ps1 -GamePath "D:\Games\OxygenNotIncluded" -Status
#>
[CmdletBinding()]
param(
    [string]$GamePath,
    [switch]$Uninstall,
    [switch]$Status,
    [switch]$Force
)

Set-StrictMode -Version 2.0
$ErrorActionPreference = 'Stop'

$SteamAppId = '457140'
$GameExe = 'OxygenNotIncluded.exe'
$Here = Split-Path -Parent $MyInvocation.MyCommand.Path

class InstallerError : System.Exception {
    InstallerError([string]$message) : base($message) {}
}

function Fail([string]$message) { throw [InstallerError]::new($message) }

function Get-Sha256([string]$path) {
    (Get-FileHash -LiteralPath $path -Algorithm SHA256).Hash.ToLowerInvariant()
}

function Short([string]$hash) { $hash.Substring(0, 12) }

# ---------------------------------------------------------------------------------------------
# What this package carries
# ---------------------------------------------------------------------------------------------

# supported-builds.txt: "<sha256 of the game's own SimDLL.dll> <game build>" per line, # comments.
function Read-SupportedBuilds {
    $file = Join-Path $Here 'supported-builds.txt'
    if (-not (Test-Path -LiteralPath $file)) { Fail "supported-builds.txt is missing from $Here. The package is incomplete; download it again." }
    $builds = @{}
    foreach ($line in Get-Content -LiteralPath $file) {
        $line = $line.Trim()
        if ($line -eq '' -or $line.StartsWith('#')) { continue }
        $parts = $line -split '\s+', 2
        if ($parts.Count -ne 2 -or $parts[0] -notmatch '^[0-9a-fA-F]{64}$') { Fail "supported-builds.txt has a line it cannot read: $line" }
        $builds[$parts[0].ToLowerInvariant()] = $parts[1]
    }
    if ($builds.Count -eq 0) { Fail 'supported-builds.txt lists no game builds.' }
    return $builds
}

# SHA256SUMS: "<sha256>  SimDLL.dll", the format sha256sum writes and checks.
function Read-Payload {
    $dll = Join-Path $Here 'SimDLL.dll'
    $sums = Join-Path $Here 'SHA256SUMS'
    if (-not (Test-Path -LiteralPath $dll)) { Fail "SimDLL.dll is missing from $Here. The package is incomplete; download it again." }
    if (-not (Test-Path -LiteralPath $sums)) { Fail "SHA256SUMS is missing from $Here. The package is incomplete; download it again." }
    $expected = $null
    foreach ($line in Get-Content -LiteralPath $sums) {
        if ($line -match '^([0-9a-fA-F]{64})\s+\*?SimDLL\.dll\s*$') { $expected = $Matches[1].ToLowerInvariant() }
    }
    if (-not $expected) { Fail 'SHA256SUMS has no entry for SimDLL.dll.' }
    $actual = Get-Sha256 $dll
    if ($actual -ne $expected) { Fail "The packaged SimDLL.dll does not match SHA256SUMS (expected $expected, found $actual). The download is damaged or has been altered; do not install it." }
    $version = 'unknown'
    $versionFile = Join-Path $Here 'VERSION'
    if (Test-Path -LiteralPath $versionFile) { $version = (Get-Content -LiteralPath $versionFile -TotalCount 1).Trim() }
    return [pscustomobject]@{ Path = $dll; Hash = $actual; Version = $version }
}

# ---------------------------------------------------------------------------------------------
# Finding the game
# ---------------------------------------------------------------------------------------------

function Get-SteamLibraries {
    $roots = New-Object System.Collections.Generic.List[string]
    foreach ($key in 'HKCU:\Software\Valve\Steam', 'HKLM:\SOFTWARE\WOW6432Node\Valve\Steam', 'HKLM:\SOFTWARE\Valve\Steam') {
        $item = Get-ItemProperty -LiteralPath $key -ErrorAction SilentlyContinue
        if (-not $item) { continue }
        foreach ($name in 'SteamPath', 'InstallPath') {
            if ($item.PSObject.Properties.Name -contains $name -and $item.$name) {
                $roots.Add(($item.$name -replace '/', '\'))
            }
        }
    }
    $libraries = New-Object System.Collections.Generic.List[string]
    foreach ($root in ($roots | Sort-Object -Unique)) {
        $libraries.Add($root)
        $vdf = Join-Path $root 'steamapps\libraryfolders.vdf'
        if (Test-Path -LiteralPath $vdf) {
            foreach ($m in [regex]::Matches((Get-Content -LiteralPath $vdf -Raw), '"path"\s+"([^"]+)"')) {
                $libraries.Add(($m.Groups[1].Value -replace '\\\\', '\'))
            }
        }
    }
    # Sort-Object -Unique compares without case; the registry and the .vdf disagree on case.
    return $libraries | Sort-Object -Unique
}

function Find-Game {
    foreach ($library in Get-SteamLibraries) {
        $manifest = Join-Path $library "steamapps\appmanifest_$SteamAppId.acf"
        $dir = Join-Path $library 'steamapps\common\OxygenNotIncluded'
        if (Test-Path -LiteralPath $manifest) {
            $m = [regex]::Match((Get-Content -LiteralPath $manifest -Raw), '"installdir"\s+"([^"]+)"')
            if ($m.Success) { $dir = Join-Path $library ('steamapps\common\' + $m.Groups[1].Value) }
        }
        if (Test-Path -LiteralPath (Join-Path $dir $GameExe)) { return $dir }
    }
    return $null
}

function Resolve-Plugins {
    $game = $GamePath
    if (-not $game) {
        $game = Find-Game
        if (-not $game) { Fail 'Could not find Oxygen Not Included through Steam. Run the installer again with -GamePath "<the folder holding OxygenNotIncluded.exe>".' }
    }
    $plugins = Join-Path $game 'OxygenNotIncluded_Data\Plugins\x86_64'
    if (-not (Test-Path -LiteralPath (Join-Path $plugins 'SimDLL.dll'))) {
        Fail "$game does not look like an Oxygen Not Included install: there is no OxygenNotIncluded_Data\Plugins\x86_64\SimDLL.dll in it."
    }
    return [pscustomobject]@{ Game = $game; Plugins = $plugins }
}

function Assert-GameClosed {
    if (Get-Process -Name 'OxygenNotIncluded' -ErrorAction SilentlyContinue) {
        Fail 'Oxygen Not Included is running. Close the game and run the installer again.'
    }
}

# ---------------------------------------------------------------------------------------------
# Replacing a file, checked
# ---------------------------------------------------------------------------------------------

# Copies $from to $to through a temporary file in the destination folder, checks the copy's
# hash, and only then moves it over $to. An interrupted run leaves either the old file or the
# new one in place, never a partial copy.
function Copy-Checked([string]$from, [string]$to, [string]$expectedHash) {
    $temp = "$to.new"
    try {
        Copy-Item -LiteralPath $from -Destination $temp -Force
        Unblock-File -LiteralPath $temp -ErrorAction SilentlyContinue
        $got = Get-Sha256 $temp
        if ($got -ne $expectedHash) { Fail "The copy of $from does not match the original ($got, expected $expectedHash). Nothing was changed." }
        Move-Item -LiteralPath $temp -Destination $to -Force
    }
    catch [System.UnauthorizedAccessException] {
        Fail "Windows refused to write $to. Run the installer as administrator, or check that no program has the file open."
    }
    finally {
        if (Test-Path -LiteralPath $temp) { Remove-Item -LiteralPath $temp -Force -ErrorAction SilentlyContinue }
    }
    if ((Get-Sha256 $to) -ne $expectedHash) { Fail "$to does not hold what was written to it. Check the folder by hand before starting the game." }
}

function Read-Marker([string]$path) {
    if (-not (Test-Path -LiteralPath $path)) { return $null }
    $parts = ((Get-Content -LiteralPath $path -TotalCount 1) -split '\s+')
    if ($parts.Count -lt 1 -or $parts[0] -notmatch '^[0-9a-f]{64}$') { return $null }
    return $parts[0]
}

# ---------------------------------------------------------------------------------------------
# The three actions
# ---------------------------------------------------------------------------------------------

function Describe([string]$hash, $builds, $payload, [string]$marker) {
    if ($builds.ContainsKey($hash)) { return "the game's own library, build $($builds[$hash])" }
    if ($hash -eq $payload.Hash) { return "the replacement library, version $($payload.Version) (this package)" }
    if ($marker -and $hash -eq $marker) { return 'the replacement library, installed from another release' }
    return 'not recognised: neither a listed game build nor a release of the replacement'
}

function Show-Status($where, $builds, $payload) {
    $live = Join-Path $where.Plugins 'SimDLL.dll'
    $backup = "$live.vanilla"
    $marker = Read-Marker "$live.sdk"
    $hash = Get-Sha256 $live
    Write-Host "Game:        $($where.Game)"
    Write-Host "SimDLL.dll:  $(Short $hash)  $(Describe $hash $builds $payload $marker)"
    if (Test-Path -LiteralPath $backup) {
        $b = Get-Sha256 $backup
        $what = if ($builds.ContainsKey($b)) { "the game's own library, build $($builds[$b])" } else { 'not a listed game build' }
        Write-Host "Backup:      $(Short $b)  $what"
    }
    else {
        Write-Host 'Backup:      none'
    }
    Write-Host "Supports:    game build $(($builds.Values | Sort-Object -Unique) -join ', ')"
}

function Install-Replacement($where, $builds, $payload) {
    $live = Join-Path $where.Plugins 'SimDLL.dll'
    $backup = "$live.vanilla"
    $markerPath = "$live.sdk"
    $marker = Read-Marker $markerPath
    $hash = Get-Sha256 $live
    $supported = ($builds.Values | Sort-Object -Unique) -join ', '

    if ($hash -eq $payload.Hash) {
        if (-not (Test-Path -LiteralPath $backup)) {
            Write-Warning "The replacement is installed but there is no SimDLL.dll.vanilla beside it. To go back to the game's own library, use Steam's 'Verify integrity of game files'."
        }
        Write-Host "Already installed: version $($payload.Version). Nothing to do."
        return
    }

    if ($builds.ContainsKey($hash)) {
        # The game's own library for a supported build. Keep it before anything else happens.
        if (Test-Path -LiteralPath $backup) {
            $old = Get-Sha256 $backup
            if ($old -ne $hash) { Write-Host "Replacing an older backup ($(Short $old)) with the game's current library." }
        }
        Copy-Checked $live $backup $hash
        Write-Host "Kept the game's own library (build $($builds[$hash])) as SimDLL.dll.vanilla."
    }
    elseif ((Test-Path -LiteralPath $backup) -and $builds.ContainsKey((Get-Sha256 $backup))) {
        # A verified backup exists, so SimDLL.dll should be a replacement this installer put there.
        if ($marker -ne $hash -and -not $Force) {
            Fail ("SimDLL.dll ($(Short $hash)) has changed since the replacement was installed, and it is not a game build this release supports ($supported). " +
                  "If the game has just been updated, the update put the game's own library back and this release does not support the new build yet. " +
                  "Nothing was changed. Run with -Force to install anyway.")
        }
        Write-Host "Updating the replacement library ($(Short $hash)); the game's own library is already kept as SimDLL.dll.vanilla."
    }
    else {
        if (-not $Force) {
            Fail ("This game build is not one this release supports. The game's SimDLL.dll has SHA-256 $hash; this release supports game build $supported. " +
                  "The library exchanges fixed-layout messages with the game, and on a different build it may misread them. Nothing was changed. " +
                  "Check the compatibility guide for a release that supports your build, or run with -Force to install anyway.")
        }
        Write-Warning "Installing on an unsupported game build because -Force was given."
        if (-not (Test-Path -LiteralPath $backup)) {
            Copy-Checked $live $backup $hash
            Write-Host 'Kept the current SimDLL.dll as SimDLL.dll.vanilla.'
        }
    }

    Copy-Checked $payload.Path $live $payload.Hash
    Set-Content -LiteralPath $markerPath -Value "$($payload.Hash) $($payload.Version)" -Encoding ASCII
    Write-Host "Installed the replacement library, version $($payload.Version) (SHA-256 $($payload.Hash))."
    Write-Host 'To go back to the game''s own library, run uninstall.cmd.'
}

function Uninstall-Replacement($where, $builds, $payload) {
    $live = Join-Path $where.Plugins 'SimDLL.dll'
    $backup = "$live.vanilla"
    $markerPath = "$live.sdk"
    $marker = Read-Marker $markerPath
    $hash = Get-Sha256 $live

    if (-not (Test-Path -LiteralPath $backup)) {
        if ($builds.ContainsKey($hash)) {
            Remove-Item -LiteralPath $markerPath -Force -ErrorAction SilentlyContinue
            Write-Host "The game's own library is already in place. Nothing to do."
            return
        }
        Fail "There is no SimDLL.dll.vanilla to restore. In Steam, open the game's Properties, then Installed Files, and choose 'Verify integrity of game files'; Steam restores the game's own SimDLL.dll."
    }

    $backupHash = Get-Sha256 $backup
    $ours = ($hash -eq $payload.Hash) -or ($marker -and $hash -eq $marker)

    if ($hash -eq $backupHash -or ($builds.ContainsKey($hash) -and -not $ours)) {
        # Steam's file check or a game update already put the game's own library back.
        Remove-Item -LiteralPath $backup, $markerPath -Force -ErrorAction SilentlyContinue
        Write-Host "The game's own library is already in place. Removed the backup, which is no longer needed."
        return
    }

    if (-not $ours -and -not $Force) {
        Fail ("SimDLL.dll ($(Short $hash)) is not a replacement this installer put there, so it was left alone, and so was the backup. " +
              "If the game has been updated since the install, the backup is for an older build and must not be restored: use Steam's 'Verify integrity of game files' instead. " +
              "Run with -Force to restore the backup anyway.")
    }

    Copy-Checked $backup $live $backupHash
    Remove-Item -LiteralPath $backup, $markerPath -Force -ErrorAction SilentlyContinue
    $what = if ($builds.ContainsKey($backupHash)) { " (build $($builds[$backupHash]))" } else { '' }
    Write-Host "Restored the game's own library$what. The replacement is uninstalled."
}

# ---------------------------------------------------------------------------------------------

try {
    $builds = Read-SupportedBuilds
    $payload = Read-Payload
    $where = Resolve-Plugins
    if ($Status) {
        Show-Status $where $builds $payload
        exit 0
    }
    Assert-GameClosed
    Write-Host "Game: $($where.Game)"
    if ($Uninstall) { Uninstall-Replacement $where $builds $payload }
    else { Install-Replacement $where $builds $payload }
    exit 0
}
catch [InstallerError] {
    Write-Host "Error: $($_.Exception.Message)" -ForegroundColor Red
    exit 1
}
catch {
    Write-Host "Unexpected error: $($_.Exception.Message)" -ForegroundColor Red
    Write-Host 'Run the installer with -Status to see the current state before trying again.'
    exit 2
}
