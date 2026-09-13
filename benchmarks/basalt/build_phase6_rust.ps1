[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [string] $TargetDir,

    [string] $ManifestPath,

    [string] $Cargo = "cargo"
)

$ErrorActionPreference = "Stop"
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$targetPath = [System.IO.Path]::GetFullPath($TargetDir)
if (-not [System.IO.Path]::IsPathRooted($targetPath)) {
    throw "TargetDir must resolve to an absolute path"
}
if ($targetPath.StartsWith($repoRoot + [System.IO.Path]::DirectorySeparatorChar, [System.StringComparison]::OrdinalIgnoreCase)) {
    throw "TargetDir must be outside the source checkout; use a dedicated run/build volume"
}
if (-not $ManifestPath) {
    $ManifestPath = Join-Path $targetPath "phase6_rust_build_manifest.json"
}
$manifestFullPath = [System.IO.Path]::GetFullPath($ManifestPath)
$cargoCommand = Get-Command $Cargo -ErrorAction Stop
$cargoExecutable = $cargoCommand.Source
$rustcExecutable = Join-Path (Split-Path -Parent $cargoExecutable) "rustc.exe"
if (-not (Test-Path -LiteralPath $rustcExecutable -PathType Leaf)) {
    throw "rustc.exe was not found beside cargo: $rustcExecutable"
}

$requiredFeatures = "basalt-lm-workspace-reuse"
$requiredRustFlags = "-C target-feature=+avx2,+fma"
$previousTargetDir = $env:CARGO_TARGET_DIR
$previousRustFlags = $env:RUSTFLAGS
$started = [DateTimeOffset]::UtcNow

try {
    $env:CARGO_TARGET_DIR = $targetPath
    $env:RUSTFLAGS = $requiredRustFlags
    Push-Location $repoRoot
    try {
        & $cargoExecutable build --locked --release --example basalt_euroc_vio_demo --features $requiredFeatures
        if ($LASTEXITCODE -ne 0) {
            throw "cargo build failed with exit code $LASTEXITCODE"
        }
    }
    finally {
        Pop-Location
    }
}
finally {
    if ($null -eq $previousTargetDir) {
        Remove-Item Env:CARGO_TARGET_DIR -ErrorAction SilentlyContinue
    } else {
        $env:CARGO_TARGET_DIR = $previousTargetDir
    }
    if ($null -eq $previousRustFlags) {
        Remove-Item Env:RUSTFLAGS -ErrorAction SilentlyContinue
    } else {
        $env:RUSTFLAGS = $previousRustFlags
    }
}

$executable = Join-Path $targetPath "release\examples\basalt_euroc_vio_demo.exe"
if (-not (Test-Path -LiteralPath $executable -PathType Leaf)) {
    throw "release executable missing after build: $executable"
}
$finished = [DateTimeOffset]::UtcNow
$gitHead = (& git -C $repoRoot rev-parse HEAD).Trim()
$gitStatus = @(& git -C $repoRoot status --porcelain=v1 --untracked-files=no)
$rustcVersion = @(& $rustcExecutable -vV)
$cargoVersion = (& $cargoExecutable -V).Trim()
$cpuNames = @(Get-CimInstance Win32_Processor | ForEach-Object { $_.Name.Trim() })
$binary = Get-Item -LiteralPath $executable
$binding = Get-FileHash -LiteralPath $executable -Algorithm SHA256

$manifest = [ordered]@{
    schema_id = "basalt.phase6.rust_release_build.v1"
    created_utc = $finished.ToString("o")
    source = [ordered]@{
        repo_root = $repoRoot
        git_head = $gitHead
        git_status_clean = ($gitStatus.Count -eq 0)
        git_status_porcelain = $gitStatus
    }
    target = [ordered]@{
        directory = $targetPath
        outside_source_checkout = $true
        executable = $binary.FullName
        executable_bytes = $binary.Length
        executable_sha256 = $binding.Hash.ToLowerInvariant()
    }
    build = [ordered]@{
        cargo = $cargoVersion
        rustc_vv = $rustcVersion
        command = @(
            "cargo", "build", "--locked", "--release", "--example",
            "basalt_euroc_vio_demo", "--features", $requiredFeatures
        )
        cargo_target_dir = $targetPath
        rustflags = $requiredRustFlags
        required_cpu_features = @("avx2", "fma")
        cargo_features = @($requiredFeatures)
        started_utc = $started.ToString("o")
        finished_utc = $finished.ToString("o")
        elapsed_seconds = ($finished - $started).TotalSeconds
    }
    host = [ordered]@{
        processor_names = $cpuNames
        os = [System.Environment]::OSVersion.VersionString
        architecture = [System.Runtime.InteropServices.RuntimeInformation]::OSArchitecture.ToString()
    }
}

$manifestDirectory = Split-Path -Parent $manifestFullPath
New-Item -ItemType Directory -Force -Path $manifestDirectory | Out-Null
$manifest | ConvertTo-Json -Depth 8 | Set-Content -LiteralPath $manifestFullPath -Encoding utf8
Write-Output $manifestFullPath
