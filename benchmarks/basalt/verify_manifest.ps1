[CmdletBinding()]
param(
  [Parameter(Mandatory = $true)]
  [string] $OracleRoot,
  [string] $ManifestPath = ''
)

$ErrorActionPreference = 'Stop'
if ([string]::IsNullOrWhiteSpace($ManifestPath)) {
  $ManifestPath = Join-Path -Path $PSScriptRoot -ChildPath 'upstream_manifest.json'
}
$manifest = Get-Content -Raw -LiteralPath $ManifestPath | ConvertFrom-Json
$expectedCommit = [string] $manifest.repository.commit
$expectedTree = [string] $manifest.repository.tree_sha1
$source = (Resolve-Path -LiteralPath $OracleRoot).Path

function Assert-Equal([string] $Label, [string] $Actual, [string] $Expected) {
  if ($Actual -ne $Expected) {
    throw "$Label mismatch: expected $Expected, got $Actual"
  }
  Write-Output "PASS $Label = $Actual"
}

function Invoke-GitText([string] $Repository, [string[]] $Arguments) {
  $rawOutput = & git --no-pager -C $Repository @Arguments 2>&1
  $exitCode = $LASTEXITCODE
  if ($exitCode -ne 0) {
    throw "git -C $Repository $($Arguments -join ' ') failed with exit code ${exitCode}: $(@($rawOutput | ForEach-Object { $_.ToString() }) -join ' ')"
  }
  return (@($rawOutput | ForEach-Object { $_.ToString() }) -join "`n").Trim()
}

function Get-LfNormalizedSha256([string] $Path) {
  $bytes = [IO.File]::ReadAllBytes($Path)
  $normalized = New-Object 'System.Collections.Generic.List[byte]'
  for ($index = 0; $index -lt $bytes.Length; $index++) {
    if ($bytes[$index] -eq 13 -and ($index + 1) -lt $bytes.Length -and $bytes[$index + 1] -eq 10) {
      continue
    }
    [void] $normalized.Add($bytes[$index])
  }
  $sha = [Security.Cryptography.SHA256]::Create()
  try {
    return (-join ($sha.ComputeHash($normalized.ToArray()) | ForEach-Object { $_.ToString('x2') }))
  } finally {
    $sha.Dispose()
  }
}

Assert-Equal 'upstream commit' (Invoke-GitText $source @('rev-parse', 'HEAD')) $expectedCommit
Assert-Equal 'upstream tree' (Invoke-GitText $source @('rev-parse', 'HEAD^{tree}')) $expectedTree

$vcpkgPath = Join-Path $source 'thirdparty\vcpkg'
$vcpkgRecord = $manifest.repository.submodules.'thirdparty/vcpkg'
if ($null -eq $vcpkgRecord) {
  throw 'manifest is missing repository.submodules.thirdparty/vcpkg'
}
$expectedVcpkg = ([string] $vcpkgRecord.commit).ToLowerInvariant()
$expectedVcpkgMode = [string] $vcpkgRecord.gitlink_mode
$expectedVcpkgBinding = [string] $vcpkgRecord.binding
if ($expectedVcpkgMode -ne '160000') {
  throw "manifest vcpkg gitlink_mode must be 160000, got $expectedVcpkgMode"
}
if ($expectedVcpkgBinding -ne 'git_index_oid') {
  throw "manifest vcpkg binding must be git_index_oid, got $expectedVcpkgBinding"
}

$indexRecord = Invoke-GitText $source @('ls-files', '--stage', '--', 'thirdparty/vcpkg')
$indexLines = @($indexRecord -split "`r?`n" | Where-Object { -not [string]::IsNullOrWhiteSpace($_) })
if ($indexLines.Count -ne 1) {
  throw "expected exactly one indexed vcpkg gitlink record, got $($indexLines.Count)"
}
$indexMatch = [regex]::Match($indexLines[0].Trim(), '^(?<mode>[0-9]{6}) (?<oid>[0-9a-f]{40}) (?<stage>[0-3])\t(?<path>.+)$')
if (-not $indexMatch.Success -or $indexMatch.Groups['path'].Value -ne 'thirdparty/vcpkg' -or $indexMatch.Groups['stage'].Value -ne '0') {
  throw "invalid indexed vcpkg gitlink record: $($indexLines[0])"
}
Assert-Equal 'vcpkg gitlink index mode' $indexMatch.Groups['mode'].Value $expectedVcpkgMode
Assert-Equal 'vcpkg gitlink index OID' $indexMatch.Groups['oid'].Value.ToLowerInvariant() $expectedVcpkg

$treeRecord = Invoke-GitText $source @('ls-tree', 'HEAD', '--', 'thirdparty/vcpkg')
$treeLines = @($treeRecord -split "`r?`n" | Where-Object { -not [string]::IsNullOrWhiteSpace($_) })
if ($treeLines.Count -ne 1) {
  throw "expected exactly one HEAD vcpkg gitlink record, got $($treeLines.Count)"
}
$treeMatch = [regex]::Match($treeLines[0].Trim(), '^(?<mode>[0-9]{6}) (?<type>[a-z]+) (?<oid>[0-9a-f]{40})\t(?<path>.+)$')
if (-not $treeMatch.Success -or $treeMatch.Groups['path'].Value -ne 'thirdparty/vcpkg') {
  throw "invalid HEAD vcpkg gitlink record: $($treeLines[0])"
}
Assert-Equal 'vcpkg gitlink HEAD-tree mode' $treeMatch.Groups['mode'].Value $expectedVcpkgMode
Assert-Equal 'vcpkg gitlink HEAD-tree OID' $treeMatch.Groups['oid'].Value.ToLowerInvariant() $expectedVcpkg

if (Test-Path -LiteralPath $vcpkgPath -PathType Leaf) {
  throw "vcpkg gitlink path is a regular file, not a submodule directory: $vcpkgPath"
}
if (-not (Test-Path -LiteralPath $vcpkgPath -PathType Container)) {
  Write-Output "PASS vcpkg worktree absent; indexed/tree gitlink OID = $expectedVcpkg"
} elseif (Test-Path -LiteralPath (Join-Path $vcpkgPath '.git')) {
  $materializedHead = (Invoke-GitText $vcpkgPath @('rev-parse', 'HEAD')).ToLowerInvariant()
  Assert-Equal 'vcpkg materialized HEAD' $materializedHead $expectedVcpkg
  $materializedRoot = [IO.Path]::GetFullPath((Invoke-GitText $vcpkgPath @('rev-parse', '--show-toplevel')))
  $expectedRoot = [IO.Path]::GetFullPath($vcpkgPath)
  if (-not [StringComparer]::OrdinalIgnoreCase.Equals($materializedRoot, $expectedRoot)) {
    throw "vcpkg materialized toplevel mismatch: expected $expectedRoot, got $materializedRoot"
  }
  Write-Output "PASS vcpkg materialized worktree root = $materializedRoot"
} else {
  $children = @(Get-ChildItem -LiteralPath $vcpkgPath -Force)
  if ($children.Count -ne 0) {
    throw "vcpkg path has materialized contents without .git metadata: $vcpkgPath"
  }
  Write-Output "PASS vcpkg worktree empty; indexed/tree gitlink OID = $expectedVcpkg"
}

foreach ($property in $manifest.source_scope.key_file_sha256.psobject.Properties) {
  $path = Join-Path $source ($property.Name -replace '/', '\\')
  if (-not (Test-Path -LiteralPath $path -PathType Leaf)) {
    throw "missing source file: $($property.Name)"
  }
  $actual = Get-LfNormalizedSha256 $path
  Assert-Equal "source $($property.Name)" $actual ([string] $property.Value)
}

$config = $manifest.inputs.upstream_config
$configPath = Join-Path $source ($config.path -replace '/', '\\')
Assert-Equal 'upstream EuRoC config SHA-256' (Get-LfNormalizedSha256 $configPath) ([string] $config.sha256)

$calib = $manifest.inputs.upstream_calibration
$calibPath = Join-Path $source ($calib.path -replace '/', '\\')
Assert-Equal 'upstream EuRoC calibration SHA-256' (Get-LfNormalizedSha256 $calibPath) ([string] $calib.sha256)

Write-Output 'Basalt upstream provenance verification passed.'
