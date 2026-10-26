# Build a new local test package; never overwrite the installed mod or older packages.
param(
    [string]$GameManagedDir = "C:\Program Files (x86)\Steam\steamapps\common\From The Depths\From_The_Depths_Data\Managed"
)
$ErrorActionPreference = "Stop"
$projectRoot = $PSScriptRoot
foreach ($project in @("PIDSupporterCore", "PIDSupporterSelector")) {
    & dotnet build (Join-Path $projectRoot "$project/$project.csproj") -c Release --nologo "-p:GameManagedDir=$GameManagedDir"
    if ($LASTEXITCODE -ne 0) { throw "Build failed: $project" }
}
$package = Join-Path $projectRoot ("PIDSupporter_Release/native-linear-autotune-" + (Get-Date -Format "yyyyMMdd-HHmmss"))
New-Item -ItemType Directory -Path $package | Out-Null
foreach ($project in @("PIDSupporterCore", "PIDSupporterSelector")) {
    Copy-Item -LiteralPath (Join-Path $projectRoot "$project/bin/Release/netstandard2.1/$project.dll") -Destination $package
}
foreach ($name in @("0Harmony.dll", "header.header", "header.jpg", "AUTOTUNE.txt", "AUTOTUNE_THEORY_KO.md", "AUTOTUNE_THEORY_EN.md")) {
    Copy-Item -LiteralPath (Join-Path $projectRoot $name) -Destination $package
}
$metadata = Get-Content -LiteralPath (Join-Path $projectRoot "plugin.json") -Raw -Encoding utf8 | ConvertFrom-Json
$metadata.workshop_id = 0
$metadata | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $package "plugin.json") -Encoding utf8
Write-Output "Local test package: $package"
Write-Output "Not installed or published. Linear identification is entirely managed C#; no Python runtime."
