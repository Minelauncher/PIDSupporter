# MakeRelease.ps1 — Workshop 업로드용 release 폴더 생성
#
# Usage:
#   PowerShell -File MakeRelease.ps1
#   또는 PowerShell 창에서:  .\MakeRelease.ps1
#
# 동작:
#   1. PIDSupporterCore 를 Release 빌드
#   2. PIDSupporter_Release/ 폴더 정리/생성
#   3. Workshop 업로드 필요 파일만 복사
#   → FTD in-game UI 에서 그 폴더 선택해 업로드

$ErrorActionPreference = "Stop"

$root = Split-Path -Parent $MyInvocation.MyCommand.Path
$release = Join-Path $root "PIDSupporter_Release"
$proj = Join-Path $root "PIDSupporterCore\PIDSupporterCore.csproj"

Write-Host "[1/3] Building Release..." -ForegroundColor Cyan
$buildLog = & dotnet build $proj -c Release 2>&1
if ($LASTEXITCODE -ne 0) {
    # post-build copy 가 게임 잠금 등으로 실패해도 CS 에러 없으면 진행
    $csErrors = $buildLog | Select-String "error CS\d+"
    if ($csErrors) {
        Write-Host "Build failed with CS errors:" -ForegroundColor Red
        $csErrors | ForEach-Object { Write-Host "  $_" -ForegroundColor Red }
        exit 1
    }
    Write-Host "  (post-build copy may have failed but compile succeeded — continuing)" -ForegroundColor Yellow
}
Write-Host "  ok" -ForegroundColor Green

Write-Host "[2/3] Preparing $release ..." -ForegroundColor Cyan
if (Test-Path $release) {
    Remove-Item "$release\*" -Recurse -Force
} else {
    New-Item -ItemType Directory -Path $release | Out-Null
}
Write-Host "  ok" -ForegroundColor Green

Write-Host "[3/3] Copying release files..." -ForegroundColor Cyan
$files = @(
    "0Harmony.dll",
    "MathNet.Numerics.dll",
    "PIDSupporterCore.dll",
    "PIDSupporterSelector.dll",
    "header.header",
    "header.jpg",          # Steam Workshop preview image (< 1MB, 권장 512x512)
    "plugin.json",
    "THEORY.md"
)

$missing = @()
foreach ($f in $files) {
    $src = Join-Path $root $f
    if (-not (Test-Path $src)) {
        $missing += $f
        continue
    }
    Copy-Item $src $release
    Write-Host "  + $f" -ForegroundColor Gray
}

if ($missing.Count -gt 0) {
    Write-Host ""
    Write-Host "WARNING: Missing files:" -ForegroundColor Yellow
    foreach ($f in $missing) {
        Write-Host "  - $f" -ForegroundColor Yellow
    }
}

Write-Host ""
Write-Host "Release folder ready:" -ForegroundColor Green
Write-Host "  $release" -ForegroundColor Green
Write-Host ""
Write-Host "Upload via FTD in-game Workshop UI (select this folder)." -ForegroundColor Cyan
