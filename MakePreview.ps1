# MakePreview.ps1 — Steam Workshop preview image 생성
#
# 입력: 같은 폴더의 header.jpg (스크린샷 또는 기타 이미지)
# 출력: 같은 파일을 512x512 + 텍스트 오버레이 로 덮어쓰기
#       (백업은 header.jpg.bak 로 자동 저장)
#
# Usage:  .\MakePreview.ps1
#         또는 PowerShell -File MakePreview.ps1

$ErrorActionPreference = "Stop"
Add-Type -AssemblyName System.Drawing

$root = Split-Path -Parent $MyInvocation.MyCommand.Path
$src = Join-Path $root "header.jpg"
$bak = Join-Path $root "header.jpg.bak"

if (-not (Test-Path $src)) {
    Write-Error "header.jpg 가 없습니다: $src"
    exit 1
}

# 백업 (덮어쓰기 안 함 — 이미 있으면 그대로)
if (-not (Test-Path $bak)) {
    Copy-Item $src $bak
    Write-Host "백업 저장: header.jpg.bak" -ForegroundColor Gray
}

Write-Host "원본 로드 중..." -ForegroundColor Cyan
$srcImg = [System.Drawing.Image]::FromFile($bak)   # 항상 백업에서 읽어서 멱등
$srcW = $srcImg.Width
$srcH = $srcImg.Height
Write-Host "  원본: ${srcW}x${srcH}" -ForegroundColor Gray

# 512x512 정사각형 (Steam Workshop 권장)
$size = 512
$bmp = New-Object System.Drawing.Bitmap($size, $size)
$g = [System.Drawing.Graphics]::FromImage($bmp)
$g.SmoothingMode = [System.Drawing.Drawing2D.SmoothingMode]::HighQuality
$g.InterpolationMode = [System.Drawing.Drawing2D.InterpolationMode]::HighQualityBicubic
$g.TextRenderingHint = [System.Drawing.Text.TextRenderingHint]::AntiAlias
$g.CompositingQuality = [System.Drawing.Drawing2D.CompositingQuality]::HighQuality

# 종횡비 fit (cover — 가장자리 crop)
$srcAspect = $srcW / $srcH
if ($srcAspect -gt 1.0) {
    # 원본이 더 wide → 세로 맞추고 좌우 crop
    $scale = $size / $srcH
    $drawW = $srcW * $scale
    $drawH = $size
    $drawX = ($size - $drawW) / 2
    $drawY = 0
} else {
    # 원본이 더 tall → 가로 맞추고 상하 crop
    $scale = $size / $srcW
    $drawW = $size
    $drawH = $srcH * $scale
    $drawX = 0
    $drawY = ($size - $drawH) / 2
}

$g.DrawImage($srcImg, [single]$drawX, [single]$drawY, [single]$drawW, [single]$drawH)

# ─── 텍스트 오버레이 ───
# 폰트 + 브러시 모두 [Type]::new() 구문 사용
# (New-Object System.Drawing.SolidBrush($colorObj) 형식은 PowerShell overload 실패해서
#  brush 가 null 되는 케이스 발견 — ::new() 가 가장 안정)
$titleFont    = [System.Drawing.Font]::new("Arial", [single]36, [System.Drawing.FontStyle]::Bold)
$subtitleFont = [System.Drawing.Font]::new("Arial", [single]13, [System.Drawing.FontStyle]::Regular)
$bandBrush    = [System.Drawing.SolidBrush]::new([System.Drawing.Color]::FromArgb(200, 0, 0, 0))
$titleBrush   = [System.Drawing.SolidBrush]::new([System.Drawing.Color]::White)
$subBrush     = [System.Drawing.SolidBrush]::new([System.Drawing.Color]::FromArgb(255, 220, 220, 220))

# 하단 검정 띠 (alpha 200 ≈ 78%)
$bandHeight = 110
$g.FillRectangle($bandBrush, [single]0, [single]($size - $bandHeight), [single]$size, [single]$bandHeight)

# 타이틀
$titleText = "PID Supporter"
$titleSize = $g.MeasureString($titleText, $titleFont)
$titleX = ($size - $titleSize.Width) / 2
$titleY = $size - $bandHeight + 8
$g.DrawString($titleText, $titleFont, $titleBrush, [single]$titleX, [single]$titleY)

# 부제
$subtitleText = "Auto PID Tuning - FRIT + Relay Feedback"
$subtitleSize = $g.MeasureString($subtitleText, $subtitleFont)
$subtitleX = ($size - $subtitleSize.Width) / 2
$subtitleY = $size - 38
$g.DrawString($subtitleText, $subtitleFont, $subBrush, [single]$subtitleX, [single]$subtitleY)

# 저장 (JPEG quality 90)
$encoder = [System.Drawing.Imaging.ImageCodecInfo]::GetImageEncoders() | Where-Object { $_.MimeType -eq "image/jpeg" }
$encParams = New-Object System.Drawing.Imaging.EncoderParameters(1)
$encParams.Param[0] = New-Object System.Drawing.Imaging.EncoderParameter([System.Drawing.Imaging.Encoder]::Quality, 90L)

$g.Flush()
$g.Dispose()
$srcImg.Dispose()
$bmp.Save($src, $encoder, $encParams)
$bmp.Dispose()

$outSize = (Get-Item $src).Length
Write-Host "완료: $src" -ForegroundColor Green
Write-Host "  크기: ${size}x${size}, $([math]::Round($outSize/1024, 1)) KB" -ForegroundColor Gray
