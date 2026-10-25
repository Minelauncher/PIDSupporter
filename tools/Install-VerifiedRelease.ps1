param([Parameter(Mandatory=$true)][string]$ContentPath)
$ErrorActionPreference='Stop'
if(Get-Process From_The_Depths -ErrorAction SilentlyContinue){throw 'Close the game before installing.'}
$source=(Resolve-Path -LiteralPath $ContentPath).Path
$target='C:\Users\kyhyg\OneDrive\Documents\From The Depths\Mods\PIDSupporter'
if(-not(Test-Path -LiteralPath (Join-Path $target 'PIDSupporterCore.dll'))){throw 'Expected installed mod not found.'}
$backup=Join-Path (Split-Path $PSScriptRoot -Parent) ('installation-backups\before-no-response-cap-'+(Get-Date -Format 'yyyyMMdd-HHmmss'))
$files=@('plugin.json','header.header','header.jpg','AUTOTUNE.txt','AUTOTUNE_THEORY_KO.md','PIDSupporterCore.dll','PIDSupporterSelector.dll','0Harmony.dll')
foreach($name in $files){if(-not(Test-Path -LiteralPath (Join-Path $source $name))){throw "Missing release file: $name"}}
New-Item -ItemType Directory -Path $backup | Out-Null
$entries=@()
foreach($name in $files){
 $dest=Join-Path $target $name
 $before=$null
 if(Test-Path -LiteralPath $dest){$before=(Get-FileHash -LiteralPath $dest).Hash;Copy-Item -LiteralPath $dest -Destination (Join-Path $backup $name);if((Get-FileHash -LiteralPath (Join-Path $backup $name)).Hash -ne $before){throw 'Backup hash mismatch'}}
 $entries+=@{name=$name;before=$before;after=(Get-FileHash -LiteralPath (Join-Path $source $name)).Hash}
}
foreach($entry in $entries){Copy-Item -LiteralPath (Join-Path $source $entry.name) -Destination (Join-Path $target $entry.name) -Force;if((Get-FileHash -LiteralPath (Join-Path $target $entry.name)).Hash -ne $entry.after){throw 'Installed hash mismatch'}}
@{source=$source;target=$target;backup=$backup;files=$entries;installed=(Get-Date).ToString('o')} | ConvertTo-Json -Depth 5 | Set-Content -LiteralPath (Join-Path $backup 'installation.json') -Encoding utf8
Write-Output "Installed and verified eight files. Backup=$backup"
