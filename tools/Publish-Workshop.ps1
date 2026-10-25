param([switch]$Publish,[string]$ChangeNotes='changelog.en.txt',[string]$ReleasePath)
$ErrorActionPreference='Stop'
$source=Split-Path $PSScriptRoot -Parent
$project=Split-Path $source -Parent
$workshop=Join-Path $source 'workshop'
$release=if($ReleasePath){(Resolve-Path -LiteralPath $ReleasePath).Path}else{throw 'Specify the verified package with -ReleasePath.'}
$version=(Get-Content -LiteralPath (Join-Path $release 'plugin.json') -Raw | ConvertFrom-Json).version
$content=Join-Path $workshop ('content-'+$version+'-md')
$releaseRecord=Join-Path $workshop ('release-'+$version+'.json')
if(-not(Test-Path -LiteralPath $releaseRecord)){
 @{version=$version;headerVersion=[guid]::NewGuid().ToString()} | ConvertTo-Json | Set-Content -LiteralPath $releaseRecord -Encoding utf8
}
$record=Get-Content -LiteralPath $releaseRecord -Raw | ConvertFrom-Json
New-Item -ItemType Directory -Path $content -Force | Out-Null
$files=@('plugin.json','header.header','AUTOTUNE.txt','AUTOTUNE_THEORY_KO.md','PIDSupporterCore.dll','PIDSupporterSelector.dll','0Harmony.dll')
foreach($name in $files){
 $fileSource=if($name -eq 'AUTOTUNE_THEORY_KO.md'){Join-Path $source $name}else{Join-Path $release $name}
 Copy-Item -LiteralPath $fileSource -Destination (Join-Path $content $name) -Force
}
Copy-Item -LiteralPath (Join-Path $workshop 'preview.jpg') -Destination (Join-Path $content 'header.jpg') -Force
$header=Get-Content -LiteralPath (Join-Path $content 'header.header') -Raw | ConvertFrom-Json
$header.Version=$record.headerVersion
$header.Description='In-game PID tune helper. Review suggestions, apply manually, and restore previous settings.'
$metadata=Get-Content -LiteralPath (Join-Path $content 'plugin.json') -Raw | ConvertFrom-Json
$metadata.workshop_id=3735379602
$metadata.gameversion='4.3.4.1'
[IO.File]::WriteAllText((Join-Path $content 'plugin.json'),($metadata | ConvertTo-Json -Compress))
[IO.File]::WriteAllText((Join-Path $content 'header.header'),($header | ConvertTo-Json -Depth 5 -Compress))
$hashes=[ordered]@{}
foreach($name in ($files+'header.jpg')){$hashes[$name]=(Get-FileHash -LiteralPath (Join-Path $content $name)).Hash}
$gallery=@()
$jobPath=Join-Path $workshop 'publish-job.json'
@{title='PIDSupporter';version=$version;content=$content;preview=(Join-Path $content 'header.jpg');gallery=@($gallery);description=(Join-Path $workshop 'description-bbcode.txt');changeNotes=(Join-Path $workshop $ChangeNotes);statePath=(Join-Path $workshop 'publish-state.json');hashes=$hashes} | ConvertTo-Json -Depth 5 | Set-Content -LiteralPath $jobPath -Encoding utf8
Write-Output "Prepared $content (eight public release files, no account data)."
if(-not $Publish){Write-Output 'Preparation only; no Steam requests made.';return}
if(Get-Process From_The_Depths -ErrorAction SilentlyContinue){throw 'User game is running; no process will be stopped.'}
$root=Join-Path ([IO.Path]::GetTempPath()) ('FR\publish-'+[guid]::NewGuid().ToString('N').Substring(0,10))
$save=Join-Path $root 'SaveData\From The Depths'
$mods=Join-Path $save 'Mods'
$publisher=Join-Path $mods 'WorkshopPublisher'
New-Item -ItemType Directory -Path $publisher -Force | Out-Null
$profile=Join-Path $project 'RankIsolationProbe\A\SaveData\From The Depths'
foreach($name in @('Options.dat','Player Profiles')){Copy-Item -LiteralPath (Join-Path $profile $name) -Destination $save -Recurse}
Copy-Item -LiteralPath (Join-Path $PSScriptRoot 'WorkshopPublisher\bin\Release\netstandard2.1\WorkshopPublisher.dll') -Destination $publisher
@{name='WorkshopPublisher';version='0.1.0';gameversion='4.3.4.1';filename='WorkshopPublisher.dll';depends=@();conflicts=@()} | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $publisher 'plugin.json')
@{Authors='Local publishing tool';Version='00000000-0000-0000-0000-000000000000';ComponentId=@{Guid='d9920ad2-6afc-4821-b88c-8148e6a5f49b';Name='WorkshopPublisher'}} | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $publisher 'header.header')
Set-Content -LiteralPath (Join-Path $root 'publisher-job-path.txt') -Value $jobPath -NoNewline -Encoding utf8
Set-Content -LiteralPath (Join-Path $root 'SaveLocation.txt') -Value (Join-Path $root 'SaveData') -NoNewline -Encoding utf8
Copy-Item -LiteralPath (Join-Path $project 'RankIsolationProbe\A\steam_appid.txt') -Destination $root
@{Settings=@(@{'$type'='BrilliantSkies.Boot.Settings.BootSetting_NoSplashScreen, Ftd'});Instructions=@();DeleteOnRead=$false} | ConvertTo-Json -Depth 5 | Set-Content -LiteralPath (Join-Path $root 'boot.json')
$game=$null
try {
 $arguments=@(('"'+(Join-Path $root 'boot.json')+'"'),'-batchmode','-nosound','-screen-fullscreen','0','-screen-width','1280','-screen-height','800','-logFile',('"'+(Join-Path $root 'game.log')+'"'))
 $game=Start-Process -FilePath 'C:\Program Files (x86)\Steam\steamapps\common\From The Depths\From_The_Depths.exe' -WorkingDirectory $root -ArgumentList $arguments -WindowStyle Hidden -PassThru
 Write-Output "PublisherRoot=$root GamePID=$($game.Id)"
 $deadline=(Get-Date).AddSeconds(300)
 while(-not(Test-Path -LiteralPath (Join-Path $root 'publisher-done.json'))){
  if($game.WaitForExit(1000)){
   if(Test-Path -LiteralPath (Join-Path $root 'publisher-done.json')){break}
   throw 'Publisher exited before completion; inspect the saved publishing journal before retrying.'
  }
  if((Get-Date) -gt $deadline){throw 'Publisher timed out; inspect the saved item ID before retrying.'}
 }
 Get-Content -LiteralPath (Join-Path $root 'publisher-done.json')
} finally {
 if($game -and -not $game.HasExited){if(-not $game.WaitForExit(5000)){Stop-Process -Id $game.Id;[void]$game.WaitForExit(5000)}}
 $archive=Join-Path $workshop ('publish-runs\'+(Get-Date -Format 'yyyyMMdd-HHmmss'))
 New-Item -ItemType Directory -Path $archive -Force | Out-Null
 foreach($name in @('game.log','publisher-progress.json','publisher-done.json')){
  if(Test-Path -LiteralPath (Join-Path $root $name)){Copy-Item -LiteralPath (Join-Path $root $name) -Destination $archive}
 }
 if(Test-Path -LiteralPath (Join-Path $save 'Logs')){Copy-Item -LiteralPath (Join-Path $save 'Logs') -Destination $archive -Recurse}
 Write-Output "Archive=$archive"
}
