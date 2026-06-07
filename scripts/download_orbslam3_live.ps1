param(
	[string]$Url = "",
	[string]$OutDir = "third_party\orbslam3_live",
	[string]$FileName = "orbslam3_live.exe",
	[string]$Sha256 = "",
	[switch]$DownloadVocabulary
)

$ErrorActionPreference = "Stop"

$officialRepo = "https://github.com/UZ-SLAMLab/ORB_SLAM3"
$officialReleases = "https://github.com/UZ-SLAMLab/ORB_SLAM3/releases"
$vocabularyUrl = "https://raw.githubusercontent.com/UZ-SLAMLab/ORB_SLAM3/master/Vocabulary/ORBvoc.txt.tar.gz"

Write-Host "ORB-SLAM3 official repo: $officialRepo"
Write-Host "ORB-SLAM3 official releases: $officialReleases"

$outPath = Join-Path $OutDir $FileName
New-Item -ItemType Directory -Force -Path $OutDir | Out-Null

function Download-OrbVocabulary {
	$vocArchive = Join-Path $OutDir "ORBvoc.txt.tar.gz"
	$vocOut = Join-Path $OutDir "ORBvoc.txt"
	Write-Host "Downloading ORB vocabulary..."
	Write-Host "  from: $vocabularyUrl"
	Write-Host "  to:   $vocArchive"
	Invoke-WebRequest -Uri $vocabularyUrl -OutFile $vocArchive
	Write-Host "Extracting ORBvoc.txt..."
	tar -xzf $vocArchive -C $OutDir
	if (!(Test-Path $vocOut)) {
		throw "ORBvoc.txt was not extracted to $vocOut"
	}
	Write-Host "Vocabulary ready: $vocOut"
}

if ($Url.Trim().Length -eq 0) {
	Write-Host ""
	Write-Host "No official Windows orbslam3_live.exe binary is published by UZ-SLAMLab."
	Write-Host "Build ORB-SLAM3, upload your orbslam3_live.exe as a release asset, then run:"
	Write-Host "  powershell -ExecutionPolicy Bypass -File scripts\download_orbslam3_live.ps1 -Url <exe-release-url>"
	Write-Host ""
	Write-Host "Expected output path:"
	Write-Host "  $outPath"
	if ($DownloadVocabulary) {
		Write-Host ""
		Download-OrbVocabulary
	} else {
		Write-Host ""
		Write-Host "Add -DownloadVocabulary to fetch and extract ORBvoc.txt from the official repo."
	}
	exit 0
}

Write-Host "Downloading ORB-SLAM3 live executable..."
Write-Host "  from: $Url"
Write-Host "  to:   $outPath"
Invoke-WebRequest -Uri $Url -OutFile $outPath

if ($Sha256.Trim().Length -gt 0) {
	$actualHash = (Get-FileHash -Algorithm SHA256 $outPath).Hash.ToLowerInvariant()
	$expectedHash = $Sha256.ToLowerInvariant()
	if ($actualHash -ne $expectedHash) {
		Remove-Item -Force $outPath
		throw "SHA256 mismatch. Expected $expectedHash but got $actualHash"
	}
	Write-Host "SHA256 verified: $actualHash"
}

if ($DownloadVocabulary) {
	Download-OrbVocabulary
}

Write-Host ""
Write-Host "Done."
Write-Host "Executable: $outPath"
Write-Host ""
Write-Host "Example:"
Write-Host "  .\.venv\Scripts\python.exe yolo_seg\pnp_pos_est_viz.py --source tello --slam-backend orbslam3 --orbslam3-exe `"$outPath`" --orbslam3-vocab `"$OutDir\ORBvoc.txt`" --orbslam3-settings path\to\tello_orbslam3.yaml"
