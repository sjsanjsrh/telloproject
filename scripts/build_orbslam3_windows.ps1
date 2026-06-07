param(
	[string]$WorkDir = "third_party\orbslam3_windows",
	[string]$RepoUrl = "https://github.com/rexdsp/ORB_SLAM3_Windows.git",
	[string]$RepoRef = "",
	[string]$Configuration = "Release",
	[string]$Platform = "x64",
	[string]$VsDevCmd = "",
	[switch]$SkipClone,
	[switch]$SkipBuild
)

$ErrorActionPreference = "Stop"

$projectRoot = Resolve-Path (Join-Path $PSScriptRoot "..")
$workRoot = Join-Path $projectRoot $WorkDir
$orbRoot = Join-Path $workRoot "ORB_SLAM3"
$solution = Join-Path $orbRoot "slam.sln"
$installDir = Join-Path $projectRoot "third_party\orbslam3_live"
$exeOut = Join-Path $installDir "orbslam3_live.exe"

function Require-Command($name) {
	if ($null -eq (Get-Command $name -ErrorAction SilentlyContinue)) {
		throw "$name was not found in PATH"
	}
}

function Find-VsDevCmd {
	if ($VsDevCmd.Trim().Length -gt 0) {
		if (-not (Test-Path $VsDevCmd)) {
			throw "VsDevCmd.bat not found: $VsDevCmd"
		}
		return $VsDevCmd
	}

	$vswhere = "C:\Program Files (x86)\Microsoft Visual Studio\Installer\vswhere.exe"
	if (Test-Path $vswhere) {
		$installPath = & $vswhere -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
		if ($LASTEXITCODE -eq 0 -and $installPath) {
			$candidate = Join-Path $installPath "Common7\Tools\VsDevCmd.bat"
			if (Test-Path $candidate) {
				return $candidate
			}
		}
	}

	foreach ($candidate in @(
		"C:\Program Files (x86)\Microsoft Visual Studio\2019\BuildTools\Common7\Tools\VsDevCmd.bat",
		"C:\Program Files\Microsoft Visual Studio\2022\Community\Common7\Tools\VsDevCmd.bat"
	)) {
		if (Test-Path $candidate) {
			return $candidate
		}
	}
	throw "Visual Studio C++ Build Tools were not found"
}

function Set-TextFile($path, $text) {
	[System.IO.File]::WriteAllText($path, $text, [System.Text.UTF8Encoding]::new($false))
}

function Patch-MatrixOperations {
	$path = Join-Path $orbRoot "g2o\core\matrix_operations.h"
	$text = Get-Content $path -Raw
	$text = $text -replace '(?m)^\s*template<>\r?\n(\s*inline void axpy\(const Eigen::MatrixXd&)', '$1'
	$text = $text -replace '(?m)^\s*template<>\r?\n(\s*inline void atxpy\(const Eigen::MatrixXd&)', '$1'
	Set-TextFile $path $text
}

function Patch-ProjectSettings {
	$path = Join-Path $orbRoot "slam.vcxproj"
	$text = Get-Content $path -Raw
	$releasePropertyGroupPattern = "(<PropertyGroup Condition=`"'`$\(Configuration\)\|`$\(Platform\)'=='Release\|x64'`" Label=`"Configuration`">[\s\S]*?)<WholeProgramOptimization>true</WholeProgramOptimization>"
	$text = [regex]::Replace($text, $releasePropertyGroupPattern, '$1<WholeProgramOptimization>false</WholeProgramOptimization>')
	if ($text -notmatch "<Optimization>Disabled</Optimization>") {
		$itemDefinitionPattern = "(<ItemDefinitionGroup Condition=`"'`$\(Configuration\)\|`$\(Platform\)'=='Release\|x64'`">\s*<ClCompile>\s*<WarningLevel>Level3</WarningLevel>)"
		$text = [regex]::Replace($text, $itemDefinitionPattern, '$1' + "`r`n      <Optimization>Disabled</Optimization>")
	}
	Set-TextFile $path $text
}

function Patch-LiveMain {
	$path = Join-Path $orbRoot "main.cpp"
	$text = Get-Content $path -Raw
	if ($text -match "live_mono_main") {
		return
	}

	$includePatch = @'
#include "global.h"

#include <chrono>
#include <fstream>
#include <map>
#include <stdexcept>
#include <string>
#include <thread>

#include <opencv2/opencv.hpp>

#include "slam/System.h"
'@
	$text = $text.Replace('#include "global.h"', $includePatch)

	$livePatch = @'

namespace {

std::map<std::string, std::string> parse_live_args(int argc, char** argv)
{
    std::map<std::string, std::string> args;
    for (int i = 1; i < argc; ++i) {
        std::string key = argv[i];
        if (key.rfind("--", 0) != 0 || i + 1 >= argc) {
            continue;
        }
        args[key.substr(2)] = argv[++i];
    }
    return args;
}

std::string live_arg(const std::map<std::string, std::string>& args, const std::string& key)
{
    const auto it = args.find(key);
    if (it == args.end() || it->second.empty()) {
        throw std::runtime_error("missing --" + key);
    }
    return it->second;
}

double live_now_seconds()
{
    using clock = std::chrono::steady_clock;
    static const auto start = clock::now();
    const auto elapsed = clock::now() - start;
    return std::chrono::duration<double>(elapsed).count();
}

cv::VideoCapture open_live_source(const std::string& source)
{
    if (source.size() == 1 && std::isdigit(static_cast<unsigned char>(source[0]))) {
        return cv::VideoCapture(std::stoi(source));
    }
    return cv::VideoCapture(source);
}

void write_live_pose(std::ofstream& out, const cv::Mat& tcw)
{
    if (tcw.empty()) {
        out << "{\"timestamp\":" << live_now_seconds()
            << ",\"tracking_state\":\"LOST\"}" << std::endl;
        return;
    }

    cv::Mat pose64;
    tcw.convertTo(pose64, CV_64F);
    cv::Mat twc = pose64.inv();

    out << "{\"timestamp\":" << live_now_seconds()
        << ",\"tracking_state\":\"OK\""
        << ",\"position\":[" << twc.at<double>(0, 3) << "," << twc.at<double>(1, 3) << "," << twc.at<double>(2, 3) << "]"
        << ",\"rotation_matrix\":[";
    for (int r = 0; r < 3; ++r) {
        if (r > 0) out << ",";
        out << "[";
        for (int c = 0; c < 3; ++c) {
            if (c > 0) out << ",";
            out << twc.at<double>(r, c);
        }
        out << "]";
    }
    out << "]}" << std::endl;
}

int live_mono_main(int argc, char** argv)
{
    try {
        const auto args = parse_live_args(argc, argv);
        const std::string mode = args.count("mode") ? args.at("mode") : "mono";
        if (mode != "mono") {
            throw std::runtime_error("live mode currently supports --mode mono");
        }

        const std::string vocab = live_arg(args, "vocab");
        const std::string settings = live_arg(args, "settings");
        const std::string source = live_arg(args, "source");
        const std::string pose_out = live_arg(args, "pose-out");

        cv::VideoCapture capture = open_live_source(source);
        if (!capture.isOpened()) {
            throw std::runtime_error("failed to open source: " + source);
        }

        std::ofstream pose_file(pose_out, std::ios::out | std::ios::app);
        if (!pose_file.is_open()) {
            throw std::runtime_error("failed to open pose output: " + pose_out);
        }

        ORB_SLAM3::System slam(vocab, settings, ORB_SLAM3::System::MONOCULAR, false);

        cv::Mat frame;
        while (capture.read(frame)) {
            if (frame.empty()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
                continue;
            }
            const cv::Mat tcw = slam.TrackMonocular(frame, live_now_seconds());
            write_live_pose(pose_file, tcw);
        }

        slam.Shutdown();
        return 0;
    } catch (const std::exception& exc) {
        std::cerr << "orbslam3 live error: " << exc.what() << std::endl;
        return 1;
    }
}

}  // namespace

'@
	$text = [regex]::Replace($text, '(?s)(int stereo_inertial_tum_vi\(int argc, char\*\* argv\);\s*)', '$1' + $livePatch)
	$text = [regex]::Replace($text, '(int main\(int argc, char\*\* argv\)\s*\{\s*)', '$1' + "`r`n    if (argc > 1 && 0 == strcmp(argv[1], `"--mode`")) {`r`n        return live_mono_main(argc, argv);`r`n    }`r`n")
	Set-TextFile $path $text
}

function Invoke-VsBuild {
	$devCmd = Find-VsDevCmd
	$tempCmd = Join-Path $env:TEMP "build_orbslam3_live_$PID.cmd"
	@"
@echo off
setlocal
call "$devCmd" -arch=amd64 || exit /b 1
cd /d "$projectRoot" || exit /b 1
msbuild "$solution" /p:Configuration=$Configuration /p:Platform=$Platform /m
if errorlevel 1 exit /b %errorlevel%
"@ | Set-Content -Encoding ASCII $tempCmd
	try {
		& cmd.exe /c $tempCmd
		if ($LASTEXITCODE -ne 0) {
			throw "MSBuild failed with exit code $LASTEXITCODE"
		}
	} finally {
		Remove-Item -Force $tempCmd -ErrorAction SilentlyContinue
	}
}

Require-Command git

Write-Host "Windows ORB-SLAM3 one-click build"
Write-Host "  repo:       $RepoUrl"
Write-Host "  work dir:   $workRoot"
Write-Host "  output exe: $exeOut"
Write-Host ""

New-Item -ItemType Directory -Force -Path $workRoot | Out-Null
New-Item -ItemType Directory -Force -Path $installDir | Out-Null

if (-not $SkipClone -and -not (Test-Path $orbRoot)) {
	Write-Host "Cloning ORB-SLAM3 Windows source..."
	git clone $RepoUrl $orbRoot
}

if ($RepoRef.Trim().Length -gt 0) {
	Push-Location $orbRoot
	git fetch --all --tags
	git checkout $RepoRef
	Pop-Location
}

if (-not (Test-Path $solution)) {
	throw "ORB-SLAM3 Windows solution was not found: $solution"
}

Write-Host "Applying Windows/MSVC live-mode patches..."
Patch-MatrixOperations
Patch-ProjectSettings
Patch-LiveMain

if (-not $SkipBuild) {
	Write-Host "Building ORB-SLAM3 live executable..."
	Invoke-VsBuild
}

$builtExe = Join-Path $orbRoot "x64\$Configuration\slam.exe"
if (-not (Test-Path $builtExe)) {
	throw "Built executable not found: $builtExe"
}

Copy-Item -Force $builtExe $exeOut

$opencvDll = Join-Path $orbRoot "x64\$Configuration\opencv_world320.dll"
if (Test-Path $opencvDll) {
	Copy-Item -Force $opencvDll (Join-Path $installDir "opencv_world320.dll")
}
$ffmpegDll = Join-Path $orbRoot "opencv\build\x64\vc14\bin\opencv_ffmpeg320_64.dll"
if (Test-Path $ffmpegDll) {
	Copy-Item -Force $ffmpegDll (Join-Path $installDir "opencv_ffmpeg320_64.dll")
}

Write-Host ""
Write-Host "Done."
Write-Host "Executable: $exeOut"
Write-Host ""
Write-Host "Run:"
Write-Host "  .\.venv\Scripts\python.exe yolo_seg\pnp_pos_est_viz.py --source tello --slam-backend orbslam3"
