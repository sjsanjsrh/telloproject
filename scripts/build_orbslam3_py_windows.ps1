param(
	[string]$Configuration = "Release"
)

$ErrorActionPreference = "Stop"

$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$orbRoot = Join-Path $projectRoot "third_party\orbslam3_windows\ORB_SLAM3"
$moduleSource = Join-Path $projectRoot "cpp\orbslam3_py\orbslam3_py.cpp"
$outDir = Join-Path $projectRoot "third_party\orbslam3_py"
$buildDir = Join-Path $projectRoot "third_party\orbslam3_py_build"

function Find-VsDevCmd {
	$candidates = @(
		"C:\Program Files (x86)\Microsoft Visual Studio\2019\BuildTools\Common7\Tools\VsDevCmd.bat",
		"C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\Common7\Tools\VsDevCmd.bat",
		"C:\Program Files\Microsoft Visual Studio\2022\Community\Common7\Tools\VsDevCmd.bat",
		"C:\Program Files\Microsoft Visual Studio\2022\BuildTools\Common7\Tools\VsDevCmd.bat"
	)
	foreach ($candidate in $candidates) {
		if (Test-Path $candidate) {
			return $candidate
		}
	}
	throw "Visual Studio Developer Command Prompt not found."
}

if (!(Test-Path $orbRoot)) {
	throw "ORB-SLAM3 Windows tree not found: $orbRoot. Run scripts\build_orbslam3_windows.ps1 first."
}
if (!(Test-Path $moduleSource)) {
	throw "Module source not found: $moduleSource"
}

New-Item -ItemType Directory -Force -Path $outDir | Out-Null
New-Item -ItemType Directory -Force -Path $buildDir | Out-Null

$pythonInfoScript = Join-Path $buildDir "python_info.py"
Set-Content -Path $pythonInfoScript -Encoding ASCII -Value @'
import json
import pathlib
import sys
import sysconfig

base = pathlib.Path(sys.base_prefix)
print(json.dumps({
    "include": sysconfig.get_paths()["include"],
    "libs": str(base / "libs"),
    "lib": "python%d%d.lib" % sys.version_info[:2],
    "suffix": sysconfig.get_config_var("EXT_SUFFIX") or ".pyd",
}))
'@
$pythonInfo = & (Join-Path $projectRoot ".venv\Scripts\python.exe") $pythonInfoScript | ConvertFrom-Json

$moduleName = "orbslam3_py$($pythonInfo.suffix)"
$outPyd = Join-Path $outDir $moduleName
$objDir = Join-Path $buildDir "obj"
New-Item -ItemType Directory -Force -Path $objDir | Out-Null

$sourceFiles = @(
	"DBoW2\DBoW2\BowVector.cpp",
	"DBoW2\DBoW2\FeatureVector.cpp",
	"DBoW2\DBoW2\FORB.cpp",
	"DBoW2\DBoW2\ScoringObject.cpp",
	"DBoW2\DUtils\Random.cpp",
	"DBoW2\DUtils\Timestamp.cpp",
	"g2o\core\batch_stats.cpp",
	"g2o\core\cache.cpp",
	"g2o\core\factory.cpp",
	"g2o\core\hyper_graph.cpp",
	"g2o\core\hyper_graph_action.cpp",
	"g2o\core\jacobian_workspace.cpp",
	"g2o\core\estimate_propagator.cpp",
	"g2o\core\marginal_covariance_cholesky.cpp",
	"g2o\core\optimizable_graph.cpp",
	"g2o\core\optimization_algorithm.cpp",
	"g2o\core\optimization_algorithm_dogleg.cpp",
	"g2o\core\optimization_algorithm_gauss_newton.cpp",
	"g2o\core\optimization_algorithm_levenberg.cpp",
	"g2o\core\optimization_algorithm_with_hessian.cpp",
	"g2o\core\parameter.cpp",
	"g2o\core\parameter_container.cpp",
	"g2o\core\robust_kernel.cpp",
	"g2o\core\robust_kernel_factory.cpp",
	"g2o\core\robust_kernel_impl.cpp",
	"g2o\core\solver.cpp",
	"g2o\core\sparse_optimizer.cpp",
	"g2o\stuff\property.cpp",
	"g2o\stuff\string_tools.cpp",
	"g2o\types\se3mat.cpp",
	"g2o\types\types_sba.cpp",
	"g2o\types\types_seven_dof_expmap.cpp",
	"g2o\types\types_six_dof_expmap.cpp",
	"g2o\stuff\timeutil.cpp",
	"global.cpp",
	"slam\Atlas.cpp",
	"slam\CameraModels\KannalaBrandt8.cpp",
	"slam\CameraModels\Pinhole.cpp",
	"slam\Converter.cpp",
	"slam\Frame.cpp",
	"slam\FrameDrawer.cpp",
	"slam\G2oTypes.cpp",
	"slam\ImuTypes.cpp",
	"slam\Initializer.cpp",
	"slam\KeyFrame.cpp",
	"slam\KeyFrameDatabase.cpp",
	"slam\LocalMapping.cpp",
	"slam\LoopClosing.cpp",
	"slam\Map.cpp",
	"slam\MapDrawer.cpp",
	"slam\MapPoint.cpp",
	"slam\MLPnPsolver.cpp",
	"slam\OptimizableTypes.cpp",
	"slam\Optimizer.cpp",
	"slam\ORBextractor.cpp",
	"slam\ORBmatcher.cpp",
	"slam\PnPsolver.cpp",
	"slam\Sim3Solver.cpp",
	"slam\System.cpp",
	"slam\Tracking.cpp",
	"slam\TwoViewReconstruction.cpp",
	"slam\Viewer.cpp"
)

$sources = $sourceFiles | ForEach-Object { "`"$(Join-Path $orbRoot $_)`"" }
$sources += "`"$moduleSource`""

$includeDirs = @(
	$orbRoot,
	(Join-Path $orbRoot "slam"),
	(Join-Path $orbRoot "slam\CameraModels"),
	(Join-Path $orbRoot "opencv\build\include"),
	(Join-Path $orbRoot "Pangolin\include"),
	(Join-Path $orbRoot "Pangolin\build\src\include"),
	(Join-Path $orbRoot "Pangolin\build\external\glew\include"),
	$pythonInfo.include
) | ForEach-Object { "/I`"$_`"" }

$libDirs = @(
	(Join-Path $orbRoot "opencv\build\x64\vc14\lib"),
	(Join-Path $orbRoot "Pangolin\lib\$Configuration"),
	(Join-Path $orbRoot "Pangolin\build\external"),
	$pythonInfo.libs
) | ForEach-Object { "/LIBPATH:`"$_`"" }

$libs = @(
	"opencv_world320.lib",
	"pangolin.lib",
	"opengl32.lib",
	"glu32.lib",
	"glew\lib\glew.lib",
	"libpng\lib\libpng16_static.lib",
	"zlib\lib\zlibstatic.lib",
	"libjpeg\lib\jpeg.lib",
	"shell32.lib",
	"user32.lib",
	"gdi32.lib",
	"advapi32.lib",
	$pythonInfo.lib
)

$compileArgs = @(
	"/c",
	"/std:c++14",
	"/EHsc",
	"/MD",
	"/O2",
	"/bigobj",
	"/DCOMPILEDWITHC11",
	"/D_CRT_SECURE_NO_WARNINGS",
	"/DNDEBUG",
	"/DNOMINMAX",
	"/D_WINDOWS",
	"/DWIN32",
	"/Fo`"$objDir\\`""
) + $includeDirs + $sources

$compileRsp = Join-Path $buildDir "orbslam3_py_compile.rsp"
Set-Content -Path $compileRsp -Value ($compileArgs -join "`r`n") -Encoding ASCII

$objects = $sourceFiles | ForEach-Object {
	$stem = [IO.Path]::GetFileNameWithoutExtension($_)
	"`"$(Join-Path $objDir "$stem.obj")`""
}
$objects += "`"$(Join-Path $objDir "orbslam3_py.obj")`""

$linkRsp = Join-Path $buildDir "orbslam3_py_link.rsp"
$linkArgs = @(
	"/DLL",
	"/NOLOGO",
	"/OUT:`"$outPyd`"",
	"/IMPLIB:`"$(Join-Path $buildDir "orbslam3_py.lib")`""
) + $libDirs + $objects + $libs
Set-Content -Path $linkRsp -Value ($linkArgs -join "`r`n") -Encoding ASCII

$devCmd = Find-VsDevCmd
$cmd = Join-Path $buildDir "build_orbslam3_py.cmd"
Set-Content -Path $cmd -Value @"
@echo off
call "$devCmd" -arch=amd64
cd /d "$orbRoot"
where cl
cl @"$compileRsp"
if errorlevel 1 exit /b %errorlevel%
link @"$linkRsp"
"@ -Encoding ASCII

Write-Host "Building ORB-SLAM3 Python extension..."
cmd /c "`"$cmd`""
if ($LASTEXITCODE -ne 0) {
	throw "ORB-SLAM3 Python extension build failed with exit code $LASTEXITCODE"
}

$opencvBin = Join-Path $orbRoot "opencv\build\x64\vc14\bin"
foreach ($dll in @("opencv_world320.dll", "opencv_ffmpeg320_64.dll")) {
	$src = Join-Path $opencvBin $dll
	if (Test-Path $src) {
		Copy-Item $src -Destination (Join-Path $outDir $dll) -Force
	}
}

Write-Host ""
Write-Host "Done."
Write-Host "Python module: $outPyd"
Write-Host ""
Write-Host "Smoke test:"
Write-Host "  .\.venv\Scripts\python.exe -c `"import sys; sys.path.insert(0, 'third_party/orbslam3_py'); import orbslam3_py; print(orbslam3_py.OrbSlam3Mono)`""
