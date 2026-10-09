param([string]$LibraryRoot = (Split-Path $PSScriptRoot -Parent))
$ErrorActionPreference = 'Stop'
$compiler = Get-ChildItem 'C:/Program Files/Microsoft Visual Studio' -Filter cl.exe -Recurse | Where-Object { $_.FullName -like '*Hostx64\x64\cl.exe' } | Sort-Object FullName -Descending | Where-Object { Test-Path (Join-Path ($_.Directory.Parent.Parent.Parent.FullName) 'include/cstdint') } | Select-Object -First 1
if (!$compiler) { throw 'Installed MSVC compiler required' }
$vcRoot = $compiler.Directory.Parent.Parent.Parent.FullName
$sdkRoot = 'C:/Program Files (x86)/Windows Kits/10'
$sdkVersion = Get-ChildItem "$sdkRoot/Include" -Directory | Sort-Object Name -Descending | Select-Object -First 1 -ExpandProperty Name
$previousInclude = $env:INCLUDE
$previousLib = $env:LIB
$outputDirectory = Join-Path ([IO.Path]::GetTempPath()) ([guid]::NewGuid().ToString())
New-Item -ItemType Directory $outputDirectory | Out-Null
try {
	$env:INCLUDE = "$PSScriptRoot;$LibraryRoot/src;$vcRoot/include;$sdkRoot/Include/$sdkVersion/ucrt;$sdkRoot/Include/$sdkVersion/shared;$sdkRoot/Include/$sdkVersion/um"
	$env:LIB = "$vcRoot/lib/x64;$sdkRoot/Lib/$sdkVersion/ucrt/x64;$sdkRoot/Lib/$sdkVersion/um/x64"
	[IO.File]::WriteAllText("$outputDirectory/HardwareSerial.h", '#include "Arduino.h"')
	& $compiler.FullName /nologo /EHsc /std:c++17 "/I$outputDirectory" "$PSScriptRoot/driver_check.cpp" "$LibraryRoot/src/STSServoDriver.cpp" "/Fe:$outputDirectory/driver_check.exe" "/Fo:$outputDirectory/"
	if ($LASTEXITCODE) { throw 'Driver check compilation failed' }
	& "$outputDirectory/driver_check.exe"
	if ($LASTEXITCODE) { throw 'Driver checks failed' }
} finally {
	$env:INCLUDE = $previousInclude
	$env:LIB = $previousLib
	if (![IO.Path]::GetFullPath($outputDirectory).StartsWith(
			[IO.Path]::GetFullPath([IO.Path]::GetTempPath()), [StringComparison]::OrdinalIgnoreCase)) {
		throw 'Test output directory must remain within system temp directory'
	}
	Remove-Item -LiteralPath $outputDirectory -Recurse -Force
}
