Clear-Host

$TotalRuntimeMinutes = 10
$EndTime = (Get-Date).AddMinutes($TotalRuntimeMinutes)

$Packages = @(
    "crypto-core",
    "net-stream",
    "auth-engine",
    "packet-parser",
    "entropy-lib",
    "hash-tools",
    "session-manager",
    "compression-utils",
    "secure-transport",
    "index-builder"
)

function Write-Step {
    param([string]$Text, [string]$Color = "DarkGreen")
    Write-Host $Text -ForegroundColor $Color
}

function Show-Progress {
    param([string]$Task, [int]$Seconds)

    for ($i = 1; $i -le $Seconds; $i++) {
        $pct = [int](($i / $Seconds) * 100)
        Write-Progress -Activity $Task -Status ("{0} percent complete" -f $pct) -PercentComplete $pct
        Start-Sleep -Seconds 1
    }
    Write-Progress -Activity $Task -Completed
}

Write-Step "Resolving dependencies..." "Green"
Start-Sleep 2

foreach ($pkg in $Packages) {
    Write-Step ("Collecting metadata for {0}" -f $pkg)
    Start-Sleep -Milliseconds (Get-Random -Minimum 300 -Maximum 900)
}

Write-Step "Dependency resolution complete." "Green"
Start-Sleep 1

foreach ($pkg in $Packages) {

    Write-Step ("Downloading {0}" -f $pkg)
    $size = Get-Random -Minimum 50 -Maximum 300
    $rate = Get-Random -Minimum 2 -Maximum 12
    $i = 0

    while ($i -lt $size) {
        $i += $rate
        if ($i -gt $size) { $i = $size }
        $pct = [int](($i / $size) * 100)
        $line = ("Downloading {0} - {1} of {2} MB at {3} MB per second - {4} percent" -f $pkg, $i, $size, $rate, $pct)
        Write-Step $line
        Start-Sleep 1
    }

    Write-Step ("Verifying checksum for {0}..." -f $pkg)
    Start-Sleep -Seconds (Get-Random -Minimum 1 -Maximum 3)

    Write-Step ("Unpacking {0}..." -f $pkg)
    Show-Progress ("Unpacking {0}" -f $pkg) (Get-Random -Minimum 5 -Maximum 12)

    Write-Step ("Installing {0}..." -f $pkg)
    Show-Progress ("Installing {0}" -f $pkg) (Get-Random -Minimum 4 -Maximum 10)

    Write-Step ("{0} installed successfully." -f $pkg) "Green"
    Start-Sleep 1
}

Write-Step "Building index..."
Show-Progress "Optimizing system" 15

Write-Step "Finalizing..."
Show-Progress "Finalizing installation" 10

Write-Step "Installation completed successfully." "Green"
