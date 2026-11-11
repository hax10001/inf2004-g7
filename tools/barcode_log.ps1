# Save the robot's USB serial output to a log file (no installs needed).
#
#   powershell -ExecutionPolicy Bypass -File tools\barcode_log.ps1 COM5 big_F_push
#
# Close any other serial monitor on that port first (only one program can
# open it). Lines are shown on screen and appended to
# tools\barcode_data\<date>\<name>.log. Press Ctrl+C to stop; run it again
# with a new name for the next barcode.
param(
    [Parameter(Mandatory = $true)][string]$Port,
    [Parameter(Mandatory = $true)][string]$Name
)

$root = Split-Path -Parent $PSScriptRoot
$folder = Join-Path $root ("tools\barcode_data\" + (Get-Date -Format "yyyy-MM-dd"))
New-Item -ItemType Directory -Force -Path $folder | Out-Null
$file = Join-Path $folder ($Name + ".log")

$serial = New-Object System.IO.Ports.SerialPort $Port, 115200
$serial.DtrEnable = $true   # the Pico's USB console only talks once DTR is set
$serial.RtsEnable = $true
$serial.ReadTimeout = 500
$serial.NewLine = "`n"
$serial.Open()
$writer = New-Object System.IO.StreamWriter($file, $true)
$writer.AutoFlush = $true
Write-Host "Logging $Port to $file  (Ctrl+C to stop)"
$passes = 0
try {
    while ($true) {
        try {
            $line = $serial.ReadLine().TrimEnd("`r")
        } catch [System.TimeoutException] {
            continue
        }
        $writer.WriteLine($line)
        if ($line -match "\[BC WAVE RAW\]") { continue }   # too many to show
        Write-Host $line
        if ($line -match "\[BC WAVE DONE\]") {
            $passes++
            Write-Host "---- pass $passes saved; ready for the next one ----" -ForegroundColor Green
        }
    }
} finally {
    $writer.Close()
    $serial.Close()
    Write-Host "Saved $passes passes to $file"
}
