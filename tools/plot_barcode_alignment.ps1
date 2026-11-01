param(
    [Parameter(Mandatory = $true)]
    [string] $DataDir
)

# Plot saved CSV data using the Windows chart library. No robot access.
Add-Type -AssemblyName System.Windows.Forms.DataVisualization
Add-Type -AssemblyName System.Drawing
$dataRoot = (Resolve-Path -LiteralPath $DataDir).Path
$wave = Import-Csv -LiteralPath (Join-Path $dataRoot 'waveform.csv')
$features = Import-Csv -LiteralPath (Join-Path $dataRoot 'features.csv')
$chart = New-Object System.Windows.Forms.DataVisualization.Charting.Chart
$chart.Width = 1600
$chart.Height = 1320
$chart.BackColor = [System.Drawing.Color]::White
$chart.Font = New-Object System.Drawing.Font('Segoe UI', 10)
$letters = @($features | Select-Object -ExpandProperty letter -Unique)
$titles = @(
    'Recorded raw ADC signal: prominent bars',
    ($letters[0] + ' model: predicted narrow-bar sampling windows'),
    ($letters[1] + ' model: predicted narrow-bar sampling windows'),
    'Distinguishing weak feature: element 16 (model positions differ)'
)

for ($panel = 0; $panel -lt 4; $panel++) {
    $area = New-Object System.Windows.Forms.DataVisualization.Charting.ChartArea
    $area.Name = 'panel' + $panel
    $area.Position.X = 3
    $area.Position.Y = 1 + 24 * $panel
    $area.Position.Width = 95
    $area.Position.Height = 23
    $area.AxisX.Title = 'Encoder-sum travel units from first retained sample'
    $area.AxisY.Title = if ($panel -eq 0) { 'ADC' } else { 'Corrected ADC' }
    $area.AxisX.Minimum = 0
    $area.AxisX.Maximum = [double]$wave[-1].units
    $area.AxisX.MajorGrid.LineColor = [System.Drawing.Color]::Gainsboro
    $area.AxisY.MajorGrid.LineColor = [System.Drawing.Color]::Gainsboro
    if ($panel -gt 0) {
        $area.AxisY.Minimum = 0
        $area.AxisY.Maximum = 100
    }
    if ($panel -eq 3) {
        $middle = @($features | Where-Object { [int]$_.i -eq 16 })
        $area.AxisX.Minimum = [Math]::Floor(
            (($middle | Measure-Object centre_units -Minimum).Minimum - 100) / 20) * 20
        $area.AxisX.Maximum = [Math]::Ceiling(
            (($middle | Measure-Object centre_units -Maximum).Maximum + 100) / 20) * 20
    }
    $chart.ChartAreas.Add($area)
    $title = New-Object System.Windows.Forms.DataVisualization.Charting.Title
    $title.Text = $titles[$panel]
    $title.DockedToChartArea = $area.Name
    $title.Docking = 'Top'
    $title.IsDockedInsideChartArea = $true
    $title.Font = New-Object System.Drawing.Font('Segoe UI', 12, [System.Drawing.FontStyle]::Bold)
    $chart.Titles.Add($title)
    $series = New-Object System.Windows.Forms.DataVisualization.Charting.Series
    $series.Name = 'signal' + $panel
    $series.ChartArea = $area.Name
    $series.ChartType = 'Line'
    $series.Color = [System.Drawing.Color]::FromArgb(35, 65, 90)
    $series.BorderWidth = 1
    foreach ($row in $wave) {
        $value = if ($panel -eq 0) { [double]$row.adc } else { [double]$row.corrected }
        [void]$series.Points.AddXY([double]$row.units, $value)
    }
    $chart.Series.Add($series)
    if ($panel -gt 0) {
        $chosen = if ($panel -eq 3) {
            @($features | Where-Object { [int]$_.i -eq 16 })
        } else {
            @($features | Where-Object { $_.letter -eq $letters[$panel - 1] })
        }
        foreach ($feature in $chosen) {
            $strip = New-Object System.Windows.Forms.DataVisualization.Charting.StripLine
            $strip.IntervalOffset = [double]$feature.centre_units - [double]$feature.half_width_units
            $strip.StripWidth = 2 * [double]$feature.half_width_units
            $rgb = if ($feature.letter -eq $letters[0]) { @(40, 110, 210) } else { @(225, 120, 30) }
            $strip.BackColor = [System.Drawing.Color]::FromArgb(45, $rgb[0], $rgb[1], $rgb[2])
            $strip.BorderColor = [System.Drawing.Color]::FromArgb($rgb[0], $rgb[1], $rgb[2])
            $strip.Text = $feature.letter + ':' + $feature.i
            $strip.TextOrientation = 'Horizontal'
            $strip.TextAlignment = 'Center'
            $area.AxisX.StripLines.Add($strip)
        }
    }
}
$chart.SaveImage((Join-Path $dataRoot 'alignment.png'), 'Png')
$chart.Dispose()
Write-Output (Join-Path $dataRoot 'alignment.png')
