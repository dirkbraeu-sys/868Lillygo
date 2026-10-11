# Protokolliert die Sendezyklen des Sensors ueber die serielle Schnittstelle des Boards.
# Jede "RX model=..."-Zeile ist ein Telegramm; doppelt dekodierte Telegramme (< 5 s Abstand)
# zaehlen einmal. Zeitpunkt = Moment, in dem die Zeile ankommt (Genauigkeit < 0,1 s).
# Aufruf: powershell -File sendezyklen-log-seriell.ps1 -Minuten 30 -Port COM16
param(
  [int]$Minuten = 25,
  [string]$Port = "COM16",
  [string]$Ausgabe = "$PSScriptRoot\..\messdaten\sendezyklen.csv"
)
$Ausgabe = [System.IO.Path]::GetFullPath($Ausgabe)
$ci = [System.Globalization.CultureInfo]::InvariantCulture
if (-not (Test-Path $Ausgabe)) {
  Add-Content -Path $Ausgabe -Encoding UTF8 -Value "Zeit;Sensor;Intervall_s;Temperatur_C;Feuchte_rF;Signal_dBm;Batterie;Hinweis"
}
$sp = New-Object System.IO.Ports.SerialPort $Port, 115200, 'None', 8, 'one'
$sp.ReadTimeout = 1000
$sp.DtrEnable = $false
$sp.RtsEnable = $false
$sp.Open()
$ende = (Get-Date).AddMinutes($Minuten)
$letzt = @{}
$intervalle = New-Object System.Collections.Generic.List[double]
Write-Output ("Start {0}, Ende {1}, Port {2}" -f (Get-Date -Format 'HH:mm:ss'), $ende.ToString('HH:mm:ss'), $Port)
$re = 'RX\s+model=(\S+)\s+ch=(\d+)\s+id=(\d+)\s+T=(-?[\d.]+)C\s+RH=(\d+)%\s+(?:RSSI=(-?\d+)dBm\s+)?Batt=(\w+)'
while ((Get-Date) -lt $ende) {
  try { $zeile = $sp.ReadLine() } catch [System.TimeoutException] { continue } catch {
    Add-Content -Path $Ausgabe -Encoding UTF8 -Value ("{0};;;;;;;Port-Fehler, Protokoll beendet" -f (Get-Date -Format "yyyy-MM-dd HH:mm:ss.fff"))
    break
  }
  $jetzt = Get-Date
  if ($zeile -match $re) {
    $key = "{0}_ch{1}_id{2}" -f $Matches[1], $Matches[2], $Matches[3]
    $alt = $letzt[$key]
    if ($null -ne $alt -and ($jetzt - $alt).TotalSeconds -lt 5) { continue }   # Doppel-Dekodierung
    $intervall = ""; $hinweis = ""
    if ($null -eq $alt) { $hinweis = "Erste Beobachtung" }
    else {
      $d = ($jetzt - $alt).TotalSeconds
      $intervall = $d.ToString("0.0", $ci)
      $intervalle.Add($d)
      if ($d -gt 100) { $hinweis = "mindestens 1 Telegramm fehlte" }
      elseif ($d -gt 70) { $hinweis = "Zyklus laenger als 70 s" }
      elseif ($d -lt 50) { $hinweis = "Zyklus kuerzer als 50 s" }
    }
    $letzt[$key] = $jetzt
    $out = "{0};{1};{2};{3};{4};{5};{6};{7}" -f $jetzt.ToString("yyyy-MM-dd HH:mm:ss.fff"), $key, $intervall, $Matches[4], $Matches[5], $Matches[6], $Matches[7], $hinweis
    Add-Content -Path $Ausgabe -Encoding UTF8 -Value $out
    Write-Output $out
  }
}
$sp.Close()
if ($intervalle.Count -gt 0) {
  $m = ($intervalle | Measure-Object -Average -Minimum -Maximum)
  $text = "ZUSAMMENFASSUNG: {0} Intervalle, Mittel {1:0.00} s, min {2:0.0} s, max {3:0.0} s" -f $m.Count, $m.Average, $m.Minimum, $m.Maximum
} else { $text = "ZUSAMMENFASSUNG: keine Intervalle erfasst" }
Write-Output $text
Set-Content -Path ($Ausgabe + ".zusammenfassung.txt") -Encoding UTF8 -Value ("{0} (Ende {1})" -f $text, (Get-Date -Format 'yyyy-MM-dd HH:mm:ss'))
