# Protokolliert die Sendezyklen ueber die Weboberflaeche des Boards (/json), z.B. wenn kein USB-Kabel steckt.
# Ein neues Telegramm erkennt das Skript daran, dass der Empfangszeitpunkt (Abfragezeit minus age_s)
# um mehr als 20 s vom letzten abweicht. Genauigkeit etwa +-1 s.
# Aufruf: powershell -File sendezyklen-log-http.ps1 -Minuten 60
param(
  [int]$Minuten = 60,
  [string]$Quelle = "http://lilygo868.local/json",
  [string]$Ausgabe = "$PSScriptRoot\..\messdaten\sendezyklen_http.csv",
  [int]$PollSekunden = 3
)
$Ausgabe = [System.IO.Path]::GetFullPath($Ausgabe)
$ci = [System.Globalization.CultureInfo]::InvariantCulture
if (-not (Test-Path $Ausgabe)) {
  Add-Content -Path $Ausgabe -Encoding UTF8 -Value "Zeit;Sensor;Zaehler;Intervall_s;Temperatur_C;Feuchte_rF;Signal_dBm;Batterie;Hinweis"
}
$ende = (Get-Date).AddMinutes($Minuten)
$letzt = @{}
$intervalle = New-Object System.Collections.Generic.List[double]
$ausfaelleInFolge = 0; $ausfaelleGesamt = 0
Write-Output ("Start {0}, Ende {1}, Quelle {2}" -f (Get-Date -Format 'HH:mm:ss'), $ende.ToString('HH:mm:ss'), $Quelle)

while ((Get-Date) -lt $ende) {
  try {
    $r = Invoke-RestMethod -Uri $Quelle -TimeoutSec 6
    if ($ausfaelleInFolge -gt 0) {
      Add-Content -Path $Ausgabe -Encoding UTF8 -Value ("{0};;;;;;;;Board wieder erreichbar nach {1} fehlgeschlagenen Abfragen" -f (Get-Date -Format "yyyy-MM-dd HH:mm:ss"), $ausfaelleInFolge)
    }
    $ausfaelleInFolge = 0
    foreach ($s in @($r)) {
      $key = "{0}_ch{1}_id{2}" -f $s.model, $s.channel, $s.id
      $jetzt = Get-Date
      $tT = $jetzt.AddSeconds(-[double]$s.age_s)
      $alt = $letzt[$key]
      if ($null -eq $alt -and [double]$s.age_s -gt 120) { continue }   # verwaister Eintrag (Sensor sendet nicht mehr unter diesem Kanal/ID)
      if ($null -eq $alt -or ($tT - $alt.Zeit).TotalSeconds -gt 20) {
        $intervall = ""; $hinweis = ""
        if ($null -eq $alt) { $hinweis = "Erste Beobachtung" }
        else {
          $d = ($tT - $alt.Zeit).TotalSeconds
          $intervall = $d.ToString("0.0", $ci); $intervalle.Add($d)
          if ([int]$s.count -lt $alt.Zaehler) { $hinweis = "Zaehler zurueckgesetzt - Board wurde neu gestartet" }
          elseif ($d -gt 100) { $hinweis = "mindestens 1 Telegramm fehlte (" + [math]::Round($d/61 - 1) + ")" }
          elseif ($d -gt 70) { $hinweis = "Zyklus laenger als 70 s" }
          elseif ($d -lt 50) { $hinweis = "Zyklus kuerzer als 50 s" }
        }
        $rssi = if ($null -ne $s.rssi) { $s.rssi } else { "" }
        $zeile = "{0};{1};{2};{3};{4};{5};{6};{7};{8}" -f $tT.ToString("yyyy-MM-dd HH:mm:ss"), $key, $s.count, $intervall, `
          ([double]$s.t).ToString("0.0", $ci), $s.rh, $rssi, $s.battery_ok, $hinweis
        Add-Content -Path $Ausgabe -Encoding UTF8 -Value $zeile
        Write-Output $zeile
        $letzt[$key] = @{ Zeit = $tT; Zaehler = [int]$s.count }
      }
    }
  } catch {
    $ausfaelleInFolge++; $ausfaelleGesamt++
    if ($ausfaelleInFolge -eq 1 -or $ausfaelleInFolge % 20 -eq 0) {
      Add-Content -Path $Ausgabe -Encoding UTF8 -Value ("{0};;;;;;;;Abfrage fehlgeschlagen (Nr. {1} in Folge)" -f (Get-Date -Format "yyyy-MM-dd HH:mm:ss"), $ausfaelleInFolge)
    }
  }
  Start-Sleep -Seconds $PollSekunden
}

if ($intervalle.Count -gt 0) {
  $m = ($intervalle | Measure-Object -Average -Minimum -Maximum)
  $text = "ZUSAMMENFASSUNG: {0} Intervalle, Mittel {1:0.00} s, min {2:0.0} s, max {3:0.0} s, fehlgeschlagene Abfragen {4}" -f $m.Count, $m.Average, $m.Minimum, $m.Maximum, $ausfaelleGesamt
} else { $text = "ZUSAMMENFASSUNG: keine Intervalle erfasst, fehlgeschlagene Abfragen $ausfaelleGesamt" }
Write-Output $text
Set-Content -Path ($Ausgabe + ".zusammenfassung.txt") -Encoding UTF8 -Value ("{0} (Ende {1})" -f $text, (Get-Date -Format 'yyyy-MM-dd HH:mm:ss'))
