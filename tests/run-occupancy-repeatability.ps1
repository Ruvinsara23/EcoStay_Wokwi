param(
  [int]$TimingToleranceMs = 3000,
  [int]$TimeoutMs = 60000
)

$ErrorActionPreference = 'Stop'

$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$wokwiCli = Join-Path $projectRoot 'wokwi-cli.exe'
$scenarioPath = 'tests/occupancy-scenario.test.yaml'
$artifactDirectory = Join-Path $PSScriptRoot '.artifacts'

if (-not (Test-Path -LiteralPath $wokwiCli)) {
  throw "Wokwi CLI not found at $wokwiCli"
}

if (-not $env:WOKWI_CLI_TOKEN) {
  throw 'WOKWI_CLI_TOKEN is not set. Create one at https://wokwi.com/dashboard/ci.'
}

New-Item -ItemType Directory -Force -Path $artifactDirectory | Out-Null

function Invoke-ScenarioRun {
  param([int]$RunNumber)

  $logPath = Join-Path $artifactDirectory "occupancy-run-$RunNumber.log"

  & $wokwiCli $projectRoot `
    --scenario $scenarioPath `
    --timeout $TimeoutMs `
    --timeout-exit-code 42 `
    --serial-log-file $logPath

  if ($LASTEXITCODE -ne 0) {
    throw "Wokwi run $RunNumber failed with exit code $LASTEXITCODE"
  }

  return $logPath
}

function Read-ScenarioTranscript {
  param([string]$LogPath)

  $events = @()
  $states = @()

  foreach ($line in Get-Content -LiteralPath $LogPath) {
    if (
      $line -match
      '^SIM_SCENARIO_EVENT elapsed_ms=(\d+) scheduled_ms=(\d+) action=(\S+)(.*?) occupancy=(\S+)$'
    ) {
      $events += [pscustomobject]@{
        ElapsedMs = [int]$Matches[1]
        ScheduledMs = [int]$Matches[2]
        Action = $Matches[3]
        Details = $Matches[4]
        Occupancy = $Matches[5]
      }
    } elseif (
      $line -match
      '^SIM_SCENARIO_STATE elapsed_ms=(\d+) from=(\S+) to=(\S+)$'
    ) {
      $states += [pscustomobject]@{
        ElapsedMs = [int]$Matches[1]
        From = $Matches[2]
        To = $Matches[3]
      }
    }
  }

  if ($events.Count -eq 0) {
    throw "No SIM_SCENARIO_EVENT records found in $LogPath"
  }

  return [pscustomobject]@{
    Events = $events
    States = $states
  }
}

function Assert-SameCount {
  param(
    [string]$RecordType,
    [int]$FirstCount,
    [int]$SecondCount
  )

  if ($FirstCount -ne $SecondCount) {
    throw "$RecordType count differs: run 1=$FirstCount, run 2=$SecondCount"
  }
}

$firstLog = Invoke-ScenarioRun -RunNumber 1
$secondLog = Invoke-ScenarioRun -RunNumber 2
$first = Read-ScenarioTranscript -LogPath $firstLog
$second = Read-ScenarioTranscript -LogPath $secondLog

Assert-SameCount 'Event' $first.Events.Count $second.Events.Count
Assert-SameCount 'State transition' $first.States.Count $second.States.Count

for ($i = 0; $i -lt $first.Events.Count; $i++) {
  $a = $first.Events[$i]
  $b = $second.Events[$i]

  $aIdentity = "$($a.ScheduledMs)|$($a.Action)|$($a.Details)|$($a.Occupancy)"
  $bIdentity = "$($b.ScheduledMs)|$($b.Action)|$($b.Details)|$($b.Occupancy)"
  if ($aIdentity -ne $bIdentity) {
    throw "Event $i differs:`nrun 1: $aIdentity`nrun 2: $bIdentity"
  }

  $timingDifference = [Math]::Abs($a.ElapsedMs - $b.ElapsedMs)
  if ($timingDifference -gt $TimingToleranceMs) {
    throw "Event $i timing differs by $timingDifference ms"
  }
}

for ($i = 0; $i -lt $first.States.Count; $i++) {
  $a = $first.States[$i]
  $b = $second.States[$i]

  if ($a.From -ne $b.From -or $a.To -ne $b.To) {
    throw "State transition $i differs: run 1=$($a.From)->$($a.To), run 2=$($b.From)->$($b.To)"
  }

  $timingDifference = [Math]::Abs($a.ElapsedMs - $b.ElapsedMs)
  if ($timingDifference -gt $TimingToleranceMs) {
    throw "State transition $i timing differs by $timingDifference ms"
  }
}

Write-Output (
  "PASS: {0} events and {1} state transitions matched across two sessions " +
  "within {2} ms. Logs: {3}, {4}" -f
  $first.Events.Count,
  $first.States.Count,
  $TimingToleranceMs,
  $firstLog,
  $secondLog
)
