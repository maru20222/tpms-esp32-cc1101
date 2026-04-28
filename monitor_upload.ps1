<#
monitor_upload.ps1
プロジェクトルートで実行する想定。Windows PowerShell 5.1 で安全に動作するように
ASCII ベースで記述しています。
#>

# logs ディレクトリを安全に作成
$dir = '.\logs'
if (-not (Test-Path $dir -PathType Container)) {
    New-Item -ItemType Directory -Path $dir | Out-Null
}

# タイムスタンプ付きファイル名
$ts = Get-Date -Format 'yyyyMMddHHmmss'
$file = Join-Path $dir ("serial_$ts.log")

# pio を探す（PATH -> 既知の場所 -> python -m platformio）
$pioCmd = $null
try { $cmd = Get-Command pio -ErrorAction Stop; $pioCmd = $cmd.Path } catch {}
if (-not $pioCmd) {
    $candidate1 = Join-Path $env:USERPROFILE '.platformio\penv\Scripts\pio.exe'
    if (Test-Path $candidate1) { $pioCmd = $candidate1 }
    else {
        $candidate2 = Join-Path $env:USERPROFILE 'AppData\Roaming\Python\Scripts\platformio.exe'
        if (Test-Path $candidate2) { $pioCmd = $candidate2 }
    }
}

if (-not $pioCmd) {
    Write-Host "pio not found in PATH; will try 'python -m platformio'" -ForegroundColor Yellow
    $usePython = $true
} else {
    Write-Host ("Using pio: " + $pioCmd) -ForegroundColor Green
    $usePython = $false
}

# upload 実行
if ($usePython) {
    python -m platformio run -t upload
    $rc = $LASTEXITCODE
} else {
    & $pioCmd run -t upload
    $rc = $LASTEXITCODE
}

if ($rc -ne 0) {
    Write-Host ("upload failed (exit " + $rc + ")") -ForegroundColor Red
    exit $rc
}

# monitor を実行してログへ追記（画面表示＋ファイル保存）
if ($usePython) {
    python -m platformio device monitor | Tee-Object -FilePath $file -Append
} else {
    & $pioCmd device monitor | Tee-Object -FilePath $file -Append
}