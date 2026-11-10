param([string]$Serial = '')
$ErrorActionPreference = 'Stop'
$taskRoot = Split-Path -Parent $PSScriptRoot
$taskMetadata = Join-Path $taskRoot 'artifacts/pico-teleop/session.json'
if (-not (Test-Path -LiteralPath $taskMetadata)) {
    throw '请先在项目根目录运行 python -m scripts.pico_teleop，再运行连接脚本。'
}
$taskSession = Get-Content -Raw -LiteralPath $taskMetadata | ConvertFrom-Json
$taskUrl = [string]$taskSession.headset_url
if ($taskUrl -notmatch '^http://localhost:[0-9]+/\?token=[A-Za-z0-9_-]+&role=input$') {
    throw '会话文件中的头显地址无效，请重新启动仿真服务。'
}
try {
    $taskHealth = Invoke-RestMethod -Uri "http://127.0.0.1:$($taskSession.port)/health" -TimeoutSec 3
    if ($taskHealth.backend -notin @('kinematic_simulation','servo_response_simulation')) { throw '不是仿真服务' }
} catch { throw '本地仿真服务未运行。请保持 start_pico_teleop.ps1 的窗口打开。' }
$taskAdb = Get-Command adb -ErrorAction SilentlyContinue
if (-not $taskAdb) { throw '未找到 adb。请安装 Android Platform Tools 并加入 PATH。' }
$taskLines = & $taskAdb.Source devices -l
if ($LASTEXITCODE -ne 0) { throw 'adb devices 执行失败。' }
$taskDevices = @()
foreach ($taskLine in $taskLines) {
    if ($taskLine -match '^(\S+)\s+(device|unauthorized|offline)\b') {
        $taskDevices += [pscustomobject]@{ Serial=$Matches[1]; State=$Matches[2] }
    }
}
if ($Serial) { $taskDevices = @($taskDevices | Where-Object { $_.Serial -eq $Serial }) }
if ($taskDevices.Count -eq 0) {
    Write-Host 'ADB 没有找到可选头显。请按顺序操作：'
    Write-Host '1. 用支持数据传输的 USB 线，将 PICO 4 连接电脑。'
    Write-Host '2. 在头显里启用开发者模式和 USB 调试。'
    Write-Host '3. 在头显弹窗里允许这台电脑调试，然后重新运行本脚本。'
    exit 2
}
if ($taskDevices.Count -gt 1) {
    $taskDevices | Format-Table
    throw '存在多台设备，请使用 -Serial <PICO序列号> 指定头显。'
}
$taskSelected = $taskDevices[0]
if ($taskSelected.State -ne 'device') {
    throw "设备状态为 $($taskSelected.State)。请在头显中允许 USB 调试，或重新插拔数据线。"
}
& $taskAdb.Source -s $taskSelected.Serial reverse "tcp:$($taskSession.port)" "tcp:$($taskSession.port)"
if ($LASTEXITCODE -ne 0) { throw 'ADB reverse 失败。' }
# Quote the URL for Android's shell as well as PowerShell; preserve its &role=input.
& $taskAdb.Source -s $taskSelected.Serial shell am start -a android.intent.action.VIEW -d "'$taskUrl'"
if ($LASTEXITCODE -ne 0) { throw '未能在头显打开网页。可在 PICO 浏览器手动输入下方地址。' }
Write-Host "头显地址：$taskUrl"
Write-Host '在 PICO 浏览器点击“进入 VR 遥操作”。只需右手柄：松开 grip/trigger 后短按 A 授权仿真。'
Write-Host '本脚本只连接 PICO，不连接任何机械臂串口。'
