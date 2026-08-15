param(
    [ValidateSet('check', 'flash-swd', 'flash-usb', 'drag')]
    [string]$Mode = 'check',
    [string]$FirmwarePath = '',
    [switch]$DryRun
)

$ErrorActionPreference = 'Stop'
[Console]::OutputEncoding = [System.Text.Encoding]::UTF8
$OutputEncoding = [System.Text.Encoding]::UTF8

$toolRoot = [System.IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$flightRoot = [System.IO.Path]::GetFullPath((Join-Path $toolRoot '..\A9固件_来源未确认'))
$logRoot = Join-Path $toolRoot 'logs'

function Write-Title([string]$Text) {
    Write-Host ''
    Write-Host '============================================================' -ForegroundColor DarkCyan
    Write-Host $Text -ForegroundColor Cyan
    Write-Host '============================================================' -ForegroundColor DarkCyan
}

function Find-Cli {
    $candidates = @(
        'D:\ST\STM32CubeCLT_1.18.0\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe',
        'C:\ST\STM32CubeCLT_1.18.0\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe',
        'C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe',
        'C:\Program Files (x86)\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe'
    )

    foreach ($candidate in $candidates) {
        if (Test-Path -LiteralPath $candidate -PathType Leaf) {
            return $candidate
        }
    }

    foreach ($root in @('D:\ST', 'C:\ST')) {
        if (Test-Path -LiteralPath $root -PathType Container) {
            $found = Get-ChildItem -LiteralPath $root -Recurse -Filter 'STM32_Programmer_CLI.exe' -File -ErrorAction SilentlyContinue | Select-Object -First 1
            if ($null -ne $found) {
                return $found.FullName
            }
        }
    }

    return $null
}

function Is-BlockedFirmware([string]$Path) {
    $text = $Path.ToLowerInvariant()
    foreach ($blocked in @('k9', 'teach', 'f570', 'bootloader', 'ac-rid', 'wheeltec')) {
        if ($text.Contains($blocked)) {
            return $true
        }
    }
    return $false
}

function Get-HexRange([string]$Path) {
    $upper = [uint32]0
    $minAddress = [uint32]::MaxValue
    $maxAddress = [uint32]0
    $recordCount = 0

    foreach ($line in Get-Content -LiteralPath $Path -ErrorAction Stop) {
        if ([string]::IsNullOrWhiteSpace($line)) { continue }
        $line = $line.Trim()
        if (-not $line.StartsWith(':') -or $line.Length -lt 11) {
            throw "不是合法 Intel HEX 行：$line"
        }

        $byteCount = [Convert]::ToInt32($line.Substring(1, 2), 16)
        $address = [Convert]::ToInt32($line.Substring(3, 4), 16)
        $type = [Convert]::ToInt32($line.Substring(7, 2), 16)

        if ($type -eq 4) {
            $upper = [Convert]::ToUInt32($line.Substring(9, 4), 16) * 0x10000
        } elseif ($type -eq 0 -and $byteCount -gt 0) {
            $start = [uint32]($upper + $address)
            $end = [uint32]($start + $byteCount - 1)
            if ($start -lt $minAddress) { $minAddress = $start }
            if ($end -gt $maxAddress) { $maxAddress = $end }
            $recordCount++
        }
    }

    if ($recordCount -eq 0) {
        throw 'HEX 文件没有可编程数据记录。'
    }

    [PSCustomObject]@{
        MinAddress = $minAddress
        MaxAddress = $maxAddress
        RecordCount = $recordCount
    }
}

function Resolve-A9Firmware([string]$RequestedPath) {
    if ([string]::IsNullOrWhiteSpace($RequestedPath)) {
        $firmware = @(Get-ChildItem -LiteralPath $flightRoot -Recurse -Filter 'Firmware_A9_*.hex' -File -ErrorAction SilentlyContinue | Sort-Object LastWriteTime -Descending | Select-Object -First 1)[0]
        if ($null -eq $firmware) {
            throw "没有找到 Firmware_A9_*.hex。"
        }
        return $firmware.FullName
    }

    $resolved = [System.IO.Path]::GetFullPath($RequestedPath)
    if (-not (Test-Path -LiteralPath $resolved -PathType Leaf)) {
        throw "固件文件不存在：$resolved"
    }
    if ([System.IO.Path]::GetExtension($resolved).ToLowerInvariant() -ne '.hex') {
        throw '只支持 .hex 固件文件。'
    }
    if (Is-BlockedFirmware $resolved) {
        throw "已拦截疑似非 A9 固件：$resolved"
    }

    $isNamedA9 = [System.IO.Path]::GetFileName($resolved) -match '^Firmware_A9_.*\.hex$'
    $sourceRoot = [System.IO.Path]::GetFullPath((Join-Path $toolRoot '..\..\02_A9源码'))
    $isA9SourceOutput = $resolved.StartsWith($sourceRoot, [System.StringComparison]::OrdinalIgnoreCase)
    if (-not $isNamedA9 -and -not $isA9SourceOutput) {
        Write-Host "未知来源 HEX：$resolved" -ForegroundColor Yellow
        $answer = Read-Host '确认这是 A9 固件请输入 YES，否则直接回车取消'
        if ($answer -cne 'YES') {
            throw '用户取消未知固件烧录。'
        }
    }
    return $resolved
}

function Assert-A9Hex([string]$Path) {
    $range = Get-HexRange $Path
    $appStart = [uint32]0x08040000
    $flashEnd = [uint32]0x08200000
    if ($range.MinAddress -lt $appStart -or $range.MaxAddress -ge $flashEnd) {
        throw ('HEX 地址范围不符合 A9 应用区：0x{0:X8} - 0x{1:X8}' -f $range.MinAddress, $range.MaxAddress)
    }
    return $range
}

function Invoke-Cli([string]$CliPath, [string[]]$Arguments, [string]$ConsoleLog) {
    Write-Host ''
    Write-Host ('执行：{0} {1}' -f $CliPath, ($Arguments -join ' ')) -ForegroundColor DarkGray
    & $CliPath @Arguments 2>&1 | Tee-Object -FilePath $ConsoleLog
    return $LASTEXITCODE
}

function Pause-Window {
    Write-Host ''
    [void](Read-Host '按回车关闭窗口')
}

Write-Title 'ACFLY A9 一键烧录工具'
$cli = Find-Cli
if ($null -eq $cli) {
    Write-Host '找不到 STM32_Programmer_CLI.exe。' -ForegroundColor Red
    Write-Host '请确认已安装 STM32CubeProgrammer 或 STM32CubeCLT。'
    Pause-Window
    exit 2
}

if ($Mode -eq 'check') {
    Write-Host "CLI：$cli" -ForegroundColor Green
    Write-Host '执行版本检查：'
    & $cli --version 2>&1
    Write-Host ''
    Write-Host '列举 ST-LINK（只读）：'
    & $cli -l st-link-only 2>&1
    Write-Host ''
    Write-Host '列举 USB（只读）：'
    & $cli -l usb 2>&1
    Pause-Window
    exit 0
}

try {
    if ($Mode -eq 'drag') {
        Write-Host '请选择烧录入口：'
        Write-Host '  1 - ST-LINK/SWD'
        Write-Host '  2 - USB-DFU'
        $choice = Read-Host '输入 1 或 2'
        if ($choice -eq '1') { $Mode = 'flash-swd' }
        elseif ($choice -eq '2') { $Mode = 'flash-usb' }
        else { throw '入口选择无效。' }
    }

    $firmware = Resolve-A9Firmware $FirmwarePath
    $range = Assert-A9Hex $firmware
    $item = Get-Item -LiteralPath $firmware
    Write-Host ''
    Write-Host '待烧录固件：' -ForegroundColor Cyan
    Write-Host "  文件：$($item.Name)"
    Write-Host "  路径：$($item.FullName)"
    Write-Host "  大小：$([Math]::Round($item.Length / 1KB, 1)) KB"
    Write-Host ('  地址：0x{0:X8} - 0x{1:X8}' -f $range.MinAddress, $range.MaxAddress)
    Write-Host "  入口：$Mode"

    $confirm = Read-Host '确认继续请输入 YES，否则直接回车取消'
    if ($confirm -cne 'YES') {
        Write-Host '已取消，未执行烧录。' -ForegroundColor Yellow
        Pause-Window
        exit 0
    }

    $stamp = Get-Date -Format 'yyyyMMdd_HHmmss'
    New-Item -ItemType Directory -Force -Path $logRoot | Out-Null
    $consoleLog = Join-Path $logRoot ("a9_{0}_{1}.log" -f ($Mode -replace 'flash-', ''), $stamp)
    $cliLog = Join-Path $logRoot ("a9_{0}_{1}_cli.log" -f ($Mode -replace 'flash-', ''), $stamp)

    if ($Mode -eq 'flash-swd') {
        $arguments = @('-c', 'port=SWD', 'mode=UR', 'freq=4000', '-w', $firmware, '-v', '-rst', '-log', $cliLog)
    } elseif ($Mode -eq 'flash-usb') {
        $arguments = @('-c', 'port=usb1', '-w', $firmware, '-v', '-rst', '-log', $cliLog)
    } else {
        throw "未知烧录模式：$Mode"
    }

    if ($DryRun) {
        Write-Host ''
        Write-Host 'DRY-RUN：不会执行烧录。' -ForegroundColor Yellow
        Write-Host ('命令参数：{0}' -f ($arguments -join ' '))
        Pause-Window
        exit 0
    }

    $exitCode = Invoke-Cli $cli $arguments $consoleLog
    if ($exitCode -eq 0) {
        Write-Host ''
        Write-Host '烧录并校验成功。' -ForegroundColor Green
        Write-Host "日志：$consoleLog"
    } else {
        Write-Host ''
        Write-Host "烧录失败，CLI 退出码：$exitCode" -ForegroundColor Red
        Write-Host "日志：$consoleLog"
    }
    Pause-Window
    exit $exitCode
} catch {
    Write-Host ''
    Write-Host ("未执行烧录：{0}" -f $_.Exception.Message) -ForegroundColor Red
    Pause-Window
    exit 3
}

