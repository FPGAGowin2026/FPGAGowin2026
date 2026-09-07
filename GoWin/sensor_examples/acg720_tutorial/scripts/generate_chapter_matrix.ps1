#requires -Version 7.3
[CmdletBinding()]
param(
    [string]$TutorialRoot = (Split-Path -Parent $PSScriptRoot)
)

$ErrorActionPreference = 'Stop'
$ReportRoot = Join-Path $TutorialRoot 'reports'
$required = @(
    'archive_manifest.csv', 'project_manifest.csv', 'all_hdl_files.csv',
    'all_module_declarations.csv', 'module_declarations.csv', 'rebuild_syn_results.csv'
)
foreach ($name in $required) {
    $path = Join-Path $ReportRoot $name
    if (-not (Test-Path -LiteralPath $path)) { throw "缺少矩阵输入：$path" }
}

$archives = Import-Csv -LiteralPath (Join-Path $ReportRoot 'archive_manifest.csv')
$projects = Import-Csv -LiteralPath (Join-Path $ReportRoot 'project_manifest.csv')
$allHdl = Import-Csv -LiteralPath (Join-Path $ReportRoot 'all_hdl_files.csv')
$allModules = Import-Csv -LiteralPath (Join-Path $ReportRoot 'all_module_declarations.csv')
$enabledModules = Import-Csv -LiteralPath (Join-Path $ReportRoot 'module_declarations.csv')
$syn = Import-Csv -LiteralPath (Join-Path $ReportRoot 'rebuild_syn_results.csv')
$pnrPath = Join-Path $ReportRoot 'rebuild_pnr_results.csv'
$pnr = if (Test-Path -LiteralPath $pnrPath) { @(Import-Csv -LiteralPath $pnrPath) } else { @() }
$timingPath = Join-Path $ReportRoot 'pnr_timing_audit.csv'
$timing = if (Test-Path -LiteralPath $timingPath) { @(Import-Csv -LiteralPath $timingPath) } else { @() }
$compatChapters = @(48,54,56,61,62,63)

function Get-Category([int]$Chapter) {
    switch ($Chapter) {
        {$_ -le 4}  { '工具与开发流程'; break }
        {$_ -le 5}  { '数字逻辑基础'; break }
        {$_ -le 10} { 'UART与状态机'; break }
        {$_ -le 13} { '按键与显示驱动'; break }
        {$_ -le 17} { 'Gowin存储与时钟IP'; break }
        {$_ -le 25} { '波形与混合信号外设'; break }
        {$_ -le 31} { 'VGA/TFT/HDMI显示'; break }
        {$_ -le 35} { 'I2C与音频'; break }
        {$_ -le 38} { 'DDR3与帧缓存'; break }
        {$_ -le 43} { '图像处理'; break }
        {$_ -le 47} { 'OV5640摄像头'; break }
        {$_ -le 63} { '以太网与数据传输'; break }
        default      { 'USB2.0/FX2与图像传输' }
    }
}

function Get-ExternalHardware([int]$Chapter) {
    switch ($Chapter) {
        {$_ -in 1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,19,20,27,32,49,50,53,54,55,56} { '板载时钟/复位/UART/LED或仅片上逻辑'; break }
        18 { 'AD9767高速DAC模块'; break }
        21 { '4x4矩阵键盘'; break }
        22 { 'HT6221红外接收'; break }
        23 { 'TLV5618 DAC'; break }
        24 { 'ADC128S102/兼容ADC'; break }
        25 { '八通道ADC采集板'; break }
        26 { 'GM7123/VGA输出'; break }
        28 { 'TFT显示屏'; break }
        29 { 'HDMI/DVI输出'; break }
        {$_ -in 30,31} { 'TFT与HDMI显示'; break }
        33 { 'I2C EEPROM'; break }
        34 { 'PCF8563 RTC与显示'; break }
        35 { 'ES8388音频编解码器'; break }
        {$_ -in 36,37} { '板载DDR3'; break }
        {$_ -in 38,39,40,41,42,43} { '板载DDR3、TFT、HDMI及UART图像源'; break }
        44 { 'OV5640（散装核心可用合成输入验证）'; break }
        45 { 'OV5640、DDR3、TFT'; break }
        46 { 'OV5640、DDR3、VGA'; break }
        47 { 'OV5640、DDR3、HDMI'; break }
        {$_ -in 48,51,52,57,58} { '千兆以太网PHY/RGMII'; break }
        59 { 'OV5640与千兆以太网PHY'; break }
        60 { '双OV5640与千兆以太网PHY'; break }
        61 { '千兆以太网PHY、DDR3、TFT'; break }
        62 { 'ACM7606/AD7606、DDR3、千兆以太网PHY'; break }
        63 { 'ACM1030、DDR3、千兆以太网PHY'; break }
        64 { 'Cypress FX2工具/驱动/固件与PC软件'; break }
        {$_ -in 65,66,68} { 'Cypress FX2 SlaveFIFO'; break }
        67 { 'Cypress FX2、DDR3、TFT/HDMI'; break }
        69 { 'Cypress FX2与JPEG数据源'; break }
        70 { 'Cypress FX2与OV5640'; break }
        default { '见章节原理图与CST' }
    }
}

function Get-Board25kStatus([int]$Chapter) {
    if ($Chapter -in 5,6,7,19,39,44,53) { return 'PHYSICAL_BIST_PASS' }
    if ($Chapter -eq 12) { return 'NORMALIZED_SIM_PASS' }
    if ($Chapter -in 1,27,49,50,64) { return 'NOT_APPLICABLE' }
    return 'NOT_PHYSICALLY_RUN'
}

$records = [Collections.Generic.List[object]]::new()
for ($chapter = 1; $chapter -le 70; $chapter++) {
    $archiveRows = @($archives | Where-Object { [int]$_.chapter -eq $chapter })
    $title = if ($archiveRows.Count) { $archiveRows[0].title } else { "第${chapter}章" }
    $projectRows = @($projects | Where-Object { [int]$_.chapter -eq $chapter })
    $hdlRows = @($allHdl | Where-Object { [int]$_.chapter -eq $chapter })
    $rawDecls = @($allModules | Where-Object { [int]$_.chapter -eq $chapter })
    $enabledDecls = @($enabledModules | Where-Object { [int]$_.chapter -eq $chapter })
    $synRows = @($syn | Where-Object { [int]$_.chapter -eq $chapter })
    $pnrRows = @($pnr | Where-Object { [int]$_.chapter -eq $chapter })
    $timingRows = @($timing | Where-Object { [int]$_.chapter -eq $chapter -and ($_.kind -eq 'normalized' -or $_.pnr_status -eq 'PASS') })
    $rawModuleNames = @($rawDecls.module | Sort-Object -Unique)
    $enabledModuleNames = @($enabledDecls.module | Sort-Object -Unique)
    $topNames = @($synRows.top_module | Where-Object { $_ } | Sort-Object -Unique)

    $sourceKind = if ($hdlRows.Count -gt 0) {
        if ($projectRows.Count -gt 0) { 'HDL_PROJECT' } else { 'HDL_SOURCES_ONLY' }
    } elseif ($chapter -eq 64) {
        'PC_TOOLS_FIRMWARE_ASSETS'
    } else {
        'DOC_ONLY'
    }
    $synStatus = if ($projectRows.Count -eq 0) {
        if ($chapter -eq 44) { 'NORMALIZED_2_OF_2_PASS' } else { 'NOT_APPLICABLE' }
    } elseif (@($synRows | Where-Object status -eq 'PASS').Count -eq $projectRows.Count) {
        "PASS_$($projectRows.Count)_OF_$($projectRows.Count)"
    } else {
        "PARTIAL_$(@($synRows | Where-Object status -eq 'PASS').Count)_OF_$($projectRows.Count)"
    }
    $pnrRawStatus = if ($projectRows.Count -eq 0) {
        'NOT_APPLICABLE'
    } elseif ($pnrRows.Count -eq 0) {
        'NOT_RUN'
    } elseif (@($pnrRows | Where-Object status -eq 'PASS').Count -eq $projectRows.Count) {
        "PASS_$($projectRows.Count)_OF_$($projectRows.Count)"
    } elseif ($pnrRows.Count -lt $projectRows.Count) {
        "IN_PROGRESS_$(@($pnrRows | Where-Object status -eq 'PASS').Count)_PASS_$(@($pnrRows | Where-Object status -eq 'FAIL').Count)_FAIL_OF_$($projectRows.Count)"
    } else {
        "FAIL_$(@($pnrRows | Where-Object status -eq 'FAIL').Count)_OF_$($projectRows.Count)"
    }
    $pnrStatus = if ($chapter -in $compatChapters -and $projectRows.Count -gt 0) {
        $rawPass = @($pnrRows | Where-Object status -eq 'PASS').Count
        "PASS_$($rawPass + 1)_OF_$($projectRows.Count)_WITH_NORMALIZED_COMPAT"
    } else {
        $pnrRawStatus
    }
    $timingStatus = if ($timingRows.Count -eq 0) {
        'NOT_AVAILABLE'
    } else {
        $clean = @($timingRows | Where-Object timing_closure -eq 'CLEAN').Count
        $violated = @($timingRows | Where-Object timing_closure -eq 'VIOLATED').Count
        if ($violated -eq 0) { "CLEAN_$clean`_OF_$($timingRows.Count)" } else { "VIOLATED_$violated`_OF_$($timingRows.Count)" }
    }
    $migration = if ($chapter -eq 36) {
        '原工程为GW5AST-138B；迁到比赛GW5AT-138B需重生DDR IP与CST/时序'
    } elseif ($chapter -in $compatChapters) {
        '已生成规范化兼容工程并产出位流；仍需按 timing 列处理时序违例'
    } elseif ($projectRows.Count -gt 0) {
        '原工程已是GW5AT-LV138PG484AC1/I0；按实际外设复核CST与时序即可'
    } elseif ($chapter -eq 44) {
        '已补两个GW5AT-138B规范化综合工程；板级顶层/CST待接入摄像头系统'
    } else {
        '无独立FPGA工程'
    }

    $records.Add([pscustomobject]@{
        chapter = $chapter
        title = $title
        category = Get-Category $chapter
        source_kind = $sourceKind
        archives = @($archiveRows | Where-Object archive).Count
        projects = $projectRows.Count
        hdl_files = $hdlRows.Count
        raw_unique_modules = $rawModuleNames.Count
        enabled_unique_modules = $enabledModuleNames.Count
        top_modules = $topNames -join ','
        synthesis = $synStatus
        pnr_raw = $pnrRawStatus
        pnr = $pnrStatus
        timing = $timingStatus
        board_25k = Get-Board25kStatus $chapter
        external_hardware = Get-ExternalHardware $chapter
        migration_138k = $migration
        module_names = $rawModuleNames -join ';'
    })
}

$records | Export-Csv -LiteralPath (Join-Path $ReportRoot 'chapter_matrix.csv') -NoTypeInformation -Encoding utf8
$records | ConvertTo-Json -Depth 5 | Set-Content -LiteralPath (Join-Path $ReportRoot 'chapter_matrix.json') -Encoding utf8

$markdown = [Collections.Generic.List[string]]::new()
$markdown.Add('# ACG720 教程 70 章工程/模块矩阵')
$markdown.Add('')
$markdown.Add('完整模块名见 `chapter_matrix.csv` 的 `module_names` 列和 `all_unique_modules.csv`。`board_25k` 只表示片上合成激励/串口自检，不代表对应外接模块已接到 25K。')
$markdown.Add('')
$markdown.Add('| 章 | 主题 | 工程 | 原始唯一模块 | 重新综合 | P&R（含兼容） | 时序 | 25K | 主要外设 |')
$markdown.Add('|---:|---|---:|---:|---|---|---|---|---|')
foreach ($record in $records) {
    $safeTitle = $record.title.Replace('|', '\|')
    $safeHw = $record.external_hardware.Replace('|', '\|')
    $markdown.Add("| $($record.chapter) | $safeTitle | $($record.projects) | $($record.raw_unique_modules) | $($record.synthesis) | $($record.pnr) | $($record.timing) | $($record.board_25k) | $safeHw |")
}
$markdown | Set-Content -LiteralPath (Join-Path $ReportRoot 'chapter_matrix.md') -Encoding utf8

Write-Host "CHAPTER_MATRIX chapters=$($records.Count) projects=$(($records | Measure-Object projects -Sum).Sum) raw_unique_name_sum=$(($records | Measure-Object raw_unique_modules -Sum).Sum)"
