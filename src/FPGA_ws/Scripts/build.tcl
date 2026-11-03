# 从任意工作目录调用；FPGA_BOARD 指定板卡，FPGA_BUILD_STAGE 指定阶段。
set root [file normalize [file join [file dirname [info script]] ..]]
set board BX72_138K_robot
if {[info exists ::env(FPGA_BOARD)]} {set board $::env(FPGA_BOARD)}
if {$board ni {BX72_138K_robot ACG720_60K_robot Sipeed_25K_robot}} {
    error "FPGA_BOARD 必须为 BX72_138K_robot、ACG720_60K_robot 或 Sipeed_25K_robot"
}
set stage full
if {[info exists ::env(FPGA_BUILD_STAGE)]} {
    set stage $::env(FPGA_BUILD_STAGE)
} elseif {[info exists ::env(BX72_BUILD_STAGE)]} {
    # 兼容已有 BX72 调用环境。
    set stage $::env(BX72_BUILD_STAGE)
}
if {$stage ni {full syn pnr}} {error "FPGA_BUILD_STAGE / BX72_BUILD_STAGE must be full, syn or pnr"}
set project [file join $root boards $board ${board}.gprj]
set variant udp
if {$board eq "BX72_138K_robot" && [info exists ::env(FPGA_VARIANT)]} {
    set variant $::env(FPGA_VARIANT)
}
if {$variant ne "udp"} {error "BX72 只提供 UDP 工程；FPGA_VARIANT 必须为 udp"}
if {$board eq "Sipeed_25K_robot"} {
    error "25K 器件及 IO_LOC 尚未核对；先使用 run_sim.py --board Sipeed_25K_robot 验证波形"
}

if {$board eq "BX72_138K_robot"} {
    # 默认布局优先可布通性，布线优先速度，同时保留时序驱动。环境变量可用于
    # 对照实验：FPGA_PLACE_OPTION=0..4、FPGA_ROUTE_OPTION=0..2、
    # FPGA_ROUTE_MAXFAN=1..100、FPGA_TIMING_DRIVEN=0|1、
    # FPGA_CLOCK_ROUTE_ORDER=0|1、FPGA_CORRECT_HOLD=0|1、
    # FPGA_REPLICATE_RESOURCES=0|1、
    # FPGA_INCREMENTAL=none|auto|place|pnr、FPGA_OPT_GOAL=auto|area|timing、
    # FPGA_MAP_OPTION=1..4、FPGA_SYN_MAX_FANOUT=positive integer、
    # FPGA_SYN_NETLIST_HIERARCHY=0|1。
    proc env_or_default {name default_value} {
        if {[info exists ::env($name)]} {return $::env($name)}
        return $default_value
    }
    set place_option [env_or_default FPGA_PLACE_OPTION 1]
    set route_option [env_or_default FPGA_ROUTE_OPTION 2]
    set route_maxfan [env_or_default FPGA_ROUTE_MAXFAN 23]
    set clock_route_order [env_or_default FPGA_CLOCK_ROUTE_ORDER 0]
    set timing_driven [env_or_default FPGA_TIMING_DRIVEN 1]
    set correct_hold [env_or_default FPGA_CORRECT_HOLD 1]
    set replicate_resources [env_or_default FPGA_REPLICATE_RESOURCES 0]
    set incremental_mode [env_or_default FPGA_INCREMENTAL none]
    set optimization_goal [env_or_default FPGA_OPT_GOAL auto]
    set map_option [env_or_default FPGA_MAP_OPTION 1]
    set retiming_resource [env_or_default FPGA_RETIMING_RESOURCE none]
    set synthesis_max_fanout [env_or_default FPGA_SYN_MAX_FANOUT 1000]
    set synthesis_netlist_hierarchy [env_or_default FPGA_SYN_NETLIST_HIERARCHY 1]
    if {![string is integer -strict $place_option] || $place_option < 0 || $place_option > 4} {
        error "FPGA_PLACE_OPTION 必须为 0..4"
    }
    if {![string is integer -strict $route_option] || $route_option < 0 || $route_option > 2} {
        error "FPGA_ROUTE_OPTION 必须为 0..2"
    }
    if {![string is integer -strict $route_maxfan] || $route_maxfan < 1 || $route_maxfan > 100} {
        error "FPGA_ROUTE_MAXFAN 必须为 1..100"
    }
    if {$clock_route_order ni {0 1}} {error "FPGA_CLOCK_ROUTE_ORDER 必须为 0 或 1"}
    if {$timing_driven ni {0 1}} {error "FPGA_TIMING_DRIVEN 必须为 0 或 1"}
    if {$correct_hold ni {0 1}} {error "FPGA_CORRECT_HOLD 必须为 0 或 1"}
    if {$replicate_resources ni {0 1}} {error "FPGA_REPLICATE_RESOURCES 必须为 0 或 1"}
    if {$incremental_mode ni {none auto place pnr}} {
        error "FPGA_INCREMENTAL 必须为 none、auto、place 或 pnr"
    }
    if {$optimization_goal ni {auto area timing}} {
        error "FPGA_OPT_GOAL 必须为 auto、area 或 timing"
    }
    if {![string is integer -strict $map_option] || $map_option < 1 || $map_option > 4} {
        error "FPGA_MAP_OPTION 必须为 1..4"
    }
    if {$retiming_resource ni {none all bsram dsp}} {
        error "FPGA_RETIMING_RESOURCE 必须为 none、all、bsram 或 dsp"
    }
    if {![string is integer -strict $synthesis_max_fanout] || $synthesis_max_fanout < 1} {
        error "FPGA_SYN_MAX_FANOUT 必须为正整数"
    }
    if {$synthesis_netlist_hierarchy ni {0 1}} {
        error "FPGA_SYN_NETLIST_HIERARCHY 必须为 0 或 1"
    }

}

# 对照实验可指定独立工程，避免覆盖活动工程的实现结果。
if {$board eq "BX72_138K_robot" && [info exists ::env(FPGA_PROJECT)]} {
    set project [file normalize $::env(FPGA_PROJECT)]
}
open_project $project
if {$board eq "BX72_138K_robot"} {set_option -output_base_name BX72_138K_robot}
if {$board eq "BX72_138K_robot"} {set_option -gen_verilog_sim_netlist 1}
# PNR 阶段也要声明同一套 SystemVerilog 解析选项。否则重新打开工程时
# Gowin 可能以 Verilog 2001 重读源文件，并在读取综合网表前隐式重新综合。
set_option -verilog_std sysv2017
set_option -top_module robot_top
set_option -use_mspi_as_gpio 1
set_option -use_sspi_as_gpio 1
set_option -use_cpu_as_gpio 1
if {$board eq "BX72_138K_robot"} {
    set_option -place_option $place_option
    set_option -route_option $route_option
    set_option -route_maxfan $route_maxfan
    set_option -clock_route_order $clock_route_order
    set_option -timing_driven $timing_driven
    set_option -correct_hold_violation $correct_hold
    set_option -replicate_resources $replicate_resources
    set_option -retiming_resource $retiming_resource
    if {$stage ne "pnr"} {
        # PNR 使用已有综合配置，避免仅调整布线时重新标记综合为过期。
        set_option -opt_goal $optimization_goal
        set_option -map_option $map_option
        set_option -max_fanout $synthesis_max_fanout
        set_option -netlist_hierarchy $synthesis_netlist_hierarchy
    }
    if {$stage ne "syn"} {
        # Gowin 会把选项留在工程状态中；每次显式设置两种增量开关，避免沿用上次运行值。
        set_option -inc_place 0
        set_option -inc_pnr 0
        if {$incremental_mode eq "auto"} {
            set pr_file [file join [file dirname $project] impl pnr ${board}.pr]
            set p_file [file join [file dirname $project] impl pnr ${board}.p]
            if {[file exists $pr_file]} {
                set_option -inc_place 0
                set_option -inc_pnr auto
            } elseif {[file exists $p_file]} {
                set_option -inc_pnr 0
                set_option -inc_place auto
            }
        } elseif {$incremental_mode eq "place"} {
            set_option -inc_pnr 0
            set_option -inc_place auto
        } elseif {$incremental_mode eq "pnr"} {
            set_option -inc_place 0
            set_option -inc_pnr auto
        }
    }
    puts "Build configuration: opt_goal=$optimization_goal map_option=$map_option retiming_resource=$retiming_resource synth_max_fanout=$synthesis_max_fanout netlist_hierarchy=$synthesis_netlist_hierarchy place=$place_option route=$route_option route_maxfan=$route_maxfan clock_route_order=$clock_route_order timing_driven=$timing_driven correct_hold=$correct_hold replicate_resources=$replicate_resources incremental=$incremental_mode"
}
foreach step {syn pnr} {
    if {($step eq "syn" && $stage eq "pnr") || ($step eq "pnr" && $stage eq "syn")} {continue}
    set started [clock milliseconds]
    puts "BUILD_START $step $started"
    flush stdout
    run $step
    puts "BUILD_TIME $step [expr {([clock milliseconds]-$started)/1000.0}] seconds"
    if {$step eq "syn" && $board eq "BX72_138K_robot" && $stage eq "full"} {
        # 沿用110592逻辑门限；已超预算就直接修源，避免继续浪费PNR时间。
        set resource_file [file join [file dirname $project] impl gwsynthesis ${board}_syn_rsc.xml]
        if {[file exists $resource_file]} {
            set resource_stream [open $resource_file r]
            set resource_text [read $resource_stream]
            close $resource_stream
            regexp {<Module[^>]*>} $resource_text resource_root
            if {[regexp {T_Lut="([0-9]+)} $resource_root]} {
                set mapped_logic 0
                foreach {attribute weight} {T_Lut 1 T_Alu 1 T_Ssram 6} {
                    if {[regexp [format {%s="([0-9]+)} $attribute] $resource_root unused count]} {
                        incr mapped_logic [expr {$count*$weight}]
                    }
                }
                puts "SYN_LOGIC_BUDGET used=$mapped_logic limit=110592"
                if {$mapped_logic > 110592} {error "mapped Logic $mapped_logic exceeds unchanged budget 110592; PNR skipped"}
            }
            foreach {attribute label limit} {T_Register Register 111276 T_Bsram BSRAM 272} {
                if {[regexp [format {%s="([0-9]+)} $attribute] $resource_root unused count]} {
                    puts "SYN_RESOURCE_BUDGET $label used=$count limit=$limit"
                    if {$count > $limit} {error "$label $count exceeds unchanged budget $limit; PNR skipped"}
                }
            }
            set mapped_dsp 0.0
            foreach {attribute weight} {T_MULT27X36 2.0 T_MULTALU27X18 1.0 T_MULT12X12 0.5} {
                if {[regexp [format {%s="([0-9]+)} $attribute] $resource_root unused count]} {
                    set mapped_dsp [expr {$mapped_dsp+$count*$weight}]
                }
            }
            puts "SYN_RESOURCE_BUDGET DSP used=$mapped_dsp limit=238.4"
            if {$mapped_dsp > 238.4} {error "DSP $mapped_dsp exceeds unchanged budget 238.4; PNR skipped"}
        }
        # 综合时序仅供诊断；在实现完整时继续PNR，以未修改SDC的最终时序判断结果。
        set timing_report [file join [file dirname $project] impl gwsynthesis ${board}_syn.rpt.html]
        set timing_gate [file join [file dirname $project] synthesis_timing_gate.json]
        set timing_checker [file join $root Scripts check_synthesis_timing.py]
        set timing_sdc [file join $root boards $board src constraint ${board}.sdc]
        set synthesis_log [file join [file dirname $project] impl gwsynthesis ${board}.log]
        set preservation_args [list --resources $resource_file --synthesis-log $synthesis_log]
        foreach instance {robot udp_request_receiver eth_udp_video_sender ov5647_capture telemetry_native_u telemetry_control_u telemetry_mux_u telemetry_packet_cdc_u} {
            lappend preservation_args --required-instance $instance
        }
        if {![info exists ::env(FPGA_PYTHON_FOR_GATES)]} {error "missing gate Python; PNR skipped"}
        if {[catch {exec $::env(FPGA_PYTHON_FOR_GATES) $timing_checker --report $timing_report --sdc $timing_sdc --output $timing_gate --allow-estimated-timing-failure {*}$preservation_args} timing_result]} {
            puts $timing_result
            flush stdout
            error "SYNTHESIS_TIMING_REJECT: required clock or implementation missing; PNR skipped"
        }
        puts $timing_result
        flush stdout
    }
}
