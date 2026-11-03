#pragma once

#include <cstdint>

namespace uav_common
{

// 整个 FlightTask 的生命周期
enum class TaskLifecycle : std::uint8_t
{
    IDLE = 0,
    RUNNING,
    SUCCEEDED,
    FAILED,
    CANCELED
};

// 一次任务执行的结果
enum class TaskResult : std::uint8_t
{
    NONE = 0,
    SUCCESS,
    FAILURE,
    TIMEOUT,
    CANCELED
};

// FlightTask 当前执行到的阶段
enum class FlightStage : std::uint8_t
{
    IDLE = 0,

    WAIT_FOR_LOCALIZATION,
    WAIT_FOR_FCU,
    PRE_FLIGHT_CHECK,

    PREPARE_OFFBOARD,
    ENTER_OFFBOARD,

    ARMING,
    TAKEOFF,

    EXECUTE,
    HOLD,

    LANDING,
    DISARMING,

    FINISHED
};

}  // namespace uav_common