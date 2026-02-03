#!/usr/bin/env python3


# Bringup launch file for main component
# - rakuda_description: robot model and visualisation
# - rakuda_control: robot controllers and state publishers
# - head_motion filter: head motion smoothing
# - rakuda_vision: head camera


from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, LogInfo, EmitEvent, TimerAction
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.events import Shutdown


def generate_launch_description():
    # 1) ros2 launch rakuda_description description.launch.py
    desc_proc = ExecuteProcess(
        cmd=["ros2", "launch", "rakuda_description", "description.launch.py"],
        output="screen",
        emulate_tty=True,
    )

    # 2) ros2 launch rakuda_control control.launch.py
    control_proc = ExecuteProcess(
        cmd=["ros2", "launch", "rakuda_control", "control.launch.py"],
        output="screen",
        emulate_tty=True,
    )

    # 3) ros2 run rakuda_tools head_motion_filter
    head_motion_filter_proc = ExecuteProcess(
        cmd=["ros2", "run", "rakuda_tools", "head_motion_filter"],
        output="screen",
        emulate_tty=True,
    )

    # 4) ros2 launch rakuda_vision head_camera_335.launch.py
    camera_proc = ExecuteProcess(
        cmd=["ros2", "launch", "rakuda_vision", "head_camera_335.launch.py"],
        output="screen",
        emulate_tty=True,
    )

    # Delay per dare tempo ai nodi "a monte" di pubblicare parametri / TF.
    # Se vuoi partire subito, porta i period a 0.0.
    delay_desc_to_control = 1.5
    delay_control_to_filter = 1.0
    delay_filter_to_camera = 1.0

    start_control_when_desc_starts = RegisterEventHandler(
        OnProcessStart(
            target_action=desc_proc,
            on_start=[
                LogInfo(msg="[bringup] description avviato -> avvio control..."),
                TimerAction(period=delay_desc_to_control, actions=[control_proc]),
            ],
        )
    )

    start_filter_when_control_starts = RegisterEventHandler(
        OnProcessStart(
            target_action=control_proc,
            on_start=[
                LogInfo(msg="[bringup] control avviato -> avvio head_motion_filter..."),
                TimerAction(period=delay_control_to_filter, actions=[head_motion_filter_proc]),
            ],
        )
    )

    start_camera_when_filter_starts = RegisterEventHandler(
        OnProcessStart(
            target_action=head_motion_filter_proc,
            on_start=[
                LogInfo(msg="[bringup] head_motion_filter avviato -> avvio head_camera_335..."),
                TimerAction(period=delay_filter_to_camera, actions=[camera_proc]),
            ],
        )
    )

    # Se muore uno dei processi, chiude tutto (bringup "fail-fast")
    shutdown_on_desc_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=desc_proc,
            on_exit=[
                LogInfo(msg="[bringup] description terminato -> shutdown"),
                EmitEvent(event=Shutdown(reason="description exited")),
            ],
        )
    )
    shutdown_on_control_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=control_proc,
            on_exit=[
                LogInfo(msg="[bringup] control terminato -> shutdown"),
                EmitEvent(event=Shutdown(reason="control exited")),
            ],
        )
    )
    shutdown_on_filter_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=head_motion_filter_proc,
            on_exit=[
                LogInfo(msg="[bringup] head_motion_filter terminato -> shutdown"),
                EmitEvent(event=Shutdown(reason="head_motion_filter exited")),
            ],
        )
    )
    shutdown_on_camera_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=camera_proc,
            on_exit=[
                LogInfo(msg="[bringup] head_camera_335 terminato -> shutdown"),
                EmitEvent(event=Shutdown(reason="camera launch exited")),
            ],
        )
    )

    return LaunchDescription([
        LogInfo(msg="[bringup] avvio sequenziale: description -> control -> head_motion_filter -> head_camera_335"),
        desc_proc,

        start_control_when_desc_starts,
        start_filter_when_control_starts,
        start_camera_when_filter_starts,

        shutdown_on_desc_exit,
        shutdown_on_control_exit,
        shutdown_on_filter_exit,
        shutdown_on_camera_exit,
    ])

