"""
CArmSingleCol / CArmDualBot — 对齐 C++ SDK 接口的 Python 包装类。

通过组合方式封装底层 :class:`Carm` 内核，对外提供与 C++
``CArmSingleCol`` / ``CArmDualBot`` 一致的方法签名与返回值约定：

* 命令类方法返回 ``int``：``1`` 表示成功，``<1`` 表示失败。
* 查询类方法返回与 C++ 对应的数据类型（``list`` / ``float`` / ``dict`` 等）。
* 输出参数通过传入可变容器（``list`` / ``dict``）就地填充，匹配 C++ 引用语义。
"""

import time
from typing import Callable
from .carm_kernel import Carm

def _to_int(success: bool) -> int:
    """bool → int（True→1, False→-1）"""
    return 1 if success else -1

# ====================================================================== #
#  CArmSingleCol — 单臂包装（对齐 C++ CArmSingleCol）
# ====================================================================== #
class CArmSingleCol:
    """
    单臂控制器，组合一个 :class:`Carm` 实例，对外接口与 C++
    ``CArmSingleCol`` 完全对齐。

    命令类方法返回 ``int``（``1`` 成功 / ``-1`` 失败），
    查询类方法返回对应数据类型，输出参数通过可变容器就地填充。
    """

    def __init__(self, server_ip: str = "10.42.0.101", port: int = 8090,
                 timeout: float = 1, arm_index: int = 0,
                 _validate_arm: bool = True):
        self._impl = Carm(addr=server_ip, arm_index=arm_index, port=port)
        self._arm_index = arm_index
        self._validate_arm = _validate_arm
        # 对齐 C++ CArmSingleCol：A3（六轴）系列单臂专用，构造时若已连接则校验臂型
        time.sleep(0.1)
        if self._validate_arm and self._impl.is_connected() \
                and not self._is_specified_arm():
            self._impl.disconnect()
            raise RuntimeError(
                "CArmSingleCol is designed for A3 series, "
                "but the arm is not A3 series")

    # ------------------------------------------------------------------ #
    #  连接 / 断开
    # ------------------------------------------------------------------ #
    def connect(self, server_ip: str = "10.42.0.101", port: int = 8090,
                timeout: float = 1) -> int:
        ok = self._impl.connect(addr=server_ip, port=port, timeout=timeout)
        time.sleep(0.1)
        # 对齐 C++ CArmSingleCol::connect：连接后校验臂型，不符合抛出异常
        if self._validate_arm and self._impl.is_connected() \
                and not self._is_specified_arm():
            self._impl.disconnect()
            raise RuntimeError(
                "CArmSingleCol is designed for A3 series, "
                "but the arm is not A3 series")
        return _to_int(ok)

    def disconnect(self) -> int:
        try:
            self._impl.disconnect()
            return 1
        except Exception:
            return -1

    def is_connected(self) -> bool:
        return self._impl.is_connected()

    # ------------------------------------------------------------------ #
    #  臂型校验（对齐 C++ CArmSingleCol::_is_specified_arm）
    # ------------------------------------------------------------------ #
    def _is_specified_arm(self) -> bool:
        """校验当前连接的机械臂是否为 A3（六轴）系列。

        状态上报周期为 20ms，这里轮询等待最多 40ms 以拿到有效状态。
        """
        status = {}
        before = time.monotonic()
        while self._impl.is_connected():
            status = self._impl._arm_state
            if status.get("arm_dof", 0) == 6 \
                    and "A3" in status.get("arm_name", ""):
                return True
            if time.monotonic() - before > 0.04:
                break
            time.sleep(0.01)
        print("CArmSingleCol is designed for A3 series, but the arm is "
              f"{status.get('arm_name', '')} with {status.get('arm_dof', 0)} dof")
        return False

    # ------------------------------------------------------------------ #
    #  基础控制
    # ------------------------------------------------------------------ #
    def set_ready(self) -> int:
        return _to_int(self._impl.set_ready())

    def set_servo_enable(self, enable: bool) -> int:
        return _to_int(self._impl.set_servo_enable(enable))

    def set_control_mode(self, mode: int) -> int:
        return _to_int(self._impl.set_control_mode(mode))

    def set_passthrough_data(self, mode: int, can_id: int, data: list) -> int:
        """透传数据。``can_id`` / ``data`` 为 list 容器，会被就地填充返回值。"""
        ok, ret_can_id, ret_data = self._impl.set_passthrough_data(mode, can_id, data)
        if isinstance(can_id, list):
            can_id.clear()
            can_id.append(ret_can_id)
        if isinstance(data, list):
            data.clear()
            data.extend(ret_data)
        return _to_int(ok)

    def set_ecat_passthrough_data(self, mode: int, frame: dict,
                                  timeout_ms: int = 100) -> int:
        """通过 EtherCAT 透传板同步发送或接收完整 CAN/CAN FD 帧。

        返回值：1 成功；-1 SDK 通信失败；-2 后端拒绝或执行失败；
        -3 后端响应格式异常；-4 Python 参数序列化失败。
        """
        code, response_frame = self._impl.set_ecat_passthrough_data(mode, frame, timeout_ms)
        if code == 1:
            frame.clear()
            frame.update(response_frame)
        return code

    def emergency_stop(self) -> int:
        return _to_int(self._impl.stop(type=3))

    def task_stop(self) -> int:
        return _to_int(self._impl.stop_task(at_once=True))

    def set_debug(self, flag: bool) -> int:
        return _to_int(self._impl.set_debug(flag))

    def set_speed_level(self, level: float, response_level: int = 20) -> int:
        return _to_int(self._impl.set_speed_level(level, response_level))

    def set_drag_params(self, torque_factor: list,
                        friction_compensation_factor: list) -> int:
        return _to_int(self._impl.set_drag_params(torque_factor,
                                                   friction_compensation_factor))

    def set_collision_config(self, enable_flag: bool = True,
                             sensitivity_level: int = 0) -> int:
        return _to_int(self._impl.set_collision_config(flag=enable_flag, level=sensitivity_level))

    def set_tool_index(self, index: int) -> int:
        return _to_int(self._impl.set_tool_index(index))

    def get_tool_index(self) -> int:
        return self._impl.tool_index

    def get_tool_coordinate(self, index: int) -> list:
        return self._impl.get_tool_coordinate(index)

    # ------------------------------------------------------------------ #
    #  版本 / 配置 / 状态
    # ------------------------------------------------------------------ #
    def get_version(self) -> str:
        return self._impl.version

    def get_config(self) -> dict:
        return self._impl.get_limits()

    def get_eeff_config(self) -> dict:
        return self._impl.get_eeff_config()

    def get_status(self) -> dict:
        return self._impl._arm_state

    # ------------------------------------------------------------------ #
    #  关节 / 末端状态查询
    # ------------------------------------------------------------------ #
    def get_joint_pos(self) -> list:
        return self._impl.joint_pos

    def get_joint_vel(self) -> list:
        return self._impl.joint_vel

    def get_joint_tau(self) -> list:
        return self._impl.joint_tau

    def get_plan_joint_pos(self) -> list:
        return self._impl.plan_joint_pos

    def get_plan_joint_vel(self) -> list:
        return self._impl.plan_joint_vel

    def get_plan_joint_tau(self) -> list:
        return self._impl.plan_joint_tau

    def get_plan_cart_pose(self) -> list:
        return self._impl.plan_cart_pose

    def get_cart_pose(self) -> list:
        return self._impl.cart_pose

    def get_joint_external_tau(self) -> list:
        return self._impl.joint_external_tau

    def get_cart_external_force(self) -> list:
        return self._impl.cart_external_force

    # ------------------------------------------------------------------ #
    #  末端执行器（通用 eeff）
    # ------------------------------------------------------------------ #
    def get_eeff_state(self) -> int:
        return self._impl.end_effector_state

    def get_eeff_pos(self) -> list:
        return self._impl.end_effector_pos

    def get_eeff_vel(self) -> list:
        return self._impl.end_effector_vel

    def get_eeff_tau(self) -> list:
        return self._impl.end_effector_tau

    def get_eeff_motor_pos(self) -> list:
        return self._impl.end_effector_motor_pos

    def get_eeff_motor_vel(self) -> list:
        return self._impl.end_effector_motor_vel

    def get_eeff_motor_tau(self) -> list:
        return self._impl.end_effector_motor_tau

    def get_plan_eeff_pos(self) -> list:
        return self._impl.plan_end_effector_pos

    def get_plan_eeff_vel(self) -> list:
        return self._impl.plan_end_effector_vel

    def get_plan_eeff_tau(self) -> list:
        return self._impl.plan_end_effector_tau

    def get_plan_eeff_motor_pos(self) -> list:
        return self._impl.plan_end_effector_motor_pos

    def get_plan_eeff_motor_vel(self) -> list:
        return self._impl.plan_end_effector_motor_vel

    def get_plan_eeff_motor_tau(self) -> list:
        return self._impl.plan_end_effector_motor_tau

    def get_eeff_type(self) -> str:
        return self._impl.end_effector_type

    def get_eeff_dof(self) -> int:
        return self._impl.end_effector_dof

    def get_eeff_connect(self) -> bool:
        return self._impl.end_effector_is_connect

    def set_eeff(self, pos: list, vel: list, tau: list, control_motor: bool = False) -> int:
        return _to_int(self._impl.set_eeff(pos, vel, tau, control_motor))

    # ------------------------------------------------------------------ #
    #  deprecated: 旧夹爪/灵巧手接口，内部转发到 eeff 等价方法
    # ------------------------------------------------------------------ #
    def get_gripper_state(self) -> int:
        return self._impl.gripper_state

    def get_gripper_pos(self) -> float:
        return self._impl.gripper_pos

    def get_gripper_vel(self) -> float:
        return self._impl.end_effector_vel[0] if self._impl.end_effector_vel else 0.0

    def get_gripper_tau(self) -> float:
        return self._impl.gripper_tau

    def get_plan_gripper_pos(self) -> float:
        return self._impl.plan_gripper_pos

    def get_plan_gripper_tau(self) -> float:
        return self._impl.plan_gripper_tau

    def set_gripper(self, pos: float, tau: float = 10) -> int:
        return _to_int(self._impl.set_gripper(pos, tau))

    def get_hand_state(self) -> int:
        return self._impl.hand_state

    def get_hand_pos(self) -> list:
        return self._impl.hand_pos

    def get_hand_vel(self) -> list:
        return self._impl.hand_vel

    def get_hand_tau(self) -> list:
        return self._impl.hand_tau

    def get_plan_hand_pos(self) -> list:
        return self._impl.plan_hand_pos

    def get_plan_hand_vel(self) -> list:
        return self._impl.plan_hand_vel

    def get_plan_hand_tau(self) -> list:
        return self._impl.plan_hand_tau

    def set_hand(self, pos: list, tau: list, vel: list) -> int:
        return _to_int(self._impl.set_hand(pos, tau, vel))

    # ------------------------------------------------------------------ #
    #  轨迹跟踪
    # ------------------------------------------------------------------ #
    def track_joint(self, targets: list, eeff_pos: float = -1) -> int:
        end_effector = eeff_pos if eeff_pos >= 0 else None
        return _to_int(self._impl.track_joint(targets, end_effector))

    def track_pose(self, targets: list, eeff_pos: float = -1) -> int:
        end_effector = eeff_pos if eeff_pos >= 0 else None
        return _to_int(self._impl.track_pose(targets, end_effector))

    # ------------------------------------------------------------------ #
    #  运动指令
    # ------------------------------------------------------------------ #
    def move_joint(self, target_pos: list, desire_time: float = -1,
                   is_sync: bool = True) -> int:
        return _to_int(self._impl.move_joint(target_pos, desire_time, is_sync))

    def move_pose(self, target_pos: list, desire_time: float = -1,
                  is_sync: bool = True) -> int:
        return _to_int(self._impl.move_pose(target_pos, desire_time, is_sync))

    def move_line_joint(self, target_pos: list, is_sync: bool = True) -> int:
        return _to_int(self._impl.move_line_joint(target_pos, is_sync))

    def move_line_pose(self, target_pos: list, is_sync: bool = True) -> int:
        return _to_int(self._impl.move_line_pose(target_pos, is_sync))

    def move_joint_traj(self, target_pos: list, eeff_pos: list = None,
                        stamps: list = None, is_sync: bool = True) -> int:
        return _to_int(self._impl.move_joint_traj(target_pos, eeff_pos, stamps, is_sync))

    def move_pose_traj(self, target_pos: list, eeff_pos: list = None,
                       stamps: list = None, is_sync: bool = True) -> int:
        return _to_int(self._impl.move_pose_traj(target_pos, eeff_pos, stamps, is_sync))

    def move_flow_pose(self, target_pos: list, line_theta_weight: float = 0.5,
                       accuracy: float = 0.0001, is_sync: bool = True) -> int:
        return _to_int(self._impl.move_flow_pose(
            target_pos, line_theta_weight, accuracy, False, is_sync))

    # ------------------------------------------------------------------ #
    #  示教 / 轨迹复现
    # ------------------------------------------------------------------ #
    def trajectory_teach(self, off_on: bool, name: str) -> int:
        return _to_int(self._impl.trajectory_teach(off_on, name))

    def trajectory_recorder(self, name: str, is_sync: bool = True) -> int:
        return _to_int(self._impl.trajectory_recorder(name, is_sync))

    def check_teach(self, traj_list: list) -> int:
        """获取已记录的示教轨迹列表，结果填充到 *traj_list*。"""
        result = self._impl.check_teach()
        if isinstance(traj_list, list):
            traj_list.clear()
            traj_list.extend(result)
        return 1 if result is not None else -1

    # ------------------------------------------------------------------ #
    #  运动学
    # ------------------------------------------------------------------ #
    def inverse_kine(self, tool_index: int, quat_pose: list,
                     ref_joint: list, jnt_value: list) -> int:
        result = self._impl.inverse_kine(quat_pose, ref_joint, tool=tool_index)
        if result is None:
            return -1
        if isinstance(jnt_value, list):
            jnt_value.clear()
            jnt_value.extend(result)
        return 1

    def forward_kine(self, tool_index: int, jnt_value: list,
                     quat_pose: list) -> int:
        result = self._impl.forward_kine(jnt_value, tool=tool_index)
        if result is None:
            return -1
        if isinstance(quat_pose, list):
            quat_pose.clear()
            quat_pose.extend(result)
        return 1

    def inverse_kine_array(self, tool_index: int, quat_pose: list,
                           ref_joint: list, jnt_value: list) -> int:
        results = []
        for pose, ref in zip(quat_pose, ref_joint):
            r = self._impl.inverse_kine(pose, ref, tool=tool_index)
            if r is None:
                return -1
            results.append(r)
        if isinstance(jnt_value, list):
            jnt_value.clear()
            jnt_value.extend(results)
        return 1

    def forward_kine_array(self, tool_index: int, jnt_value: list,
                           quat_pose: list) -> int:
        results = []
        for joints in jnt_value:
            r = self._impl.forward_kine(joints, tool=tool_index)
            if r is None:
                return -1
            results.append(r)
        if isinstance(quat_pose, list):
            quat_pose.clear()
            quat_pose.extend(results)
        return 1

    # ------------------------------------------------------------------ #
    #  回调注册（对齐 C++ 单回调语义）
    # ------------------------------------------------------------------ #
    def register_joint_cbk(self, cbk: Callable) -> None:
        self._register_keyed_cbk("get_joint", cbk, "_joint_cbks",
                                 lambda c: self._impl.on_update(
                                     lambda msg: self._dispatch_joint(c, msg)))

    def release_joint_cbk(self) -> None:
        self._release_keyed_cbk("get_joint", "_joint_cbks")

    def register_pose_cbk(self, cbk: Callable) -> None:
        self._register_keyed_cbk("get_pose", cbk, "_pose_cbks",
                                 lambda c: self._impl.on_update(
                                     lambda msg: self._dispatch_pose(c, msg)))

    def release_pose_cbk(self) -> None:
        self._release_keyed_cbk("get_pose", "_pose_cbks")

    def register_plan_joint_cbk(self, cbk: Callable) -> None:
        self._register_keyed_cbk("get_cmd_joint", cbk, "_plan_joint_cbks",
                                 lambda c: self._impl.on_update(
                                     lambda msg: self._dispatch_plan_joint(c, msg)))

    def release_plan_joint_cbk(self) -> None:
        self._release_keyed_cbk("get_cmd_joint", "_plan_joint_cbks")

    def register_plan_pose_cbk(self, cbk: Callable) -> None:
        self._register_keyed_cbk("get_cmd_pose", cbk, "_plan_pose_cbks",
                                 lambda c: self._impl.on_update(
                                     lambda msg: self._dispatch_plan_pose(c, msg)))

    def release_plan_pose_cbk(self) -> None:
        self._release_keyed_cbk("get_cmd_pose", "_plan_pose_cbks")

    def register_external_force_cbk(self, cbk: Callable) -> None:
        self._register_keyed_cbk("get_external_force", cbk, "_ext_force_cbks",
                                 lambda c: self._impl.on_update(
                                     lambda msg: self._dispatch_ext_force(c, msg)))

    def release_external_force_cbk(self) -> None:
        self._release_keyed_cbk("get_external_force", "_ext_force_cbks")

    def register_error_cbk(self, key: str, cbk: Callable) -> None:
        self._register_keyed_cbk(key, cbk, "_error_cbks",
                                 lambda c: self._impl.on_error(
                                     lambda code, msg: self._dispatch_error(c, code, msg)))

    def release_error_cbk(self, key: str) -> None:
        self._release_keyed_cbk(key, "_error_cbks")

    def register_completion_cbk(self, key: str, cbk: Callable) -> None:
        self._register_keyed_cbk(key, cbk, "_completion_cbks",
                                 lambda c: self._impl.on_task_finish(
                                     lambda task_key: self._dispatch_completion(c, task_key)))

    def release_completion_cbk(self, key: str) -> None:
        self._release_keyed_cbk(key, "_completion_cbks")

    # ---- 回调内部辅助 ----
    def _register_keyed_cbk(self, key, cbk, attr_name, registrator):
        if not hasattr(self, attr_name):
            setattr(self, attr_name, {})
        store = getattr(self, attr_name)
        if not store:
            # 第一次注册时才挂载底层回调
            registrator(store)
        store[key] = cbk

    def _release_keyed_cbk(self, key, attr_name):
        store = getattr(self, attr_name, {})
        store.pop(key, None)

    def _dispatch_joint(self, store, msg):
        st = self._impl._arm_state
        t = st.get("Unix_time_stamp", 0.0)
        reality = st.get("reality", {})
        for cbk in store.values():
            cbk(t, reality.get("pose", []), reality.get("vel", []),
                 reality.get("torque", []))

    def _dispatch_pose(self, store, msg):
        st = self._impl._arm_state
        t = st.get("Unix_time_stamp", 0.0)
        for cbk in store.values():
            cbk(t, st.get("pose", []))

    def _dispatch_plan_joint(self, store, msg):
        st = self._impl._arm_state
        t = st.get("Unix_time_stamp", 0.0)
        plan = st.get("plan", {})
        for cbk in store.values():
            cbk(t, plan.get("pose", []), plan.get("vel", []),
                 plan.get("torque", []))

    def _dispatch_plan_pose(self, store, msg):
        st = self._impl._arm_state
        t = st.get("Unix_time_stamp", 0.0)
        for cbk in store.values():
            cbk(t, st.get("plan", {}).get("cart_pose", []))

    def _dispatch_ext_force(self, store, msg):
        st = self._impl._arm_state
        t = st.get("Unix_time_stamp", 0.0)
        for cbk in store.values():
            cbk(t, st.get("joint_external_tau", []),
                 st.get("cart_external_force", []))

    def _dispatch_error(self, store, code, msg):
        for cbk in store.values():
            cbk(code, msg)

    def _dispatch_completion(self, store, task_key):
        for cbk in store.values():
            cbk(task_key)

    # ------------------------------------------------------------------ #
    #  底层透传接口
    # ------------------------------------------------------------------ #
    def set_low_mode(self, flag: bool) -> int:
        return _to_int(self._impl.set_low_mode(flag))

    def low_pv_command(self, pos: list, vel: list, data: dict) -> int:
        ok, low = self._impl.low_pv_command(pos, vel)
        if isinstance(data, dict):
            data.clear()
            data.update(low if isinstance(low, dict) else {})
        return _to_int(ok)

    def low_mit_command(self, pos: list, vel: list, tau: list,
                        kp: list, kd: list, data: dict) -> int:
        ok, low = self._impl.low_mit_command(pos, vel, tau, kp, kd)
        if isinstance(data, dict):
            data.clear()
            data.update(low if isinstance(low, dict) else {})
        return _to_int(ok)

    def low_pf_command(self, pos: list, vel: list, tau: list, data: dict) -> int:
        ok, low = self._impl.low_pf_command(pos, vel, tau)
        if isinstance(data, dict):
            data.clear()
            data.update(low if isinstance(low, dict) else {})
        return _to_int(ok)

    def low_current_command(self, tau: list, data: dict) -> int:
        ok, low = self._impl.low_current_command(tau)
        if isinstance(data, dict):
            data.clear()
            data.update(low if isinstance(low, dict) else {})
        return _to_int(ok)

    def low_refresh(self, data: dict) -> int:
        ok, low = self._impl.low_refresh()
        if isinstance(data, dict):
            data.clear()
            data.update(low if isinstance(low, dict) else {})
        return _to_int(ok)

    def low_set_end_effector_ctr(self, pos: list, vel: list, tau: list,
                                 data: dict) -> int:
        ok, low = self._impl.low_set_end_effector_ctr(pos, vel, tau)
        if isinstance(data, dict):
            data.clear()
            data.update(low if isinstance(low, dict) else {})
        return _to_int(ok)

    def low_set_robot_mode(self, mode: int) -> int:
        return _to_int(self._impl.low_set_robot_mode(mode))

    def low_set_end_effector_mode(self, mode: int) -> int:
        return _to_int(self._impl.low_set_end_effector_mode(mode))

    def low_set_servo_enable(self, status: bool) -> int:
        return _to_int(self._impl.low_set_servo_enable(status))

    def low_reset(self, cnt: int = 5) -> int:
        return _to_int(self._impl.low_reset(cnt))

    def low_get_servo_status(self, status: dict) -> int:
        result = self._impl.low_get_servo_status()
        if isinstance(status, dict):
            status.clear()
            status.update(result if isinstance(result, dict) else {})
        return 1 if result is not None else -1

    def low_get_inverse_kine(self, pose: list, refer_pos: list,
                             jnt_value: list, tool: int = -1) -> int:
        ok, result = self._impl.low_get_inverse_kine(pose, refer_pos, tool)
        if isinstance(jnt_value, list):
            jnt_value.clear()
            jnt_value.extend(result if result else [])
        return _to_int(ok)

    def low_get_forward_kine(self, joint_pos: list, quat_pose: list,
                             tool: int = -1) -> int:
        ok, result = self._impl.low_get_forward_kine(joint_pos, tool)
        if isinstance(quat_pose, list):
            quat_pose.clear()
            quat_pose.extend(result if result else [])
        return _to_int(ok)

    def low_get_dynamics(self, joint_pos: list, joint_vel: list,
                         joint_acc: list, tool: list, m_force: list,
                         c_force: list, g_force: list) -> int:
        ok, result = self._impl.low_get_dynamics(joint_pos, joint_vel, joint_acc)
        if not ok:
            return -1
        if isinstance(tool, list):
            tool.clear()
            tool.append(result.get("tool", -1))
        for name, container in [("m_force", m_force), ("c_force", c_force),
                                ("g_force", g_force)]:
            if isinstance(container, list):
                container.clear()
                container.extend(result.get(name, []))
        return 1

    def low_get_jacobian(self, joint_pos: list, tool: list, mat: dict) -> int:
        ok, result = self._impl.low_get_jacobian(joint_pos)
        if not ok:
            return -1
        if isinstance(tool, list):
            tool.clear()
            tool.append(result.get("tool", -1))
        if isinstance(mat, dict):
            mat.clear()
            mat.update(result)
        return 1

    def low_get_nullspace(self, joint_pos: list, tolerance: float,
                          tool: list, mat: dict) -> int:
        ok, result = self._impl.low_get_nullspace(joint_pos, tolerance)
        if not ok:
            return -1
        if isinstance(tool, list):
            tool.clear()
            tool.append(result.get("tool", -1))
        if isinstance(mat, dict):
            mat.clear()
            mat.update(result)
        return 1


# ====================================================================== #
#  CArmDualBot — 双臂包装（对齐 C++ CArmDualBot）
# ====================================================================== #
class CArmDualBot:
    """
    双臂控制器，组合两个 :class:`CArmSingleCol`（左/右），对外接口与
    C++ ``CArmDualBot`` 完全对齐。

    * 共享方法（connect / set_ready / set_speed_level 等）同时操作两个臂，
      全部成功才返回 1。
    * 单臂方法以 ``{verb}_{side}_{noun}`` 命名，与 C++ 一致，例如
      ``get_left_joint_pos``、``track_left_joint``、``move_left_joint``、
      ``low_left_pv_command``、``inverse_kine_left``。
    """

    def __init__(self, server_ip: str = "10.42.0.101", port: int = 8090,
                 timeout: float = 1, left_index: int = 0,
                 right_index: int = 1, _validate_arm: bool = True):
        # 内部组合的两个单臂不单独做 A3 校验，由 CArmDualBot 统一做 D3 校验
        self._left = CArmSingleCol(server_ip, port, timeout, arm_index=left_index,
                                   _validate_arm=False)
        time.sleep(0.1)  # 避免两个实例同时连接时出现端口冲突
        self._right = CArmSingleCol(server_ip, port, timeout, arm_index=right_index,
                                    _validate_arm=False)
        self._validate_arm = _validate_arm
        time.sleep(0.1)
        # 对齐 C++ CArmDualBot：D3（人形双臂）专用，构造时若双臂已连接则校验臂型
        if self._validate_arm and self._left.is_connected() \
                and self._right.is_connected() and not self._is_specified_arm():
            self._left.disconnect()
            self._right.disconnect()
            raise RuntimeError(
                "CArmDualBot is designed for D3 series, "
                "but the arm is not D3 series")

    # ================================================================== #
    #  共享方法
    # ================================================================== #
    def connect(self, server_ip: str = "10.42.0.101", port: int = 8090,
                timeout: float = 1) -> int:
        rl = self._left.connect(server_ip, port, timeout)
        time.sleep(0.1)  # 避免两个实例同时连接时出现端口冲突
        rr = self._right.connect(server_ip, port, timeout)
        time.sleep(0.1)
        # 对齐 C++ CArmDualBot::connect：双臂连接后校验臂型，不符合抛出异常
        if self._validate_arm and self._left.is_connected() \
                and self._right.is_connected() and not self._is_specified_arm():
            self._left.disconnect()
            self._right.disconnect()
            raise RuntimeError(
                "CArmDualBot is designed for D3 series, "
                "but the arm is not D3 series")
        return 1 if rl == 1 and rr == 1 else -1

    def disconnect(self) -> int:
        rl = self._left.disconnect()
        time.sleep(0.1)  # 避免两个实例同时连接时出现端口冲突
        rr = self._right.disconnect()
        return 1 if rl == 1 and rr == 1 else -1

    def is_connected(self) -> bool:
        return self._left.is_connected() and self._right.is_connected()

    # ------------------------------------------------------------------ #
    #  臂型校验（对齐 C++ CArmDualBot::_is_specified_arm）
    # ------------------------------------------------------------------ #
    def _is_specified_arm(self) -> bool:
        """校验左右臂是否均为 D3（七轴人形双臂）系列。

        状态上报周期为 20ms，这里轮询等待最多 40ms 以拿到有效状态。
        """
        before = time.monotonic()
        status_l = self._left._impl._arm_state
        status_r = self._right._impl._arm_state
        while self._left.is_connected() and self._right.is_connected():
            status_l = self._left._impl._arm_state
            status_r = self._right._impl._arm_state
            if status_l.get("arm_dof", 0) == 7 \
                    and "D3" in status_l.get("arm_name", "") \
                    and status_r.get("arm_dof", 0) == 7 \
                    and "D3" in status_r.get("arm_name", ""):
                return True
            if time.monotonic() - before > 0.04:
                break
            time.sleep(0.01)
        print("CArmDualBot left arm is designed for D3 series, but the arm is "
              f"{status_l.get('arm_name', '')} with {status_l.get('arm_dof', 0)} dof")
        print("CArmDualBot right arm is designed for D3 series, but the arm is "
              f"{status_r.get('arm_name', '')} with {status_r.get('arm_dof', 0)} dof")
        return False

    def set_ready(self) -> int:
        rl = self._left.set_ready()
        rr = self._right.set_ready()
        return 1 if rl == 1 and rr == 1 else -1

    def set_servo_enable(self, enable: bool) -> int:
        rl = self._left.set_servo_enable(enable)
        rr = self._right.set_servo_enable(enable)
        return 1 if rl == 1 and rr == 1 else -1

    def set_left_servo_enable(self, enable: bool) -> int:
        return self._left.set_servo_enable(enable)

    def set_right_servo_enable(self, enable: bool) -> int:
        return self._right.set_servo_enable(enable)

    def set_control_mode(self, mode: int) -> int:
        rl = self._left.set_control_mode(mode)
        rr = self._right.set_control_mode(mode)
        return 1 if rl == 1 and rr == 1 else -1

    def set_left_control_mode(self, mode: int) -> int:
        return self._left.set_control_mode(mode)

    def set_right_control_mode(self, mode: int) -> int:
        return self._right.set_control_mode(mode)

    # def set_passthrough_data(self, mode: int, can_id: int, data: list) -> int:
    #     rl = self._left.set_passthrough_data(mode, can_id, data)
    #     return 1 if rl == 1 else -1

    def set_left_ecat_passthrough_data(self, mode: int, frame: dict,
                                       timeout_ms: int = 100) -> int:
        """通过左臂 EtherCAT CAN 总线同步透传完整帧。"""
        return self._left.set_ecat_passthrough_data(mode, frame, timeout_ms)

    def set_right_ecat_passthrough_data(self, mode: int, frame: dict,
                                        timeout_ms: int = 100) -> int:
        """通过右臂 EtherCAT CAN 总线同步透传完整帧。"""
        return self._right.set_ecat_passthrough_data(mode, frame, timeout_ms)

    def get_version(self) -> str:
        return self._left.get_version() + self._right.get_version()

    def emergency_stop(self) -> int:
        rl = self._left.emergency_stop()
        rr = self._right.emergency_stop()
        return 1 if rl == 1 and rr == 1 else -1

    def task_stop(self) -> int:
        rl = self._left.task_stop()
        rr = self._right.task_stop()
        return 1 if rl == 1 and rr == 1 else -1

    def set_debug(self, flag: bool) -> int:
        return self._left.set_debug(flag)

    def set_speed_level(self, level: float, response_level: int = 20) -> int:
        rl = self._left.set_speed_level(level, response_level)
        rr = self._right.set_speed_level(level, response_level)
        return 1 if rl == 1 and rr == 1 else -1

    def set_collision_config(self, enable_flag: bool = True,
                             sensitivity_level: int = 0) -> int:
        rl = self._left.set_collision_config(enable_flag, sensitivity_level)
        rr = self._right.set_collision_config(enable_flag, sensitivity_level)
        return 1 if rl == 1 and rr == 1 else -1

    def set_left_collision_config(self, enable_flag: bool = True,
                                  sensitivity_level: int = 0) -> int:
        return self._left.set_collision_config(enable_flag, sensitivity_level)

    def set_right_collision_config(self, enable_flag: bool = True,
                                   sensitivity_level: int = 0) -> int:
        return self._right.set_collision_config(enable_flag, sensitivity_level)

    def set_low_mode(self, flag: bool) -> int:
        return self._left.set_low_mode(flag)

    def register_error_cbk(self, key: str, cbk: Callable) -> None:
        self._left.register_error_cbk(key, cbk)
        self._right.register_error_cbk(key, cbk)

    def release_error_cbk(self, key: str) -> None:
        self._left.release_error_cbk(key)
        self._right.release_error_cbk(key)

    def register_completion_cbk(self, key: str, cbk: Callable) -> None:
        self._left.register_completion_cbk(key, cbk)
        self._right.register_completion_cbk(key, cbk)

    def release_completion_cbk(self, key: str) -> None:
        self._left.release_completion_cbk(key)
        self._right.release_completion_cbk(key)

    def check_teach(self, left_traj_list: list, right_traj_list: list) -> int:
        rl = self._left.check_teach(left_traj_list)
        rr = self._right.check_teach(right_traj_list)
        return 1 if rl == 1 and rr == 1 else -1

    # ================================================================== #
    #  左臂方法（命名规则与 C++ 一致：get_left_* / set_left_* / track_left_* 等）
    # ================================================================== #
    def get_left_config(self) -> dict:
        return self._left.get_config()

    def get_left_eeff_config(self) -> dict:
        return self._left.get_eeff_config()

    def get_left_status(self) -> dict:
        return self._left.get_status()

    def get_left_joint_pos(self) -> list:
        return self._left.get_joint_pos()

    def get_left_joint_vel(self) -> list:
        return self._left.get_joint_vel()

    def get_left_joint_tau(self) -> list:
        return self._left.get_joint_tau()

    def get_left_plan_joint_pos(self) -> list:
        return self._left.get_plan_joint_pos()

    def get_left_plan_joint_vel(self) -> list:
        return self._left.get_plan_joint_vel()

    def get_left_plan_joint_tau(self) -> list:
        return self._left.get_plan_joint_tau()

    def get_left_plan_cart_pose(self) -> list:
        return self._left.get_plan_cart_pose()

    def get_left_cart_pose(self) -> list:
        return self._left.get_cart_pose()

    def get_left_joint_external_tau(self) -> list:
        return self._left.get_joint_external_tau()

    def get_left_cart_external_force(self) -> list:
        return self._left.get_cart_external_force()

    def register_left_joint_cbk(self, cbk: Callable) -> None:
        self._left.register_joint_cbk(cbk)

    def release_left_joint_cbk(self) -> None:
        self._left.release_joint_cbk()

    def register_left_pose_cbk(self, cbk: Callable) -> None:
        self._left.register_pose_cbk(cbk)

    def release_left_pose_cbk(self) -> None:
        self._left.release_pose_cbk()

    def register_left_plan_joint_cbk(self, cbk: Callable) -> None:
        self._left.register_plan_joint_cbk(cbk)

    def release_left_plan_joint_cbk(self) -> None:
        self._left.release_plan_joint_cbk()

    def register_left_plan_pose_cbk(self, cbk: Callable) -> None:
        self._left.register_plan_pose_cbk(cbk)

    def release_left_plan_pose_cbk(self) -> None:
        self._left.release_plan_pose_cbk()

    def register_left_external_force_cbk(self, cbk: Callable) -> None:
        self._left.register_external_force_cbk(cbk)

    def release_left_external_force_cbk(self) -> None:
        self._left.release_external_force_cbk()

    # ------------------------------------------------------------------ #
    #  左臂末端执行器（通用 eeff）
    # ------------------------------------------------------------------ #
    def get_left_eeff_state(self) -> int:
        return self._left.get_eeff_state()

    def get_left_eeff_pos(self) -> list:
        return self._left.get_eeff_pos()

    def get_left_eeff_vel(self) -> list:
        return self._left.get_eeff_vel()

    def get_left_eeff_tau(self) -> list:
        return self._left.get_eeff_tau()

    def get_left_eeff_motor_pos(self) -> list:
        return self._left.get_eeff_motor_pos()

    def get_left_eeff_motor_vel(self) -> list:
        return self._left.get_eeff_motor_vel()

    def get_left_eeff_motor_tau(self) -> list:
        return self._left.get_eeff_motor_tau()

    def get_left_plan_eeff_pos(self) -> list:
        return self._left.get_plan_eeff_pos()

    def get_left_plan_eeff_vel(self) -> list:
        return self._left.get_plan_eeff_vel()

    def get_left_plan_eeff_tau(self) -> list:
        return self._left.get_plan_eeff_tau()

    def get_left_plan_eeff_motor_pos(self) -> list:
        return self._left.get_plan_eeff_motor_pos()

    def get_left_plan_eeff_motor_vel(self) -> list:
        return self._left.get_plan_eeff_motor_vel()

    def get_left_plan_eeff_motor_tau(self) -> list:
        return self._left.get_plan_eeff_motor_tau()

    def get_left_eeff_type(self) -> str:
        return self._left.get_eeff_type()

    def get_left_eeff_dof(self) -> int:
        return self._left.get_eeff_dof()

    def get_left_eeff_connect(self) -> bool:
        return self._left.get_eeff_connect()

    def set_left_eeff(self, pos: list, vel: list, tau: list,
                      control_motor: bool = False) -> int:
        return self._left.set_eeff(pos, vel, tau, control_motor)

    # ------------------------------------------------------------------ #
    #  deprecated: 左臂旧夹爪/灵巧手接口
    # ------------------------------------------------------------------ #
    def get_left_gripper_state(self) -> int:
        return self._left.get_gripper_state()

    def get_left_gripper_pos(self) -> float:
        return self._left.get_gripper_pos()

    def get_left_gripper_vel(self) -> float:
        return self._left.get_gripper_vel()

    def get_left_gripper_tau(self) -> float:
        return self._left.get_gripper_tau()

    def get_left_plan_gripper_pos(self) -> float:
        return self._left.get_plan_gripper_pos()

    def get_left_plan_gripper_tau(self) -> float:
        return self._left.get_plan_gripper_tau()

    def get_left_hand_state(self) -> int:
        return self._left.get_hand_state()

    def get_left_hand_pos(self) -> list:
        return self._left.get_hand_pos()

    def get_left_hand_vel(self) -> list:
        return self._left.get_hand_vel()

    def get_left_hand_tau(self) -> list:
        return self._left.get_hand_tau()

    def get_left_plan_hand_pos(self) -> list:
        return self._left.get_plan_hand_pos()

    def get_left_plan_hand_vel(self) -> list:
        return self._left.get_plan_hand_vel()

    def get_left_plan_hand_tau(self) -> list:
        return self._left.get_plan_hand_tau()

    def track_left_joint(self, targets: list, eeff_pos: float = -1) -> int:
        return self._left.track_joint(targets, eeff_pos)

    def track_left_pose(self, targets: list, eeff_pos: float = -1) -> int:
        return self._left.track_pose(targets, eeff_pos)

    def move_left_joint(self, target_pos: list, desire_time: float = -1,
                        is_sync: bool = True) -> int:
        return self._left.move_joint(target_pos, desire_time, is_sync)

    def move_left_pose(self, target_pos: list, desire_time: float = -1,
                       is_sync: bool = True) -> int:
        return self._left.move_pose(target_pos, desire_time, is_sync)

    def move_left_joint_traj(self, target_pos: list, eeff_pos: list = None,
                             stamps: list = None, is_sync: bool = True) -> int:
        return self._left.move_joint_traj(target_pos, eeff_pos, stamps, is_sync)

    def move_left_pose_traj(self, target_pos: list, eeff_pos: list = None,
                            stamps: list = None, is_sync: bool = True) -> int:
        return self._left.move_pose_traj(target_pos, eeff_pos, stamps, is_sync)

    def move_left_flow_pose(self, target_pos: list,
                            line_theta_weight: float = 0.5,
                            accuracy: float = 0.0001,
                            is_sync: bool = True) -> int:
        return self._left.move_flow_pose(target_pos, line_theta_weight,
                                         accuracy, is_sync)

    def set_left_gripper(self, pos: float, tau: float = 10) -> int:
        return self._left.set_gripper(pos, tau)

    def set_left_hand(self, pos: list, tau: list, vel: list) -> int:
        return self._left.set_hand(pos, tau, vel)

    def set_left_tool_index(self, index: int) -> int:
        return self._left.set_tool_index(index)

    def get_left_tool_index(self) -> int:
        return self._left.get_tool_index()

    def get_left_tool_coordinate(self, index: int) -> list:
        return self._left.get_tool_coordinate(index)

    def trajectory_teach_left(self, off_on: bool, name: str) -> int:
        return self._left.trajectory_teach(off_on, name)

    def trajectory_recorder_left(self, name: str, is_sync: bool = True) -> int:
        return self._left.trajectory_recorder(name, is_sync)

    def inverse_kine_left(self, tool_index: int, quat_pose: list,
                          ref_joint: list, jnt_value: list) -> int:
        return self._left.inverse_kine(tool_index, quat_pose,
                                       ref_joint, jnt_value)

    def forward_kine_left(self, tool_index: int, jnt_value: list,
                          quat_pose: list) -> int:
        return self._left.forward_kine(tool_index, jnt_value, quat_pose)

    def inverse_kine_left_array(self, tool_index: int, quat_pose: list,
                                ref_joint: list, jnt_value: list) -> int:
        return self._left.inverse_kine_array(tool_index, quat_pose,
                                             ref_joint, jnt_value)

    def forward_kine_left_array(self, tool_index: int, jnt_value: list,
                                quat_pose: list) -> int:
        return self._left.forward_kine_array(tool_index, jnt_value, quat_pose)

    def low_left_pv_command(self, pos: list, vel: list, data: dict) -> int:
        return self._left.low_pv_command(pos, vel, data)

    def low_left_mit_command(self, pos: list, vel: list, tau: list,
                             kp: list, kd: list, data: dict) -> int:
        return self._left.low_mit_command(pos, vel, tau, kp, kd, data)

    def low_left_pf_command(self, pos: list, vel: list, tau: list,
                            data: dict) -> int:
        return self._left.low_pf_command(pos, vel, tau, data)

    def low_left_current_command(self, tau: list, data: dict) -> int:
        return self._left.low_current_command(tau, data)

    def low_left_refresh(self, data: dict) -> int:
        return self._left.low_refresh(data)

    def low_left_set_end_effector_ctr(self, pos: list, vel: list, tau: list,
                                      data: dict) -> int:
        return self._left.low_set_end_effector_ctr(pos, vel, tau, data)

    def low_left_set_robot_mode(self, mode: int) -> int:
        return self._left.low_set_robot_mode(mode)

    def low_left_set_end_effector_mode(self, mode: int) -> int:
        return self._left.low_set_end_effector_mode(mode)

    def low_left_set_servo_enable(self, status: bool) -> int:
        return self._left.low_set_servo_enable(status)

    def low_left_reset(self, cnt: int = 5) -> int:
        return self._left.low_reset(cnt)

    def low_left_get_servo_status(self, status: dict) -> int:
        return self._left.low_get_servo_status(status)

    def low_left_get_inverse_kine(self, pose: list, refer_pos: list,
                                  jnt_value: list, tool: int = -1) -> int:
        return self._left.low_get_inverse_kine(pose, refer_pos, jnt_value, tool)

    def low_left_get_forward_kine(self, joint_pos: list, quat_pose: list,
                                  tool: int = -1) -> int:
        return self._left.low_get_forward_kine(joint_pos, quat_pose, tool)

    def low_left_get_dynamics(self, joint_pos: list, joint_vel: list,
                              joint_acc: list, tool: list, m_force: list,
                              c_force: list, g_force: list) -> int:
        return self._left.low_get_dynamics(joint_pos, joint_vel, joint_acc,
                                           tool, m_force, c_force, g_force)

    def low_left_get_jacobian(self, joint_pos: list, tool: list,
                              mat: dict) -> int:
        return self._left.low_get_jacobian(joint_pos, tool, mat)

    def low_left_get_nullspace(self, joint_pos: list, tolerance: float,
                               tool: list, mat: dict) -> int:
        return self._left.low_get_nullspace(joint_pos, tolerance, tool, mat)

    # ================================================================== #
    #  右臂方法（命名规则与 C++ 一致：get_right_* / set_right_* / track_right_* 等）
    # ================================================================== #
    def get_right_config(self) -> dict:
        return self._right.get_config()

    def get_right_eeff_config(self) -> dict:
        return self._right.get_eeff_config()

    def get_right_status(self) -> dict:
        return self._right.get_status()

    def get_right_joint_pos(self) -> list:
        return self._right.get_joint_pos()

    def get_right_joint_vel(self) -> list:
        return self._right.get_joint_vel()

    def get_right_joint_tau(self) -> list:
        return self._right.get_joint_tau()

    def get_right_plan_joint_pos(self) -> list:
        return self._right.get_plan_joint_pos()

    def get_right_plan_joint_vel(self) -> list:
        return self._right.get_plan_joint_vel()

    def get_right_plan_joint_tau(self) -> list:
        return self._right.get_plan_joint_tau()

    def get_right_plan_cart_pose(self) -> list:
        return self._right.get_plan_cart_pose()

    def get_right_cart_pose(self) -> list:
        return self._right.get_cart_pose()

    def get_right_joint_external_tau(self) -> list:
        return self._right.get_joint_external_tau()

    def get_right_cart_external_force(self) -> list:
        return self._right.get_cart_external_force()

    def register_right_joint_cbk(self, cbk: Callable) -> None:
        self._right.register_joint_cbk(cbk)

    def release_right_joint_cbk(self) -> None:
        self._right.release_joint_cbk()

    def register_right_pose_cbk(self, cbk: Callable) -> None:
        self._right.register_pose_cbk(cbk)

    def release_right_pose_cbk(self) -> None:
        self._right.release_pose_cbk()

    def register_right_plan_joint_cbk(self, cbk: Callable) -> None:
        self._right.register_plan_joint_cbk(cbk)

    def release_right_plan_joint_cbk(self) -> None:
        self._right.release_plan_joint_cbk()

    def register_right_plan_pose_cbk(self, cbk: Callable) -> None:
        self._right.register_plan_pose_cbk(cbk)

    def release_right_plan_pose_cbk(self) -> None:
        self._right.release_plan_pose_cbk()

    def register_right_external_force_cbk(self, cbk: Callable) -> None:
        self._right.register_external_force_cbk(cbk)

    def release_right_external_force_cbk(self) -> None:
        self._right.release_external_force_cbk()

    # ------------------------------------------------------------------ #
    #  右臂末端执行器（通用 eeff）
    # ------------------------------------------------------------------ #
    def get_right_eeff_state(self) -> int:
        return self._right.get_eeff_state()

    def get_right_eeff_pos(self) -> list:
        return self._right.get_eeff_pos()

    def get_right_eeff_vel(self) -> list:
        return self._right.get_eeff_vel()

    def get_right_eeff_tau(self) -> list:
        return self._right.get_eeff_tau()

    def get_right_eeff_motor_pos(self) -> list:
        return self._right.get_eeff_motor_pos()

    def get_right_eeff_motor_vel(self) -> list:
        return self._right.get_eeff_motor_vel()

    def get_right_eeff_motor_tau(self) -> list:
        return self._right.get_eeff_motor_tau()

    def get_right_plan_eeff_pos(self) -> list:
        return self._right.get_plan_eeff_pos()

    def get_right_plan_eeff_vel(self) -> list:
        return self._right.get_plan_eeff_vel()

    def get_right_plan_eeff_tau(self) -> list:
        return self._right.get_plan_eeff_tau()

    def get_right_plan_eeff_motor_pos(self) -> list:
        return self._right.get_plan_eeff_motor_pos()

    def get_right_plan_eeff_motor_vel(self) -> list:
        return self._right.get_plan_eeff_motor_vel()

    def get_right_plan_eeff_motor_tau(self) -> list:
        return self._right.get_plan_eeff_motor_tau()

    def get_right_eeff_type(self) -> str:
        return self._right.get_eeff_type()

    def get_right_eeff_dof(self) -> int:
        return self._right.get_eeff_dof()

    def get_right_eeff_connect(self) -> bool:
        return self._right.get_eeff_connect()

    def set_right_eeff(self, pos: list, vel: list, tau: list,
                       control_motor: bool = False) -> int:
        return self._right.set_eeff(pos, vel, tau, control_motor)

    # ------------------------------------------------------------------ #
    #  deprecated: 右臂旧夹爪/灵巧手接口
    # ------------------------------------------------------------------ #
    def get_right_gripper_state(self) -> int:
        return self._right.get_gripper_state()

    def get_right_gripper_pos(self) -> float:
        return self._right.get_gripper_pos()

    def get_right_gripper_vel(self) -> float:
        return self._right.get_gripper_vel()

    def get_right_gripper_tau(self) -> float:
        return self._right.get_gripper_tau()

    def get_right_plan_gripper_pos(self) -> float:
        return self._right.get_plan_gripper_pos()

    def get_right_plan_gripper_tau(self) -> float:
        return self._right.get_plan_gripper_tau()

    def get_right_hand_state(self) -> int:
        return self._right.get_hand_state()

    def get_right_hand_pos(self) -> list:
        return self._right.get_hand_pos()

    def get_right_hand_vel(self) -> list:
        return self._right.get_hand_vel()

    def get_right_hand_tau(self) -> list:
        return self._right.get_hand_tau()

    def get_right_plan_hand_pos(self) -> list:
        return self._right.get_plan_hand_pos()

    def get_right_plan_hand_vel(self) -> list:
        return self._right.get_plan_hand_vel()

    def get_right_plan_hand_tau(self) -> list:
        return self._right.get_plan_hand_tau()

    def track_right_joint(self, targets: list, eeff_pos: float = -1) -> int:
        return self._right.track_joint(targets, eeff_pos)

    def track_right_pose(self, targets: list, eeff_pos: float = -1) -> int:
        return self._right.track_pose(targets, eeff_pos)

    def track_joint(self, left_targets: list, right_targets: list,
                    left_eeff_pos: float = -1,
                    right_eeff_pos: float = -1) -> int:
        """跟随运动，周期性发送目标关节位置，双臂同时运动（对齐 C++ CArmDualBot::track_joint）。"""
        ret = self.track_left_joint(left_targets, left_eeff_pos)
        if ret < 0:
            return ret
        return self.track_right_joint(right_targets, right_eeff_pos)

    def track_pose(self, left_targets: list, right_targets: list,
                   left_eeff_pos: float = -1,
                   right_eeff_pos: float = -1) -> int:
        """跟随运动，周期性发送目标位姿，法兰相对基座，双臂同时运动（对齐 C++ CArmDualBot::track_pose）。"""
        ret = self.track_left_pose(left_targets, left_eeff_pos)
        if ret < 0:
            return ret
        return self.track_right_pose(right_targets, right_eeff_pos)

    def move_right_joint(self, target_pos: list, desire_time: float = -1,
                         is_sync: bool = True) -> int:
        return self._right.move_joint(target_pos, desire_time, is_sync)

    def move_right_pose(self, target_pos: list, desire_time: float = -1,
                        is_sync: bool = True) -> int:
        return self._right.move_pose(target_pos, desire_time, is_sync)

    def move_right_joint_traj(self, target_pos: list, eeff_pos: list = None,
                              stamps: list = None,
                              is_sync: bool = True) -> int:
        return self._right.move_joint_traj(target_pos, eeff_pos,
                                           stamps, is_sync)

    def move_right_pose_traj(self, target_pos: list, eeff_pos: list = None,
                             stamps: list = None,
                             is_sync: bool = True) -> int:
        return self._right.move_pose_traj(target_pos, eeff_pos,
                                          stamps, is_sync)

    def move_right_flow_pose(self, target_pos: list,
                             line_theta_weight: float = 0.5,
                             accuracy: float = 0.0001,
                             is_sync: bool = True) -> int:
        return self._right.move_flow_pose(target_pos, line_theta_weight,
                                          accuracy, is_sync)

    def set_right_gripper(self, pos: float, tau: float = 10) -> int:
        return self._right.set_gripper(pos, tau)

    def set_right_hand(self, pos: list, tau: list, vel: list) -> int:
        return self._right.set_hand(pos, tau, vel)

    def set_right_tool_index(self, index: int) -> int:
        return self._right.set_tool_index(index)

    def get_right_tool_index(self) -> int:
        return self._right.get_tool_index()

    def get_right_tool_coordinate(self, index: int) -> list:
        return self._right.get_tool_coordinate(index)

    def trajectory_teach_right(self, off_on: bool, name: str) -> int:
        return self._right.trajectory_teach(off_on, name)

    def trajectory_teach(self, off_on: bool, name: str) -> int:
        """使用同一轨迹名同步开始或停止左右臂示教。"""
        rl = self._left.trajectory_teach(off_on, name)
        rr = self._right.trajectory_teach(off_on, name)
        return 1 if rl == 1 and rr == 1 else -1

    def trajectory_recorder_right(self, name: str, is_sync: bool = True) -> int:
        return self._right.trajectory_recorder(name, is_sync)

    def inverse_kine_right(self, tool_index: int, quat_pose: list,
                           ref_joint: list, jnt_value: list) -> int:
        return self._right.inverse_kine(tool_index, quat_pose,
                                        ref_joint, jnt_value)

    def forward_kine_right(self, tool_index: int, jnt_value: list,
                           quat_pose: list) -> int:
        return self._right.forward_kine(tool_index, jnt_value, quat_pose)

    def inverse_kine_right_array(self, tool_index: int, quat_pose: list,
                                 ref_joint: list, jnt_value: list) -> int:
        return self._right.inverse_kine_array(tool_index, quat_pose,
                                              ref_joint, jnt_value)

    def forward_kine_right_array(self, tool_index: int, jnt_value: list,
                                 quat_pose: list) -> int:
        return self._right.forward_kine_array(tool_index, jnt_value, quat_pose)

    def low_right_pv_command(self, pos: list, vel: list, data: dict) -> int:
        return self._right.low_pv_command(pos, vel, data)

    def low_right_mit_command(self, pos: list, vel: list, tau: list,
                              kp: list, kd: list, data: dict) -> int:
        return self._right.low_mit_command(pos, vel, tau, kp, kd, data)

    def low_right_pf_command(self, pos: list, vel: list, tau: list,
                             data: dict) -> int:
        return self._right.low_pf_command(pos, vel, tau, data)

    def low_right_current_command(self, tau: list, data: dict) -> int:
        return self._right.low_current_command(tau, data)

    def low_right_refresh(self, data: dict) -> int:
        return self._right.low_refresh(data)

    def low_right_set_end_effector_ctr(self, pos: list, vel: list, tau: list,
                                       data: dict) -> int:
        return self._right.low_set_end_effector_ctr(pos, vel, tau, data)

    def low_right_set_robot_mode(self, mode: int) -> int:
        return self._right.low_set_robot_mode(mode)

    def low_right_set_end_effector_mode(self, mode: int) -> int:
        return self._right.low_set_end_effector_mode(mode)

    def low_right_set_servo_enable(self, status: bool) -> int:
        return self._right.low_set_servo_enable(status)

    def low_right_reset(self, cnt: int = 5) -> int:
        return self._right.low_reset(cnt)

    def low_right_get_servo_status(self, status: dict) -> int:
        return self._right.low_get_servo_status(status)

    def low_right_get_inverse_kine(self, pose: list, refer_pos: list,
                                   jnt_value: list, tool: int = -1) -> int:
        return self._right.low_get_inverse_kine(pose, refer_pos,
                                                jnt_value, tool)

    def low_right_get_forward_kine(self, joint_pos: list, quat_pose: list,
                                   tool: int = -1) -> int:
        return self._right.low_get_forward_kine(joint_pos, quat_pose, tool)

    def low_right_get_dynamics(self, joint_pos: list, joint_vel: list,
                               joint_acc: list, tool: list, m_force: list,
                               c_force: list, g_force: list) -> int:
        return self._right.low_get_dynamics(joint_pos, joint_vel, joint_acc,
                                            tool, m_force, c_force, g_force)

    def low_right_get_jacobian(self, joint_pos: list, tool: list,
                               mat: dict) -> int:
        return self._right.low_get_jacobian(joint_pos, tool, mat)

    def low_right_get_nullspace(self, joint_pos: list, tolerance: float,
                                tool: list, mat: dict) -> int:
        return self._right.low_get_nullspace(joint_pos, tolerance, tool, mat)


# ====================================================================== #
#  CArmBust — 上半身包装（左臂、右臂、两自由度腰部）
# ====================================================================== #
class CArmBust(CArmDualBot):
    """上半身控制器。

    组合左右七自由度手臂和两自由度腰部。左右臂接口继承自
    :class:`CArmDualBot`；腰部仅提供关节空间运动、关节状态、示教与底层
    关节控制接口，不提供工具、末端执行器、透传或任务空间接口。
    """

    def __init__(self, server_ip: str = "10.42.0.101", port: int = 8090,
                 timeout: float = 1, left_index: int = 0, right_index: int = 1,
                 waist_index: int = 2, _validate_arm: bool = True):
        super().__init__(server_ip, port, timeout, left_index, right_index,
                         _validate_arm=False)
        time.sleep(0.1)
        self._waist = CArmSingleCol(server_ip, port, timeout, arm_index=waist_index,
                                    _validate_arm=False)
        self._validate_arm = _validate_arm
        time.sleep(0.1)
        if self._validate_arm and self.is_connected() and not self._is_specified_arm():
            self.disconnect()
            raise RuntimeError("CArmBust is designed for D3 arms with a 2 dof waist")

    def _is_specified_arm(self) -> bool:
        before = time.monotonic()
        status_l = self._left.get_status()
        status_r = self._right.get_status()
        status_w = self._waist.get_status()
        while self.is_connected():
            status_l = self._left.get_status()
            status_r = self._right.get_status()
            status_w = self._waist.get_status()
            if status_l.get("arm_dof", 0) == 7 and "D3" in status_l.get("arm_name", "") \
                    and status_r.get("arm_dof", 0) == 7 \
                    and "D3" in status_r.get("arm_name", "") \
                    and status_w.get("arm_dof", 0) == 2:
                return True
            if time.monotonic() - before > 0.04:
                break
            time.sleep(0.01)
        return False

    def connect(self, server_ip: str = "10.42.0.101", port: int = 8090,
                timeout: float = 1) -> int:
        left_ret = self._left.connect(server_ip, port, timeout)
        time.sleep(0.1)
        right_ret = self._right.connect(server_ip, port, timeout)
        time.sleep(0.1)
        waist_ret = self._waist.connect(server_ip, port, timeout)
        if self._validate_arm and self.is_connected() and not self._is_specified_arm():
            self.disconnect()
            raise RuntimeError("CArmBust is designed for D3 arms with a 2 dof waist")
        return 1 if left_ret == 1 and right_ret == 1 and waist_ret == 1 else -1

    def disconnect(self) -> int:
        dual_ret = super().disconnect()
        waist_ret = self._waist.disconnect()
        return 1 if dual_ret == 1 and waist_ret == 1 else -1

    def is_connected(self) -> bool:
        return super().is_connected() and self._waist.is_connected()

    def set_ready(self) -> int:
        dual_ret = super().set_ready()
        waist_ret = self._waist.set_ready()
        return 1 if dual_ret == 1 and waist_ret == 1 else -1

    def set_servo_enable(self, enable: bool) -> int:
        dual_ret = super().set_servo_enable(enable)
        waist_ret = self._waist.set_servo_enable(enable)
        return 1 if dual_ret == 1 and waist_ret == 1 else -1

    def set_waist_servo_enable(self, enable: bool) -> int:
        return self._waist.set_servo_enable(enable)

    def set_control_mode(self, mode: int) -> int:
        dual_ret = super().set_control_mode(mode)
        waist_ret = self._waist.set_control_mode(mode)
        return 1 if dual_ret == 1 and waist_ret == 1 else -1

    def set_waist_control_mode(self, mode: int) -> int:
        return self._waist.set_control_mode(mode)

    def get_version(self) -> str:
        return super().get_version() + self._waist.get_version()

    def emergency_stop(self) -> int:
        dual_ret = super().emergency_stop()
        waist_ret = self._waist.emergency_stop()
        return 1 if dual_ret == 1 and waist_ret == 1 else -1

    def task_stop(self) -> int:
        dual_ret = super().task_stop()
        waist_ret = self._waist.task_stop()
        return 1 if dual_ret == 1 and waist_ret == 1 else -1

    def set_speed_level(self, level: float, response_level: int = 20) -> int:
        dual_ret = super().set_speed_level(level, response_level)
        waist_ret = self._waist.set_speed_level(level, response_level)
        return 1 if dual_ret == 1 and waist_ret == 1 else -1

    def set_collision_config(self, enable_flag: bool = True,
                             sensitivity_level: int = 0) -> int:
        dual_ret = super().set_collision_config(enable_flag, sensitivity_level)
        waist_ret = self._waist.set_collision_config(enable_flag, sensitivity_level)
        return 1 if dual_ret == 1 and waist_ret == 1 else -1

    def set_waist_collision_config(self, enable_flag: bool = True,
                                   sensitivity_level: int = 0) -> int:
        return self._waist.set_collision_config(enable_flag, sensitivity_level)

    def register_error_cbk(self, key: str, cbk: Callable) -> None:
        super().register_error_cbk(key, cbk)
        self._waist.register_error_cbk(key, cbk)

    def release_error_cbk(self, key: str) -> None:
        super().release_error_cbk(key)
        self._waist.release_error_cbk(key)

    def register_completion_cbk(self, key: str, cbk: Callable) -> None:
        super().register_completion_cbk(key, cbk)
        self._waist.register_completion_cbk(key, cbk)

    def release_completion_cbk(self, key: str) -> None:
        super().release_completion_cbk(key)
        self._waist.release_completion_cbk(key)

    def trajectory_teach(self, off_on: bool, name: str) -> int:
        dual_ret = super().trajectory_teach(off_on, name)
        waist_ret = self._waist.trajectory_teach(off_on, name)
        return 1 if dual_ret == 1 and waist_ret == 1 else -1

    def trajectory_teach_waist(self, off_on: bool, name: str) -> int:
        return self._waist.trajectory_teach(off_on, name)

    def trajectory_recorder_waist(self, name: str, is_sync: bool = True) -> int:
        return self._waist.trajectory_recorder(name, is_sync)

    def check_teach(self, left_traj_list: list, right_traj_list: list,
                    waist_traj_list: list) -> int:
        dual_ret = super().check_teach(left_traj_list, right_traj_list)
        waist_ret = self._waist.check_teach(waist_traj_list)
        return 1 if dual_ret == 1 and waist_ret == 1 else -1

    def get_waist_config(self) -> dict:
        return self._waist.get_config()

    def get_waist_status(self) -> dict:
        return self._waist.get_status()

    def get_waist_joint_pos(self) -> list:
        return self._waist.get_joint_pos()

    def get_waist_joint_vel(self) -> list:
        return self._waist.get_joint_vel()

    def get_waist_joint_tau(self) -> list:
        return self._waist.get_joint_tau()

    def get_waist_plan_joint_pos(self) -> list:
        return self._waist.get_plan_joint_pos()

    def get_waist_plan_joint_vel(self) -> list:
        return self._waist.get_plan_joint_vel()

    def get_waist_plan_joint_tau(self) -> list:
        return self._waist.get_plan_joint_tau()

    def get_waist_joint_external_tau(self) -> list:
        return self._waist.get_joint_external_tau()

    def register_waist_joint_cbk(self, cbk: Callable) -> None:
        self._waist.register_joint_cbk(cbk)

    def release_waist_joint_cbk(self) -> None:
        self._waist.release_joint_cbk()

    def register_waist_plan_joint_cbk(self, cbk: Callable) -> None:
        self._waist.register_plan_joint_cbk(cbk)

    def release_waist_plan_joint_cbk(self) -> None:
        self._waist.release_plan_joint_cbk()

    def track_waist_joint(self, targets: list) -> int:
        if len(targets) != 2:
            return -1
        return self._waist.track_joint(targets)

    def move_waist_joint(self, target_pos: list, desire_time: float = -1,
                         is_sync: bool = True) -> int:
        if len(target_pos) != 2:
            return -1
        return self._waist.move_joint(target_pos, desire_time, is_sync)

    def move_waist_joint_traj(self, target_pos: list, stamps: list = None,
                              is_sync: bool = True) -> int:
        if not target_pos or any(len(point) != 2 for point in target_pos):
            return -1
        return self._waist.move_joint_traj(target_pos, [], stamps, is_sync)

    def low_waist_pv_command(self, pos: list, vel: list, data: dict) -> int:
        return self._waist.low_pv_command(pos, vel, data)

    def low_waist_mit_command(self, pos: list, vel: list, tau: list,
                              kp: list, kd: list, data: dict) -> int:
        return self._waist.low_mit_command(pos, vel, tau, kp, kd, data)

    def low_waist_pf_command(self, pos: list, vel: list, tau: list,
                             data: dict) -> int:
        return self._waist.low_pf_command(pos, vel, tau, data)

    def low_waist_current_command(self, tau: list, data: dict) -> int:
        return self._waist.low_current_command(tau, data)

    def low_waist_refresh(self, data: dict) -> int:
        return self._waist.low_refresh(data)

    def low_waist_set_robot_mode(self, mode: int) -> int:
        return self._waist.low_set_robot_mode(mode)

    def low_waist_set_servo_enable(self, status: bool) -> int:
        return self._waist.low_set_servo_enable(status)

    def low_waist_reset(self, cnt: int = 5) -> int:
        return self._waist.low_reset(cnt)

    def low_waist_get_servo_status(self, status: dict) -> int:
        return self._waist.low_get_servo_status(status)
