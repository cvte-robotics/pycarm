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
        """初始化机械臂控制对象。

        :param server_ip: [输入] 控制器IPv4地址。
        :param port: [输入] 控制器TCP端口。
        :param timeout: [输入] 连接超时时间；单位s。
        :param arm_index: [输入] 目标机械臂编号。
        :param _validate_arm: [输入] 是否校验设备型号。
        :return: 无返回值。
        """
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
        """连接机械臂控制器并校验设备类型。

        :param server_ip: [输入] 控制器IPv4地址。
        :param port: [输入] 控制器TCP端口。
        :param timeout: [输入] 连接超时时间；单位s。
        :return: int，1表示成功，-1表示失败。
        """
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
        """断开机械臂控制器连接。

        :return: int，1表示成功，-1表示失败。
        """
        try:
            self._impl.disconnect()
            return 1
        except Exception:
            return -1

    def is_connected(self) -> bool:
        """检查机械臂控制器是否已连接。

        :return: bool，True表示已连接，False表示未连接。
        """
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
        """将机械臂切换到就绪状态。

        :return: int，1表示成功，-1表示失败。
        """
        return _to_int(self._impl.set_ready())

    def set_servo_enable(self, enable: bool) -> int:
        """设置机械臂伺服使能状态。

        :param enable: [输入] 是否使能；True为使能，False为失能。
        :return: int，1表示成功，-1表示失败。
        """
        return _to_int(self._impl.set_servo_enable(enable))

    def set_control_mode(self, mode: int) -> int:
        """设置机械臂控制模式。

        :param mode: [输入] 控制或透传模式枚举值。
        :return: int，1表示成功，-1表示失败。
        """
        return _to_int(self._impl.set_control_mode(mode))

    def set_passthrough_data(self, mode: int, can_id: int, data: list) -> int:
        """通过控制器透传CAN数据。

        :param mode: [输入] 透传模式枚举值。
        :param can_id: [输入/输出] CAN帧ID。
        :param data: [输入/输出] CAN负载字节列表；单位byte，成功时写入响应负载。
        :return: int，1表示成功，-1表示失败。
        """
        ok, _ret_can_id, ret_data = self._impl.set_passthrough_data(
            mode, can_id, data)
        if ok:
            # TODO: Python int 不可变，需确定响应 CAN ID 的输出方式后再实现回写。
            can_id = _ret_can_id
            if ret_data is not None and isinstance(data, list):
                data.clear()
                data.extend(ret_data)
        return _to_int(ok)

    def set_ecat_passthrough_data(self, mode: int, frame: dict,
                                  timeout_ms: int = 100) -> int:
        """通过 EtherCAT 透传板同步发送或接收完整 CAN/CAN FD 帧。

        :param mode: [输入] 透传模式枚举值。
        :param frame: [输入/输出] CAN/CAN FD帧字典；数据字段单位byte，成功时写入响应帧。
        :param timeout_ms: [输入] 响应超时时间；单位ms。
        :return: int，1成功；-1通信失败；-2后端拒绝或执行失败；
            -3响应格式异常；-4参数序列化失败。
        """
        code, response_frame = self._impl.set_ecat_passthrough_data(mode, frame, timeout_ms)
        if code == 1:
            frame.clear()
            frame.update(response_frame)
        return code

    def emergency_stop(self) -> int:
        """触发机械臂急停。

        :return: int，1表示成功，-1表示失败。
        """
        return _to_int(self._impl.stop(type=3))

    def task_stop(self) -> int:
        """立即停止机械臂当前任务。

        :return: int，1表示成功，-1表示失败。
        """
        return _to_int(self._impl.stop_task(at_once=True))

    def set_debug(self, flag: bool) -> int:
        """设置机械臂调试模式。

        :param flag: [输入] 功能开关；True为开启，False为关闭。
        :return: int，1表示成功，-1表示失败。
        """
        return _to_int(self._impl.set_debug(flag))

    def set_speed_level(self, level: float, response_level: int = 20) -> int:
        """设置机械臂速度等级。

        :param level: [输入] 全局速度十分比，范围由控制器定义。
        :param response_level: [输入] 速度响应等级。
        :return: int，1表示成功，-1表示失败。
        """
        return _to_int(self._impl.set_speed_level(level, response_level))

    def set_drag_params(self, torque_factor: list,
                        friction_compensation_factor: list) -> int:
        """设置机械臂拖动参数。

        :param torque_factor: [输入] 机械臂拖动转矩缩放系数，长度=dof。
        :param friction_compensation_factor: [输入] 机械臂摩擦补偿系数，长度=dof。
        :return: int，1表示成功，-1表示失败。
        """
        return _to_int(self._impl.set_drag_params(torque_factor,
                                                   friction_compensation_factor))

    def set_collision_config(self, enable_flag: bool = True,
                             sensitivity_level: int = 0) -> int:
        """设置机械臂碰撞检测配置。

        :param enable_flag: [输入] 是否启用碰撞检测。
        :param sensitivity_level: [输入] 碰撞灵敏度等级。
        :return: int，1表示成功，-1表示失败。
        """
        return _to_int(self._impl.set_collision_config(flag=enable_flag, level=sensitivity_level))

    def set_tool_index(self, index: int) -> int:
        """设置机械臂工具编号。

        :param index: [输入] 工具编号。
        :return: int，1表示成功，-1表示失败。
        """
        return _to_int(self._impl.set_tool_index(index))

    def get_tool_index(self) -> int:
        """获取工具编号。

        :return: int，枚举值、编号或自由度。
        """
        return self._impl.tool_index

    def get_tool_coordinate(self, index: int) -> list:
        """获取工具坐标。

        :param index: [输入] 工具编号。
        :return: list，[x,y,z,qx,qy,qz,qw]；位置单位m，姿态为归一化四元数。
        """
        return self._impl.get_tool_coordinate(index)

    # ------------------------------------------------------------------ #
    #  版本 / 配置 / 状态
    # ------------------------------------------------------------------ #
    def get_version(self) -> str:
        """获取SDK与控制器版本信息。

        :return: str，版本信息。
        """
        return self._impl.version

    def get_config(self) -> dict:
        """获取配置。

        :return: dict，配置或状态字典；各字段沿用对应物理单位。
        """
        return self._impl.get_limits()

    def get_eeff_config(self) -> dict:
        """获取末端执行器配置。

        :return: dict，配置或状态字典；各字段沿用对应物理单位。
        """
        return self._impl.get_eeff_config()

    def get_status(self) -> dict:
        """获取状态。

        :return: dict，配置或状态字典；各字段沿用对应物理单位。
        """
        return self._impl._arm_state

    # ------------------------------------------------------------------ #
    #  关节 / 末端状态查询
    # ------------------------------------------------------------------ #
    def get_joint_pos(self) -> list:
        """获取实际关节位置。

        :return: list或float，位置；单位rad。
        """
        return self._impl.joint_pos

    def get_joint_vel(self) -> list:
        """获取实际关节速度。

        :return: list或float，速度；单位rad/s。
        """
        return self._impl.joint_vel

    def get_joint_tau(self) -> list:
        """获取实际关节力矩。

        :return: list或float，力矩；单位Nm。
        """
        return self._impl.joint_tau

    def get_plan_joint_pos(self) -> list:
        """获取规划关节位置。

        :return: list或float，位置；单位rad。
        """
        return self._impl.plan_joint_pos

    def get_plan_joint_vel(self) -> list:
        """获取规划关节速度。

        :return: list或float，速度；单位rad/s。
        """
        return self._impl.plan_joint_vel

    def get_plan_joint_tau(self) -> list:
        """获取规划关节力矩。

        :return: list或float，力矩；单位Nm。
        """
        return self._impl.plan_joint_tau

    def get_plan_cart_pose(self) -> list:
        """获取规划末端位姿。

        :return: list，[x,y,z,qx,qy,qz,qw]；位置单位m，姿态为归一化四元数。
        """
        return self._impl.plan_cart_pose

    def get_cart_pose(self) -> list:
        """获取实际末端位姿。

        :return: list，[x,y,z,qx,qy,qz,qw]；位置单位m，姿态为归一化四元数。
        """
        return self._impl.cart_pose

    def get_joint_external_tau(self) -> list:
        """获取关节外力矩。

        :return: list或float，力矩；单位Nm。
        """
        return self._impl.joint_external_tau

    def get_cart_external_force(self) -> list:
        """获取末端外力/力矩。

        :return: list，[Fx,Fy,Fz,Tx,Ty,Tz]；单位N和N·m。
        """
        return self._impl.cart_external_force

    # ------------------------------------------------------------------ #
    #  末端执行器（通用 eeff）
    # ------------------------------------------------------------------ #
    def get_eeff_state(self) -> int:
        """获取末端执行器状态。

        :return: int，枚举值、编号或自由度。
        """
        return self._impl.end_effector_state

    def get_eeff_pos(self) -> list:
        """获取末端执行器位置。

        :return: list或float，位置；单位rad或m。
        """
        return self._impl.end_effector_pos

    def get_eeff_vel(self) -> list:
        """获取末端执行器速度。

        :return: list或float，速度；单位rad/s或m/s。
        """
        return self._impl.end_effector_vel

    def get_eeff_tau(self) -> list:
        """获取末端执行器力矩/力。

        :return: list或float，力矩/力；单位N·m或N。
        """
        return self._impl.end_effector_tau

    def get_eeff_motor_pos(self) -> list:
        """获取末端执行器电机位置。

        :return: list或float，位置；单位rad或m。
        """
        return self._impl.end_effector_motor_pos

    def get_eeff_motor_vel(self) -> list:
        """获取末端执行器电机速度。

        :return: list或float，速度；单位rad/s或m/s。
        """
        return self._impl.end_effector_motor_vel

    def get_eeff_motor_tau(self) -> list:
        """获取末端执行器电机力矩/力。

        :return: list或float，力矩/力；单位N·m或N。
        """
        return self._impl.end_effector_motor_tau

    def get_plan_eeff_pos(self) -> list:
        """获取规划末端执行器位置。

        :return: list或float，位置；单位rad或m。
        """
        return self._impl.plan_end_effector_pos

    def get_plan_eeff_vel(self) -> list:
        """获取规划末端执行器速度。

        :return: list或float，速度；单位rad/s或m/s。
        """
        return self._impl.plan_end_effector_vel

    def get_plan_eeff_tau(self) -> list:
        """获取规划末端执行器力矩/力。

        :return: list或float，力矩/力；单位N·m或N。
        """
        return self._impl.plan_end_effector_tau

    def get_plan_eeff_motor_pos(self) -> list:
        """获取规划末端执行器电机位置。

        :return: list或float，位置；单位rad或m。
        """
        return self._impl.plan_end_effector_motor_pos

    def get_plan_eeff_motor_vel(self) -> list:
        """获取规划末端执行器电机速度。

        :return: list或float，速度；单位rad/s或m/s。
        """
        return self._impl.plan_end_effector_motor_vel

    def get_plan_eeff_motor_tau(self) -> list:
        """获取规划末端执行器电机力矩/力。

        :return: list或float，力矩/力；单位N·m或N。
        """
        return self._impl.plan_end_effector_motor_tau

    def get_eeff_type(self) -> str:
        """获取末端执行器类型。

        :return: str，类型名称。
        """
        return self._impl.end_effector_type

    def get_eeff_dof(self) -> int:
        """获取末端执行器自由度。

        :return: int，枚举值、编号或自由度。
        """
        return self._impl.end_effector_dof

    def get_eeff_connect(self) -> bool:
        """获取末端执行器连接状态。

        :return: bool，True表示已连接，False表示未连接。
        """
        return self._impl.end_effector_is_connect

    def set_eeff(self, pos: list, vel: list, tau: list, control_motor: bool = False) -> int:
        """设置机械臂末端执行器。

        :param pos: [输入] 目标位置；关节/旋转电机单位rad，直线电机单位m。
        :param vel: [输入] 目标速度；旋转部分单位rad/s，直线部分单位m/s。
        :param tau: [输入] 目标力矩/力；单位N·m或N。
        :param control_motor: [输入] 是否直接使用电机坐标控制末端执行器。
        :return: int，1表示成功，-1表示失败。
        """
        return _to_int(self._impl.set_eeff(pos, vel, tau, control_motor))

    # ------------------------------------------------------------------ #
    #  deprecated: 旧夹爪/灵巧手接口，内部转发到 eeff 等价方法
    # ------------------------------------------------------------------ #
    def get_gripper_state(self) -> int:
        """获取夹爪状态。

        :return: int，枚举值、编号或自由度。
        """
        return self._impl.gripper_state

    def get_gripper_pos(self) -> float:
        """获取夹爪位置。

        :return: list或float，位置；单位rad或m。
        """
        return self._impl.gripper_pos

    def get_gripper_vel(self) -> float:
        """获取夹爪速度。

        :return: list或float，速度；单位rad/s或m/s。
        """
        return self._impl.end_effector_vel[0] if self._impl.end_effector_vel else 0.0

    def get_gripper_tau(self) -> float:
        """获取夹爪力矩/力。

        :return: list或float，力矩/力；单位N·m或N。
        """
        return self._impl.gripper_tau

    def get_plan_gripper_pos(self) -> float:
        """获取规划夹爪位置。

        :return: list或float，位置；单位rad或m。
        """
        return self._impl.plan_gripper_pos

    def get_plan_gripper_tau(self) -> float:
        """获取规划夹爪力矩/力。

        :return: list或float，力矩/力；单位N·m或N。
        """
        return self._impl.plan_gripper_tau

    def set_gripper(self, pos: float, tau: float = 10) -> int:
        """设置机械臂夹爪。

        :param pos: [输入] 目标位置；关节/旋转电机单位rad，直线电机单位m。
        :param tau: [输入] 目标力矩/力；单位N·m或N。
        :return: int，1表示成功，-1表示失败。
        """
        return _to_int(self._impl.set_gripper(pos, tau))

    def get_hand_state(self) -> int:
        """获取灵巧手状态。

        :return: int，枚举值、编号或自由度。
        """
        return self._impl.hand_state

    def get_hand_pos(self) -> list:
        """获取灵巧手位置。

        :return: list或float，位置；单位rad或m。
        """
        return self._impl.hand_pos

    def get_hand_vel(self) -> list:
        """获取灵巧手速度。

        :return: list或float，速度；单位rad/s或m/s。
        """
        return self._impl.hand_vel

    def get_hand_tau(self) -> list:
        """获取灵巧手力矩/力。

        :return: list或float，力矩/力；单位N·m或N。
        """
        return self._impl.hand_tau

    def get_plan_hand_pos(self) -> list:
        """获取规划灵巧手位置。

        :return: list或float，位置；单位rad或m。
        """
        return self._impl.plan_hand_pos

    def get_plan_hand_vel(self) -> list:
        """获取规划灵巧手速度。

        :return: list或float，速度；单位rad/s或m/s。
        """
        return self._impl.plan_hand_vel

    def get_plan_hand_tau(self) -> list:
        """获取规划灵巧手力矩/力。

        :return: list或float，力矩/力；单位N·m或N。
        """
        return self._impl.plan_hand_tau

    def set_hand(self, pos: list, tau: list, vel: list) -> int:
        """设置机械臂灵巧手。

        :param pos: [输入] 目标位置；关节/旋转电机单位rad，直线电机单位m。
        :param tau: [输入] 目标力矩/力；单位N·m或N。
        :param vel: [输入] 目标速度；旋转部分单位rad/s，直线部分单位m/s。
        :return: int，1表示成功，-1表示失败。
        """
        return _to_int(self._impl.set_hand(pos, tau, vel))

    # ------------------------------------------------------------------ #
    #  轨迹跟踪
    # ------------------------------------------------------------------ #
    def track_joint(self, targets: list, eeff_pos: float = -1) -> int:
        """实时跟随关节目标。

        :param targets: [输入] 目标关节位置，长度=dof；单位rad。
        :param eeff_pos: [输入] 末端执行器目标位置；单位rad或m；负值表示不随本次指令更新。
        :return: int，1表示成功，-1表示失败。
        """
        end_effector = eeff_pos if eeff_pos >= 0 else None
        return _to_int(self._impl.track_joint(targets, end_effector))

    def track_pose(self, targets: list, eeff_pos: float = -1) -> int:
        """实时跟随位姿目标。

        :param targets: [输入] 目标位姿[x,y,z,qx,qy,qz,qw]；位置单位m，姿态为归一化四元数。
        :param eeff_pos: [输入] 末端执行器目标位置；单位rad或m；负值表示不随本次指令更新。
        :return: int，1表示成功，-1表示失败。
        """
        end_effector = eeff_pos if eeff_pos >= 0 else None
        return _to_int(self._impl.track_pose(targets, end_effector))

    # ------------------------------------------------------------------ #
    #  运动指令
    # ------------------------------------------------------------------ #
    def move_joint(self, target_pos: list, desire_time: float = -1,
                   is_sync: bool = True) -> int:
        """执行关节运动。

        :param target_pos: [输入] 目标关节位置，长度=dof；单位rad。
        :param desire_time: [输入] 期望运动时间；单位s；负值表示由控制器自动规划。
        :param is_sync: [输入] 是否同步等待任务完成。
        :return: int，1表示成功，-1表示失败。
        """
        return _to_int(self._impl.move_joint(target_pos, desire_time, is_sync))

    def move_pose(self, target_pos: list, desire_time: float = -1,
                  is_sync: bool = True) -> int:
        """执行位姿运动。

        :param target_pos: [输入] 目标位姿或位姿轨迹[x,y,z,qx,qy,qz,qw]；位置单位m，姿态为归一化四元数。
        :param desire_time: [输入] 期望运动时间；单位s；负值表示由控制器自动规划。
        :param is_sync: [输入] 是否同步等待任务完成。
        :return: int，1表示成功，-1表示失败。
        """
        return _to_int(self._impl.move_pose(target_pos, desire_time, is_sync))

    def move_line_joint(self, target_pos: list, is_sync: bool = True) -> int:
        """执行关节空间直线运动。

        :param target_pos: [输入] 目标关节位置，长度=dof；单位rad。
        :param is_sync: [输入] 是否同步等待任务完成。
        :return: int，1表示成功，-1表示失败。
        """
        return _to_int(self._impl.move_line_joint(target_pos, is_sync))

    def move_line_pose(self, target_pos: list, is_sync: bool = True) -> int:
        """执行任务空间直线运动。

        :param target_pos: [输入] 目标位姿或位姿轨迹[x,y,z,qx,qy,qz,qw]；位置单位m，姿态为归一化四元数。
        :param is_sync: [输入] 是否同步等待任务完成。
        :return: int，1表示成功，-1表示失败。
        """
        return _to_int(self._impl.move_line_pose(target_pos, is_sync))

    def move_joint_traj(self, target_pos: list, eeff_pos: list = None,
                        stamps: list = None, is_sync: bool = True) -> int:
        """执行关节轨迹运动。

        :param target_pos: [输入] 目标关节位置轨迹；各轨迹点长度=dof，单位rad。
        :param eeff_pos: [输入] 末端执行器目标位置轨迹；单位rad或m；None表示不随本次指令更新。
        :param stamps: [输入] 各轨迹点时间戳；单位s；None表示自动规划。
        :param is_sync: [输入] 是否同步等待任务完成。
        :return: int，1表示成功，-1表示失败。
        """
        return _to_int(self._impl.move_joint_traj(target_pos, eeff_pos, stamps, is_sync))

    def move_pose_traj(self, target_pos: list, eeff_pos: list = None,
                       stamps: list = None, is_sync: bool = True) -> int:
        """执行位姿轨迹运动。

        :param target_pos: [输入] 目标位姿或位姿轨迹[x,y,z,qx,qy,qz,qw]；位置单位m，姿态为归一化四元数。
        :param eeff_pos: [输入] 末端执行器目标位置；单位rad或m；负值表示不随本次指令更新。
        :param stamps: [输入] 各轨迹点时间戳；单位s；None表示自动规划。
        :param is_sync: [输入] 是否同步等待任务完成。
        :return: int，1表示成功，-1表示失败。
        """
        return _to_int(self._impl.move_pose_traj(target_pos, eeff_pos, stamps, is_sync))

    def move_flow_pose(self, target_pos: list, line_theta_weight: float = 0.5,
                       accuracy: float = 0.0001, is_sync: bool = True) -> int:
        """执行连续位姿运动。

        :param target_pos: [输入] 目标位姿或位姿轨迹[x,y,z,qx,qy,qz,qw]；位置单位m，姿态为归一化四元数。
        :param line_theta_weight: [输入] 直线运动姿态插值权重。
        :param accuracy: [输入] 路径精度阈值；单位m。
        :param is_sync: [输入] 是否同步等待任务完成。
        :return: int，1表示成功，-1表示失败。
        """
        return _to_int(self._impl.move_flow_pose(
            target_pos, line_theta_weight, accuracy, False, is_sync))

    # ------------------------------------------------------------------ #
    #  示教 / 轨迹复现
    # ------------------------------------------------------------------ #
    def trajectory_teach(self, off_on: bool, name: str) -> int:
        """开始或停止机械臂示教轨迹录制。

        :param off_on: [输入] True开始录制，False停止录制。
        :param name: [输入] 示教轨迹名称。
        :return: int，1表示成功，-1表示失败。
        """
        return _to_int(self._impl.trajectory_teach(off_on, name))

    def trajectory_recorder(self, name: str, is_sync: bool = True) -> int:
        """复现机械臂已录制的示教轨迹。

        :param name: [输入] 示教轨迹名称。
        :param is_sync: [输入] 是否同步等待任务完成。
        :return: int，1表示成功，-1表示失败。
        """
        return _to_int(self._impl.trajectory_recorder(name, is_sync))

    def check_teach(self, traj_list: list) -> int:
        """获取已记录的示教轨迹列表，结果填充到 *traj_list*。"""
        result = self._impl.check_teach()
        if isinstance(traj_list, list):
            traj_list.clear()
            if result is not None:
                traj_list.extend(result)
        return 1 if result is not None else -1

    # ------------------------------------------------------------------ #
    #  运动学
    # ------------------------------------------------------------------ #
    def inverse_kine(self, tool_index: int, quat_pose: list,
                     ref_joint: list, jnt_value: list) -> int:
        """计算机械臂逆运动学。

        :param tool_index: [输入] 工具编号。
        :param quat_pose: [输入] 笛卡尔位姿或位姿数组[x,y,z,qx,qy,qz,qw]；位置单位m，姿态为归一化四元数。
        :param ref_joint: [输入] 逆解参考关节位置或其数组；单位rad。
        :param jnt_value: [输出] 关节位置或其数组；单位rad。
        :return: int，1表示成功，-1表示失败。
        """
        result = self._impl.inverse_kine(quat_pose, ref_joint, tool=tool_index)
        if not result:
            return -1
        if isinstance(jnt_value, list):
            jnt_value.clear()
            jnt_value.extend(result)
        return 1

    def forward_kine(self, tool_index: int, jnt_value: list,
                     quat_pose: list) -> int:
        """计算机械臂正运动学。

        :param tool_index: [输入] 工具编号。
        :param jnt_value: [输入] 关节位置或其数组；单位rad。
        :param quat_pose: [输出] 笛卡尔位姿或位姿数组[x,y,z,qx,qy,qz,qw]；位置单位m，姿态为归一化四元数。
        :return: int，1表示成功，-1表示失败。
        """
        result = self._impl.forward_kine(jnt_value, tool=tool_index)
        if not result:
            return -1
        if isinstance(quat_pose, list):
            quat_pose.clear()
            quat_pose.extend(result)
        return 1

    def inverse_kine_array(self, tool_index: int, quat_pose: list,
                           ref_joint: list, jnt_value: list) -> int:
        """批量计算机械臂逆运动学。

        :param tool_index: [输入] 工具编号。
        :param quat_pose: [输入] 笛卡尔位姿或位姿数组[x,y,z,qx,qy,qz,qw]；位置单位m，姿态为归一化四元数。
        :param ref_joint: [输入] 逆解参考关节位置或其数组；单位rad。
        :param jnt_value: [输出] 关节位置或其数组；单位rad。
        :return: int，1表示成功，-1表示失败。
        """
        results = []
        for pose, ref in zip(quat_pose, ref_joint):
            r = self._impl.inverse_kine(pose, ref, tool=tool_index)
            if not r:
                return -1
            results.append(r)
        if isinstance(jnt_value, list):
            jnt_value.clear()
            jnt_value.extend(results)
        return 1

    def forward_kine_array(self, tool_index: int, jnt_value: list,
                           quat_pose: list) -> int:
        """批量计算机械臂正运动学。

        :param tool_index: [输入] 工具编号。
        :param jnt_value: [输入] 关节位置或其数组；单位rad。
        :param quat_pose: [输出] 笛卡尔位姿或位姿数组[x,y,z,qx,qy,qz,qw]；位置单位m，姿态为归一化四元数。
        :return: int，1表示成功，-1表示失败。
        """
        results = []
        for joints in jnt_value:
            r = self._impl.forward_kine(joints, tool=tool_index)
            if not r:
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
        """注册关节状态回调。

        :param cbk: [输入] 回调函数；参数单位由对应状态量定义。
        :return: 无返回值。
        """
        self._register_keyed_cbk("get_joint", cbk, "_joint_cbks",
                                 lambda c: self._impl.on_update(
                                     lambda msg: self._dispatch_joint(c, msg)))

    def release_joint_cbk(self) -> None:
        """注销关节状态回调。

        :return: 无返回值。
        """
        self._release_keyed_cbk("get_joint", "_joint_cbks")

    def register_pose_cbk(self, cbk: Callable) -> None:
        """注册位姿状态回调。

        :param cbk: [输入] 回调函数；参数单位由对应状态量定义。
        :return: 无返回值。
        """
        self._register_keyed_cbk("get_pose", cbk, "_pose_cbks",
                                 lambda c: self._impl.on_update(
                                     lambda msg: self._dispatch_pose(c, msg)))

    def release_pose_cbk(self) -> None:
        """注销位姿状态回调。

        :return: 无返回值。
        """
        self._release_keyed_cbk("get_pose", "_pose_cbks")

    def register_plan_joint_cbk(self, cbk: Callable) -> None:
        """注册规划关节状态回调。

        :param cbk: [输入] 回调函数；参数单位由对应状态量定义。
        :return: 无返回值。
        """
        self._register_keyed_cbk("get_cmd_joint", cbk, "_plan_joint_cbks",
                                 lambda c: self._impl.on_update(
                                     lambda msg: self._dispatch_plan_joint(c, msg)))

    def release_plan_joint_cbk(self) -> None:
        """注销规划关节状态回调。

        :return: 无返回值。
        """
        self._release_keyed_cbk("get_cmd_joint", "_plan_joint_cbks")

    def register_plan_pose_cbk(self, cbk: Callable) -> None:
        """注册规划位姿回调。

        :param cbk: [输入] 回调函数；参数单位由对应状态量定义。
        :return: 无返回值。
        """
        self._register_keyed_cbk("get_cmd_pose", cbk, "_plan_pose_cbks",
                                 lambda c: self._impl.on_update(
                                     lambda msg: self._dispatch_plan_pose(c, msg)))

    def release_plan_pose_cbk(self) -> None:
        """注销规划位姿回调。

        :return: 无返回值。
        """
        self._release_keyed_cbk("get_cmd_pose", "_plan_pose_cbks")

    def register_external_force_cbk(self, cbk: Callable) -> None:
        """注册外力回调。

        :param cbk: [输入] 回调函数；参数单位由对应状态量定义。
        :return: 无返回值。
        """
        self._register_keyed_cbk("get_external_force", cbk, "_ext_force_cbks",
                                 lambda c: self._impl.on_update(
                                     lambda msg: self._dispatch_ext_force(c, msg)))

    def release_external_force_cbk(self) -> None:
        """注销外力回调。

        :return: 无返回值。
        """
        self._release_keyed_cbk("get_external_force", "_ext_force_cbks")

    def register_error_cbk(self, key: str, cbk: Callable) -> None:
        """注册错误回调。

        :param key: [输入] 回调唯一标识。
        :param cbk: [输入] 回调函数；参数单位由对应状态量定义。
        :return: 无返回值。
        """
        self._register_keyed_cbk(key, cbk, "_error_cbks",
                                 lambda c: self._impl.on_error(
                                     lambda code, msg: self._dispatch_error(c, code, msg)))

    def release_error_cbk(self, key: str) -> None:
        """注销错误回调。

        :param key: [输入] 回调唯一标识。
        :return: 无返回值。
        """
        self._release_keyed_cbk(key, "_error_cbks")

    def register_completion_cbk(self, key: str, cbk: Callable) -> None:
        """注册任务完成回调。

        :param key: [输入] 回调唯一标识。
        :param cbk: [输入] 回调函数；参数单位由对应状态量定义。
        :return: 无返回值。
        """
        self._register_keyed_cbk(key, cbk, "_completion_cbks",
                                 lambda c: self._impl.on_task_finish(
                                     lambda task_key: self._dispatch_completion(c, task_key)))

    def release_completion_cbk(self, key: str) -> None:
        """注销任务完成回调。

        :param key: [输入] 回调唯一标识。
        :return: 无返回值。
        """
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
        """开启或关闭 Low Mode。

        :param flag: [输入] ``True`` 进入 Low Mode，``False`` 退出。
        :return: 1 表示成功，-1 表示失败。
        """
        return _to_int(self._impl.set_low_mode(flag))

    def low_set_safety_params(self, updates: dict, current: dict) -> int:
        """设置 Low Mode 安全参数，支持部分更新。

        仅允许在 Low Mode 下调用，``updates`` 至少包含以下一个字段：

        :param updates: [输入] 待更新参数字典。支持
            ``low_watchdog_timeout``（ms，有限正数）、
            ``low_pos_gap_limit``（rad，长度=dof）、
            ``low_vel_gap_limit``（rad/s，长度=dof）、
            ``low_tau_limit``（Nm，长度=dof）。三个数组的元素必须为有限非负数。
        :param current: [输出] 当前完整安全参数字典，字段含义及单位与 ``updates`` 相同。
        :return: 1 表示成功，-1 表示参数无效或请求失败。
        """
        if not isinstance(updates, dict) or not isinstance(current, dict):
            return -1
        ok, params = self._impl.low_set_safety_params(updates)
        if not ok:
            return -1
        current.clear()
        current.update(params)
        return 1

    def low_pv_command(self, pos: list, vel: list, data: dict) -> int:
        """发送底层位置/速度（PV）控制指令。

        :param pos: [输入] 目标关节位置，长度=dof；单位rad。
        :param vel: [输入] 目标关节速度，长度=dof；单位rad/s。
        :param data: [输出] 本次指令返回的底层硬件状态字典，各状态量沿用对应物理单位。
        :return: 1 表示成功，-1 表示失败。
        """
        ok, low = self._impl.low_pv_command(pos, vel)
        if isinstance(data, dict):
            data.clear()
            data.update(low if isinstance(low, dict) else {})
        return _to_int(ok)

    def low_mit_command(self, pos: list, vel: list, tau: list,
                        kp: list, kd: list, data: dict) -> int:
        """发送底层 MIT 位置、速度和力矩综合控制指令。

        :param pos: [输入] 目标关节位置，长度=dof；单位rad。
        :param vel: [输入] 目标关节速度，长度=dof；单位rad/s。
        :param tau: [输入] 前馈关节力矩，长度=dof；单位Nm。
        :param kp: [输入] 关节位置刚度系数，长度=dof；单位Nm/rad。
        :param kd: [输入] 关节速度阻尼系数，长度=dof；单位Nm·s/rad。
        :param data: [输出] 本次指令返回的底层硬件状态字典，各状态量沿用对应物理单位。
        :return: 1 表示成功，-1 表示失败。
        """
        ok, low = self._impl.low_mit_command(pos, vel, tau, kp, kd)
        if isinstance(data, dict):
            data.clear()
            data.update(low if isinstance(low, dict) else {})
        return _to_int(ok)

    def low_pf_command(self, pos: list, vel: list, tau: list, data: dict) -> int:
        """发送底层位置/力矩（PF）混合控制指令。

        :param pos: [输入] 目标关节位置，长度=dof；单位rad。
        :param vel: [输入] 目标关节速度，长度=dof；单位rad/s。
        :param tau: [输入] 前馈关节力矩，长度=dof；单位Nm。
        :param data: [输出] 本次指令返回的底层硬件状态字典，各状态量沿用对应物理单位。
        :return: 1 表示成功，-1 表示失败。
        """
        ok, low = self._impl.low_pf_command(pos, vel, tau)
        if isinstance(data, dict):
            data.clear()
            data.update(low if isinstance(low, dict) else {})
        return _to_int(ok)

    def low_current_command(self, tau: list, data: dict) -> int:
        """发送底层 Current 关节力矩控制指令。

        :param tau: [输入] 目标关节力矩，长度=dof；单位Nm。
        :param data: [输出] 本次指令返回的底层硬件状态字典，各状态量沿用对应物理单位。
        :return: 1 表示成功，-1 表示失败。
        """
        ok, low = self._impl.low_current_command(tau)
        if isinstance(data, dict):
            data.clear()
            data.update(low if isinstance(low, dict) else {})
        return _to_int(ok)

    def low_refresh(self, data: dict) -> int:
        """刷新一帧底层硬件状态，不下发控制目标。

        :param data: [输出] 最新底层硬件状态字典，各状态量沿用对应物理单位。
        :return: 1 表示成功，-1 表示失败。
        """
        ok, low = self._impl.low_refresh()
        if isinstance(data, dict):
            data.clear()
            data.update(low if isinstance(low, dict) else {})
        return _to_int(ok)

    def low_set_end_effector_ctr(self, pos: list, vel: list, tau: list,
                                 data: dict) -> int:
        """发送底层末端执行器电机控制指令。

        :param pos: [输入] 末端执行器电机目标位置；旋转电机单位rad，直线电机单位m。
        :param vel: [输入] 末端执行器电机目标速度；单位rad/s或m/s。
        :param tau: [输入] 末端执行器电机目标力矩/力；单位N·m或N。
        :param data: [输出] 本次指令返回的底层硬件状态字典，各状态量沿用对应物理单位。
        :return: 1 表示成功，-1 表示失败。
        """
        ok, low = self._impl.low_set_end_effector_ctr(pos, vel, tau)
        if isinstance(data, dict):
            data.clear()
            data.update(low if isinstance(low, dict) else {})
        return _to_int(ok)

    def low_set_robot_mode(self, mode: int) -> int:
        """设置机器人底层运行模式。

        :param mode: [输入] 模式枚举值：0-IDLE、1-PV、2-MIT、3-CURRENT、4-PF。
        :return: 1 表示成功，-1 表示失败。
        """
        return _to_int(self._impl.low_set_robot_mode(mode))

    def low_set_end_effector_mode(self, mode: int) -> int:
        """设置末端执行器底层运行模式。

        :param mode: [输入] 末端执行器模式枚举值，具体取值由末端执行器定义。
        :return: 1 表示成功，-1 表示失败。
        """
        return _to_int(self._impl.low_set_end_effector_mode(mode))

    def low_set_servo_enable(self, status: bool) -> int:
        """设置底层伺服使能状态。

        :param status: [输入] ``True`` 上使能，``False`` 下使能。
        :return: 1 表示成功，-1 表示失败。
        """
        return _to_int(self._impl.low_set_servo_enable(status))

    def low_reset(self, cnt: int = 5) -> int:
        """复位底层错误。

        :param cnt: [输入] 最大复位尝试次数，默认5；单位为次。
        :return: 1 表示成功，-1 表示失败。
        """
        return _to_int(self._impl.low_reset(cnt))

    def low_get_servo_status(self, status: dict) -> int:
        """获取底层伺服状态。

        :param status: [输出] 伺服状态字典，包含控制参数、连接/使能状态、温度（°C）、
            母线电压（V）及错误信息等字段。
        :return: 1 表示成功，-1 表示失败。
        """
        result = self._impl.low_get_servo_status()
        if isinstance(status, dict):
            status.clear()
            status.update(result if isinstance(result, dict) else {})
        return 1 if result else -1

    def low_get_inverse_kine(self, pose: list, refer_pos: list,
                             jnt_value: list, tool: int = -1) -> int:
        """计算目标位姿对应的关节位置。

        :param pose: [输入] 目标位姿 ``[x, y, z, qx, qy, qz, qw]``；位置单位m，姿态为归一化四元数。
        :param refer_pos: [输入] 多解选优参考关节位置，长度=dof；单位rad。
        :param jnt_value: [输出] 求解得到的关节位置，长度=dof；单位rad。
        :param tool: [输入] 工具编号，-1表示当前工具。
        :return: 1 表示成功，-1 表示失败。
        """
        ok, _, result = self._impl.low_get_inverse_kine(pose, refer_pos, tool)
        if isinstance(jnt_value, list):
            jnt_value.clear()
            jnt_value.extend(result if result else [])
        return _to_int(ok)

    def low_get_forward_kine(self, joint_pos: list, quat_pose: list,
                             tool: int = -1) -> int:
        """计算关节位置对应的末端位姿。

        :param joint_pos: [输入] 关节位置，长度=dof；单位rad。
        :param quat_pose: [输出] 位姿 ``[x, y, z, qx, qy, qz, qw]``；位置单位m，姿态为归一化四元数。
        :param tool: [输入] 工具编号，-1表示当前工具。
        :return: 1 表示成功，-1 表示失败。
        """
        ok, _, result = self._impl.low_get_forward_kine(joint_pos, tool)
        if isinstance(quat_pose, list):
            quat_pose.clear()
            quat_pose.extend(result if result else [])
        return _to_int(ok)

    def low_get_dynamics(self, joint_pos: list, joint_vel: list,
                         joint_acc: list, tool: list, m_force: list,
                         c_force: list, g_force: list) -> int:
        """计算关节动力学的惯性、科里奥利/离心力及重力分量。

        :param joint_pos: [输入] 关节位置，长度=dof；单位rad。
        :param joint_vel: [输入] 关节速度，长度=dof；单位rad/s。
        :param joint_acc: [输入] 关节加速度，长度=dof；单位rad/s²。
        :param tool: [输出] 使用的工具编号列表（单元素）。
        :param m_force: [输出] 惯性项关节力矩，长度=dof；单位Nm。
        :param c_force: [输出] 科里奥利及离心项关节力矩，长度=dof；单位Nm。
        :param g_force: [输出] 重力项关节力矩，长度=dof；单位Nm。
        :return: 1 表示成功，-1 表示失败。
        """
        ok, result_tool, result_m, result_c, result_g = self._impl.low_get_dynamics(
            joint_pos, joint_vel, joint_acc)
        if not ok:
            return -1
        if isinstance(tool, list):
            tool.clear()
            tool.append(result_tool)
        for result, container in [(result_m, m_force), (result_c, c_force),
                                  (result_g, g_force)]:
            if isinstance(container, list):
                container.clear()
                container.extend(result)
        return 1

    def low_get_jacobian(self, joint_pos: list, tool: list, mat: list) -> int:
        """计算雅可比矩阵。

        :param joint_pos: [输入] 关节位置，长度=dof；单位rad。
        :param tool: [输出] 使用的工具编号列表（单元素）。
        :param mat: [输出] 二维雅可比矩阵，形状为rows×cols；各元素单位取决于其映射的
            末端速度分量和关节速度分量。
        :return: 1 表示成功，-1 表示失败。
        """
        ok, result_tool, matrix = self._impl.low_get_jacobian(joint_pos)
        if not ok:
            return -1
        if isinstance(tool, list):
            tool.clear()
            tool.append(result_tool)
        if isinstance(mat, list):
            mat.clear()
            mat.extend(matrix)
        return 1

    def low_get_nullspace(self, joint_pos: list, tolerance: float,
                          tool: list, mat: list) -> int:
        """计算关节零空间矩阵。

        :param joint_pos: [输入] 关节位置，长度=dof；单位rad。
        :param tolerance: [输入] 奇异值判定公差。
        :param tool: [输出] 使用的工具编号列表（单元素）。
        :param mat: [输出] 二维零空间矩阵，形状为rows×cols。
        :return: 1 表示成功，-1 表示失败。
        """
        ok, result_tool, matrix = self._impl.low_get_nullspace(joint_pos, tolerance)
        if not ok:
            return -1
        if isinstance(tool, list):
            tool.clear()
            tool.append(result_tool)
        if isinstance(mat, list):
            mat.clear()
            mat.extend(matrix)
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
        """初始化机械臂控制对象。

        :param server_ip: [输入] 控制器IPv4地址。
        :param port: [输入] 控制器TCP端口。
        :param timeout: [输入] 连接超时时间；单位s。
        :param left_index: [输入] 左臂编号。
        :param right_index: [输入] 右臂编号。
        :param _validate_arm: [输入] 是否校验设备型号。
        :return: 无返回值。
        """
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
        """连接机械臂控制器并校验设备类型。

        :param server_ip: [输入] 控制器IPv4地址。
        :param port: [输入] 控制器TCP端口。
        :param timeout: [输入] 连接超时时间；单位s。
        :return: int，1表示成功，-1表示失败。
        """
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
        """断开机械臂控制器连接。

        :return: int，1表示成功，-1表示失败。
        """
        rl = self._left.disconnect()
        time.sleep(0.1)  # 避免两个实例同时连接时出现端口冲突
        rr = self._right.disconnect()
        return 1 if rl == 1 and rr == 1 else -1

    def is_connected(self) -> bool:
        """检查机械臂控制器是否已连接。

        :return: bool，True表示已连接，False表示未连接。
        """
        return self._left.is_connected() and self._right.is_connected()

    # ------------------------------------------------------------------ #
    #  臂型校验（对齐 C++ CArmDualBot::_is_specified_arm）
    # ------------------------------------------------------------------ #
    def _is_specified_arm(self) -> bool:
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
        """将机械臂切换到就绪状态。

        :return: int，1表示成功，-1表示失败。
        """
        rl = self._left.set_ready()
        rr = self._right.set_ready()
        return 1 if rl == 1 and rr == 1 else -1

    def set_servo_enable(self, enable: bool) -> int:
        """设置机械臂伺服使能状态。

        :param enable: [输入] 是否使能；True为使能，False为失能。
        :return: int，1表示成功，-1表示失败。
        """
        rl = self._left.set_servo_enable(enable)
        rr = self._right.set_servo_enable(enable)
        return 1 if rl == 1 and rr == 1 else -1

    def set_left_servo_enable(self, enable: bool) -> int:
        """设置左臂伺服使能状态。

        :param enable: [输入] 是否使能；True为使能，False为失能。
        :return: int，1表示成功，-1表示失败。
        """
        return self._left.set_servo_enable(enable)

    def set_right_servo_enable(self, enable: bool) -> int:
        """设置右臂伺服使能状态。

        :param enable: [输入] 是否使能；True为使能，False为失能。
        :return: int，1表示成功，-1表示失败。
        """
        return self._right.set_servo_enable(enable)

    def set_control_mode(self, mode: int) -> int:
        """设置机械臂控制模式。

        :param mode: [输入] 控制或透传模式枚举值。
        :return: int，1表示成功，-1表示失败。
        """
        rl = self._left.set_control_mode(mode)
        rr = self._right.set_control_mode(mode)
        return 1 if rl == 1 and rr == 1 else -1

    def set_left_control_mode(self, mode: int) -> int:
        """设置左臂控制模式。

        :param mode: [输入] 控制或透传模式枚举值。
        :return: int，1表示成功，-1表示失败。
        """
        return self._left.set_control_mode(mode)

    def set_right_control_mode(self, mode: int) -> int:
        """设置右臂控制模式。

        :param mode: [输入] 控制或透传模式枚举值。
        :return: int，1表示成功，-1表示失败。
        """
        return self._right.set_control_mode(mode)

    # def set_passthrough_data(self, mode: int, can_id: int, data: list) -> int:
    #     rl = self._left.set_passthrough_data(mode, can_id, data)
    #     return 1 if rl == 1 else -1

    def set_left_ecat_passthrough_data(self, mode: int, frame: dict,
                                       timeout_ms: int = 100) -> int:
        """通过左臂EtherCAT CAN总线同步透传完整帧。

        :param mode: [输入] 透传模式枚举值。
        :param frame: [输入/输出] CAN/CAN FD帧字典；数据字段单位byte，成功时写入响应帧。
        :param timeout_ms: [输入] 响应超时时间；单位ms。
        :return: int，1表示成功，负值表示失败。
        """
        return self._left.set_ecat_passthrough_data(mode, frame, timeout_ms)

    def set_right_ecat_passthrough_data(self, mode: int, frame: dict,
                                        timeout_ms: int = 100) -> int:
        """通过右臂EtherCAT CAN总线同步透传完整帧。

        :param mode: [输入] 透传模式枚举值。
        :param frame: [输入/输出] CAN/CAN FD帧字典；数据字段单位byte，成功时写入响应帧。
        :param timeout_ms: [输入] 响应超时时间；单位ms。
        :return: int，1表示成功，负值表示失败。
        """
        return self._right.set_ecat_passthrough_data(mode, frame, timeout_ms)

    def get_version(self) -> str:
        """获取SDK与控制器版本信息。

        :return: str，版本信息。
        """
        return self._left.get_version() + self._right.get_version()

    def emergency_stop(self) -> int:
        """触发机械臂急停。

        :return: int，1表示成功，-1表示失败。
        """
        rl = self._left.emergency_stop()
        rr = self._right.emergency_stop()
        return 1 if rl == 1 and rr == 1 else -1

    def task_stop(self) -> int:
        """立即停止机械臂当前任务。

        :return: int，1表示成功，-1表示失败。
        """
        rl = self._left.task_stop()
        rr = self._right.task_stop()
        return 1 if rl == 1 and rr == 1 else -1

    def set_debug(self, flag: bool) -> int:
        """设置机械臂调试模式。

        :param flag: [输入] 功能开关；True为开启，False为关闭。
        :return: int，1表示成功，-1表示失败。
        """
        return self._left.set_debug(flag)

    def set_speed_level(self, level: float, response_level: int = 20) -> int:
        """设置机械臂速度等级。

        :param level: [输入] 全局速度百分比，范围由控制器定义；单位%。
        :param response_level: [输入] 速度响应等级。
        :return: int，1表示成功，-1表示失败。
        """
        rl = self._left.set_speed_level(level, response_level)
        rr = self._right.set_speed_level(level, response_level)
        return 1 if rl == 1 and rr == 1 else -1

    def set_collision_config(self, enable_flag: bool = True,
                             sensitivity_level: int = 0) -> int:
        """设置机械臂碰撞检测配置。

        :param enable_flag: [输入] 是否启用碰撞检测。
        :param sensitivity_level: [输入] 碰撞灵敏度等级。
        :return: int，1表示成功，-1表示失败。
        """
        rl = self._left.set_collision_config(enable_flag, sensitivity_level)
        rr = self._right.set_collision_config(enable_flag, sensitivity_level)
        return 1 if rl == 1 and rr == 1 else -1

    def set_left_collision_config(self, enable_flag: bool = True,
                                  sensitivity_level: int = 0) -> int:
        """设置左臂碰撞检测配置。

        :param enable_flag: [输入] 是否启用碰撞检测。
        :param sensitivity_level: [输入] 碰撞灵敏度等级。
        :return: int，1表示成功，-1表示失败。
        """
        return self._left.set_collision_config(enable_flag, sensitivity_level)

    def set_right_collision_config(self, enable_flag: bool = True,
                                   sensitivity_level: int = 0) -> int:
        """设置右臂碰撞检测配置。

        :param enable_flag: [输入] 是否启用碰撞检测。
        :param sensitivity_level: [输入] 碰撞灵敏度等级。
        :return: int，1表示成功，-1表示失败。
        """
        return self._right.set_collision_config(enable_flag, sensitivity_level)

    def set_low_mode(self, flag: bool) -> int:
        """单路下发 Low Mode 开关指令。

        :param flag: [输入] ``True`` 进入 Low Mode，``False`` 退出。
        :return: 1 表示成功，-1 表示失败。
        """
        return self._left.set_low_mode(flag)

    def register_error_cbk(self, key: str, cbk: Callable) -> None:
        """注册错误回调。

        :param key: [输入] 回调唯一标识。
        :param cbk: [输入] 回调函数；参数单位由对应状态量定义。
        :return: 无返回值。
        """
        self._left.register_error_cbk(key, cbk)
        self._right.register_error_cbk(key, cbk)

    def release_error_cbk(self, key: str) -> None:
        """注销错误回调。

        :param key: [输入] 回调唯一标识。
        :return: 无返回值。
        """
        self._left.release_error_cbk(key)
        self._right.release_error_cbk(key)

    def register_completion_cbk(self, key: str, cbk: Callable) -> None:
        """注册任务完成回调。

        :param key: [输入] 回调唯一标识。
        :param cbk: [输入] 回调函数；参数单位由对应状态量定义。
        :return: 无返回值。
        """
        self._left.register_completion_cbk(key, cbk)
        self._right.register_completion_cbk(key, cbk)

    def release_completion_cbk(self, key: str) -> None:
        """注销任务完成回调。

        :param key: [输入] 回调唯一标识。
        :return: 无返回值。
        """
        self._left.release_completion_cbk(key)
        self._right.release_completion_cbk(key)

    def check_teach(self, left_traj_list: list, right_traj_list: list) -> int:
        """检查机械臂示教轨迹是否存在。

        :param left_traj_list: [输出] 左臂示教轨迹名称列表。
        :param right_traj_list: [输出] 右臂示教轨迹名称列表。
        :return: int，1表示成功，-1表示失败。
        """
        rl = self._left.check_teach(left_traj_list)
        rr = self._right.check_teach(right_traj_list)
        return 1 if rl == 1 and rr == 1 else -1

    # ================================================================== #
    #  左臂方法（命名规则与 C++ 一致：get_left_* / set_left_* / track_left_* 等）
    # ================================================================== #
    def get_left_config(self) -> dict:
        """获取左臂配置。

        :return: dict，配置或状态字典；各字段沿用对应物理单位。
        """
        return self._left.get_config()

    def get_left_eeff_config(self) -> dict:
        """获取左臂末端执行器配置。

        :return: dict，配置或状态字典；各字段沿用对应物理单位。
        """
        return self._left.get_eeff_config()

    def get_left_status(self) -> dict:
        """获取左臂状态。

        :return: dict，配置或状态字典；各字段沿用对应物理单位。
        """
        return self._left.get_status()

    def get_left_joint_pos(self) -> list:
        """获取左臂实际关节位置。

        :return: list或float，位置；单位rad。
        """
        return self._left.get_joint_pos()

    def get_left_joint_vel(self) -> list:
        """获取左臂实际关节速度。

        :return: list或float，速度；单位rad/s。
        """
        return self._left.get_joint_vel()

    def get_left_joint_tau(self) -> list:
        """获取左臂实际关节力矩。

        :return: list或float，力矩；单位Nm。
        """
        return self._left.get_joint_tau()

    def get_left_plan_joint_pos(self) -> list:
        """获取左臂规划关节位置。

        :return: list或float，位置；单位rad。
        """
        return self._left.get_plan_joint_pos()

    def get_left_plan_joint_vel(self) -> list:
        """获取左臂规划关节速度。

        :return: list或float，速度；单位rad/s。
        """
        return self._left.get_plan_joint_vel()

    def get_left_plan_joint_tau(self) -> list:
        """获取左臂规划关节力矩。

        :return: list或float，力矩；单位Nm。
        """
        return self._left.get_plan_joint_tau()

    def get_left_plan_cart_pose(self) -> list:
        """获取左臂规划末端位姿。

        :return: list，[x,y,z,qx,qy,qz,qw]；位置单位m，姿态为归一化四元数。
        """
        return self._left.get_plan_cart_pose()

    def get_left_cart_pose(self) -> list:
        """获取左臂实际末端位姿。

        :return: list，[x,y,z,qx,qy,qz,qw]；位置单位m，姿态为归一化四元数。
        """
        return self._left.get_cart_pose()

    def get_left_joint_external_tau(self) -> list:
        """获取左臂关节外力矩。

        :return: list或float，力矩；单位Nm。
        """
        return self._left.get_joint_external_tau()

    def get_left_cart_external_force(self) -> list:
        """获取左臂末端外力/力矩。

        :return: list，[Fx,Fy,Fz,Tx,Ty,Tz]；单位N和N·m。
        """
        return self._left.get_cart_external_force()

    def register_left_joint_cbk(self, cbk: Callable) -> None:
        """注册左臂关节状态回调。

        :param cbk: [输入] 回调函数；参数单位由对应状态量定义。
        :return: 无返回值。
        """
        self._left.register_joint_cbk(cbk)

    def release_left_joint_cbk(self) -> None:
        """注销左臂关节状态回调。

        :return: 无返回值。
        """
        self._left.release_joint_cbk()

    def register_left_pose_cbk(self, cbk: Callable) -> None:
        """注册左臂位姿状态回调。

        :param cbk: [输入] 回调函数；参数单位由对应状态量定义。
        :return: 无返回值。
        """
        self._left.register_pose_cbk(cbk)

    def release_left_pose_cbk(self) -> None:
        """注销左臂位姿状态回调。

        :return: 无返回值。
        """
        self._left.release_pose_cbk()

    def register_left_plan_joint_cbk(self, cbk: Callable) -> None:
        """注册左臂规划关节状态回调。

        :param cbk: [输入] 回调函数；参数单位由对应状态量定义。
        :return: 无返回值。
        """
        self._left.register_plan_joint_cbk(cbk)

    def release_left_plan_joint_cbk(self) -> None:
        """注销左臂规划关节状态回调。

        :return: 无返回值。
        """
        self._left.release_plan_joint_cbk()

    def register_left_plan_pose_cbk(self, cbk: Callable) -> None:
        """注册左臂规划位姿回调。

        :param cbk: [输入] 回调函数；参数单位由对应状态量定义。
        :return: 无返回值。
        """
        self._left.register_plan_pose_cbk(cbk)

    def release_left_plan_pose_cbk(self) -> None:
        """注销左臂规划位姿回调。

        :return: 无返回值。
        """
        self._left.release_plan_pose_cbk()

    def register_left_external_force_cbk(self, cbk: Callable) -> None:
        """注册左臂外力回调。

        :param cbk: [输入] 回调函数；参数单位由对应状态量定义。
        :return: 无返回值。
        """
        self._left.register_external_force_cbk(cbk)

    def release_left_external_force_cbk(self) -> None:
        """注销左臂外力回调。

        :return: 无返回值。
        """
        self._left.release_external_force_cbk()

    # ------------------------------------------------------------------ #
    #  左臂末端执行器（通用 eeff）
    # ------------------------------------------------------------------ #
    def get_left_eeff_state(self) -> int:
        """获取左臂末端执行器状态。

        :return: int，枚举值、编号或自由度。
        """
        return self._left.get_eeff_state()

    def get_left_eeff_pos(self) -> list:
        """获取左臂末端执行器位置。

        :return: list或float，位置；单位rad或m。
        """
        return self._left.get_eeff_pos()

    def get_left_eeff_vel(self) -> list:
        """获取左臂末端执行器速度。

        :return: list或float，速度；单位rad/s或m/s。
        """
        return self._left.get_eeff_vel()

    def get_left_eeff_tau(self) -> list:
        """获取左臂末端执行器力矩/力。

        :return: list或float，力矩/力；单位N·m或N。
        """
        return self._left.get_eeff_tau()

    def get_left_eeff_motor_pos(self) -> list:
        """获取左臂末端执行器电机位置。

        :return: list或float，位置；单位rad或m。
        """
        return self._left.get_eeff_motor_pos()

    def get_left_eeff_motor_vel(self) -> list:
        """获取左臂末端执行器电机速度。

        :return: list或float，速度；单位rad/s或m/s。
        """
        return self._left.get_eeff_motor_vel()

    def get_left_eeff_motor_tau(self) -> list:
        """获取左臂末端执行器电机力矩/力。

        :return: list或float，力矩/力；单位N·m或N。
        """
        return self._left.get_eeff_motor_tau()

    def get_left_plan_eeff_pos(self) -> list:
        """获取左臂规划末端执行器位置。

        :return: list或float，位置；单位rad或m。
        """
        return self._left.get_plan_eeff_pos()

    def get_left_plan_eeff_vel(self) -> list:
        """获取左臂规划末端执行器速度。

        :return: list或float，速度；单位rad/s或m/s。
        """
        return self._left.get_plan_eeff_vel()

    def get_left_plan_eeff_tau(self) -> list:
        """获取左臂规划末端执行器力矩/力。

        :return: list或float，力矩/力；单位N·m或N。
        """
        return self._left.get_plan_eeff_tau()

    def get_left_plan_eeff_motor_pos(self) -> list:
        """获取左臂规划末端执行器电机位置。

        :return: list或float，位置；单位rad或m。
        """
        return self._left.get_plan_eeff_motor_pos()

    def get_left_plan_eeff_motor_vel(self) -> list:
        """获取左臂规划末端执行器电机速度。

        :return: list或float，速度；单位rad/s或m/s。
        """
        return self._left.get_plan_eeff_motor_vel()

    def get_left_plan_eeff_motor_tau(self) -> list:
        """获取左臂规划末端执行器电机力矩/力。

        :return: list或float，力矩/力；单位N·m或N。
        """
        return self._left.get_plan_eeff_motor_tau()

    def get_left_eeff_type(self) -> str:
        """获取左臂末端执行器类型。

        :return: str，类型名称。
        """
        return self._left.get_eeff_type()

    def get_left_eeff_dof(self) -> int:
        """获取左臂末端执行器自由度。

        :return: int，枚举值、编号或自由度。
        """
        return self._left.get_eeff_dof()

    def get_left_eeff_connect(self) -> bool:
        """获取左臂末端执行器连接状态。

        :return: bool，True表示已连接，False表示未连接。
        """
        return self._left.get_eeff_connect()

    def set_left_eeff(self, pos: list, vel: list, tau: list,
                      control_motor: bool = False) -> int:
        """设置左臂末端执行器。

        :param pos: [输入] 目标位置；关节/旋转电机单位rad，直线电机单位m。
        :param vel: [输入] 目标速度；旋转部分单位rad/s，直线部分单位m/s。
        :param tau: [输入] 目标力矩/力；单位N·m或N。
        :param control_motor: [输入] 是否直接使用电机坐标控制末端执行器。
        :return: int，1表示成功，-1表示失败。
        """
        return self._left.set_eeff(pos, vel, tau, control_motor)

    # ------------------------------------------------------------------ #
    #  deprecated: 左臂旧夹爪/灵巧手接口
    # ------------------------------------------------------------------ #
    def get_left_gripper_state(self) -> int:
        """获取左臂夹爪状态。

        :return: int，枚举值、编号或自由度。
        """
        return self._left.get_gripper_state()

    def get_left_gripper_pos(self) -> float:
        """获取左臂夹爪位置。

        :return: list或float，位置；单位rad或m。
        """
        return self._left.get_gripper_pos()

    def get_left_gripper_vel(self) -> float:
        """获取左臂夹爪速度。

        :return: list或float，速度；单位rad/s或m/s。
        """
        return self._left.get_gripper_vel()

    def get_left_gripper_tau(self) -> float:
        """获取左臂夹爪力矩/力。

        :return: list或float，力矩/力；单位N·m或N。
        """
        return self._left.get_gripper_tau()

    def get_left_plan_gripper_pos(self) -> float:
        """获取左臂规划夹爪位置。

        :return: list或float，位置；单位rad或m。
        """
        return self._left.get_plan_gripper_pos()

    def get_left_plan_gripper_tau(self) -> float:
        """获取左臂规划夹爪力矩/力。

        :return: list或float，力矩/力；单位N·m或N。
        """
        return self._left.get_plan_gripper_tau()

    def get_left_hand_state(self) -> int:
        """获取左臂灵巧手状态。

        :return: int，枚举值、编号或自由度。
        """
        return self._left.get_hand_state()

    def get_left_hand_pos(self) -> list:
        """获取左臂灵巧手位置。

        :return: list或float，位置；单位rad或m。
        """
        return self._left.get_hand_pos()

    def get_left_hand_vel(self) -> list:
        """获取左臂灵巧手速度。

        :return: list或float，速度；单位rad/s或m/s。
        """
        return self._left.get_hand_vel()

    def get_left_hand_tau(self) -> list:
        """获取左臂灵巧手力矩/力。

        :return: list或float，力矩/力；单位N·m或N。
        """
        return self._left.get_hand_tau()

    def get_left_plan_hand_pos(self) -> list:
        """获取左臂规划灵巧手位置。

        :return: list或float，位置；单位rad或m。
        """
        return self._left.get_plan_hand_pos()

    def get_left_plan_hand_vel(self) -> list:
        """获取左臂规划灵巧手速度。

        :return: list或float，速度；单位rad/s或m/s。
        """
        return self._left.get_plan_hand_vel()

    def get_left_plan_hand_tau(self) -> list:
        """获取左臂规划灵巧手力矩/力。

        :return: list或float，力矩/力；单位N·m或N。
        """
        return self._left.get_plan_hand_tau()

    def track_left_joint(self, targets: list, eeff_pos: float = -1) -> int:
        """实时跟随左臂关节目标。

        :param targets: [输入] 目标关节位置，长度=左臂dof；单位rad。
        :param eeff_pos: [输入] 末端执行器目标位置；单位rad或m；负值表示不随本次指令更新。
        :return: int，1表示成功，-1表示失败。
        """
        return self._left.track_joint(targets, eeff_pos)

    def track_left_pose(self, targets: list, eeff_pos: float = -1) -> int:
        """实时跟随左臂位姿目标。

        :param targets: [输入] 目标位姿[x,y,z,qx,qy,qz,qw]；位置单位m，姿态为归一化四元数。
        :param eeff_pos: [输入] 末端执行器目标位置；单位rad或m；负值表示不随本次指令更新。
        :return: int，1表示成功，-1表示失败。
        """
        return self._left.track_pose(targets, eeff_pos)

    def move_left_joint(self, target_pos: list, desire_time: float = -1,
                        is_sync: bool = True) -> int:
        """执行左臂关节运动。

        :param target_pos: [输入] 目标关节位置，长度=左臂dof；单位rad。
        :param desire_time: [输入] 期望运动时间；单位s；负值表示由控制器自动规划。
        :param is_sync: [输入] 是否同步等待任务完成。
        :return: int，1表示成功，-1表示失败。
        """
        return self._left.move_joint(target_pos, desire_time, is_sync)

    def move_left_pose(self, target_pos: list, desire_time: float = -1,
                       is_sync: bool = True) -> int:
        """执行左臂位姿运动。

        :param target_pos: [输入] 目标位姿或位姿轨迹[x,y,z,qx,qy,qz,qw]；位置单位m，姿态为归一化四元数。
        :param desire_time: [输入] 期望运动时间；单位s；负值表示由控制器自动规划。
        :param is_sync: [输入] 是否同步等待任务完成。
        :return: int，1表示成功，-1表示失败。
        """
        return self._left.move_pose(target_pos, desire_time, is_sync)

    def move_left_joint_traj(self, target_pos: list, eeff_pos: list = None,
                             stamps: list = None, is_sync: bool = True) -> int:
        """执行左臂关节轨迹运动。

        :param target_pos: [输入] 目标关节位置轨迹；各轨迹点长度=左臂dof，单位rad。
        :param eeff_pos: [输入] 末端执行器目标位置轨迹；单位rad或m；None表示不随本次指令更新。
        :param stamps: [输入] 各轨迹点时间戳；单位s；None表示自动规划。
        :param is_sync: [输入] 是否同步等待任务完成。
        :return: int，1表示成功，-1表示失败。
        """
        return self._left.move_joint_traj(target_pos, eeff_pos, stamps, is_sync)

    def move_left_pose_traj(self, target_pos: list, eeff_pos: list = None,
                            stamps: list = None, is_sync: bool = True) -> int:
        """执行左臂位姿轨迹运动。

        :param target_pos: [输入] 目标位姿或位姿轨迹[x,y,z,qx,qy,qz,qw]；位置单位m，姿态为归一化四元数。
        :param eeff_pos: [输入] 末端执行器目标位置；单位rad或m；负值表示不随本次指令更新。
        :param stamps: [输入] 各轨迹点时间戳；单位s；None表示自动规划。
        :param is_sync: [输入] 是否同步等待任务完成。
        :return: int，1表示成功，-1表示失败。
        """
        return self._left.move_pose_traj(target_pos, eeff_pos, stamps, is_sync)

    def move_left_flow_pose(self, target_pos: list,
                            line_theta_weight: float = 0.5,
                            accuracy: float = 0.0001,
                            is_sync: bool = True) -> int:
        """执行左臂连续位姿运动。

        :param target_pos: [输入] 目标位姿或位姿轨迹[x,y,z,qx,qy,qz,qw]；位置单位m，姿态为归一化四元数。
        :param line_theta_weight: [输入] 直线运动姿态插值权重。
        :param accuracy: [输入] 路径精度阈值；单位m。
        :param is_sync: [输入] 是否同步等待任务完成。
        :return: int，1表示成功，-1表示失败。
        """
        return self._left.move_flow_pose(target_pos, line_theta_weight,
                                         accuracy, is_sync)

    def set_left_gripper(self, pos: float, tau: float = 10) -> int:
        """设置左臂夹爪。

        :param pos: [输入] 目标位置；关节/旋转电机单位rad，直线电机单位m。
        :param tau: [输入] 目标力矩/力；单位N·m或N。
        :return: int，1表示成功，-1表示失败。
        """
        return self._left.set_gripper(pos, tau)

    def set_left_hand(self, pos: list, tau: list, vel: list) -> int:
        """设置左臂灵巧手。

        :param pos: [输入] 目标位置；关节/旋转电机单位rad，直线电机单位m。
        :param tau: [输入] 目标力矩/力；单位N·m或N。
        :param vel: [输入] 目标速度；旋转部分单位rad/s，直线部分单位m/s。
        :return: int，1表示成功，-1表示失败。
        """
        return self._left.set_hand(pos, tau, vel)

    def set_left_tool_index(self, index: int) -> int:
        """设置左臂工具编号。

        :param index: [输入] 工具编号。
        :return: int，1表示成功，-1表示失败。
        """
        return self._left.set_tool_index(index)

    def get_left_tool_index(self) -> int:
        """获取左臂工具编号。

        :return: int，枚举值、编号或自由度。
        """
        return self._left.get_tool_index()

    def get_left_tool_coordinate(self, index: int) -> list:
        """获取左臂工具坐标。

        :param index: [输入] 工具编号。
        :return: list，[x,y,z,qx,qy,qz,qw]；位置单位m，姿态为归一化四元数。
        """
        return self._left.get_tool_coordinate(index)

    def trajectory_teach_left(self, off_on: bool, name: str) -> int:
        """执行左臂示教轨迹录制。

        :param off_on: [输入] True开始录制，False停止录制。
        :param name: [输入] 示教轨迹名称。
        :return: int，1表示成功，-1表示失败。
        """
        return self._left.trajectory_teach(off_on, name)

    def trajectory_recorder_left(self, name: str, is_sync: bool = True) -> int:
        """执行左臂示教轨迹复现。

        :param name: [输入] 示教轨迹名称。
        :param is_sync: [输入] 是否同步等待任务完成。
        :return: int，1表示成功，-1表示失败。
        """
        return self._left.trajectory_recorder(name, is_sync)

    def inverse_kine_left(self, tool_index: int, quat_pose: list,
                          ref_joint: list, jnt_value: list) -> int:
        """执行左臂逆运动学。

        :param tool_index: [输入] 工具编号。
        :param quat_pose: [输入] 笛卡尔位姿或位姿数组[x,y,z,qx,qy,qz,qw]；位置单位m，姿态为归一化四元数。
        :param ref_joint: [输入] 逆解参考关节位置或其数组；单位rad。
        :param jnt_value: [输出] 关节位置或其数组；单位rad。
        :return: int，1表示成功，-1表示失败。
        """
        return self._left.inverse_kine(tool_index, quat_pose,
                                       ref_joint, jnt_value)

    def forward_kine_left(self, tool_index: int, jnt_value: list,
                          quat_pose: list) -> int:
        """执行左臂正运动学。

        :param tool_index: [输入] 工具编号。
        :param jnt_value: [输入] 关节位置或其数组；单位rad。
        :param quat_pose: [输出] 笛卡尔位姿或位姿数组[x,y,z,qx,qy,qz,qw]；位置单位m，姿态为归一化四元数。
        :return: int，1表示成功，-1表示失败。
        """
        return self._left.forward_kine(tool_index, jnt_value, quat_pose)

    def inverse_kine_left_array(self, tool_index: int, quat_pose: list,
                                ref_joint: list, jnt_value: list) -> int:
        """执行左臂批量逆运动学。

        :param tool_index: [输入] 工具编号。
        :param quat_pose: [输入] 笛卡尔位姿或位姿数组[x,y,z,qx,qy,qz,qw]；位置单位m，姿态为归一化四元数。
        :param ref_joint: [输入] 逆解参考关节位置或其数组；单位rad。
        :param jnt_value: [输出] 关节位置或其数组；单位rad。
        :return: int，1表示成功，-1表示失败。
        """
        return self._left.inverse_kine_array(tool_index, quat_pose,
                                             ref_joint, jnt_value)

    def forward_kine_left_array(self, tool_index: int, jnt_value: list,
                                quat_pose: list) -> int:
        """执行左臂批量正运动学。

        :param tool_index: [输入] 工具编号。
        :param jnt_value: [输入] 关节位置或其数组；单位rad。
        :param quat_pose: [输出] 笛卡尔位姿或位姿数组[x,y,z,qx,qy,qz,qw]；位置单位m，姿态为归一化四元数。
        :return: int，1表示成功，-1表示失败。
        """
        return self._left.forward_kine_array(tool_index, jnt_value, quat_pose)

    def low_left_pv_command(self, pos: list, vel: list, data: dict) -> int:
        """发送左臂PV指令；pos[输入]单位rad，vel[输入]单位rad/s，
        data[输出]为底层状态字典；数组长度均为左臂dof。返回1成功、-1失败。
        """
        return self._left.low_pv_command(pos, vel, data)

    def low_left_mit_command(self, pos: list, vel: list, tau: list,
                             kp: list, kd: list, data: dict) -> int:
        """发送左臂MIT指令。

        pos/vel/tau/kp/kd均为[输入]且长度=左臂dof，单位依次为rad、rad/s、
        Nm、Nm/rad、Nm·s/rad；data[输出]为底层状态字典。
        返回1成功、-1失败。
        """
        return self._left.low_mit_command(pos, vel, tau, kp, kd, data)

    def low_left_pf_command(self, pos: list, vel: list, tau: list,
                            data: dict) -> int:
        """发送左臂PF指令；pos/vel/tau[输入]单位依次为rad、rad/s、
        Nm，长度=左臂dof；data[输出]为底层状态字典。返回1成功、-1失败。
        """
        return self._left.low_pf_command(pos, vel, tau, data)

    def low_left_current_command(self, tau: list, data: dict) -> int:
        """发送左臂Current指令；tau[输入]为关节力矩，单位Nm、长度=左臂dof；
        data[输出]为底层状态字典。返回1成功、-1失败。
        """
        return self._left.low_current_command(tau, data)

    def low_left_set_safety_params(self, updates: dict, current: dict) -> int:
        """设置左臂Low Mode安全参数，支持部分更新。

        updates[输入]可包含看门狗超时(ms)、位置差上限(rad)、速度差上限(rad/s)
        和力矩/力上限(Nm)；current[输出]为四项完整当前参数。返回1成功、-1失败。
        """
        return self._left.low_set_safety_params(updates, current)

    def low_left_refresh(self, data: dict) -> int:
        """刷新左臂底层状态；data[输出]为最新状态字典，各字段沿用对应物理单位。
        返回1成功、-1失败。
        """
        return self._left.low_refresh(data)

    def low_left_set_end_effector_ctr(self, pos: list, vel: list, tau: list,
                                      data: dict) -> int:
        """控制左臂末端电机；pos/vel/tau[输入]单位依次为rad或m、rad/s或m/s、N·m或N；
        data[输出]为底层状态字典。返回1成功、-1失败。
        """
        return self._left.low_set_end_effector_ctr(pos, vel, tau, data)

    def low_left_set_robot_mode(self, mode: int) -> int:
        """设置左臂底层模式；mode[输入]为0-IDLE、1-PV、2-MIT、3-CURRENT、4-PF。
        返回1成功、-1失败。
        """
        return self._left.low_set_robot_mode(mode)

    def low_left_set_end_effector_mode(self, mode: int) -> int:
        """设置左臂末端模式；mode[输入]为末端定义的枚举值。返回1成功、-1失败。"""
        return self._left.low_set_end_effector_mode(mode)

    def low_left_set_servo_enable(self, status: bool) -> int:
        """设置左臂伺服；status[输入]为True上使能、False下使能。
        返回1成功、-1失败。
        """
        return self._left.low_set_servo_enable(status)

    def low_left_reset(self, cnt: int = 5) -> int:
        """复位左臂底层错误；cnt[输入]为最大尝试次数，单位次。返回1成功、-1失败。"""
        return self._left.low_reset(cnt)

    def low_left_get_servo_status(self, status: dict) -> int:
        """获取左臂伺服状态；status[输出]包含模式、连接/使能、温度(°C)、电压(V)和错误信息。
        返回1成功、-1失败。
        """
        return self._left.low_get_servo_status(status)

    def low_left_get_inverse_kine(self, pose: list, refer_pos: list,
                                  jnt_value: list, tool: int = -1) -> int:
        """计算左臂逆运动学。

        :param pose: [输入] 位姿[x,y,z,qx,qy,qz,qw]；位置单位m，姿态为归一化四元数。
        :param refer_pos: [输入] 参考关节位置；单位rad。
        :param jnt_value: [输出] 逆解关节位置；单位rad。
        :param tool: [输入] 工具编号，-1表示当前工具。
        :return: int，1表示成功，-1表示失败。
        """
        return self._left.low_get_inverse_kine(pose, refer_pos, jnt_value, tool)

    def low_left_get_forward_kine(self, joint_pos: list, quat_pose: list,
                                  tool: int = -1) -> int:
        """计算左臂正运动学。

        :param joint_pos: [输入] 关节位置；单位rad。
        :param quat_pose: [输出] 位姿[x,y,z,qx,qy,qz,qw]；位置单位m，姿态为归一化四元数。
        :param tool: [输入] 工具编号，-1表示当前工具。
        :return: int，1表示成功，-1表示失败。
        """
        return self._left.low_get_forward_kine(joint_pos, quat_pose, tool)

    def low_left_get_dynamics(self, joint_pos: list, joint_vel: list,
                              joint_acc: list, tool: list, m_force: list,
                              c_force: list, g_force: list) -> int:
        """计算左臂动力学。

        :param joint_pos: [输入] 关节位置；单位rad。
        :param joint_vel: [输入] 关节速度；单位rad/s。
        :param joint_acc: [输入] 关节加速度；单位rad/s²。
        :param tool: [输出] 工具编号列表。
        :param m_force: [输出] 惯性项力矩；单位Nm。
        :param c_force: [输出] 科里奥利/离心项力矩；单位Nm。
        :param g_force: [输出] 重力项力矩；单位Nm。
        :return: int，1表示成功，-1表示失败。
        """
        return self._left.low_get_dynamics(joint_pos, joint_vel, joint_acc,
                                           tool, m_force, c_force, g_force)

    def low_left_get_jacobian(self, joint_pos: list, tool: list,
                              mat: list) -> int:
        """计算左臂雅可比矩阵。

        :param joint_pos: [输入] 关节位置；单位rad。
        :param tool: [输出] 工具编号列表。
        :param mat: [输出] rows×cols二维矩阵，元素单位由映射分量决定。
        :return: int，1表示成功，-1表示失败。
        """
        return self._left.low_get_jacobian(joint_pos, tool, mat)

    def low_left_get_nullspace(self, joint_pos: list, tolerance: float,
                               tool: list, mat: list) -> int:
        """计算左臂零空间矩阵。

        :param joint_pos: [输入] 关节位置；单位rad。
        :param tolerance: [输入] 奇异值判定公差。
        :param tool: [输出] 工具编号列表。
        :param mat: [输出] rows×cols二维矩阵。
        :return: int，1表示成功，-1表示失败。
        """
        return self._left.low_get_nullspace(joint_pos, tolerance, tool, mat)

    # ================================================================== #
    #  右臂方法（命名规则与 C++ 一致：get_right_* / set_right_* / track_right_* 等）
    # ================================================================== #
    def get_right_config(self) -> dict:
        """获取右臂配置。

        :return: dict，配置或状态字典；各字段沿用对应物理单位。
        """
        return self._right.get_config()

    def get_right_eeff_config(self) -> dict:
        """获取右臂末端执行器配置。

        :return: dict，配置或状态字典；各字段沿用对应物理单位。
        """
        return self._right.get_eeff_config()

    def get_right_status(self) -> dict:
        """获取右臂状态。

        :return: dict，配置或状态字典；各字段沿用对应物理单位。
        """
        return self._right.get_status()

    def get_right_joint_pos(self) -> list:
        """获取右臂实际关节位置。

        :return: list或float，位置；单位rad。
        """
        return self._right.get_joint_pos()

    def get_right_joint_vel(self) -> list:
        """获取右臂实际关节速度。

        :return: list或float，速度；单位rad/s。
        """
        return self._right.get_joint_vel()

    def get_right_joint_tau(self) -> list:
        """获取右臂实际关节力矩。

        :return: list或float，力矩；单位Nm。
        """
        return self._right.get_joint_tau()

    def get_right_plan_joint_pos(self) -> list:
        """获取右臂规划关节位置。

        :return: list或float，位置；单位rad。
        """
        return self._right.get_plan_joint_pos()

    def get_right_plan_joint_vel(self) -> list:
        """获取右臂规划关节速度。

        :return: list或float，速度；单位rad/s。
        """
        return self._right.get_plan_joint_vel()

    def get_right_plan_joint_tau(self) -> list:
        """获取右臂规划关节力矩。

        :return: list或float，力矩；单位Nm。
        """
        return self._right.get_plan_joint_tau()

    def get_right_plan_cart_pose(self) -> list:
        """获取右臂规划末端位姿。

        :return: list，[x,y,z,qx,qy,qz,qw]；位置单位m，姿态为归一化四元数。
        """
        return self._right.get_plan_cart_pose()

    def get_right_cart_pose(self) -> list:
        """获取右臂实际末端位姿。

        :return: list，[x,y,z,qx,qy,qz,qw]；位置单位m，姿态为归一化四元数。
        """
        return self._right.get_cart_pose()

    def get_right_joint_external_tau(self) -> list:
        """获取右臂关节外力矩。

        :return: list或float，力矩；单位Nm。
        """
        return self._right.get_joint_external_tau()

    def get_right_cart_external_force(self) -> list:
        """获取右臂末端外力/力矩。

        :return: list，[Fx,Fy,Fz,Tx,Ty,Tz]；单位N和N·m。
        """
        return self._right.get_cart_external_force()

    def register_right_joint_cbk(self, cbk: Callable) -> None:
        """注册右臂关节状态回调。

        :param cbk: [输入] 回调函数；参数单位由对应状态量定义。
        :return: 无返回值。
        """
        self._right.register_joint_cbk(cbk)

    def release_right_joint_cbk(self) -> None:
        """注销右臂关节状态回调。

        :return: 无返回值。
        """
        self._right.release_joint_cbk()

    def register_right_pose_cbk(self, cbk: Callable) -> None:
        """注册右臂位姿状态回调。

        :param cbk: [输入] 回调函数；参数单位由对应状态量定义。
        :return: 无返回值。
        """
        self._right.register_pose_cbk(cbk)

    def release_right_pose_cbk(self) -> None:
        """注销右臂位姿状态回调。

        :return: 无返回值。
        """
        self._right.release_pose_cbk()

    def register_right_plan_joint_cbk(self, cbk: Callable) -> None:
        """注册右臂规划关节状态回调。

        :param cbk: [输入] 回调函数；参数单位由对应状态量定义。
        :return: 无返回值。
        """
        self._right.register_plan_joint_cbk(cbk)

    def release_right_plan_joint_cbk(self) -> None:
        """注销右臂规划关节状态回调。

        :return: 无返回值。
        """
        self._right.release_plan_joint_cbk()

    def register_right_plan_pose_cbk(self, cbk: Callable) -> None:
        """注册右臂规划位姿回调。

        :param cbk: [输入] 回调函数；参数单位由对应状态量定义。
        :return: 无返回值。
        """
        self._right.register_plan_pose_cbk(cbk)

    def release_right_plan_pose_cbk(self) -> None:
        """注销右臂规划位姿回调。

        :return: 无返回值。
        """
        self._right.release_plan_pose_cbk()

    def register_right_external_force_cbk(self, cbk: Callable) -> None:
        """注册右臂外力回调。

        :param cbk: [输入] 回调函数；参数单位由对应状态量定义。
        :return: 无返回值。
        """
        self._right.register_external_force_cbk(cbk)

    def release_right_external_force_cbk(self) -> None:
        """注销右臂外力回调。

        :return: 无返回值。
        """
        self._right.release_external_force_cbk()

    # ------------------------------------------------------------------ #
    #  右臂末端执行器（通用 eeff）
    # ------------------------------------------------------------------ #
    def get_right_eeff_state(self) -> int:
        """获取右臂末端执行器状态。

        :return: int，枚举值、编号或自由度。
        """
        return self._right.get_eeff_state()

    def get_right_eeff_pos(self) -> list:
        """获取右臂末端执行器位置。

        :return: list或float，位置；单位rad或m。
        """
        return self._right.get_eeff_pos()

    def get_right_eeff_vel(self) -> list:
        """获取右臂末端执行器速度。

        :return: list或float，速度；单位rad/s或m/s。
        """
        return self._right.get_eeff_vel()

    def get_right_eeff_tau(self) -> list:
        """获取右臂末端执行器力矩/力。

        :return: list或float，力矩/力；单位N·m或N。
        """
        return self._right.get_eeff_tau()

    def get_right_eeff_motor_pos(self) -> list:
        """获取右臂末端执行器电机位置。

        :return: list或float，位置；单位rad或m。
        """
        return self._right.get_eeff_motor_pos()

    def get_right_eeff_motor_vel(self) -> list:
        """获取右臂末端执行器电机速度。

        :return: list或float，速度；单位rad/s或m/s。
        """
        return self._right.get_eeff_motor_vel()

    def get_right_eeff_motor_tau(self) -> list:
        """获取右臂末端执行器电机力矩/力。

        :return: list或float，力矩/力；单位N·m或N。
        """
        return self._right.get_eeff_motor_tau()

    def get_right_plan_eeff_pos(self) -> list:
        """获取右臂规划末端执行器位置。

        :return: list或float，位置；单位rad或m。
        """
        return self._right.get_plan_eeff_pos()

    def get_right_plan_eeff_vel(self) -> list:
        """获取右臂规划末端执行器速度。

        :return: list或float，速度；单位rad/s或m/s。
        """
        return self._right.get_plan_eeff_vel()

    def get_right_plan_eeff_tau(self) -> list:
        """获取右臂规划末端执行器力矩/力。

        :return: list或float，力矩/力；单位N·m或N。
        """
        return self._right.get_plan_eeff_tau()

    def get_right_plan_eeff_motor_pos(self) -> list:
        """获取右臂规划末端执行器电机位置。

        :return: list或float，位置；单位rad或m。
        """
        return self._right.get_plan_eeff_motor_pos()

    def get_right_plan_eeff_motor_vel(self) -> list:
        """获取右臂规划末端执行器电机速度。

        :return: list或float，速度；单位rad/s或m/s。
        """
        return self._right.get_plan_eeff_motor_vel()

    def get_right_plan_eeff_motor_tau(self) -> list:
        """获取右臂规划末端执行器电机力矩/力。

        :return: list或float，力矩/力；单位N·m或N。
        """
        return self._right.get_plan_eeff_motor_tau()

    def get_right_eeff_type(self) -> str:
        """获取右臂末端执行器类型。

        :return: str，类型名称。
        """
        return self._right.get_eeff_type()

    def get_right_eeff_dof(self) -> int:
        """获取右臂末端执行器自由度。

        :return: int，枚举值、编号或自由度。
        """
        return self._right.get_eeff_dof()

    def get_right_eeff_connect(self) -> bool:
        """获取右臂末端执行器连接状态。

        :return: bool，True表示已连接，False表示未连接。
        """
        return self._right.get_eeff_connect()

    def set_right_eeff(self, pos: list, vel: list, tau: list,
                       control_motor: bool = False) -> int:
        """设置右臂末端执行器。

        :param pos: [输入] 目标位置；关节/旋转电机单位rad，直线电机单位m。
        :param vel: [输入] 目标速度；旋转部分单位rad/s，直线部分单位m/s。
        :param tau: [输入] 目标力矩/力；单位N·m或N。
        :param control_motor: [输入] 是否直接使用电机坐标控制末端执行器。
        :return: int，1表示成功，-1表示失败。
        """
        return self._right.set_eeff(pos, vel, tau, control_motor)

    # ------------------------------------------------------------------ #
    #  deprecated: 右臂旧夹爪/灵巧手接口
    # ------------------------------------------------------------------ #
    def get_right_gripper_state(self) -> int:
        """获取右臂夹爪状态。

        :return: int，枚举值、编号或自由度。
        """
        return self._right.get_gripper_state()

    def get_right_gripper_pos(self) -> float:
        """获取右臂夹爪位置。

        :return: list或float，位置；单位rad或m。
        """
        return self._right.get_gripper_pos()

    def get_right_gripper_vel(self) -> float:
        """获取右臂夹爪速度。

        :return: list或float，速度；单位rad/s或m/s。
        """
        return self._right.get_gripper_vel()

    def get_right_gripper_tau(self) -> float:
        """获取右臂夹爪力矩/力。

        :return: list或float，力矩/力；单位N·m或N。
        """
        return self._right.get_gripper_tau()

    def get_right_plan_gripper_pos(self) -> float:
        """获取右臂规划夹爪位置。

        :return: list或float，位置；单位rad或m。
        """
        return self._right.get_plan_gripper_pos()

    def get_right_plan_gripper_tau(self) -> float:
        """获取右臂规划夹爪力矩/力。

        :return: list或float，力矩/力；单位N·m或N。
        """
        return self._right.get_plan_gripper_tau()

    def get_right_hand_state(self) -> int:
        """获取右臂灵巧手状态。

        :return: int，枚举值、编号或自由度。
        """
        return self._right.get_hand_state()

    def get_right_hand_pos(self) -> list:
        """获取右臂灵巧手位置。

        :return: list或float，位置；单位rad或m。
        """
        return self._right.get_hand_pos()

    def get_right_hand_vel(self) -> list:
        """获取右臂灵巧手速度。

        :return: list或float，速度；单位rad/s或m/s。
        """
        return self._right.get_hand_vel()

    def get_right_hand_tau(self) -> list:
        """获取右臂灵巧手力矩/力。

        :return: list或float，力矩/力；单位N·m或N。
        """
        return self._right.get_hand_tau()

    def get_right_plan_hand_pos(self) -> list:
        """获取右臂规划灵巧手位置。

        :return: list或float，位置；单位rad或m。
        """
        return self._right.get_plan_hand_pos()

    def get_right_plan_hand_vel(self) -> list:
        """获取右臂规划灵巧手速度。

        :return: list或float，速度；单位rad/s或m/s。
        """
        return self._right.get_plan_hand_vel()

    def get_right_plan_hand_tau(self) -> list:
        """获取右臂规划灵巧手力矩/力。

        :return: list或float，力矩/力；单位N·m或N。
        """
        return self._right.get_plan_hand_tau()

    def track_right_joint(self, targets: list, eeff_pos: float = -1) -> int:
        """实时跟随右臂关节目标。

        :param targets: [输入] 目标关节位置，长度=右臂dof；单位rad。
        :param eeff_pos: [输入] 末端执行器目标位置；单位rad或m；负值表示不随本次指令更新。
        :return: int，1表示成功，-1表示失败。
        """
        return self._right.track_joint(targets, eeff_pos)

    def track_right_pose(self, targets: list, eeff_pos: float = -1) -> int:
        """实时跟随右臂位姿目标。

        :param targets: [输入] 目标位姿[x,y,z,qx,qy,qz,qw]；位置单位m，姿态为归一化四元数。
        :param eeff_pos: [输入] 末端执行器目标位置；单位rad或m；负值表示不随本次指令更新。
        :return: int，1表示成功，-1表示失败。
        """
        return self._right.track_pose(targets, eeff_pos)

    def track_joint(self, left_targets: list, right_targets: list,
                    left_eeff_pos: float = -1,
                    right_eeff_pos: float = -1) -> int:
        """周期性发送双臂目标关节位置。

        :param left_targets: [输入] 左臂目标关节位置，长度=左臂dof；单位rad。
        :param right_targets: [输入] 右臂目标关节位置，长度=右臂dof；单位rad。
        :param left_eeff_pos: [输入] 左臂末端执行器目标位置；单位rad或m；负值表示不更新。
        :param right_eeff_pos: [输入] 右臂末端执行器目标位置；单位rad或m；负值表示不更新。
        :return: int，1表示成功，-1表示失败。
        """
        ret = self.track_left_joint(left_targets, left_eeff_pos)
        if ret < 0:
            return ret
        return self.track_right_joint(right_targets, right_eeff_pos)

    def track_pose(self, left_targets: list, right_targets: list,
                   left_eeff_pos: float = -1,
                   right_eeff_pos: float = -1) -> int:
        """周期性发送双臂法兰相对基座的目标位姿。

        :param left_targets: [输入] 左臂位姿[x,y,z,qx,qy,qz,qw]；位置单位m，姿态为归一化四元数。
        :param right_targets: [输入] 右臂位姿[x,y,z,qx,qy,qz,qw]；位置单位m，姿态为归一化四元数。
        :param left_eeff_pos: [输入] 左臂末端执行器目标位置；单位rad或m；负值表示不更新。
        :param right_eeff_pos: [输入] 右臂末端执行器目标位置；单位rad或m；负值表示不更新。
        :return: int，1表示成功，-1表示失败。
        """
        ret = self.track_left_pose(left_targets, left_eeff_pos)
        if ret < 0:
            return ret
        return self.track_right_pose(right_targets, right_eeff_pos)

    def move_right_joint(self, target_pos: list, desire_time: float = -1,
                         is_sync: bool = True) -> int:
        """执行右臂关节运动。

        :param target_pos: [输入] 目标关节位置，长度=右臂dof；单位rad。
        :param desire_time: [输入] 期望运动时间；单位s；负值表示由控制器自动规划。
        :param is_sync: [输入] 是否同步等待任务完成。
        :return: int，1表示成功，-1表示失败。
        """
        return self._right.move_joint(target_pos, desire_time, is_sync)

    def move_right_pose(self, target_pos: list, desire_time: float = -1,
                        is_sync: bool = True) -> int:
        """执行右臂位姿运动。

        :param target_pos: [输入] 目标位姿或位姿轨迹[x,y,z,qx,qy,qz,qw]；位置单位m，姿态为归一化四元数。
        :param desire_time: [输入] 期望运动时间；单位s；负值表示由控制器自动规划。
        :param is_sync: [输入] 是否同步等待任务完成。
        :return: int，1表示成功，-1表示失败。
        """
        return self._right.move_pose(target_pos, desire_time, is_sync)

    def move_right_joint_traj(self, target_pos: list, eeff_pos: list = None,
                              stamps: list = None,
                              is_sync: bool = True) -> int:
        """执行右臂关节轨迹运动。

        :param target_pos: [输入] 目标关节位置轨迹；各轨迹点长度=右臂dof，单位rad。
        :param eeff_pos: [输入] 末端执行器目标位置轨迹；单位rad或m；None表示不随本次指令更新。
        :param stamps: [输入] 各轨迹点时间戳；单位s；None表示自动规划。
        :param is_sync: [输入] 是否同步等待任务完成。
        :return: int，1表示成功，-1表示失败。
        """
        return self._right.move_joint_traj(target_pos, eeff_pos,
                                           stamps, is_sync)

    def move_right_pose_traj(self, target_pos: list, eeff_pos: list = None,
                             stamps: list = None,
                             is_sync: bool = True) -> int:
        """执行右臂位姿轨迹运动。

        :param target_pos: [输入] 目标位姿或位姿轨迹[x,y,z,qx,qy,qz,qw]；位置单位m，姿态为归一化四元数。
        :param eeff_pos: [输入] 末端执行器目标位置；单位rad或m；负值表示不随本次指令更新。
        :param stamps: [输入] 各轨迹点时间戳；单位s；None表示自动规划。
        :param is_sync: [输入] 是否同步等待任务完成。
        :return: int，1表示成功，-1表示失败。
        """
        return self._right.move_pose_traj(target_pos, eeff_pos,
                                          stamps, is_sync)

    def move_right_flow_pose(self, target_pos: list,
                             line_theta_weight: float = 0.5,
                             accuracy: float = 0.0001,
                             is_sync: bool = True) -> int:
        """执行右臂连续位姿运动。

        :param target_pos: [输入] 目标位姿或位姿轨迹[x,y,z,qx,qy,qz,qw]；位置单位m，姿态为归一化四元数。
        :param line_theta_weight: [输入] 直线运动姿态插值权重。
        :param accuracy: [输入] 路径精度阈值；单位m。
        :param is_sync: [输入] 是否同步等待任务完成。
        :return: int，1表示成功，-1表示失败。
        """
        return self._right.move_flow_pose(target_pos, line_theta_weight,
                                          accuracy, is_sync)

    def set_right_gripper(self, pos: float, tau: float = 10) -> int:
        """设置右臂夹爪。

        :param pos: [输入] 目标位置；关节/旋转电机单位rad，直线电机单位m。
        :param tau: [输入] 目标力矩/力；单位N·m或N。
        :return: int，1表示成功，-1表示失败。
        """
        return self._right.set_gripper(pos, tau)

    def set_right_hand(self, pos: list, tau: list, vel: list) -> int:
        """设置右臂灵巧手。

        :param pos: [输入] 目标位置；关节/旋转电机单位rad，直线电机单位m。
        :param tau: [输入] 目标力矩/力；单位N·m或N。
        :param vel: [输入] 目标速度；旋转部分单位rad/s，直线部分单位m/s。
        :return: int，1表示成功，-1表示失败。
        """
        return self._right.set_hand(pos, tau, vel)

    def set_right_tool_index(self, index: int) -> int:
        """设置右臂工具编号。

        :param index: [输入] 工具编号。
        :return: int，1表示成功，-1表示失败。
        """
        return self._right.set_tool_index(index)

    def get_right_tool_index(self) -> int:
        """获取右臂工具编号。

        :return: int，枚举值、编号或自由度。
        """
        return self._right.get_tool_index()

    def get_right_tool_coordinate(self, index: int) -> list:
        """获取右臂工具坐标。

        :param index: [输入] 工具编号。
        :return: list，[x,y,z,qx,qy,qz,qw]；位置单位m，姿态为归一化四元数。
        """
        return self._right.get_tool_coordinate(index)

    def trajectory_teach_right(self, off_on: bool, name: str) -> int:
        """执行右臂示教轨迹录制。

        :param off_on: [输入] True开始录制，False停止录制。
        :param name: [输入] 示教轨迹名称。
        :return: int，1表示成功，-1表示失败。
        """
        return self._right.trajectory_teach(off_on, name)

    def trajectory_teach(self, off_on: bool, name: str) -> int:
        """使用同一轨迹名同步开始或停止左右臂示教。

        :param off_on: [输入] True开始录制，False停止录制。
        :param name: [输入] 左右臂共用的示教轨迹名称。
        :return: int，1表示成功，-1表示失败。
        """
        rl = self._left.trajectory_teach(off_on, name)
        rr = self._right.trajectory_teach(off_on, name)
        return 1 if rl == 1 and rr == 1 else -1

    def trajectory_recorder_right(self, name: str, is_sync: bool = True) -> int:
        """执行右臂示教轨迹复现。

        :param name: [输入] 示教轨迹名称。
        :param is_sync: [输入] 是否同步等待任务完成。
        :return: int，1表示成功，-1表示失败。
        """
        return self._right.trajectory_recorder(name, is_sync)

    def inverse_kine_right(self, tool_index: int, quat_pose: list,
                           ref_joint: list, jnt_value: list) -> int:
        """执行右臂逆运动学。

        :param tool_index: [输入] 工具编号。
        :param quat_pose: [输入] 笛卡尔位姿或位姿数组[x,y,z,qx,qy,qz,qw]；位置单位m，姿态为归一化四元数。
        :param ref_joint: [输入] 逆解参考关节位置或其数组；单位rad。
        :param jnt_value: [输出] 关节位置或其数组；单位rad。
        :return: int，1表示成功，-1表示失败。
        """
        return self._right.inverse_kine(tool_index, quat_pose,
                                        ref_joint, jnt_value)

    def forward_kine_right(self, tool_index: int, jnt_value: list,
                           quat_pose: list) -> int:
        """执行右臂正运动学。

        :param tool_index: [输入] 工具编号。
        :param jnt_value: [输入] 关节位置或其数组；单位rad。
        :param quat_pose: [输出] 笛卡尔位姿或位姿数组[x,y,z,qx,qy,qz,qw]；位置单位m，姿态为归一化四元数。
        :return: int，1表示成功，-1表示失败。
        """
        return self._right.forward_kine(tool_index, jnt_value, quat_pose)

    def inverse_kine_right_array(self, tool_index: int, quat_pose: list,
                                 ref_joint: list, jnt_value: list) -> int:
        """执行右臂批量逆运动学。

        :param tool_index: [输入] 工具编号。
        :param quat_pose: [输入] 笛卡尔位姿或位姿数组[x,y,z,qx,qy,qz,qw]；位置单位m，姿态为归一化四元数。
        :param ref_joint: [输入] 逆解参考关节位置或其数组；单位rad。
        :param jnt_value: [输出] 关节位置或其数组；单位rad。
        :return: int，1表示成功，-1表示失败。
        """
        return self._right.inverse_kine_array(tool_index, quat_pose,
                                              ref_joint, jnt_value)

    def forward_kine_right_array(self, tool_index: int, jnt_value: list,
                                 quat_pose: list) -> int:
        """执行右臂批量正运动学。

        :param tool_index: [输入] 工具编号。
        :param jnt_value: [输入] 关节位置或其数组；单位rad。
        :param quat_pose: [输出] 笛卡尔位姿或位姿数组[x,y,z,qx,qy,qz,qw]；位置单位m，姿态为归一化四元数。
        :return: int，1表示成功，-1表示失败。
        """
        return self._right.forward_kine_array(tool_index, jnt_value, quat_pose)

    def low_right_pv_command(self, pos: list, vel: list, data: dict) -> int:
        """发送右臂PV指令；pos[输入]单位rad，vel[输入]单位rad/s，
        data[输出]为底层状态字典；数组长度均为右臂dof。返回1成功、-1失败。
        """
        return self._right.low_pv_command(pos, vel, data)

    def low_right_mit_command(self, pos: list, vel: list, tau: list,
                              kp: list, kd: list, data: dict) -> int:
        """发送右臂MIT指令。

        pos/vel/tau/kp/kd均为[输入]且长度=右臂dof，单位依次为rad、rad/s、
        Nm、Nm/rad、Nm·s/rad；data[输出]为底层状态字典。
        返回1成功、-1失败。
        """
        return self._right.low_mit_command(pos, vel, tau, kp, kd, data)

    def low_right_pf_command(self, pos: list, vel: list, tau: list,
                             data: dict) -> int:
        """发送右臂PF指令；pos/vel/tau[输入]单位依次为rad、rad/s、
        Nm，长度=右臂dof；data[输出]为底层状态字典。返回1成功、-1失败。
        """
        return self._right.low_pf_command(pos, vel, tau, data)

    def low_right_current_command(self, tau: list, data: dict) -> int:
        """发送右臂Current指令；tau[输入]为关节力矩，单位Nm、长度=右臂dof；
        data[输出]为底层状态字典。返回1成功、-1失败。
        """
        return self._right.low_current_command(tau, data)

    def low_right_set_safety_params(self, updates: dict, current: dict) -> int:
        """设置右臂Low Mode安全参数，支持部分更新。

        updates[输入]可包含看门狗超时(ms)、位置差上限(rad)、速度差上限(rad/s)
        和力矩/力上限(Nm)；current[输出]为四项完整当前参数。返回1成功、-1失败。
        """
        return self._right.low_set_safety_params(updates, current)

    def low_right_refresh(self, data: dict) -> int:
        """刷新右臂底层状态；data[输出]为最新状态字典，各字段沿用对应物理单位。
        返回1成功、-1失败。
        """
        return self._right.low_refresh(data)

    def low_right_set_end_effector_ctr(self, pos: list, vel: list, tau: list,
                                       data: dict) -> int:
        """控制右臂末端电机；pos/vel/tau[输入]单位依次为rad或m、rad/s或m/s、N·m或N；
        data[输出]为底层状态字典。返回1成功、-1失败。
        """
        return self._right.low_set_end_effector_ctr(pos, vel, tau, data)

    def low_right_set_robot_mode(self, mode: int) -> int:
        """设置右臂底层模式；mode[输入]为0-IDLE、1-PV、2-MIT、3-CURRENT、4-PF。
        返回1成功、-1失败。
        """
        return self._right.low_set_robot_mode(mode)

    def low_right_set_end_effector_mode(self, mode: int) -> int:
        """设置右臂末端模式；mode[输入]为末端定义的枚举值。返回1成功、-1失败。"""
        return self._right.low_set_end_effector_mode(mode)

    def low_right_set_servo_enable(self, status: bool) -> int:
        """设置右臂伺服；status[输入]为True上使能、False下使能。
        返回1成功、-1失败。
        """
        return self._right.low_set_servo_enable(status)

    def low_right_reset(self, cnt: int = 5) -> int:
        """复位右臂底层错误；cnt[输入]为最大尝试次数，单位次。返回1成功、-1失败。"""
        return self._right.low_reset(cnt)

    def low_right_get_servo_status(self, status: dict) -> int:
        """获取右臂伺服状态；status[输出]包含模式、连接/使能、温度(°C)、电压(V)和错误信息。
        返回1成功、-1失败。
        """
        return self._right.low_get_servo_status(status)

    def low_right_get_inverse_kine(self, pose: list, refer_pos: list,
                                   jnt_value: list, tool: int = -1) -> int:
        """计算右臂逆运动学。

        :param pose: [输入] 位姿[x,y,z,qx,qy,qz,qw]；位置单位m，姿态为归一化四元数。
        :param refer_pos: [输入] 参考关节位置；单位rad。
        :param jnt_value: [输出] 逆解关节位置；单位rad。
        :param tool: [输入] 工具编号，-1表示当前工具。
        :return: int，1表示成功，-1表示失败。
        """
        return self._right.low_get_inverse_kine(pose, refer_pos,
                                                jnt_value, tool)

    def low_right_get_forward_kine(self, joint_pos: list, quat_pose: list,
                                   tool: int = -1) -> int:
        """计算右臂正运动学。

        :param joint_pos: [输入] 关节位置；单位rad。
        :param quat_pose: [输出] 位姿[x,y,z,qx,qy,qz,qw]；位置单位m，姿态为归一化四元数。
        :param tool: [输入] 工具编号，-1表示当前工具。
        :return: int，1表示成功，-1表示失败。
        """
        return self._right.low_get_forward_kine(joint_pos, quat_pose, tool)

    def low_right_get_dynamics(self, joint_pos: list, joint_vel: list,
                               joint_acc: list, tool: list, m_force: list,
                               c_force: list, g_force: list) -> int:
        """计算右臂动力学。

        :param joint_pos: [输入] 关节位置；单位rad。
        :param joint_vel: [输入] 关节速度；单位rad/s。
        :param joint_acc: [输入] 关节加速度；单位rad/s²。
        :param tool: [输出] 工具编号列表。
        :param m_force: [输出] 惯性项力矩；单位Nm。
        :param c_force: [输出] 科里奥利/离心项力矩；单位Nm。
        :param g_force: [输出] 重力项力矩；单位Nm。
        :return: int，1表示成功，-1表示失败。
        """
        return self._right.low_get_dynamics(joint_pos, joint_vel, joint_acc,
                                            tool, m_force, c_force, g_force)

    def low_right_get_jacobian(self, joint_pos: list, tool: list,
                               mat: list) -> int:
        """计算右臂雅可比矩阵。

        :param joint_pos: [输入] 关节位置；单位rad。
        :param tool: [输出] 工具编号列表。
        :param mat: [输出] rows×cols二维矩阵，元素单位由映射分量决定。
        :return: int，1表示成功，-1表示失败。
        """
        return self._right.low_get_jacobian(joint_pos, tool, mat)

    def low_right_get_nullspace(self, joint_pos: list, tolerance: float,
                                tool: list, mat: list) -> int:
        """计算右臂零空间矩阵。

        :param joint_pos: [输入] 关节位置；单位rad。
        :param tolerance: [输入] 奇异值判定公差。
        :param tool: [输出] 工具编号列表。
        :param mat: [输出] rows×cols二维矩阵。
        :return: int，1表示成功，-1表示失败。
        """
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
        """初始化机械臂控制对象。

        :param server_ip: [输入] 控制器IPv4地址。
        :param port: [输入] 控制器TCP端口。
        :param timeout: [输入] 连接超时时间；单位s。
        :param left_index: [输入] 左臂编号。
        :param right_index: [输入] 右臂编号。
        :param waist_index: [输入] 腰部编号。
        :param _validate_arm: [输入] 是否校验设备型号。
        :return: 无返回值。
        """
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
        """连接机械臂控制器并校验设备类型。

        :param server_ip: [输入] 控制器IPv4地址。
        :param port: [输入] 控制器TCP端口。
        :param timeout: [输入] 连接超时时间；单位s。
        :return: int，1表示成功，-1表示失败。
        """
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
        """断开机械臂控制器连接。

        :return: int，1表示成功，-1表示失败。
        """
        dual_ret = super().disconnect()
        waist_ret = self._waist.disconnect()
        return 1 if dual_ret == 1 and waist_ret == 1 else -1

    def is_connected(self) -> bool:
        """检查机械臂控制器是否已连接。

        :return: bool，True表示已连接，False表示未连接。
        """
        return super().is_connected() and self._waist.is_connected()

    def set_ready(self) -> int:
        """将机械臂切换到就绪状态。

        :return: int，1表示成功，-1表示失败。
        """
        dual_ret = super().set_ready()
        waist_ret = self._waist.set_ready()
        return 1 if dual_ret == 1 and waist_ret == 1 else -1

    def set_servo_enable(self, enable: bool) -> int:
        """设置机械臂伺服使能状态。

        :param enable: [输入] 是否使能；True为使能，False为失能。
        :return: int，1表示成功，-1表示失败。
        """
        dual_ret = super().set_servo_enable(enable)
        waist_ret = self._waist.set_servo_enable(enable)
        return 1 if dual_ret == 1 and waist_ret == 1 else -1

    def set_waist_servo_enable(self, enable: bool) -> int:
        """设置腰部伺服使能状态。

        :param enable: [输入] 是否使能；True为使能，False为失能。
        :return: int，1表示成功，-1表示失败。
        """
        return self._waist.set_servo_enable(enable)

    def set_control_mode(self, mode: int) -> int:
        """设置机械臂控制模式。

        :param mode: [输入] 控制或透传模式枚举值。
        :return: int，1表示成功，-1表示失败。
        """
        dual_ret = super().set_control_mode(mode)
        waist_ret = self._waist.set_control_mode(mode)
        return 1 if dual_ret == 1 and waist_ret == 1 else -1

    def set_waist_control_mode(self, mode: int) -> int:
        """设置腰部控制模式。

        :param mode: [输入] 控制或透传模式枚举值。
        :return: int，1表示成功，-1表示失败。
        """
        return self._waist.set_control_mode(mode)

    def get_version(self) -> str:
        """获取SDK与控制器版本信息。

        :return: str，版本信息。
        """
        return super().get_version() + self._waist.get_version()

    def emergency_stop(self) -> int:
        """触发机械臂急停。

        :return: int，1表示成功，-1表示失败。
        """
        dual_ret = super().emergency_stop()
        waist_ret = self._waist.emergency_stop()
        return 1 if dual_ret == 1 and waist_ret == 1 else -1

    def task_stop(self) -> int:
        """立即停止机械臂当前任务。

        :return: int，1表示成功，-1表示失败。
        """
        dual_ret = super().task_stop()
        waist_ret = self._waist.task_stop()
        return 1 if dual_ret == 1 and waist_ret == 1 else -1

    def set_speed_level(self, level: float, response_level: int = 20) -> int:
        """设置机械臂速度等级。

        :param level: [输入] 全局速度百分比，范围由控制器定义；单位%。
        :param response_level: [输入] 速度响应等级。
        :return: int，1表示成功，-1表示失败。
        """
        dual_ret = super().set_speed_level(level, response_level)
        waist_ret = self._waist.set_speed_level(level, response_level)
        return 1 if dual_ret == 1 and waist_ret == 1 else -1

    def set_collision_config(self, enable_flag: bool = True,
                             sensitivity_level: int = 0) -> int:
        """设置机械臂碰撞检测配置。

        :param enable_flag: [输入] 是否启用碰撞检测。
        :param sensitivity_level: [输入] 碰撞灵敏度等级。
        :return: int，1表示成功，-1表示失败。
        """
        dual_ret = super().set_collision_config(enable_flag, sensitivity_level)
        waist_ret = self._waist.set_collision_config(enable_flag, sensitivity_level)
        return 1 if dual_ret == 1 and waist_ret == 1 else -1

    def set_waist_collision_config(self, enable_flag: bool = True,
                                   sensitivity_level: int = 0) -> int:
        """设置腰部碰撞检测配置。

        :param enable_flag: [输入] 是否启用碰撞检测。
        :param sensitivity_level: [输入] 碰撞灵敏度等级。
        :return: int，1表示成功，-1表示失败。
        """
        return self._waist.set_collision_config(enable_flag, sensitivity_level)

    def register_error_cbk(self, key: str, cbk: Callable) -> None:
        """注册错误回调。

        :param key: [输入] 回调唯一标识。
        :param cbk: [输入] 回调函数；参数单位由对应状态量定义。
        :return: 无返回值。
        """
        super().register_error_cbk(key, cbk)
        self._waist.register_error_cbk(key, cbk)

    def release_error_cbk(self, key: str) -> None:
        """注销错误回调。

        :param key: [输入] 回调唯一标识。
        :return: 无返回值。
        """
        super().release_error_cbk(key)
        self._waist.release_error_cbk(key)

    def register_completion_cbk(self, key: str, cbk: Callable) -> None:
        """注册任务完成回调。

        :param key: [输入] 回调唯一标识。
        :param cbk: [输入] 回调函数；参数单位由对应状态量定义。
        :return: 无返回值。
        """
        super().register_completion_cbk(key, cbk)
        self._waist.register_completion_cbk(key, cbk)

    def release_completion_cbk(self, key: str) -> None:
        """注销任务完成回调。

        :param key: [输入] 回调唯一标识。
        :return: 无返回值。
        """
        super().release_completion_cbk(key)
        self._waist.release_completion_cbk(key)

    def trajectory_teach(self, off_on: bool, name: str) -> int:
        """开始或停止机械臂示教轨迹录制。

        :param off_on: [输入] True开始录制，False停止录制。
        :param name: [输入] 示教轨迹名称。
        :return: int，1表示成功，-1表示失败。
        """
        dual_ret = super().trajectory_teach(off_on, name)
        waist_ret = self._waist.trajectory_teach(off_on, name)
        return 1 if dual_ret == 1 and waist_ret == 1 else -1

    def trajectory_teach_waist(self, off_on: bool, name: str) -> int:
        """开始或停止腰部示教轨迹录制。

        :param off_on: [输入] True开始录制，False停止录制。
        :param name: [输入] 示教轨迹名称。
        :return: int，1表示成功，-1表示失败。
        """
        return self._waist.trajectory_teach(off_on, name)

    def trajectory_recorder_waist(self, name: str, is_sync: bool = True) -> int:
        """复现腰部已录制的示教轨迹。

        :param name: [输入] 示教轨迹名称。
        :param is_sync: [输入] 是否同步等待任务完成。
        :return: int，1表示成功，-1表示失败。
        """
        return self._waist.trajectory_recorder(name, is_sync)

    def check_teach(self, left_traj_list: list, right_traj_list: list,
                    waist_traj_list: list) -> int:
        """检查机械臂示教轨迹是否存在。

        :param left_traj_list: [输出] 左臂示教轨迹名称列表。
        :param right_traj_list: [输出] 右臂示教轨迹名称列表。
        :param waist_traj_list: [输出] 腰部示教轨迹名称列表。
        :return: int，1表示成功，-1表示失败。
        """
        dual_ret = super().check_teach(left_traj_list, right_traj_list)
        waist_ret = self._waist.check_teach(waist_traj_list)
        return 1 if dual_ret == 1 and waist_ret == 1 else -1

    def get_waist_config(self) -> dict:
        """获取腰部配置。

        :return: dict，配置或状态字典；各字段沿用对应物理单位。
        """
        return self._waist.get_config()

    def get_waist_status(self) -> dict:
        """获取腰部状态。

        :return: dict，配置或状态字典；各字段沿用对应物理单位。
        """
        return self._waist.get_status()

    def get_waist_joint_pos(self) -> list:
        """获取腰部实际关节位置。

        :return: list或float，位置；单位rad。
        """
        return self._waist.get_joint_pos()

    def get_waist_joint_vel(self) -> list:
        """获取腰部实际关节速度。

        :return: list或float，速度；单位rad/s。
        """
        return self._waist.get_joint_vel()

    def get_waist_joint_tau(self) -> list:
        """获取腰部实际关节力矩。

        :return: list或float，力矩；单位Nm。
        """
        return self._waist.get_joint_tau()

    def get_waist_plan_joint_pos(self) -> list:
        """获取腰部规划关节位置。

        :return: list或float，位置；单位rad。
        """
        return self._waist.get_plan_joint_pos()

    def get_waist_plan_joint_vel(self) -> list:
        """获取腰部规划关节速度。

        :return: list或float，速度；单位rad/s。
        """
        return self._waist.get_plan_joint_vel()

    def get_waist_plan_joint_tau(self) -> list:
        """获取腰部规划关节力矩。

        :return: list或float，力矩；单位Nm。
        """
        return self._waist.get_plan_joint_tau()

    def get_waist_joint_external_tau(self) -> list:
        """获取腰部关节外力矩。

        :return: list或float，力矩；单位Nm。
        """
        return self._waist.get_joint_external_tau()

    def register_waist_joint_cbk(self, cbk: Callable) -> None:
        """注册腰部关节状态回调。

        :param cbk: [输入] 回调函数；参数单位由对应状态量定义。
        :return: 无返回值。
        """
        self._waist.register_joint_cbk(cbk)

    def release_waist_joint_cbk(self) -> None:
        """注销腰部关节状态回调。

        :return: 无返回值。
        """
        self._waist.release_joint_cbk()

    def register_waist_plan_joint_cbk(self, cbk: Callable) -> None:
        """注册腰部规划关节状态回调。

        :param cbk: [输入] 回调函数；参数单位由对应状态量定义。
        :return: 无返回值。
        """
        self._waist.register_plan_joint_cbk(cbk)

    def release_waist_plan_joint_cbk(self) -> None:
        """注销腰部规划关节状态回调。

        :return: 无返回值。
        """
        self._waist.release_plan_joint_cbk()

    def track_waist_joint(self, targets: list) -> int:
        """实时跟随腰部关节目标。

        :param targets: [输入] 目标关节位置，长度=腰部dof；单位rad。
        :return: int，1表示成功，-1表示失败。
        """
        if len(targets) != 2:
            return -1
        return self._waist.track_joint(targets)

    def move_waist_joint(self, target_pos: list, desire_time: float = -1,
                         is_sync: bool = True) -> int:
        """执行腰部关节运动。

        :param target_pos: [输入] 目标关节位置，长度=腰部dof；单位rad。
        :param desire_time: [输入] 期望运动时间；单位s；负值表示由控制器自动规划。
        :param is_sync: [输入] 是否同步等待任务完成。
        :return: int，1表示成功，-1表示失败。
        """
        if len(target_pos) != 2:
            return -1
        return self._waist.move_joint(target_pos, desire_time, is_sync)

    def move_waist_joint_traj(self, target_pos: list, stamps: list = None,
                              is_sync: bool = True) -> int:
        """执行腰部关节轨迹运动。

        :param target_pos: [输入] 目标关节位置轨迹；各轨迹点长度=腰部dof，单位rad。
        :param stamps: [输入] 各轨迹点时间戳；单位s；None表示自动规划。
        :param is_sync: [输入] 是否同步等待任务完成。
        :return: int，1表示成功，-1表示失败。
        """
        if not target_pos or any(len(point) != 2 for point in target_pos):
            return -1
        return self._waist.move_joint_traj(target_pos, [], stamps, is_sync)

    def low_waist_pv_command(self, pos: list, vel: list, data: dict) -> int:
        """发送腰部PV指令；pos[输入]单位rad，vel[输入]单位rad/s，长度=腰部dof；
        data[输出]为底层状态字典。返回1成功、-1失败。
        """
        return self._waist.low_pv_command(pos, vel, data)

    def low_waist_mit_command(self, pos: list, vel: list, tau: list,
                              kp: list, kd: list, data: dict) -> int:
        """发送腰部MIT指令。

        pos/vel/tau/kp/kd均为[输入]且长度=腰部dof，单位依次为rad、rad/s、
        Nm、Nm/rad、Nm·s/rad；data[输出]为底层状态字典。
        返回1成功、-1失败。
        """
        return self._waist.low_mit_command(pos, vel, tau, kp, kd, data)

    def low_waist_pf_command(self, pos: list, vel: list, tau: list,
                             data: dict) -> int:
        """发送腰部PF指令；pos/vel/tau[输入]单位依次为rad、rad/s、Nm，
        长度=腰部dof；data[输出]为底层状态字典。返回1成功、-1失败。
        """
        return self._waist.low_pf_command(pos, vel, tau, data)

    def low_waist_current_command(self, tau: list, data: dict) -> int:
        """发送腰部Current指令；tau[输入]为关节力矩，单位Nm、长度=腰部dof；
        data[输出]为底层状态字典。返回1成功、-1失败。
        """
        return self._waist.low_current_command(tau, data)

    def low_waist_set_safety_params(self, updates: dict, current: dict) -> int:
        """设置腰部Low Mode安全参数，支持部分更新。

        updates[输入]可包含看门狗超时(ms)、位置差上限(rad)、速度差上限(rad/s)
        和力矩/力上限(Nm)；current[输出]为四项完整当前参数。返回1成功、-1失败。
        """
        return self._waist.low_set_safety_params(updates, current)

    def low_waist_refresh(self, data: dict) -> int:
        """刷新腰部底层状态；data[输出]为最新状态字典，各字段沿用对应物理单位。
        返回1成功、-1失败。
        """
        return self._waist.low_refresh(data)

    def low_waist_set_robot_mode(self, mode: int) -> int:
        """设置腰部底层模式；mode[输入]为0-IDLE、1-PV、2-MIT、3-CURRENT、4-PF。
        返回1成功、-1失败。
        """
        return self._waist.low_set_robot_mode(mode)

    def low_waist_set_servo_enable(self, status: bool) -> int:
        """设置腰部伺服；status[输入]为True上使能、False下使能。
        返回1成功、-1失败。
        """
        return self._waist.low_set_servo_enable(status)

    def low_waist_reset(self, cnt: int = 5) -> int:
        """复位腰部底层错误；cnt[输入]为最大尝试次数，单位次。返回1成功、-1失败。"""
        return self._waist.low_reset(cnt)

    def low_waist_get_servo_status(self, status: dict) -> int:
        """获取腰部伺服状态；status[输出]包含模式、连接/使能、温度(°C)、电压(V)和错误信息。
        返回1成功、-1失败。
        """
        return self._waist.low_get_servo_status(status)
