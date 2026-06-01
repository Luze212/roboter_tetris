"""Minimal Robotiq 2F-series gripper driver over Modbus RTU (USB/RS-485).

Vendored, dependency-light replacement for the subset of the ``pyrobotiqgripper``
API used by this package. Implemented directly on ``pymodbus`` + ``pyserial`` to
avoid that library's hard dependency on ``numpy>=2.0`` (which conflicts with the
ROS image's apt-installed numpy 1.26.4 and cannot be overridden through the AICA
build) and its desktop-GUI dependencies (pygame/pyautogui/pynput).

Only the methods consumed by :mod:`roboter_tetris.robotiq_gripper` are provided,
with matching names/signatures so the component code stays unchanged.

Reference: Robotiq 2F-85 / 2F-140 Instruction Manual, "Read & write registers"
(Modbus RTU). Robot-output (command) registers start at 0x03E8, robot-input
(status) registers at 0x07D0; each block is three 16-bit registers (six bytes).
"""

import time
from typing import Dict, List, Optional

from pymodbus.client import ModbusSerialClient
from serial.tools import list_ports

# --- Modbus layout (Robotiq 2F) ---
_SLAVE_ID = 9
_CMD_ADDR = 0x03E8        # robot output / functional registers (write)
_STATUS_ADDR = 0x07D0     # robot input / status registers (read)
_NUM_REGS = 3

# Action-request bits (high byte of the first command register)
_ACT = 0x01               # rACT: activation bit (must stay set once activated)
_GTO = 0x08               # rGTO: "go to" the requested position

_STA_ACTIVATION_COMPLETE = 0x03   # gSTA
_OBJ_IN_MOTION = 0x00             # gOBJ: fingers still moving


class RobotiqConnectionError(RuntimeError):
    """Raised on serial/Modbus failures or when no gripper responds."""


class RobotiqGripper:
    """Subset-compatible Robotiq 2F driver (Modbus RTU over a serial port)."""

    def __init__(
        self,
        com_port: str = "auto",
        slave_id: int = _SLAVE_ID,
        baudrate: int = 115200,
        timeout: float = 1.0,
    ) -> None:
        self._com_port = com_port
        self._slave = slave_id
        self._baudrate = baudrate
        self._timeout = timeout
        self._client: Optional[ModbusSerialClient] = None
        # Bit calibration: nominal full range until measured by calibrate_bit().
        self._open_bit = 0
        self._close_bit = 255
        # Linear bit<->mm mapping, set by calibrate_mm().
        self._a_coef: Optional[float] = None
        self._b_coef: Optional[float] = None

    # -- connection -----------------------------------------------------------

    def connect(self) -> None:
        port = self._autodetect_port() if self._com_port == "auto" else self._com_port
        client = self._make_client(port)
        if not client.connect():
            raise RobotiqConnectionError(f"Could not open serial port {port}")
        self._client = client
        if self._read_status_registers() is None:
            client.close()
            self._client = None
            raise RobotiqConnectionError(f"No Robotiq gripper responding on {port}")

    def disconnect(self) -> None:
        if self._client is not None:
            self._client.close()
            self._client = None

    def _make_client(self, port: str) -> ModbusSerialClient:
        return ModbusSerialClient(
            port=port,
            baudrate=self._baudrate,
            bytesize=8,
            parity="N",
            stopbits=1,
            timeout=self._timeout,
        )

    def _autodetect_port(self) -> str:
        for info in list_ports.comports():
            client = self._make_client(info.device)
            try:
                if not client.connect():
                    continue
                rr = client.read_holding_registers(
                    address=_STATUS_ADDR, count=_NUM_REGS, slave=self._slave
                )
                if rr is not None and not rr.isError():
                    return info.device
            except Exception:
                continue
            finally:
                client.close()
        raise RobotiqConnectionError("No Robotiq gripper found on any serial port")

    # -- low-level Modbus -----------------------------------------------------

    def _write_command(self, action: int, position: int, speed: int, force: int) -> None:
        if self._client is None:
            raise RobotiqConnectionError("Gripper is not connected")
        reg0 = (action & 0xFF) << 8                       # [action][reserved]
        reg1 = position & 0xFF                            # [reserved][rPR]
        reg2 = ((speed & 0xFF) << 8) | (force & 0xFF)     # [rSP][rFR]
        rq = self._client.write_registers(
            address=_CMD_ADDR, values=[reg0, reg1, reg2], slave=self._slave
        )
        if rq is None or rq.isError():
            raise RobotiqConnectionError("Modbus write to gripper failed")

    def _read_status_registers(self) -> Optional[List[int]]:
        if self._client is None:
            return None
        rr = self._client.read_holding_registers(
            address=_STATUS_ADDR, count=_NUM_REGS, slave=self._slave
        )
        if rr is None or rr.isError():
            return None
        return list(rr.registers)

    # -- status ---------------------------------------------------------------

    def status(self, refreshStatus: bool = True) -> Dict[str, int]:
        regs = self._read_status_registers()
        if regs is None:
            raise RobotiqConnectionError("Could not read gripper status")
        status_byte = (regs[0] >> 8) & 0xFF
        fault_byte = (regs[1] >> 8) & 0xFF
        return {
            "gACT": status_byte & 0x01,
            "gGTO": (status_byte >> 3) & 0x01,
            "gSTA": (status_byte >> 4) & 0x03,
            "gOBJ": (status_byte >> 6) & 0x03,
            "gFLT": fault_byte & 0x0F,
            "gPR": regs[1] & 0xFF,          # requested-position echo
            "gPO": (regs[2] >> 8) & 0xFF,   # actual position
            "gCU": regs[2] & 0xFF,          # motor current
        }

    def position(self) -> int:
        return self.status()["gPO"]

    # -- activation -----------------------------------------------------------

    def activate(self, timeout: float = 10.0) -> None:
        """Clear, then set the activation bit and wait until activation completes.

        The gripper performs its internal open/close reference run during
        activation, so the fingers must be free to move.
        """
        self._write_command(0x00, 0, 0, 0)   # clear rACT
        time.sleep(0.1)
        self._write_command(_ACT, 0, 0, 0)   # set rACT
        start = time.monotonic()
        while time.monotonic() - start < timeout:
            st = self.status()
            if st["gACT"] == 1 and st["gSTA"] == _STA_ACTIVATION_COMPLETE:
                return
            time.sleep(0.1)
        raise RobotiqConnectionError("Gripper activation timed out")

    def stop(self) -> None:
        """Hold the current position (stay activated, drop the GTO bit)."""
        self._write_command(_ACT, 0, 0, 0)

    # -- motion ---------------------------------------------------------------

    def move(
        self,
        position: int,
        speed: int = 255,
        force: int = 255,
        wait: bool = True,
        timeout: float = 10.0,
    ) -> None:
        position = max(0, min(255, int(position)))
        speed = max(0, min(255, int(speed)))
        force = max(0, min(255, int(force)))
        self._write_command(_ACT | _GTO, position, speed, force)
        if wait:
            self._wait_motion_complete(timeout)

    def move_mm(
        self,
        positionmm: float,
        speed: int = 255,
        force: int = 255,
        wait: bool = True,
        timeout: float = 10.0,
    ) -> None:
        if self._a_coef is None or self._b_coef is None:
            raise RobotiqConnectionError("Gripper is not mm-calibrated")
        bit = round(self._a_coef * float(positionmm) + self._b_coef)
        self.move(bit, speed, force, wait=wait, timeout=timeout)

    def _wait_motion_complete(self, timeout: float) -> None:
        start = time.monotonic()
        time.sleep(0.05)  # let the motion start before sampling gOBJ
        while time.monotonic() - start < timeout:
            if self.status()["gOBJ"] != _OBJ_IN_MOTION:
                return
            time.sleep(0.05)

    # -- calibration ----------------------------------------------------------

    def calibrate_bit(
        self, openbit: Optional[int] = None, closebit: Optional[int] = None
    ) -> None:
        """Record the actual position-register values at the open/close extremes.

        With explicit values, just store them. Otherwise measure them by driving
        the gripper fully open, then fully closed (gentle force).
        """
        if openbit is not None and closebit is not None:
            self._open_bit, self._close_bit = openbit, closebit
            return
        self.move(0, speed=255, force=0, wait=True)
        self._open_bit = self.status()["gPO"]
        self.move(255, speed=255, force=0, wait=True)
        self._close_bit = self.status()["gPO"]

    def calibrate_mm(self, closemm: float, openmm: float) -> None:
        """Define the linear bit<->mm mapping from the bit-calibration extremes."""
        if self._open_bit == self._close_bit:
            raise RobotiqConnectionError("Invalid bit calibration (open == close)")
        self._a_coef = (self._close_bit - self._open_bit) / (closemm - openmm)
        self._b_coef = self._open_bit - self._a_coef * openmm
