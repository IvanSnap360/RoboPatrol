# -*- coding: utf-8 -*-

from .async_plugin_manager import AsyncPluginManager

from . import action
from . import calibration
from . import camera
from . import core
from . import failure
from . import follow_me
from . import ftp
from . import geofence
from . import gimbal
from . import info
from . import log_files
from . import manual_control
from . import mission
from . import mission_raw
from . import mocap
from . import offboard
from . import param
from . import shell
from . import telemetry
from . import tracking_server
from . import transponder
from . import tune

from . import bin


class System:
    """
    Instantiate a System object, that will serve as a proxy to
    all the MAVSDK plugins.

    Parameters
    ----------
    mavsdk_server_address: str
        Address of a running mavsdk_server instance. If None,
        an instance of mavsdk_server will be automatically
        started (on localhost).

    port: int
        Port of the running mavsdk_server instance specified by
        mavsdk_server_address.

    """
    def __init__(self, mavsdk_server_address=None, port=50051):
        self._mavsdk_server_address = mavsdk_server_address
        self._port = port

        self._plugins = {}
        self._server_process = None

    def __del__(self):
        self._stop_mavsdk_server()

    async def connect(self, system_address=None):
        """
        Connect the System object to a remote system.

        Parameters
        ----------
        system_address: str
            The address of the remote system. If None, it will
            default to udp://:14540. Supported URL formats:

                - Serial: serial:///path/to/serial/dev[:baudrate]
                - UDP: udp://[bind_host][:bind_port]
                - TCP: tcp://[server_host][:server_port]

        """

        if self._server_process is not None:
            # a mavsdk_server have already been launch by this instance:
            # --> clean all before trying to reconnect
            self._stop_mavsdk_server()

            # add a delay to be sure recourses have been freed and restart mavsdk_server
            import time; time.sleep(1)

        if self._mavsdk_server_address is None:
            self._mavsdk_server_address = 'localhost'
            self._server_process = self._start_mavsdk_server(system_address,self._port)

        await self._init_plugins(self._mavsdk_server_address, self._port)

    def _stop_mavsdk_server(self):
        """
        kill the running mavsdk_server and clean the whole instance
        """
        import subprocess
        if isinstance(self._server_process,subprocess.Popen):
            self._server_process.kill()
            self.__init__(port = self._port)

    async def _init_plugins(self, host, port):
        plugin_manager = await AsyncPluginManager.create(host=host, port=port)

        self._plugins = {}
        self._plugins["action"] = action.Action(plugin_manager)
        self._plugins["calibration"] = calibration.Calibration(plugin_manager)
        self._plugins["camera"] = camera.Camera(plugin_manager)
        self._plugins["core"] = core.Core(plugin_manager)
        self._plugins["failure"] = failure.Failure(plugin_manager)
        self._plugins["follow_me"] = follow_me.FollowMe(plugin_manager)
        self._plugins["ftp"] = ftp.Ftp(plugin_manager)
        self._plugins["geofence"] = geofence.Geofence(plugin_manager)
        self._plugins["gimbal"] = gimbal.Gimbal(plugin_manager)
        self._plugins["info"] = info.Info(plugin_manager)
        self._plugins["log_files"] = log_files.LogFiles(plugin_manager)
        self._plugins["manual_control"] = manual_control.ManualControl(plugin_manager)
        self._plugins["mission"] = mission.Mission(plugin_manager)
        self._plugins["mission_raw"] = mission_raw.MissionRaw(plugin_manager)
        self._plugins["mocap"] = mocap.Mocap(plugin_manager)
        self._plugins["offboard"] = offboard.Offboard(plugin_manager)
        self._plugins["param"] = param.Param(plugin_manager)
        self._plugins["shell"] = shell.Shell(plugin_manager)
        self._plugins["telemetry"] = telemetry.Telemetry(plugin_manager)
        self._plugins["tracking_server"] = tracking_server.TrackingServer(plugin_manager)
        self._plugins["transponder"] = transponder.Transponder(plugin_manager)
        self._plugins["tune"] = tune.Tune(plugin_manager)

    @staticmethod
    def error_uninitialized(plugin_name: str) -> str:
        return "{plugin_name} plugin has not been initialized!" \
            "Did you run `System.connect()`?"

    @property
    def action(self) -> action.Action:
        if "action" not in self._plugins:
            raise RuntimeError(self.error_uninitialized("Action"))
        return self._plugins["action"]

    @property
    def calibration(self) -> calibration.Calibration:
        if "calibration" not in self._plugins:
            raise RuntimeError(self.error_uninitialized("Calibration"))
        return self._plugins["calibration"]

    @property
    def camera(self) -> camera.Camera:
        if "camera" not in self._plugins:
            raise RuntimeError(self.error_uninitialized("Camera"))
        return self._plugins["camera"]

    @property
    def core(self) -> core.Core:
        if "core" not in self._plugins:
            raise RuntimeError(self.error_uninitialized("Core"))
        return self._plugins["core"]

    @property
    def failure(self) -> failure.Failure:
        if "failure" not in self._plugins:
            raise RuntimeError(self.error_uninitialized("Failure"))
        return self._plugins["failure"]
    
    @property
    def follow_me(self) -> follow_me.FollowMe:
        if "follow_me" not in self._plugins:
            raise RuntimeError(self.error_uninitialized("FollowMe"))
        return self._plugins["follow_me"]

    @property
    def ftp(self) -> ftp.Ftp:
        if "ftp" not in self._plugins:
            raise RuntimeError(self.error_uninitialized("Ftp"))
        return self._plugins["ftp"]

    @property
    def geofence(self) -> geofence.Geofence:
        if "geofence" not in self._plugins:
            raise RuntimeError(self.error_uninitialized("Geofence"))
        return self._plugins["geofence"]

    @property
    def gimbal(self) -> gimbal.Gimbal:
        if "gimbal" not in self._plugins:
            raise RuntimeError(self.error_uninitialized("Gimbal"))
        return self._plugins["gimbal"]

    @property
    def info(self) -> info.Info:
        if "info" not in self._plugins:
            raise RuntimeError(self.error_uninitialized("Info"))
        return self._plugins["info"]

    @property
    def log_files(self) -> log_files.LogFiles:
        if "log_files" not in self._plugins:
            raise RuntimeError(self.error_uninitialized("LogFiles"))
        return self._plugins["log_files"]

    @property
    def manual_control(self) -> manual_control.ManualControl:
        if "manual_control" not in self._plugins:
            raise RuntimeError(self.error_uninitialized("ManualControl"))
        return self._plugins["manual_control"]

    @property
    def mission(self) -> mission.Mission:
        if "mission" not in self._plugins:
            raise RuntimeError(self.error_uninitialized("Mission"))
        return self._plugins["mission"]

    @property
    def mission_raw(self) -> mission_raw.MissionRaw:
        if "mission_raw" not in self._plugins:
            raise RuntimeError(self.error_uninitialized("MissionRaw"))
        return self._plugins["mission_raw"]

    @property
    def mocap(self) -> mocap.Mocap:
        if "mocap" not in self._plugins:
            raise RuntimeError(self.error_uninitialized("Mocap"))
        return self._plugins["mocap"]

    @property
    def offboard(self) -> offboard.Offboard:
        if "offboard" not in self._plugins:
            raise RuntimeError(self.error_uninitialized("Offboard"))
        return self._plugins["offboard"]

    @property
    def param(self) -> param.Param:
        if "param" not in self._plugins:
            raise RuntimeError(self.error_uninitialized("Param"))
        return self._plugins["param"]

    @property
    def shell(self) -> shell.Shell:
        if "shell" not in self._plugins:
            raise RuntimeError(self.error_uninitialized("Shell"))
        return self._plugins["shell"]

    @property
    def telemetry(self) -> telemetry.Telemetry:
        if "telemetry" not in self._plugins:
            raise RuntimeError(self.error_uninitialized("Telemetry"))
        return self._plugins["telemetry"]

    @property
    def tracking_server(self) -> tracking_server.TrackingServer:
        if "tracking_server" not in self._plugins:
            raise RuntimeError(self.error_uninitialized("TrackingServer"))
        return self._plugins["tracking_server"]

    @property
    def transponder(self) -> transponder.Transponder:
        if "transponder" not in self._plugins:
            raise RuntimeError(self.error_uninitialized("Transponder"))
        return self._plugins["transponder"]

    @property
    def tune(self) -> tune.Tune:
        if "tune" not in self._plugins:
            raise RuntimeError(self.error_uninitialized("Tune"))
        return self._plugins["tune"]

    @staticmethod
    def _start_mavsdk_server(system_address=None,port=50051):
        """
        Starts the gRPC server in a subprocess, listening on localhost:port
        port parameter can be specified now to allow multiple mavsdk servers to be spawned via code
        """
        import atexit
        import os
        import subprocess
        import sys

        if sys.version_info >= (3, 7):
            from importlib.resources import path
        else:
            from importlib_resources import path

        try:
            with path(bin, 'mavsdk_server') as backend:
                bin_path_and_args = [os.fspath(backend), "-p", str(port)]
                if system_address:
                    bin_path_and_args.append(system_address)
                p = subprocess.Popen(bin_path_and_args,
                                     shell=False,
                                     stdout=subprocess.DEVNULL,
                                     stderr=subprocess.DEVNULL)
        except FileNotFoundError:
            print("""
This installation does not provide an embedded 'mavsdk_server' binary.
If you installed using pip, this means that 'mavsdk_server' is not distributed
for your platform yet (e.g. arm).

You will need to get and run the 'mavsdk_server' binary manually:

  1. Download 'mavsdk_server' from: https://github.com/mavlink/mavsdk/releases
     or build it from source.

  2. Run it, e.g. on port 50051:
     './mavsdk_server -p 50051'

  3. Set the 'mavsdk_server_address' and port when creating the System:
     'drone = System(mavsdk_server_address='localhost', port=50051)'
""")
            sys.exit(1)

        def cleanup():
            p.kill()

        atexit.register(cleanup)

        return p
