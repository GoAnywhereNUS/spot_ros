from sensor_msgs.msg import Joy
from bosdyn.api.spot import robot_command_pb2 as spot_command_pb2
import time

class TeleopFuncs:
    """
    Handles commands to execute discrete services (e.g. power-on/sit/stand 
    etc.) from joystick commands. Complementary to teleop_twist_joy, which 
    handles control (velocity) commands.
    """

    def __init__(
        self, 
        spot_wrapper, 
        movement_query_fn,
        refractory_period_secs=0.75,
    ) -> None:
        self.spot_wrapper = spot_wrapper
        self.movement_query_fn = movement_query_fn
        self.refractory_period_secs = refractory_period_secs
        self.time_since_last_trigger_secs = time.time()

        self.enable = False
        self.toggle_power = False
        self.toggle_sit_stand = False
        self.toggle_locomotion_mode = False
        self.toggle_stairs_mode = False

        self.valid_locomotion_hints = [
            (spot_command_pb2.LocomotionHint.HINT_AUTO, "AUTO"),
            (spot_command_pb2.LocomotionHint.HINT_TROT, "TROT"),
            (spot_command_pb2.LocomotionHint.HINT_SPEED_SELECT_TROT, "TROT WITH STOP"),
            (spot_command_pb2.LocomotionHint.HINT_CRAWL, "CRAWL"),
            (spot_command_pb2.LocomotionHint.HINT_SPEED_SELECT_CRAWL, "CRAWL WITH STOP"),
            (spot_command_pb2.LocomotionHint.HINT_AMBLE, "AMBLE"),
            (spot_command_pb2.LocomotionHint.HINT_SPEED_SELECT_AMBLE, "AMBLE WITH STOP"),
        ]
        self.locomotion_mode_idx = 0

        self.valid_stair_hints = [
            (spot_command_pb2.StairsMode.STAIRS_MODE_OFF, "OFF"),
            (spot_command_pb2.StairsMode.STAIRS_MODE_ON,  "ON"),
            (spot_command_pb2.StairsMode.STAIRS_MODE_AUTO, "AUTOSELECT"),
        ]
        self.stair_mode_idx = 0

    def handle_joy(self, joy_msg):
        self.enable = (joy_msg.buttons[4] == 1 and joy_msg.buttons[5] == 1)

        if self.enable:
            # Check requests that will trigger robot actions with refractory period.
            if time.time() - self.time_since_last_trigger_secs > self.refractory_period_secs:
                self.toggle_power = self.buttons[6] == 1 # Power on/off request
                self.toggle_sit_stand = self.buttons[7] == 1 # Sit/stand request

            # Check all other requests with low latency.
            self.toggle_locomotion_mode = self.buttons[9] == 1 # Locomotion mode change request
            self.toggle_stairs_mode = self.buttons[10] == 1 # Stairs mode change request

            # Handle all mode change requests
            if self.toggle_power:
                self._handle_toggle_power()
            if self.toggle_sit_stand:
                self._handle_toggle_sit_stand()
            if self.toggle_locomotion_mode:
                self._handle_toggle_locomotion_mode()
            if self.toggle_stairs_mode:
                self._handle_toggle_stairs_mode()

    def _handle_toggle_power(self):
        if self.spot_wrapper.check_is_powered_on():
            resp = self.spot_wrapper.safe_power_off()
            print("ON --> OFF", resp[0], resp[1])
        else:
            resp = self.spot_wrapper.power_on()
            print("OFF --> ON", resp[0], resp[1])

    def _handle_toggle_sit_stand(self):
        if not self.movement_query_fn(autonomous_command=False):
            return "Not changing sit/stand. Robot motion not allowed!"
        
        # We check if it is sitting first. There can be occasions
        # where it is registered as both in sitting and standing
        # states by the wrapper. In these ambiguous situations, it
        # is safer to try and stand (from a known sitting position,
        # or a standing position) than to try and sit into a position
        # of unknown stability.
        if self.spot_wrapper.is_sitting():
            resp = self.spot_wrapper.stand()
            print("SIT --> STAND:", resp[0], resp[1])
        else:
            resp = self.spot_wrapper.sit()
            print("STAND --> SIT", resp[0], resp[1])

    def _handle_toggle_locomotion_mode(self):
        self.locomotion_mode_idx = (self.locomotion_mode_idx + 1) % len(self.valid_locomotion_hints)
        locomotion_hint, msg = self.valid_locomotion_hints[self.locomotion_mode_idx]
        try:
            mobility_params = self.spot_wrapper.get_mobility_params()
            mobility_params.locomotion_hint = locomotion_hint
            self.spot_wrapper.set_mobility_params(mobility_params)
            print("Set locomotion mode to: ", msg)
        except Exception as e:
            print("Error setting locomotion mode:{}".format(e))

    def _handle_toggle_stairs_mode(self):
        self.stair_mode_idx = (self.stair_mode_idx + 1) % len(self.valid_stair_hints)
        stair_hint, msg = self.valid_stair_hints[self.stair_mode_idx]
        try:
            mobility_params = self.spot_wrapper.get_mobility_params()
            mobility_params.stair_hint = stair_hint
            self.spot_wrapper.set_mobility_params(mobility_params)
            print("Set stair mode to: ", msg)
        except Exception as e:
            print("Error setting stair mode:{}".format(e))