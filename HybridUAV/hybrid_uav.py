#!/usr/bin/env python

"""
Hybrid UAV SITL test script for ArduPilot autotest framework.
Tests basic functionality of a hybrid quadplane with power generation.
"""
from __future__ import print_function
import os
import sys

script_dir = os.path.dirname(os.path.realpath(__file__))
ardupilot_dir = os.path.join(script_dir, '..', '..')
sys.path.insert(0, os.path.join(ardupilot_dir, 'Tools', 'autotest'))

import os
import time

from pymavlink import mavutil
from array import array

from common import AutoTest
from common import NotAchievedException
from common import AutoTestTimeoutException

class HybridUAVTest(AutoTest):
    def __init__(self):
        super(HybridUAVTest, self).__init__()
        self.frame = 'quadplane'
        self.init_vehicle_test_variables()

    def init_vehicle_test_variables(self):
        """Initialize test-specific variables."""
        self.homeloc = None
        self.distance_to_waypoint = 0
        self.last_wp_distance = 0
        self.last_wp_time = 0
        self.timeout = 300  # 5 minutes timeout for tests

    def default_frame_params(self):
        """Default parameters specific to hybrid frame"""
        return {
            "Q_ENABLE": 1,
            "Q_TILT_TYPE": 0,
            "ENG_RUN": 1,
            "ENG_OUTPUT": 800,
            "ENG_PWR_GEN_ENABLE": 1,
            "BATT_MONITOR": 4
        }

    def test_power_generation(self):
        """Test power generation system during hover"""
        self.progress("Testing power generation system")
        self.change_mode('QLOITER')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        
        # Take off to test height
        target_alt = 10
        self.set_rc(3, 1500)  # Mid throttle
        self.wait_altitude(target_alt - 1,
                         target_alt + 1,
                         relative=True)
        
        # Check engine parameters
        eng_output = self.get_parameter("ENG_OUTPUT")
        if eng_output < 500:
            raise NotAchievedException("Engine output too low")
            
        # Monitor battery voltage during hover
        start_voltage = self.get_battery_voltage()
        self.progress(f"Initial battery voltage: {start_voltage}")
        
        # Hover for 60 seconds while monitoring power
        tstart = self.get_sim_time()
        while self.get_sim_time_cached() < tstart + 60:
            current_voltage = self.get_battery_voltage()
            if current_voltage < start_voltage - 0.5:
                raise NotAchievedException("Power generation not maintaining voltage")
            time.sleep(1)
            
        self.progress("Power generation test successful")
        
        # Land and disarm
        self.change_mode('QLAND')
        self.wait_disarmed()
        
    def test_mission(self):
        """Test basic mission capabilities"""
        self.progress("Testing mission execution")
        
        # Upload test mission
        self.load_mission("hybrid_test_mission.txt")
        self.change_mode('AUTO')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        
        # Monitor mission progress
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time_cached() - tstart > self.timeout:
                raise AutoTestTimeoutException("Mission timeout")
                
            m = self.mav.recv_match(type=['MISSION_CURRENT','VFR_HUD'],
                                  blocking=True)
            if m is None:
                continue
                
            if m.get_type() == 'MISSION_CURRENT':
                seq = m.seq
                if seq == self.mission.num_commands()-1:
                    break
        
        self.progress("Mission complete")
        self.wait_disarmed()

    def tests(self):
        """Return list of all tests"""
        ret = super(HybridUAVTest, self).tests()
        ret.extend([
            ("TestPowerGeneration", "Test power generation system", self.test_power_generation),
            ("TestMission", "Test basic mission capabilities", self.test_mission),
        ])
        return ret

def generate_frame_file():
    """Generate frame configuration file for SITL"""
    return '''<?xml version="1.0"?>
<frame name="hybrid_uav" class="copter">
  <description>Hybrid UAV with power generation</description>
  <version>1</version>
  <default_params>
    <param name="MOT_PWM_MIN" value="1000" />
    <param name="MOT_PWM_MAX" value="2000" />
    <param name="MOT_SPIN_ARM" value="0.1" />
    <param name="MOT_SPIN_MIN" value="0.15" />
    <param name="MOT_SPIN_MAX" value="0.95" />
  </default_params>
</frame>
'''

if __name__ == "__main__":
    test = HybridUAVTest()
    test.run_tests()