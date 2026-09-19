import math
import unittest
from types import SimpleNamespace as NS
from unittest.mock import patch
from xml.etree import ElementTree as ET

import carla
import lane_offset_motion as motion


class LaneOffsetMotionTest(unittest.TestCase):
    def test_zero_relative_station_does_not_advance_carla_waypoint(self):
        from srunner.tools.openscenario_parser import OpenScenarioParser
        def disallowed(distance):
            raise AssertionError('zero station offset must not call waypoint.next/previous')
        waypoint=NS(road_id=1,lane_id=-1,s=50,transform=carla.Transform(carla.Location(x=50,y=1.75)),
                    next=disallowed,previous=disallowed)
        roadmap=NS(get_waypoint=lambda _:waypoint,get_waypoint_xodr=lambda *args:waypoint)
        position=ET.fromstring('<Position><RelativeLanePosition entityRef="reference" dLane="0" ds="0" offset="-3.5"/></Position>')
        reference=NS(rolename='reference',transform=carla.Transform())
        with patch.object(motion.CarlaDataProvider,'get_map',return_value=roadmap):
            transform=OpenScenarioParser.convert_position_to_transform(position,[reference])
        self.assertAlmostEqual(transform.location.x,50)
        self.assertAlmostEqual(transform.location.y,5.25)

    def test_sinusoid_starts_and_ends_without_jumps(self):
        start, target, acc = 0.3, -1.4, 0.8
        duration=math.pi*math.sqrt(abs(target-start)/(2*acc))
        self.assertEqual(motion.sinusoidal_offset(start,target,acc,0),start)
        self.assertAlmostEqual(motion.sinusoidal_offset(start,target,acc,duration),target)
        self.assertAlmostEqual(motion.sinusoidal_offset(start,target,acc,100),target)
        self.assertAlmostEqual(motion.sinusoidal_offset(start,target,acc,duration/2),(target+start)/2)
        step=1e-4
        for i in range(1,100):
            t=duration*i/100
            actual=(motion.sinusoidal_offset(start,target,acc,t+step)-2*motion.sinusoidal_offset(start,target,acc,t)+motion.sinusoidal_offset(start,target,acc,t-step))/(step*step)
            self.assertLessEqual(abs(actual),acc+1e-6)

    def test_invalid_acceleration_and_noncontinuous_motion_fail(self):
        with self.assertRaises(ValueError):motion.sinusoidal_offset(0,1,0,1)
        with self.assertRaises(ValueError):motion.sinusoidal_offset(0,float('nan'),1,1)
        with self.assertRaises(ValueError):motion.SinusoidalLaneOffset(None,1,1,continuous=False)

    def test_xodr_offset_sign_matches_both_lane_directions(self):
        actor=NS(get_location=lambda:carla.Location())
        for lane,target,expected in [(-2,1.4,-1.4),(2,-1.4,-1.4),(-1,-1.4,1.4),(1,1.4,1.4)]:
            roadmap=NS(get_waypoint=lambda loc:NS(lane_id=lane))
            with patch.object(motion.CarlaDataProvider,'get_map',return_value=roadmap), patch.object(motion.ChangeActorLaneOffset,'__init__',return_value=None) as init:
                motion.SinusoidalLaneOffset(actor,target,.8)
                self.assertAlmostEqual(init.call_args.args[1],expected)


if __name__=='__main__':
    unittest.main()
