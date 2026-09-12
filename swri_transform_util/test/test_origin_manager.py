#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2026, Southwest Research Institute (SwRI)
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of Southwest Research Institute (SwRI) nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL SOUTHWEST RESEARCH INSTITUTE BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""Unit tests for the origin distance diagnostic in OriginManager."""

import math

from diagnostic_msgs.msg import DiagnosticStatus
from gps_msgs.msg import GPSFix, GPSStatus
import pytest
import rclpy
from sensor_msgs.msg import NavSatFix, NavSatStatus

from swri_transform_util.origin_manager import (DEFAULT_MAX_ORIGIN_DISTANCE,
                                                EARTH_MEAN_RADIUS,
                                                OriginManager,
                                                planar_distance)

SWRI = {'latitude': 29.45196669, 'longitude': -98.61370577, 'altitude': 233.719}

# One degree of latitude along a sphere of the earth's mean radius.
ONE_DEGREE = EARTH_MEAN_RADIUS * math.pi / 180.0


@pytest.fixture
def manager(request):
    """Provide an OriginManager on a real node, with publishing captured."""
    rclpy.init()
    node = rclpy.create_node('test_origin_manager')
    max_distance = getattr(request, 'param', DEFAULT_MAX_ORIGIN_DISTANCE)
    mgr = OriginManager(node, 'map', max_origin_distance=max_distance)
    # Capture what would be published rather than spinning, so that the
    # diagnostic can be inspected synchronously.
    mgr.published = []
    mgr.diagnostic_pub.publish = mgr.published.append
    yield mgr
    node.destroy_node()
    rclpy.shutdown()


def status_of(mgr):
    """Publish one diagnostic and return the single status in it."""
    mgr.published.clear()
    mgr._publish_diagnostic()
    assert len(mgr.published) == 1
    assert len(mgr.published[0].status) == 1
    return mgr.published[0].status[0]


def values_of(status):
    return {kv.key: kv.value for kv in status.values}


def test_distance_between_identical_points_is_zero():
    assert planar_distance(SWRI['latitude'], SWRI['longitude'],
                             SWRI['latitude'], SWRI['longitude']) == pytest.approx(0.0)


def test_one_degree_of_latitude():
    # A degree of latitude is the same length at any longitude.
    assert planar_distance(0.0, 0.0, 1.0, 0.0) == pytest.approx(ONE_DEGREE, abs=1.0)
    assert planar_distance(29.0, -98.0, 30.0, -98.0) == pytest.approx(ONE_DEGREE, abs=1.0)


def test_one_degree_of_longitude_shrinks_away_from_the_equator():
    # A degree of longitude is a full degree at the equator and shrinks by
    # cos(latitude) as you move away from it.
    assert planar_distance(0.0, 0.0, 0.0, 1.0) == pytest.approx(ONE_DEGREE, abs=1.0)
    assert planar_distance(60.0, 0.0, 60.0, 1.0) == pytest.approx(ONE_DEGREE / 2.0, rel=1e-3)


def test_distance_is_symmetric():
    forward = planar_distance(29.0, -98.0, 29.1, -98.2)
    backward = planar_distance(29.1, -98.2, 29.0, -98.0)
    assert forward == pytest.approx(backward)


def test_error_reported_when_there_is_no_origin(manager):
    # Nothing is set on the manager, so this is the state the node is in while
    # it waits for its first fix.
    status = status_of(manager)
    assert status.name == 'LocalXY Origin'
    assert status.level == DiagnosticStatus.ERROR
    assert status.message == 'No Origin'
    assert status.values == []


def test_no_distance_reported_until_a_position_is_known(manager):
    manager.set_origin('manual', SWRI['latitude'], SWRI['longitude'], SWRI['altitude'])
    assert 'Distance From Origin' not in values_of(status_of(manager))


def test_distance_reported_while_within_the_limit(manager):
    manager.set_origin('navsat', SWRI['latitude'], SWRI['longitude'], SWRI['altitude'])
    # Roughly 1.1 km north of the origin, well inside the default 10 km.
    manager.update_current_position(SWRI['latitude'] + 0.01, SWRI['longitude'])

    status = status_of(manager)
    values = values_of(status)
    assert status.level == DiagnosticStatus.OK
    assert float(values['Distance From Origin']) == pytest.approx(1111.9, abs=5.0)
    assert float(values['Max Distance From Origin']) == DEFAULT_MAX_ORIGIN_DISTANCE
    assert 'farther than' not in status.message


def test_warns_once_past_the_limit(manager):
    manager.set_origin('navsat', SWRI['latitude'], SWRI['longitude'], SWRI['altitude'])
    # Half a degree of latitude is about 55 km, well past the default limit.
    manager.update_current_position(SWRI['latitude'] + 0.5, SWRI['longitude'])

    status = status_of(manager)
    assert status.level == DiagnosticStatus.WARN
    assert 'farther than' in status.message
    assert float(values_of(status)['Distance From Origin']) == pytest.approx(55597.0, abs=100.0)


def test_a_manual_origin_stays_at_least_warn_when_far(manager):
    # A manual origin is already WARN, so the distance must not quietly
    # downgrade it, and the message should still explain the distance.
    manager.set_origin('manual', SWRI['latitude'], SWRI['longitude'], SWRI['altitude'])
    manager.update_current_position(SWRI['latitude'] + 0.5, SWRI['longitude'])

    status = status_of(manager)
    assert status.level == DiagnosticStatus.WARN
    assert 'Origin Was Set Manually' in status.message
    assert 'farther than' in status.message


@pytest.mark.parametrize('manager', [100000.0], indirect=True)
def test_the_limit_is_configurable(manager):
    manager.set_origin('navsat', SWRI['latitude'], SWRI['longitude'], SWRI['altitude'])
    # 55 km would trip the 10 km default but is inside this 100 km limit.
    manager.update_current_position(SWRI['latitude'] + 0.5, SWRI['longitude'])

    status = status_of(manager)
    assert status.level == DiagnosticStatus.OK
    assert 'farther than' not in status.message
    assert float(values_of(status)['Max Distance From Origin']) == 100000.0


def test_position_updates_track_the_latest_fix(manager):
    manager.set_origin('navsat', SWRI['latitude'], SWRI['longitude'], SWRI['altitude'])
    manager.update_current_position(SWRI['latitude'] + 0.5, SWRI['longitude'])
    assert status_of(manager).level == DiagnosticStatus.WARN

    # Coming back inside the limit clears the warning.
    manager.update_current_position(SWRI['latitude'], SWRI['longitude'])
    assert status_of(manager).level == DiagnosticStatus.OK


def test_invalid_fixes_do_not_update_the_position(manager):
    manager.set_origin('navsat', SWRI['latitude'], SWRI['longitude'], SWRI['altitude'])

    gps = GPSFix()
    gps.status.status = GPSStatus.STATUS_NO_FIX
    gps.latitude = SWRI['latitude'] + 0.5
    gps.longitude = SWRI['longitude']
    manager.update_current_position_from_gps(gps)

    navsat = NavSatFix()
    navsat.status.status = NavSatStatus.STATUS_NO_FIX
    navsat.latitude = SWRI['latitude'] + 0.5
    navsat.longitude = SWRI['longitude']
    manager.update_current_position_from_navsat(navsat)

    assert manager.current_position is None
    assert 'Distance From Origin' not in values_of(status_of(manager))


def test_valid_fixes_update_the_position(manager):
    manager.set_origin('navsat', SWRI['latitude'], SWRI['longitude'], SWRI['altitude'])

    gps = GPSFix()
    gps.status.status = GPSStatus.STATUS_FIX
    gps.latitude = SWRI['latitude'] + 0.5
    gps.longitude = SWRI['longitude']
    manager.update_current_position_from_gps(gps)
    assert status_of(manager).level == DiagnosticStatus.WARN

    navsat = NavSatFix()
    navsat.status.status = NavSatStatus.STATUS_FIX
    navsat.latitude = SWRI['latitude']
    navsat.longitude = SWRI['longitude']
    manager.update_current_position_from_navsat(navsat)
    assert status_of(manager).level == DiagnosticStatus.OK
