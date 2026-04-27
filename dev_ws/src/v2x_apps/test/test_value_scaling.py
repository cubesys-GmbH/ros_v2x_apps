import math

import pytest

import etsi_its_cpm_ts_msgs.msg as cpm_msg
import etsi_its_vam_ts_msgs.msg as vam_msg

from v2x_apps.cpm_provider import (
    CpmAltitudeValue,
    CpmLatitudeValue,
    CpmLongitudeValue,
    CpmSemiAxisLengthValue,
    CpmValue,
)
from v2x_apps.vam_provider import (
    VamAltitudeValue,
    VamLatitudeValue,
    VamLongitudeValue,
    VamSemiAxisLengthValue,
    VamValue,
)


# CpmValue and VamValue are independent classes with the same contract,
# so the base-class behaviour is exercised against both.
@pytest.mark.parametrize('cls', [CpmValue, VamValue])
class TestScaledValue:
    def test_finite_in_range_is_scaled_and_rounded(self, cls):
        v = cls(1.5, scale=10.0, unavailable=99)
        v.range(0, 100)
        assert v.get() == 15

    def test_value_above_max_returns_out_of_range(self, cls):
        v = cls(50.0, scale=1.0, unavailable=99)
        v.range(0, 10, out_of_range=42)
        assert v.get() == 42

    def test_value_below_min_returns_out_of_range(self, cls):
        v = cls(-50.0, scale=1.0, unavailable=99)
        v.range(0, 10, out_of_range=42)
        assert v.get() == 42

    def test_out_of_range_falls_back_to_unavailable_when_unset(self, cls):
        v = cls(50.0, scale=1.0, unavailable=99)
        v.range(0, 10)
        assert v.get() == 99

    def test_nan_returns_unavailable(self, cls):
        v = cls(math.nan, scale=1.0, unavailable=99)
        v.range(0, 10)
        assert v.get() == 99

    def test_inf_returns_unavailable(self, cls):
        v = cls(math.inf, scale=1.0, unavailable=99)
        v.range(0, 10)
        assert v.get() == 99

    def test_rounds_to_nearest(self, cls):
        v_low = cls(1.4, scale=1.0, unavailable=99)
        v_low.range(0, 10)
        v_high = cls(1.6, scale=1.0, unavailable=99)
        v_high.range(0, 10)
        assert v_low.get() == 1
        assert v_high.get() == 2

    def test_returns_python_int(self, cls):
        v = cls(1.0, scale=1.0, unavailable=99)
        v.range(0, 10)
        assert isinstance(v.get(), int)


# Subclasses pin the right ETSI scaling factor and sentinel constants.
@pytest.mark.parametrize('subclass,msg_module,attr', [
    (CpmLatitudeValue, cpm_msg, 'Latitude'),
    (CpmLongitudeValue, cpm_msg, 'Longitude'),
    (VamLatitudeValue, vam_msg, 'Latitude'),
    (VamLongitudeValue, vam_msg, 'Longitude'),
])
def test_lat_lon_uses_tenths_of_microdegrees(subclass, msg_module, attr):
    # 1e-7 deg encodes to 1 in ETSI's 0.1-microdegree units.
    assert subclass(1e-7).get() == 1


@pytest.mark.parametrize('subclass,msg_module,attr', [
    (CpmLatitudeValue, cpm_msg, 'Latitude'),
    (CpmLongitudeValue, cpm_msg, 'Longitude'),
    (VamLatitudeValue, vam_msg, 'Latitude'),
    (VamLongitudeValue, vam_msg, 'Longitude'),
])
def test_lat_lon_nan_returns_msg_unavailable(subclass, msg_module, attr):
    expected = getattr(msg_module, attr).UNAVAILABLE
    assert subclass(math.nan).get() == expected


@pytest.mark.parametrize('subclass,msg_module', [
    (CpmSemiAxisLengthValue, cpm_msg),
    (VamSemiAxisLengthValue, vam_msg),
])
def test_semi_axis_overflow_uses_out_of_range_not_unavailable(subclass, msg_module):
    # A clearly oversized confidence should hit OUT_OF_RANGE, not UNAVAILABLE.
    assert subclass(1e9).get() == msg_module.SemiAxisLength.OUT_OF_RANGE


@pytest.mark.parametrize('subclass', [CpmAltitudeValue, VamAltitudeValue])
def test_altitude_uses_centimetres(subclass):
    # 100 m -> 10 000 cm.
    assert subclass(100.0).get() == 10000


@pytest.mark.parametrize('subclass,msg_module', [
    (CpmAltitudeValue, cpm_msg),
    (VamAltitudeValue, vam_msg),
])
def test_altitude_nan_returns_msg_unavailable(subclass, msg_module):
    assert subclass(math.nan).get() == msg_module.AltitudeValue.UNAVAILABLE
