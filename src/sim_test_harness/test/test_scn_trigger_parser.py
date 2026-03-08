"""Tests for the SCN trigger parser."""

import os
import tempfile
from sim_test_harness.scn_trigger_parser import parse_triggers


SCN_SNIPPET = """\
<?xml version="1.0"?>
<scenario>
  <environment>
    <ned latitude="0.0" longitude="0.0" />
  </environment>

  <static name="Ground" type="plane">
    <material name="steel" />
    <world_transform rpy="0.0 0.0 0.0" xyz="0.0 0.0 3.0" />
  </static>

  <volumetric_trigger name="test_gate" timeout_s="15.0">
    <origin rpy="0.0 0.0 1.57" xyz="1.0 2.0 3.0" />
    <dimensions xyz="4.0 5.0 6.0" />
  </volumetric_trigger>

  <volumetric_trigger name="test_marker" timeout_s="25.5">
    <origin rpy="0.0 0.0 0.0" xyz="10.0 20.0 1.5" />
    <dimensions xyz="2.0 2.0 2.0" />
  </volumetric_trigger>
</scenario>
"""


def test_parse_triggers_count():
    with tempfile.NamedTemporaryFile(mode="w", suffix=".scn", delete=False) as f:
        f.write(SCN_SNIPPET)
        f.flush()
        triggers = parse_triggers(f.name)
    os.unlink(f.name)

    assert len(triggers) == 2


def test_parse_trigger_fields():
    with tempfile.NamedTemporaryFile(mode="w", suffix=".scn", delete=False) as f:
        f.write(SCN_SNIPPET)
        f.flush()
        triggers = parse_triggers(f.name)
    os.unlink(f.name)

    t0 = triggers[0]
    assert t0.name == "test_gate"
    assert t0.timeout_s == 15.0
    assert t0.position == (1.0, 2.0, 3.0)
    assert t0.rotation_rpy == (0.0, 0.0, 1.57)
    assert t0.dimensions == (4.0, 5.0, 6.0)

    t1 = triggers[1]
    assert t1.name == "test_marker"
    assert t1.timeout_s == 25.5
    assert t1.position == (10.0, 20.0, 1.5)
    assert t1.dimensions == (2.0, 2.0, 2.0)


def test_parse_no_triggers():
    scn = '<?xml version="1.0"?>\n<scenario></scenario>'
    with tempfile.NamedTemporaryFile(mode="w", suffix=".scn", delete=False) as f:
        f.write(scn)
        f.flush()
        triggers = parse_triggers(f.name)
    os.unlink(f.name)

    assert len(triggers) == 0
