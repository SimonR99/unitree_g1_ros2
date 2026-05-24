"""
Pre-rclpy DDS initialization for the Unitree SDK.

# Problem

When ROS 2 Foxy is configured with `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
(the standard setup for talking to a Unitree robot — see unitree_ros2 README)
both rclpy and `unitree_sdk2py` use cyclonedds under the hood. cyclonedds
allows only one explicit `Domain` object per (process, domain_id). The SDK's
`ChannelFactoryInitialize` always calls `Domain(id, config)`, and so does
`rmw_cyclonedds_cpp` on first node creation — whichever runs first wins, and
the other throws "create domain error" / "Precondition Not Met".

# Fix

We monkey-patch `unitree_sdk2py.core.channel.ChannelFactory.Init` so the SDK
side skips the explicit `Domain(...)` call and just creates a
`DomainParticipant` against the process-wide cyclonedds domain. Both stacks
then share a single domain.

Network interface selection: if the user passes `interface:=<iface>` (via
`--ros-args -p`) or sets `$G1_INTERFACE`, we synthesize a tiny CycloneDDS XML
config and point `$CYCLONEDDS_URI` at it BEFORE any cyclonedds import. This
matches the convention in `unitree_ros2/setup.sh`. If neither is set, we
fall back to the existing `$CYCLONEDDS_URI` (or cyclonedds' auto-detect).

Each bridge process should call `init_dds_from_args()` exactly once, at the
top of `main()`, BEFORE `rclpy.init()`.
"""

import os
import sys
import tempfile

_CYCLONEDDS_XML_TEMPLATE = """\
<CycloneDDS>
  <Domain>
    <General>
      <Interfaces>
        <NetworkInterface name="{iface}" priority="default" multicast="default"/>
      </Interfaces>
    </General>
  </Domain>
</CycloneDDS>
"""


def _arg_value(args, key):
    """Look for `<key>:=<value>` (with or without preceding `-p`)."""
    needle = f"{key}:="
    for i, a in enumerate(args):
        if a.startswith(needle):
            return a.split(":=", 1)[1]
        if a == "-p" and i + 1 < len(args) and args[i + 1].startswith(needle):
            return args[i + 1].split(":=", 1)[1]
    return None


def _resolve(default_interface: str, default_domain: int) -> tuple:
    args = sys.argv
    iface = _arg_value(args, "interface")
    if not iface:
        iface = os.environ.get("G1_INTERFACE", default_interface)
    dom_str = _arg_value(args, "domain_id")
    domain = int(dom_str) if dom_str is not None else default_domain
    return domain, iface


def _ensure_cyclonedds_uri(iface: str) -> None:
    """If `iface` is set and no CYCLONEDDS_URI points at that iface already,
    write a temp XML and point $CYCLONEDDS_URI at it. Must run before any
    cyclonedds import.
    """
    if not iface:
        return
    existing = os.environ.get("CYCLONEDDS_URI", "")
    if iface in existing:
        return
    xml = _CYCLONEDDS_XML_TEMPLATE.format(iface=iface)
    f = tempfile.NamedTemporaryFile(
        prefix="g1_ros2_bridge_cyclonedds_", suffix=".xml",
        mode="w", delete=False)
    f.write(xml)
    f.close()
    os.environ["CYCLONEDDS_URI"] = f.name


def _patch_channel_factory():
    """Replace ChannelFactory.Init so it skips Domain() and reuses the shared one."""
    from unitree_sdk2py.core import channel as ch

    def _patched_init(self, id, networkInterface=None, qos=None):
        cls = ch.ChannelFactory
        # Access the name-mangled class attributes set in the SDK source.
        if cls._ChannelFactory__initialized:
            return True
        with cls._ChannelFactory__init_lock:
            if cls._ChannelFactory__initialized:
                return True
            from cyclonedds.domain import DomainParticipant
            try:
                cls._ChannelFactory__participant = DomainParticipant(id)
            except Exception as e:
                print(f"[g1_ros2_bridge.dds_init] participant create error: {e}", flush=True)
                return False
            cls._ChannelFactory__qos = qos
            cls._ChannelFactory__initialized = True
            return True

    ch.ChannelFactory.Init = _patched_init


def prepare_dds(default_interface: str = "", default_domain: int = 0) -> tuple:
    """Step 1 (call BEFORE `rclpy.init()`): resolve args, set CYCLONEDDS_URI, monkey-patch the SDK.
    Does NOT yet create any cyclonedds participant — that has to wait until
    rclpy has created the domain (i.e., until the first rclpy.Node exists).
    Returns `(domain, interface)`.
    """
    domain, iface = _resolve(default_interface, default_domain)
    _ensure_cyclonedds_uri(iface)
    _patch_channel_factory()
    return domain, iface


def finalize_dds(domain: int, iface: str) -> None:
    """Step 2 (call AFTER an rclpy.Node has been constructed): create the SDK's
    DomainParticipant in the already-existing cyclonedds domain.
    """
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    ChannelFactoryInitialize(domain, iface)


def init_dds_from_args(default_interface: str = "", default_domain: int = 0) -> tuple:
    """Convenience: prepare + finalize in one call. Only use when no rclpy
    domain exists yet (e.g. in a standalone non-ROS Python script).
    For rclpy-based bridges call `prepare_dds()` before `rclpy.init()` and
    `finalize_dds()` after the first Node is constructed.
    """
    domain, iface = prepare_dds(default_interface, default_domain)
    finalize_dds(domain, iface)
    return domain, iface
