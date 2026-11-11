#!/usr/bin/env python3
"""FAST-LIVO2's colored scans (/cloud_registered: x y z rgb, frame camera_init) merged into one voxel map for its RViz,
published as /cloud_map every PERIOD s of sim time (ros1_ws/fast_livo2.sh starts it with the RViz). RViz keeping every
scan itself (its config's Decay Time) grew past 6 GB in 7 minutes and was killed by the container's memory cap; here a
voxel keeps the latest point that fell into it (as the saved map does, patches/fast_livo2.patch compactMap), so memory
grows with the explored volume only.
  python3 color_map.py [--selftest]"""
import sys

import numpy as np

VOX = 0.1      # m, = pcd_save/filter_size_pcd
PERIOD = 1.0   # s, sim time (use_sim_time): ~2.5 s of wall time at the sim's 0.4x
B = 1 << 20    # voxel index offset: 21 bits per axis, +-105 km at 0.1 m
WHITE = np.array([0xffffff], np.uint32).view(np.float32)[0]


def merge(old, new):
    """(N,4) float32 x y z rgb (rgb = packed uint32 seen as float32), new rows last: one row per voxel, the latest."""
    pts = np.concatenate([old, new])[::-1]                      # latest first: np.unique keeps the first occurrence
    k = np.floor(pts[:, :3] / VOX).astype(np.int64) + B
    _, first = np.unique((k[:, 0] << 42) | (k[:, 1] << 21) | k[:, 2], return_index=True)
    return pts[first]


def selftest():
    red, blue = np.array([0xff0000, 0x0000ff], np.uint32).view(np.float32)
    old = np.array([[0.01, 0.01, 0.01, red], [5.0, 5.0, 5.0, red]], np.float32)
    new = np.array([[0.05, 0.02, 0.03, blue], [-0.05, 0.0, 0.0, blue]], np.float32)   # same voxel as old[0]; x < 0
    m = merge(old, new)
    assert len(m) == 3, m                                                            # 3 voxels
    first = m[np.all(np.floor(m[:, :3] / VOX) == 0, axis=1)]
    assert len(first) == 1 and first[0, 3].view(np.uint32) == 0x0000ff, first         # latest color wins
    assert merge(np.zeros((0, 4), np.float32), new).shape == (2, 4)
    print("color_map selftest ok")


def main():
    import rospy
    from sensor_msgs.msg import PointCloud2, PointField
    rospy.init_node("color_map")
    pub = rospy.Publisher("/cloud_map", PointCloud2, queue_size=1, latch=True)
    st = {"map": np.zeros((0, 4), np.float32), "buf": []}
    fields = [PointField(n, 4 * i, PointField.FLOAT32, 1) for i, n in enumerate(("x", "y", "z", "rgb"))]   # as PCL

    def on_scan(m):
        n = m.width * m.height
        if n == 0:                                    # in LIVO mode the LIO step publishes empty clouds
            return
        off = {f.name: f.offset for f in m.fields if f.name in ("x", "y", "z", "rgb")}   # no rgb: LIO only (no images)
        a = np.frombuffer(m.data, np.dtype({"names": list(off), "formats": ["<f4"] * len(off),
                                            "offsets": list(off.values()), "itemsize": m.point_step}), count=n)
        rgb = a["rgb"] if "rgb" in off else np.full(n, WHITE)
        st["buf"].append(np.stack([a["x"], a["y"], a["z"], rgb], axis=1))

    def tick(_):
        if not st["buf"]:
            return
        new, st["buf"] = np.concatenate(st["buf"]), []
        st["map"] = merge(st["map"], new)
        out = PointCloud2(height=1, width=len(st["map"]), fields=fields, is_bigendian=False, point_step=16,
                          row_step=16 * len(st["map"]), is_dense=True, data=st["map"].tobytes())
        out.header.stamp, out.header.frame_id = rospy.Time.now(), "camera_init"
        pub.publish(out)

    rospy.Subscriber("/cloud_registered", PointCloud2, on_scan, queue_size=50)
    rospy.Timer(rospy.Duration(PERIOD), tick)
    rospy.spin()


if __name__ == "__main__":
    selftest() if "--selftest" in sys.argv else main()
