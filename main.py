import argparse

from lidar import Config, jalankan_sistem


def parse_args():
    parser = argparse.ArgumentParser(
        description="Visualisasi LiDAR (mode real/sim/auto)"
    )
    parser.add_argument(
        "--mode",
        choices=["auto", "real", "sim"],
        default="auto",
        help="auto: coba real lalu fallback ke simulasi",
    )
    parser.add_argument(
        "--port",
        default=None,
        help="Port serial LiDAR (contoh: COM9 atau /dev/ttyUSB0)",
    )

    return parser.parse_args()


# ======================================================
# MAIN
# ======================================================
if __name__ == "__main__":
    args = parse_args()
    config = Config(port_name=args.port) if args.port else Config()
    jalankan_sistem(mode=args.mode, config=config)
