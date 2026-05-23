import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

import matplotlib.pyplot as plt
import numpy as np

try:
    from rplidar import RPLidar
except Exception:
    RPLidar = None

Scan = List[Tuple[int, float, float]]


# ======================================================
# KONFIGURASI
# ======================================================
@dataclass
class Config:
    port_name: str = "COM9"
    jarak_min: int = 100
    jarak_max: int = 12000
    buffer_scan: int = 3000
    maks_titik: int = 600
    threshold_pindah: int = 100
    rmax_plot: int = 4000

    # Simulasi
    points_per_scan: int = 360
    simulasi_radius: int = 1500
    simulasi_noise: int = 40
    simulasi_step_interval: int = 120


# ======================================================
# SUMBER DATA LIDAR
# ======================================================
class RealLidarSource:
    def __init__(self, config: Config):
        if RPLidar is None:
            raise RuntimeError("Paket rplidar belum terpasang")

        self.config = config
        self.lidar = RPLidar(config.port_name)
        self.lidar.start_motor()
        time.sleep(2)

    def iter_scans(self, max_buf_meas: Optional[int] = None):
        buffer_size = self.config.buffer_scan if max_buf_meas is None else max_buf_meas
        for scan in self.lidar.iter_scans(max_buf_meas=buffer_size):
            yield scan

    def close(self):
        self.lidar.stop()
        self.lidar.stop_motor()
        self.lidar.disconnect()


class SimulatedLidarSource:
    def __init__(self, config: Config):
        self.config = config
        self.base_radius = config.simulasi_radius
        self.scan_count = 0
        self.phase = 0.0

    def iter_scans(self, max_buf_meas: Optional[int] = None):
        while True:
            self.scan_count += 1

            # Simulasikan pindah posisi tiap beberapa scan
            if self.scan_count % self.config.simulasi_step_interval == 0:
                shift = np.random.randint(-300, 300)
                self.base_radius = int(
                    np.clip(
                        self.base_radius + shift,
                        self.config.jarak_min + 200,
                        self.config.jarak_max - 200,
                    )
                )

            angles = np.linspace(0, 359, self.config.points_per_scan)
            noise = np.random.normal(
                0,
                self.config.simulasi_noise,
                size=angles.shape,
            )

            distances = (
                self.base_radius
                + 100 * np.sin(np.radians(angles * 3 + self.phase))
                + noise
            )

            distances = np.clip(
                distances,
                self.config.jarak_min + 1,
                self.config.jarak_max - 1,
            )

            self.phase = (self.phase + 3) % 360

            scan: Scan = [(15, float(a), float(d)) for a, d in zip(angles, distances)]

            yield scan
            time.sleep(0.05)

    def close(self):
        pass


def create_scan_source(mode: str, config: Config):
    mode = mode.lower()

    if mode == "sim":
        return SimulatedLidarSource(config)

    if mode == "real":
        return RealLidarSource(config)

    if mode == "auto":
        try:
            return RealLidarSource(config)
        except Exception as e:
            print(f"Mode auto: gagal inisialisasi LiDAR ({e}). Menggunakan simulasi.")
            return SimulatedLidarSource(config)

    raise ValueError("Mode tidak dikenal. Gunakan: auto, real, sim")


# ======================================================
# VISUALISASI
# ======================================================
class LidarVisualizer:
    def __init__(self, config: Config):
        self.config = config
        plt.ion()
        self.fig = plt.figure(figsize=(14, 7))

        # Polar plot
        self.ax_polar = self.fig.add_subplot(121, projection="polar")
        self.ax_polar.set_theta_zero_location("N")
        self.ax_polar.set_rmax(config.rmax_plot)

        # Text panel
        self.ax_text = self.fig.add_subplot(122)
        self.ax_text.axis("off")

    def update_plot(self, angles, distances, nomor_posisi: int):
        self.ax_polar.clear()
        self.ax_polar.set_theta_zero_location("N")
        self.ax_polar.set_rmax(self.config.rmax_plot)

        self.ax_polar.scatter(
            angles,
            distances,
            c="red",
            s=5,
            alpha=0.7,
        )

        self.ax_polar.set_title(
            f"Visualisasi Penampang TUTSIT\nPosisi {nomor_posisi}",
            pad=20,
            fontsize=14,
        )

    def update_text(
        self,
        radius,
        diameter,
        sudut,
        jarak,
        jumlah_titik,
        nomor_posisi: int,
    ):
        self.ax_text.clear()
        self.ax_text.axis("off")

        tampilan = (
            f"DATA PENGUKURAN REAL-TIME\n"
            f"============================\n\n"
            f"POSISI AKTIF:\n"
            f"Posisi : {nomor_posisi}\n\n"
            f"POSISI LASER:\n"
            f"Sudut Saat Ini : {sudut:.2f}°\n"
            f"Jarak Terakhir : {jarak:.2f} mm\n\n"
            f"HASIL DIMENSI:\n"
            f"Radius Rerata : {radius:.2f} mm\n"
            f"Diameter      : {diameter:.2f} mm\n"
            f"Diameter (cm) : {diameter / 10:.2f} cm\n\n"
            f"STATISTIK:\n"
            f"Jumlah Titik : {jumlah_titik}\n"
            f"Status       : AKTIF\n\n"
            f"INFO:\n"
            f"Jika LiDAR dipindah,\n"
            f"visualisasi lama otomatis dibersihkan."
        )

        self.ax_text.text(
            0.05,
            0.5,
            tampilan,
            transform=self.ax_text.transAxes,
            fontsize=12,
            family="monospace",
            verticalalignment="center",
            bbox=dict(
                facecolor="white",
                alpha=0.85,
                edgecolor="black",
                boxstyle="round",
            ),
        )

    def refresh(self):
        plt.draw()
        plt.pause(0.001)

    def close(self):
        plt.ioff()
        plt.show()


# ======================================================
# LOGIKA PEMROSESAN
# ======================================================
@dataclass
class ScanStats:
    radius_rerata: float
    diameter: float
    sudut_terakhir: float
    jarak_terakhir: float
    jumlah_titik: int


class LidarProcessor:
    def __init__(self, config: Config):
        self.config = config
        self.all_angles: List[float] = []
        self.all_distances: List[float] = []
        self.last_radius: Optional[float] = None
        self.counter_pindah = 0
        self.nomor_posisi = 1

    def process_scan(self, scan: Scan) -> Optional[ScanStats]:
        current_angles: List[float] = []
        current_distances: List[float] = []

        for _, angle, distance in scan:
            if self.config.jarak_min < distance < self.config.jarak_max:
                current_angles.append(np.radians(angle))
                current_distances.append(distance)

        if not current_distances:
            return None

        radius_rerata = float(np.median(current_distances))
        diameter = radius_rerata * 2
        sudut_terakhir = float(np.degrees(current_angles[-1]))
        jarak_terakhir = float(current_distances[-1])

        # Deteksi pindah posisi
        if self.last_radius is not None:
            selisih = abs(radius_rerata - self.last_radius)

            if selisih > self.config.threshold_pindah:
                self.counter_pindah += 1
            else:
                self.counter_pindah = 0

            if self.counter_pindah >= 3:
                print("\nPosisi baru terdeteksi")
                print("Membersihkan visualisasi lama...\n")

                self.all_angles.clear()
                self.all_distances.clear()
                self.nomor_posisi += 1
                self.counter_pindah = 0

        self.last_radius = radius_rerata

        # Tambah titik
        self.all_angles.extend(current_angles)
        self.all_distances.extend(current_distances)

        if len(self.all_angles) > self.config.maks_titik:
            overflow = len(self.all_angles) - self.config.maks_titik
            del self.all_angles[:overflow]
            del self.all_distances[:overflow]

        return ScanStats(
            radius_rerata=radius_rerata,
            diameter=diameter,
            sudut_terakhir=sudut_terakhir,
            jarak_terakhir=jarak_terakhir,
            jumlah_titik=len(self.all_angles),
        )


# ======================================================
# PROGRAM UTAMA
# ======================================================
def jalankan_sistem(mode: str = "auto", config: Optional[Config] = None):
    config = config or Config()

    source = None
    visualizer = None

    try:
        source = create_scan_source(mode, config)
        visualizer = LidarVisualizer(config)
        processor = LidarProcessor(config)

        print("=" * 50)
        print("SISTEM TUTSIT LiDAR AKTIF")
        print("Geser LiDAR untuk membuat scan baru")
        print("Tekan CTRL+C untuk berhenti")
        print(f"Mode: {mode.upper()}")
        print("=" * 50)

        time.sleep(1)

        for scan in source.iter_scans(max_buf_meas=config.buffer_scan):
            stats = processor.process_scan(scan)
            if stats is None:
                plt.pause(0.001)
                continue

            visualizer.update_plot(
                processor.all_angles,
                processor.all_distances,
                processor.nomor_posisi,
            )

            visualizer.update_text(
                stats.radius_rerata,
                stats.diameter,
                stats.sudut_terakhir,
                stats.jarak_terakhir,
                stats.jumlah_titik,
                processor.nomor_posisi,
            )

            visualizer.refresh()

    except KeyboardInterrupt:
        print("\nProgram dihentikan pengguna.")

    except Exception as e:
        print(f"\nERROR: {e}")

    finally:
        try:
            if source is not None:
                source.close()
        except Exception:
            pass

        if visualizer is not None:
            visualizer.close()
