<<<<<<< HEAD
import matplotlib.pyplot as plt
import numpy as np
from rplidar import RPLidar
import time

# ======================================================
# KONFIGURASI
# ======================================================
PORT_NAME = 'COM9'

JARAK_MIN = 100
JARAK_MAX = 12000

BUFFER_SCAN = 3000

MAKS_TITIK = 600

# Selisih radius untuk deteksi pindah posisi
THRESHOLD_PINDAH = 100

# ======================================================
# INISIALISASI LIDAR
# ======================================================
lidar = RPLidar(PORT_NAME)

lidar.start_motor()
time.sleep(2)

# ======================================================
# GRAFIK
# ======================================================
plt.ion()

fig = plt.figure(figsize=(14,7))

# Polar plot
ax_polar = fig.add_subplot(121, projection='polar')
ax_polar.set_theta_zero_location('N')
ax_polar.set_rmax(4000)

# Text panel
ax_text = fig.add_subplot(122)
ax_text.axis('off')

# ======================================================
# DATA
# ======================================================
all_angles = []
all_distances = []

last_radius = None
counter_pindah = 0

nomor_posisi = 1

# ======================================================
# UPDATE GRAFIK
# ======================================================
def update_visualisasi():

    ax_polar.clear()

    ax_polar.set_theta_zero_location('N')
    ax_polar.set_rmax(4000)

    ax_polar.scatter(
        all_angles,
        all_distances,
        c='red',
        s=5,
        alpha=0.7
    )

    ax_polar.set_title(
        f"Visualisasi Penampang TUTSIT\nPosisi {nomor_posisi}",
        pad=20,
        fontsize=14
    )

# ======================================================
# UPDATE TEKS
# ======================================================
def update_teks(
    radius,
    diameter,
    sudut,
    jarak,
    jumlah_titik
):

    ax_text.clear()
    ax_text.axis('off')

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
        f"Diameter (cm) : {diameter/10:.2f} cm\n\n"

        f"STATISTIK:\n"
        f"Jumlah Titik : {jumlah_titik}\n"
        f"Status       : AKTIF\n\n"

        f"INFO:\n"
        f"Jika LiDAR dipindah,\n"
        f"visualisasi lama otomatis dibersihkan."
    )

    ax_text.text(
        0.05,
        0.5,
        tampilan,
        transform=ax_text.transAxes,
        fontsize=12,
        family='monospace',
        verticalalignment='center',
        bbox=dict(
            facecolor='white',
            alpha=0.85,
            edgecolor='black',
            boxstyle='round'
        )
    )

# ======================================================
# PROGRAM UTAMA
# ======================================================
def jalankan_sistem():

    global last_radius
    global counter_pindah
    global nomor_posisi

    try:

        print("=" * 50)
        print("SISTEM TUTSIT LiDAR AKTIF")
        print("Geser LiDAR untuk membuat scan baru")
        print("Tekan CTRL+C untuk berhenti")
        print("=" * 50)

        time.sleep(1)

        for scan in lidar.iter_scans(max_buf_meas=BUFFER_SCAN):

            current_angles = []
            current_distances = []

            # ==========================================
            # FILTER DATA
            # ==========================================
            for (_, angle, distance) in scan:

                if JARAK_MIN < distance < JARAK_MAX:

                    current_angles.append(
                        np.radians(angle)
                    )

                    current_distances.append(distance)

            if not current_distances:
                plt.pause(0.001)
                continue

            # ==========================================
            # HITUNG DIMENSI
            # ==========================================
            radius_rerata = np.median(current_distances)

            diameter = radius_rerata * 2

            sudut_terakhir = np.degrees(
                current_angles[-1]
            )

            jarak_terakhir = current_distances[-1]

            # ==========================================
            # DETEKSI PINDAH POSISI
            # ==========================================
            if last_radius is not None:

                selisih = abs(
                    radius_rerata - last_radius
                )

                # Jika berubah signifikan
                if selisih > THRESHOLD_PINDAH:

                    counter_pindah += 1

                else:

                    counter_pindah = 0

                # Jika stabil beberapa scan
                if counter_pindah >= 3:

                    print(f"\nPosisi baru terdeteksi")
                    print("Membersihkan visualisasi lama...\n")

                    # HAPUS VISUALISASI LAMA
                    all_angles.clear()
                    all_distances.clear()

                    nomor_posisi += 1

                    counter_pindah = 0

            last_radius = radius_rerata

            # ==========================================
            # TAMBAH TITIK
            # ==========================================
            all_angles.extend(current_angles)
            all_distances.extend(current_distances)

            # Batasi titik
            if len(all_angles) > MAKS_TITIK:

                del all_angles[:len(all_angles)-MAKS_TITIK]
                del all_distances[:len(all_distances)-MAKS_TITIK]

            # ==========================================
            # UPDATE VISUAL
            # ==========================================
            update_visualisasi()

            update_teks(
                radius_rerata,
                diameter,
                sudut_terakhir,
                jarak_terakhir,
                len(all_angles)
            )

            plt.draw()
            plt.pause(0.001)

    except KeyboardInterrupt:

        print("\nProgram dihentikan pengguna.")

    except Exception as e:

        print(f"\nERROR: {e}")

    finally:

        try:
            lidar.stop()
            lidar.stop_motor()
            lidar.disconnect()
        except:
            pass

        plt.ioff()
        plt.show()

# ======================================================
# MAIN
# ======================================================
if __name__ == '__main__':
    jalankan_sistem()
=======
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
>>>>>>> 236a48ba950288e6a780348173a362ceb12e526c
