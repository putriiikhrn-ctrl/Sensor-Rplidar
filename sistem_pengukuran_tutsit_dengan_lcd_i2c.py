import argparse
import os
import random
import time
import matplotlib.pyplot as plt
import numpy as np

# --- IMPORT LIBRARY DENGAN PENANGANAN ERROR ---
# Library RPLidar
try:
    from rplidar import RPLidar, RPLidarException
except ImportError:
    print("[PERINGATAN] Library 'rplidar' tidak terinstal. Jalankan: pip install rplidar")
    RPLidar = None
    RPLidarException = Exception

# Library LCD I2C
try:
    from rpi_lcd import LCD
except ImportError:
    print("[PERINGATAN] Library 'rpi-lcd' tidak terinstal. Jalankan: pip install rpi-lcd")
    LCD = None


# --- KELAS KONFIGURASI ---
class Config:
    def __init__(self, port_name=None):
        # Jika di Raspberry Pi, port default biasanya '/dev/ttyUSB0'
        # Jika di Windows, biasanya 'COM9' atau sejenisnya
        if port_name:
            self.port_name = port_name
        else:
            self.port_name = '/dev/ttyUSB0' if os.name != 'nt' else 'COM9'


# --- INISIALISASI LCD I2C ---
def inisialisasi_lcd():
    if LCD is None:
        print("[INFO] LCD dinonaktifkan (Library rpi-lcd tidak terdeteksi).")
        return None
    try:
        lcd_obj = LCD()
        lcd_obj.clear()
        lcd_obj.text("SISTEM TUTSIT", 1)
        lcd_obj.text("READY...", 2)
        time.sleep(2)
        return lcd_obj
    except Exception as e:
        print(f"[ERROR] Gagal terhubung ke LCD I2C: {e}")
        print("Saran: Periksa kabel SDA/SCL dan pastikan I2C sudah aktif di raspi-config.")
        return None


# --- FUNGSI UTAMA PENGUKURAN ---
def jalankan_sistem(mode="auto", config=None):
    if config is None:
        config = Config()

    # Inisialisasi LCD
    lcd = inisialisasi_lcd()

    # Inisialisasi Grafik Matplotlib (Fullscreen ready)
    plt.ion()
    fig = plt.figure(figsize=(12, 7))
    ax_polar = fig.add_subplot(121, projection='polar') # Grafik radar (Kiri)
    ax_text = fig.add_subplot(122)                      # Info text (Kanan)
    ax_text.axis('off')

    # Maksimalkan jendela tampilan di monitor jika memungkinkan
    try:
        fig_manager = plt.get_current_fig_manager()
        fig_manager.full_screen_toggle()
    except Exception:
        pass

    lidar = None
    is_real_mode = False

    # Menentukan Mode Operasi (Real atau Simulasi)
    if mode in ["real", "auto"]:
        if RPLidar is None:
            print("[INFO] Tidak dapat masuk ke mode REAL karena library rplidar absen.")
        else:
            try:
                print(f"Mencoba menghubungkan LiDAR pada port: {config.port_name}...")
                if lcd:
                    lcd.clear()
                    lcd.text("CONNECTING...", 1)
                    lcd.text(config.port_name, 2)

                lidar = RPLidar(config.port_name)
                lidar.reset()
                time.sleep(1.5) # Beri jeda rebooting sensor
                
                # Cek koneksi dengan mencoba scan awal
                for scan in lidar.iter_scans(max_buf_meas=100):
                    break
                
                is_real_mode = True
                print("Sensor LiDAR berhasil terhubung! Memulai pemindaian fisik.")
            except Exception as e:
                print(f"[PERINGATAN] Gagal inisialisasi LiDAR: {e}")
                if lidar:
                    try:
                        lidar.stop()
                        lidar.disconnect()
                    except Exception:
                        pass
                
                if mode == "auto":
                    print("Beralih ke MODE SIMULASI (Fallback)...")
                    is_real_mode = False
                else:
                    print("Koneksi gagal. Program dihentikan.")
                    if lcd:
                        lcd.clear()
                        lcd.text("CONN ERROR!", 1)
                    return

    # Mempersiapkan visualisasi
    all_angles = []
    all_distances = []

    try:
        if is_real_mode and lidar:
            # --- JALUR DATA REAL SENSOR ---
            if lcd:
                lcd.clear()
                lcd.text("MODE: REAL SCAN", 1)
                time.sleep(1)

            for scan in lidar.iter_scans(max_buf_meas=1000):
                current_angles = []
                current_distances = []
                
                for (_, angle, distance) in scan:
                    # Filter: Rentang 15cm - 12m (150mm - 12000mm)
                    if 150 < distance < 12000:
                        rad = np.radians(angle)
                        current_angles.append(rad)
                        current_distances.append(distance)
                
                if current_distances:
                    all_angles = current_angles
                    all_distances = current_distances
                    
                    # Penghitungan Metrologi
                    rerata_radius = np.mean(all_distances)
                    diameter_mm = rerata_radius * 2
                    diameter_cm = diameter_mm / 10
                    sudut_terakhir = np.degrees(all_angles[-1])

                    # 1. Update Monitor (Grafik Radar)
                    ax_polar.clear()
                    ax_polar.set_theta_zero_location('N')
                    ax_polar.set_rmax(4000) # Batas visual radial 4 meter
                    ax_polar.scatter(all_angles, all_distances, c='red', s=4, alpha=0.7)
                    ax_polar.set_title("PENAMPANG DALAM TUTSIT (REAL)", pad=15, fontsize=12, fontweight='bold')

                    # 2. Update Monitor (Teks Kanan)
                    ax_text.clear()
                    ax_text.axis('off')
                    info_panel = (
                        f"PENGUKURAN DI LAPANGAN\n"
                        f"======================\n\n"
                        f"Sudut Laser : {sudut_terakhir:.1f}°\n"
                        f"Jml Titik   : {len(current_distances)} Pts\n\n"
                        f"DIAMETER TERUKUR:\n"
                        f"-> {diameter_mm:.2f} mm\n"
                        f"-> {diameter_cm:.2f} cm\n\n"
                        f"Status      : REAL TIME\n"
                        f"Koneksi     : TERHUBUNG"
                    )
                    ax_text.text(0.1, 0.5, info_panel, transform=ax_text.transAxes, 
                                fontsize=12, family='monospace', verticalalignment='center',
                                bbox=dict(facecolor='white', alpha=0.8, edgecolor='black', boxstyle='round'))
                    
                    plt.draw()

                    # 3. UPDATE TAMPILAN LCD (Hanya diameter saja)
                    if lcd:
                        lcd.text(f"D: {diameter_mm:.1f} mm", 1)
                        lcd.text(f"D: {diameter_cm:.2f} cm", 2)

                plt.pause(0.001)

        else:
            # --- JALUR DATA SIMULASI (TANPA SENSOR) ---
            if lcd:
                lcd.clear()
                lcd.text("MODE: SIMULATION", 1)
                time.sleep(1)

            base_diameter_simulasi = 3000.0 # Simulasi diameter tangki 3 meter (Radius 1500mm)
            
            while True:
                # Membuat 360 titik sudut simulasi
                sim_angles = np.linspace(0, 2*np.pi, 360)
                # Beri variasi noise +/- 12 mm agar grafik terlihat hidup
                sim_distances = [ (base_diameter_simulasi/2) + random.uniform(-12, 12) for _ in range(360) ]
                
                # Penghitungan Metrologi Simulasi
                rerata_radius = np.mean(sim_distances)
                diameter_mm = rerata_radius * 2
                diameter_cm = diameter_mm / 10
                
                # 1. Update Monitor (Grafik Radar)
                ax_polar.clear()
                ax_polar.set_theta_zero_location('N')
                ax_polar.set_rmax(4000)
                ax_polar.scatter(sim_angles, sim_distances, c='blue', s=3, alpha=0.6)
                ax_polar.set_title("PENAMPANG DALAM TUTSIT (SIMULASI)", pad=15, fontsize=12, fontweight='bold')

                # 2. Update Monitor (Teks Kanan)
                ax_text.clear()
                ax_text.axis('off')
                info_panel = (
                    f"SIMULASI ALAT UKUR\n"
                    f"==================\n\n"
                    f"Sudut Laser : 360.0° (Loop)\n"
                    f"Jml Titik   : 360 Pts\n\n"
                    f"DIAMETER ESTIMASI:\n"
                    f"-> {diameter_mm:.2f} mm\n"
                    f"-> {diameter_cm:.2f} cm\n\n"
                    f"Status      : DEMO MODE\n"
                    f"Koneksi     : DISCONNECTED"
                )
                ax_text.text(0.1, 0.5, info_panel, transform=ax_text.transAxes, 
                            fontsize=12, family='monospace', verticalalignment='center',
                            bbox=dict(facecolor='white', alpha=0.8, edgecolor='black', boxstyle='round'))
                
                plt.draw()

                # 3. UPDATE TAMPILAN LCD (Hanya diameter saja)
                if lcd:
                    lcd.text(f"D: {diameter_mm:.1f} mm", 1)
                    lcd.text(f"D: {diameter_cm:.2f} cm", 2)

                plt.pause(0.4) # Beri jeda 0.4 detik agar LCD tidak berkedip terlalu cepat

    except KeyboardInterrupt:
        print("\nProgram dihentikan oleh pengguna.")
    finally:
        # --- PROSES CLEANUP HARDWARE YANG AMAN ---
        print("Membersihkan resource dan mematikan layar...")
        if lcd:
            try:
                lcd.clear()
                lcd.text("SISTEM MATI", 1)
                time.sleep(1)
                lcd.clear()
            except Exception:
                pass
        
        if lidar:
            try:
                lidar.stop()
                lidar.disconnect()
            except Exception:
                pass
        
        plt.close('all')


# --- ARGUMENT PARSER ---
def parse_args():
    parser = argparse.ArgumentParser(
        description="Aplikasi Pengukuran Diameter TUTSIT dengan Visualisasi Monitor dan LCD I2C"
    )
    parser.add_argument(
        "--mode",
        choices=["auto", "real", "sim"],
        default="auto",
        help="auto: coba real terlebih dahulu, jika gagal otomatis simulasi",
    )
    parser.add_argument(
        "--port",
        default=None,
        help="Port serial LiDAR (contoh: /dev/ttyUSB0 atau COM9)",
    )
    return parser.parse_args()


# --- ENTRI UTAMA ---
if __name__ == "__main__":
    args = parse_args()
    config = Config(port_name=args.port) if args.port else Config()
    jalankan_sistem(mode=args.mode, config=config)