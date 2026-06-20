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


# --- FUNGSI FITTING LINGKARAN KUADRAT TERKECIL (LEAST SQUARES) ---
def hitung_diameter_off_center(angles_rad, distances_mm):
    """
    Menghitung diameter & pusat lingkaran sesungguhnya meskipun posisi sensor tidak di tengah.
    Menggunakan algoritma Least Squares Circle Fitting (Regresi Linear Kuadrat Terkecil).
    
    Parameter:
    angles_rad   : Array sudut dalam radian (theta)
    distances_mm : Array jarak dalam mm (r)
    
    Output:
    x_c          : Koordinat pusat lingkaran sumbu X relatif terhadap sensor (mm)
    y_c          : Koordinat pusat lingkaran sumbu Y relatif terhadap sensor (mm)
    diameter     : Diameter lingkaran hasil fitting (mm)
    """
    if len(distances_mm) < 3:
        # Minimal butuh 3 titik koordinat untuk membentuk lingkaran unik
        return 0.0, 0.0, 0.0

    # 1. Konversi data Polar ke Kartesius (Posisi sensor dianggap sebagai koordinat 0,0)
    x = distances_mm * np.cos(angles_rad)
    y = distances_mm * np.sin(angles_rad)
    
    # 2. Menyusun matriks Design M dan vektor target b
    # Persamaan: x*A + y*B + C = x^2 + y^2
    M = np.column_stack((x, y, np.ones_like(x)))
    b = x**2 + y**2
    
    # 3. Menyelesaikan persamaan Least Squares (M * p = b)
    # p akan berisi estimasi nilai parameter [A, B, C]
    p, _, _, _ = np.linalg.lstsq(M, b, rcond=None)
    A, B, C = p[0], p[1], p[2]
    
    # 4. Ekstraksi koordinat pusat lingkaran dan radiusnya
    x_c = A / 2
    y_c = B / 2
    
    # Proteksi nilai di bawah akar dari angka negatif akibat noise ekstrim
    val_dalam_akar = C + x_c**2 + y_c**2
    if val_dalam_akar < 0:
        return 0.0, 0.0, 0.0
        
    radius = np.sqrt(val_dalam_akar)
    diameter = 2 * radius
    
    return x_c, y_c, diameter


# --- FUNGSI UTAMA PENGUKURAN ---
def jalankan_sistem(mode="auto", config=None):
    if config is None:
        config = Config()

    # Inisialisasi LCD
    lcd = inisialisasi_lcd()

    # Inisialisasi Grafik Matplotlib (Fullscreen ready)
    plt.ion()
    fig = plt.figure(figsize=(13, 7))
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
                
                if len(current_distances) >= 3:
                    all_angles = np.array(current_angles)
                    all_distances = np.array(current_distances)
                    
                    # Penghitungan Metrologi dengan Least Squares Circle Fitting
                    x_c, y_c, diameter_mm = hitung_diameter_off_center(all_angles, all_distances)
                    diameter_cm = diameter_mm / 10
                    # Menghitung jarak pergeseran (eksentrisitas) sensor dari pusat lingkaran asli
                    jarak_pergeseran_mm = np.sqrt(x_c**2 + y_c**2)
                    sudut_terakhir = np.degrees(all_angles[-1])

                    # 1. Update Monitor (Grafik Radar)
                    ax_polar.clear()
                    ax_polar.set_theta_zero_location('N')
                    ax_polar.set_rmax(4000) # Batas visual radial 4 meter
                    ax_polar.scatter(all_angles, all_distances, c='red', s=4, alpha=0.7, label='Data Laser')
                    
                    # Visualisasi perkiraan titik tengah tangki pada grafik radar
                    sudut_pusat = np.arctan2(y_c, x_c)
                    ax_polar.scatter(sudut_pusat, jarak_pergeseran_mm, c='green', marker='x', s=100, fontweight='bold', label='Pusat Tangki')
                    ax_polar.legend(loc='lower right')
                    ax_polar.set_title("PENAMPANG DALAM TUTSIT (REAL)", pad=15, fontsize=12, fontweight='bold')

                    # 2. Update Monitor (Teks Kanan)
                    ax_text.clear()
                    ax_text.axis('off')
                    info_panel = (
                        f"PENGUKURAN DI LAPANGAN (FITTING)\n"
                        f"=================================\n\n"
                        f"Sudut Laser : {sudut_terakhir:.1f}°\n"
                        f"Jml Titik   : {len(current_distances)} Pts\n\n"
                        f"DIAMETER TERUKUR (FITTED):\n"
                        f"-> {diameter_mm:.2f} mm\n"
                        f"-> {diameter_cm:.2f} cm\n\n"
                        f"ANALISIS EKSENTRISITAS SENSOR:\n"
                        f"-> Pusat X   : {x_c:.1f} mm\n"
                        f"-> Pusat Y   : {y_c:.1f} mm\n"
                        f"-> Pergeseran: {jarak_pergeseran_mm:.1f} mm\n\n"
                        f"Status      : REAL TIME\n"
                        f"Koneksi     : TERHUBUNG"
                    )
                    ax_text.text(0.05, 0.5, info_panel, transform=ax_text.transAxes, 
                                fontsize=11, family='monospace', verticalalignment='center',
                                bbox=dict(facecolor='white', alpha=0.8, edgecolor='black', boxstyle='round'))
                    
                    plt.draw()

                    # 3. UPDATE TAMPILAN LCD (Diameter hasil fitting terkompensasi)
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

            # Skenario Simulasi: Tangki memiliki diameter nyata 3000 mm (Radius 1500 mm)
            # Namun, sensor diletakkan melenceng secara sengaja di koordinat X = 150.0 mm dan Y = -250.0 mm
            base_radius_simulasi = 1500.0
            x_c_nyata = 150.0
            y_c_nyata = -250.0
            
            # Jarak pergeseran teoritis dari titik (0,0) adalah sqrt(150^2 + (-250)^2) = ~291.55 mm
            eksentrisitas_teoritis = np.sqrt(x_c_nyata**2 + y_c_nyata**2)
            
            while True:
                # Membuat 360 titik sudut simulasi (0 s.d 360 derajat)
                sim_angles = np.linspace(0, 2*np.pi, 360)
                
                # Menghitung jarak radial r dari sensor (0,0) ke dinding tangki eksentris
                # Persamaan kuadratik r^2 + b*r + c = 0
                b_coef = -2 * (x_c_nyata * np.cos(sim_angles) + y_c_nyata * np.sin(sim_angles))
                c_coef = x_c_nyata**2 + y_c_nyata**2 - base_radius_simulasi**2
                
                # Mengambil akar positif untuk mendapatkan jarak radial simulasi asli
                sim_distances = (-b_coef + np.sqrt(b_coef**2 - 4*1*c_coef)) / 2
                
                # Memberikan variasi noise acak +/- 12 mm pada pembacaan sensor agar realistis
                sim_distances += np.array([random.uniform(-12, 12) for _ in range(360)])
                
                # Penghitungan Metrologi Simulasi Menggunakan Algoritma Least Squares
                x_c_fit, y_c_fit, diameter_mm = hitung_diameter_off_center(sim_angles, sim_distances)
                diameter_cm = diameter_mm / 10
                jarak_pergeseran_fit = np.sqrt(x_c_fit**2 + y_c_fit**2)
                
                # Hitung juga versi model rata-rata lama untuk perbandingan di konsol (menunjukkan error-nya)
                diameter_rata_rata_lama = np.mean(sim_distances) * 2
                error_rata_rata_lama = abs(diameter_rata_rata_lama - (base_radius_simulasi * 2))
                error_fitting_baru = abs(diameter_mm - (base_radius_simulasi * 2))
                
                print(f"[METROLOGI] Model Lama: {diameter_rata_rata_lama:.1f} mm (Error: {error_rata_rata_lama:.1f} mm) | "
                      f"Model Fitting LS: {diameter_mm:.1f} mm (Error: {error_fitting_baru:.1f} mm)")
                
                # 1. Update Monitor (Grafik Radar)
                ax_polar.clear()
                ax_polar.set_theta_zero_location('N')
                ax_polar.set_rmax(4000)
                ax_polar.scatter(sim_angles, sim_distances, c='blue', s=3, alpha=0.6, label='Data Simulasi')
                
                # Menampilkan titik pusat hasil fitting kalkulasi (X) pada radar
                sudut_pusat_fit = np.arctan2(y_c_fit, x_c_fit)
                ax_polar.scatter(sudut_pusat_fit, jarak_pergeseran_fit, c='magenta', marker='x', s=100, fontweight='bold', label='Pusat Terkalkulasi')
                ax_polar.legend(loc='lower right')
                ax_polar.set_title("PENAMPANG DALAM TUTSIT (SIMULASI)", pad=15, fontsize=12, fontweight='bold')

                # 2. Update Monitor (Teks Kanan)
                ax_text.clear()
                ax_text.axis('off')
                info_panel = (
                    f"SIMULASI ALAT UKUR (OFF-CENTER)\n"
                    f"=================================\n\n"
                    f"Sudut Laser : 360.0° (Loop)\n"
                    f"Jml Titik   : 360 Pts\n\n"
                    f"DIAMETER ESTIMASI (FITTED):\n"
                    f"-> {diameter_mm:.2f} mm\n"
                    f"-> {diameter_cm:.2f} cm\n\n"
                    f"IDENTIFIKASI PERGESERAN ALAT:\n"
                    f"-> Pusat X   : {x_c_fit:.1f} mm\n"
                    f"-> Pusat Y   : {y_c_fit:.1f} mm\n"
                    f"-> Pergeseran: {jarak_pergeseran_fit:.1f} mm\n\n"
                    f"Status      : DEMO MODE\n"
                    f"Koneksi     : DISCONNECTED"
                )
                ax_text.text(0.05, 0.5, info_panel, transform=ax_text.transAxes, 
                            fontsize=11, family='monospace', verticalalignment='center',
                            bbox=dict(facecolor='white', alpha=0.8, edgecolor='black', boxstyle='round'))
                
                plt.draw()

                # 3. UPDATE TAMPILAN LCD (Diameter hasil fitting terkompensasi)
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
        description="Aplikasi Pengukuran Diameter TUTSIT dengan Visualisasi Monitor dan LCD I2C (Anti-Eksentris)"
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