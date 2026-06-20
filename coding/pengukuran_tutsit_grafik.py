import argparse
import os
import random
import time
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


# --- FUNGSI GRAFIK TERMINAL (ASCII SCATTER PLOT) ---
def buat_grafik_ascii(angles_rad, distances_mm, x_c, y_c, lebar=54, tinggi=20):
    """
    Menghasilkan graf teks ASCII 2D yang memplot bentuk penampang lingkaran tangki,
    kedudukan sensor, dan pusat bulatan terhitung secara langsung di terminal.
    """
    # Inisialisasi kanvas kosong dengan ruang spasi
    grid = [[" " for _ in range(lebar)] for _ in range(tinggi)]
    
    # Konversi data Polar ke Kartesius
    x = distances_mm * np.cos(angles_rad)
    y = distances_mm * np.sin(angles_rad)
    
    # Had batas visualisasi (-2000 mm hingga +2000 mm) untuk memuat tangki berdiameter 3 meter
    had_min, had_max = -2000.0, 2000.0
    
    # Fungsi pembantu memetakan koordinat fizikal (mm) ke koordinat indeks grid terminal
    def ke_indeks_grid(px, py):
        # Normalisasi ke julat 0.0 hingga 1.0
        nx = (px - had_min) / (had_max - had_min)
        ny = (py - had_min) / (had_max - had_min)
        
        # Hitung indeks kolum dan baris
        col = int(nx * (lebar - 1))
        row = int((1.0 - ny) * (tinggi - 1)) # Terbalikkan paksi Y untuk terminal
        return col, row

    # 1. Plot sempadan kotak grafik
    for r in range(tinggi):
        for c in range(lebar):
            if r == 0 or r == tinggi - 1:
                grid[r][c] = "-"
            elif c == 0 or c == lebar - 1:
                grid[r][c] = "|"
                
    # 2. Plot titik-titik pemindaian laser tangki (simbol: •)
    for i in range(len(x)):
        c, r = ke_indeks_grid(x[i], y[i])
        if 0 < c < lebar - 1 and 0 < r < tinggi - 1:
            grid[r][c] = "•"
            
    # 3. Plot kedudukan fisik sensor laser (simbol: +, diletakkan di koordinat 0,0)
    c_sens, r_sens = ke_indeks_grid(0.0, 0.0)
    if 0 < c_sens < lebar - 1 and 0 < r_sens < tinggi - 1:
        grid[r_sens][c_sens] = "+"
        
    # 4. Plot anggaran titik pusat lingkaran tangki (simbol: C)
    c_ctr, r_ctr = ke_indeks_grid(x_c, y_c)
    if 0 < c_ctr < lebar - 1 and 0 < r_ctr < tinggi - 1:
        grid[r_ctr][c_ctr] = "C"
        
    # Satukan grid baris menjadi satu string panjang
    output_grafik = "\n".join("".join(baris) for baris in grid)
    return output_grafik


# --- FUNGSI MEMBERSIHKAN LAYAR TERMINAL ---
def bersihkan_terminal():
    # Menggunakan perintah sistem untuk membersihkan layar terminal agar tampak seperti aplikasi GUI teks
    os.system('cls' if os.name == 'nt' else 'clear')


# --- FUNGSI UTAMA PENGUKURAN ---
def jalankan_sistem(mode="auto", config=None):
    if config is None:
        config = Config()

    # Inisialisasi LCD
    lcd = inisialisasi_lcd()

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
                time.sleep(1)
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
                    time.sleep(1.5)
                else:
                    print("Koneksi gagal. Program dihentikan.")
                    if lcd:
                        lcd.clear()
                        lcd.text("CONN ERROR!", 1)
                    return

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

                    # Hasilkan representasi grafik ASCII dari penampang lingkaran
                    grafik_terminal = buat_grafik_ascii(all_angles, all_distances, x_c, y_c)

                    # 1. TAMPILKAN DASHBOARD DI TERMINAL DENGAN GRAFIK ASCII
                    bersihkan_terminal()
                    print("=========================================================================")
                    print("           SISTEM PENGUKURAN TUTSIT (DASHBOARD GRAFIK KONSOL)           ")
                    print("=========================================================================")
                    print(grafik_terminal)
                    print(" PETUNJUK GRAFIK:  [•] Dinding Tangki  [+] Posisi Alat  [C] Pusat Tangki")
                    print("-------------------------------------------------------------------------")
                    print(f" Status Sistem       : AKTIF (REAL-TIME-SCAN)")
                    print(f" Jumlah Titik Data   : {len(current_distances)} Pts | Sudut Laser: {sudut_terakhir:.1f}°")
                    print("-------------------------------------------------------------------------")
                    print(" DIAMETER DALAM TANGKI (FITTED):")
                    print(f"  -> {diameter_mm:10.2f} mm  ({diameter_cm:.2f} cm)")
                    print(" ANALISIS KEDUDUKAN SENSOR (OFF-CENTER):")
                    print(f"  -> Koordinat Pusat : X: {x_c:+.1f} mm, Y: {y_c:+.1f} mm")
                    print(f"  -> Sesaran / Ofset : {jarak_pergeseran_mm:10.1f} mm")
                    print("=========================================================================")
                    print(" Tekan Ctrl+C untuk menghentikan program.")

                    # 2. UPDATE TAMPILAN LCD
                    if lcd:
                        lcd.text(f"D: {diameter_mm:.1f} mm", 1)
                        lcd.text(f"D: {diameter_cm:.2f} cm", 2)

        else:
            # --- JALUR DATA SIMULASI (TANPA SENSOR) ---
            if lcd:
                lcd.clear()
                lcd.text("MODE: SIMULATION", 1)
                time.sleep(1)

            # Skenario Simulasi: Tangki memiliki diameter nyata 3000 mm (Radius 1500 mm)
            # Namun, sensor diletakkan melenceng secara sengaja di koordinat X = 200.0 mm dan Y = -350.0 mm
            base_radius_simulasi = 1500.0
            x_c_nyata = 200.0
            y_c_nyata = -350.0
            
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
                
                # Hitung versi model rata-rata lama untuk menunjukkan perbandingan galat di terminal
                diameter_rata_rata_lama = np.mean(sim_distances) * 2
                error_rata_rata_lama = abs(diameter_rata_rata_lama - (base_radius_simulasi * 2))
                error_fitting_baru = abs(diameter_mm - (base_radius_simulasi * 2))

                # Hasilkan representasi grafik ASCII dari penampang lingkaran simulasi
                grafik_terminal = buat_grafik_ascii(sim_angles, sim_distances, x_c_fit, y_c_fit)

                # 1. TAMPILKAN DASHBOARD DI TERMINAL DENGAN GRAFIK ASCII
                bersihkan_terminal()
                print("=========================================================================")
                print("         SISTEM PENGUKURAN TUTSIT (DASHBOARD GRAFIK SIMULASI)          ")
                print("=========================================================================")
                print(grafik_terminal)
                print(" PETUNJUK GRAFIK:  [•] Dinding Tangki  [+] Posisi Alat  [C] Pusat Tangki")
                print("-------------------------------------------------------------------------")
                print(f" Status Sistem       : DEMO MODE (SIMULASI)")
                print(f" Jumlah Titik Data   : 360 Pts (Generated)  | Jarak Laser: 360.0° (Loop)")
                print("-------------------------------------------------------------------------")
                print(" DIAMETER ESTIMASI (FITTED):")
                print(f"  -> {diameter_mm:10.2f} mm  [Ralat Anggaran LS : {error_fitting_baru:.2f} mm]")
                print(f"  -> {diameter_cm:10.2f} cm")
                print(" ESTIMASI METODE LAMA (RATA-RATA MUDAH):")
                print(f"  -> {diameter_rata_rata_lama:10.2f} mm  [Ralat Rata-Rata Lama: {error_rata_rata_lama:.2f} mm]")
                print(" ANALISIS KEDUDUKAN SENSOR (OFF-CENTER):")
                print(f"  -> Koordinat Pusat : X: {x_c_fit:+.1f} mm, Y: {y_c_fit:+.1f} mm")
                print(f"  -> Sesaran / Ofset : {jarak_pergeseran_fit:10.1f} mm")
                print("=========================================================================")
                print(" Tekan Ctrl+C untuk menghentikan program.")
                
                # 2. UPDATE TAMPILAN LCD
                if lcd:
                    lcd.text(f"D: {diameter_mm:.1f} mm", 1)
                    lcd.text(f"D: {diameter_cm:.2f} cm", 2)

                time.sleep(0.4) # Jeda masa pembaharuan skrin konsol dan LCD

    except KeyboardInterrupt:
        print("\nProgram dihentikan oleh pengguna.")
    finally:
        # --- PROSES CLEANUP HARDWARE YANG AMAN ---
        print("\nMembersihkan resource dan mematikan layar...")
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