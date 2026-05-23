# Sensor-Rplidar

Visualisasi data LiDAR (RPLidar) dalam bentuk polar plot + panel teks real-time. Mendukung mode **real**, **simulasi**, dan **auto** (fallback ke simulasi jika perangkat tidak tersedia).

## Prasyarat
- Python 3.8+ (disarankan)
- Paket Python:
  ```bash
  pip install -r requirements.txt
  ```

## Menjalankan Program
Gunakan `main.py` dengan opsi berikut:

### 1) Mode simulasi (tanpa sensor)
```bash
python main.py --mode sim
```

### 2) Mode otomatis (coba sensor, fallback ke simulasi)
```bash
python main.py --mode auto
```

### 3) Mode real (wajib ada sensor)
```bash
python main.py --mode real --port COM9
```
> Di Linux biasanya port serial seperti `/dev/ttyUSB0` atau `/dev/ttyACM0`.

## Opsi CLI
| Opsi | Nilai | Default | Keterangan |
|------|------|---------|------------|
| `--mode` | `auto` / `real` / `sim` | `auto` | Mode operasi program |
| `--port` | contoh: `COM9`, `/dev/ttyUSB0` | (kosong) | Port serial LiDAR (hanya untuk mode real/auto) |

## Catatan Matplotlib (GUI)
Jika muncul peringatan:
```
FigureCanvasAgg is non-interactive, and thus cannot be shown
```
Artinya backend matplotlib non-GUI. Solusi cepat:
```bash
MPLBACKEND=TkAgg python main.py --mode sim
```
Atau gunakan backend GUI lain seperti `QtAgg` (pastikan dependensinya terpasang).

## Struktur Singkat
- `main.py` → entry point, parsing argumen
- `lidar.py` → konfigurasi, sumber data (real/simulasi), pemrosesan, visualisasi

## Contoh Singkat
```bash
# Simulasi
python main.py --mode sim

# Sensor nyata
python main.py --mode real --port COM9
```
