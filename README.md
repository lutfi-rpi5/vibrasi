# Vibration Monitoring System — ISO 10816

> **Platform:** ESP32-C3 Super Mini · ADXL345 · OLED SSD1306 0.96"  
> **Standar:** ISO 10816 / ISO 20816 (Vibration Severity – Machinery)  
> **Author:** Muhammad Lutfi Nur Anendi · v1.0.0 · 2026

---

## Daftar Isi

1. [Gambaran Umum](#1-gambaran-umum)
2. [Arsitektur Sistem](#2-arsitektur-sistem)
3. [Hardware & Wiring](#3-hardware--wiring)
4. [Struktur File Proyek](#4-struktur-file-proyek)
5. [Flow Data: Dari Sensor ke Display](#5-flow-data-dari-sensor-ke-display)
   - 5.1 [RMS Velocity](#51-flow-parameter-rms-velocity-mms)
   - 5.2 [Peak Acceleration](#52-flow-parameter-peak-acceleration-ms²)
   - 5.3 [Dominant Frequency](#53-flow-parameter-dominant-frequency-hz)
6. [Penjelasan Matematis Lengkap](#6-penjelasan-matematis-lengkap)
   - 6.1 [Tahap 1 – Akuisisi Sensor ADXL345](#61-tahap-1--akuisisi-sensor-adxl345)
   - 6.2 [Tahap 2 – Kalibrasi Statis](#62-tahap-2--kalibrasi-statis)
   - 6.3 [Tahap 3 – Ekstraksi Skalar 3-Axis](#63-tahap-3--ekstraksi-skalar-3-axis)
   - 6.4 [Tahap 4 – Filter Sinyal](#64-tahap-4--filter-sinyal)
   - 6.5 [Tahap 5 – Kalkulasi RMS Velocity](#65-tahap-5--kalkulasi-rms-velocity)
   - 6.6 [Tahap 6 – Kalkulasi Peak Acceleration](#66-tahap-6--kalkulasi-peak-acceleration)
   - 6.7 [Tahap 7 – Kalkulasi Dominant Frequency (FFT)](#67-tahap-7--kalkulasi-dominant-frequency-fft)
7. [Perbedaan RMS, Peak, dan Frequency](#7-perbedaan-rms-peak-dan-frequency)
8. [Kenapa Satuan Berbeda-Beda?](#8-kenapa-satuan-berbeda-beda)
9. [Standar ISO 10816 – Alarm Threshold](#9-standar-iso-10816--alarm-threshold)
10. [Parameter Konfigurasi](#10-parameter-konfigurasi)
11. [Keterbatasan Sistem](#11-keterbatasan-sistem)
12. [Library yang Digunakan](#12-library-yang-digunakan)

---

## 1. Gambaran Umum

Sistem ini adalah alat **pemantau getaran mesin** berbasis mikrokontroler ESP32-C3 yang mengukur intensitas getaran secara real-time sesuai standar industri ISO 10816. Alat ini membaca akselerasi 3-axis dari sensor ADXL345, memproses sinyal melalui pipeline DSP (Digital Signal Processing), dan menampilkan tiga parameter utama di layar OLED:

| Parameter | Nilai | Satuan | Makna |
|---|---|---|---|
| **RMS Velocity** | mis. `2.34` | mm/s | Seberapa kencang mesin bergetar (energi rata-rata) |
| **Peak Acceleration** | mis. `0.87` | m/s² | Benturan/lonjakan getaran terkuat yang terdeteksi |
| **Dominant Frequency** | mis. `49.2` | Hz | Pada frekuensi berapa getaran paling dominan terjadi |

Ketiga angka ini adalah "vital sign" sebuah mesin yang berputar — seperti tekanan darah, denyut nadi, dan suhu tubuh manusia. Mesin yang sehat menghasilkan angka rendah dan stabil; mesin yang mulai rusak memperlihatkan kenaikan yang jelas.

---

## 2. Arsitektur Sistem

Sistem dirancang menggunakan pola **node-based** yang terinspirasi dari ROS2. Setiap modul memiliki satu tanggung jawab, berkomunikasi lewat data buffer, dan berjalan secara non-blocking (tidak ada `delay()` dalam loop utama).

```
┌─────────┐   raw m/s²    ┌─────────────┐  corrected   ┌────────┐
│  Sensor │ ─────────────▶│ Calibration │─────────────▶│ Filter │
│  Node   │               │    Node     │              │  Node  │
└─────────┘               └─────────────┘              └───┬────┘
    ▲                                                       │ filtered
    │ isReady() poll                                        │ accelBuffer[512]
    │ @ 800 Hz                                              ▼
    │                                         ┌─────────────────────────┐
    │                                         │     Processing Node     │
    │                                         │  integrate → vel → RMS  │
    │                                         │  peak accel             │
    │                                         └──────────┬──────────────┘
    │                                                    │
    │                                       ┌────────────┴────────────┐
    │                                       │                         │
    │                                  ┌────▼─────┐             ┌─────▼────┐
    │                                  │ FFT Node │             │ Display  │
    │                                  │ (freq)   │             │  Node    │
    │                                  └──────────┘             └──────────┘
    │
  [loop()]
```

**Tabel Node:**

| Node | File | Fungsi |
|---|---|---|
| `Sensor` | `sensor.cpp/.h` | Baca raw data ADXL345 via I2C, konversi ke m/s² |
| `Calibration` | `calibration.cpp/.h` | Koreksi offset statis + kompensasi gravitasi |
| `Filter` | `filter.cpp/.h` | DC removal, High-Pass Filter, Low-Pass Filter |
| `Processing` | `processing.cpp/.h` | Integrasi akselerasi → kecepatan, hitung RMS & Peak |
| `FFTNode` | `fft.cpp/.h` | Hanning window + FFT → frekuensi dominan |
| `Display` | `display.cpp/.h` | Render hasil ke OLED tiap 500 ms |

**Siklus kerja sistem:**

1. **Fase Kalibrasi** (startup, ~640 ms, blocking): 512 sampel diambil saat sensor diam untuk menghitung offset.
2. **Loop Akuisisi** (non-blocking, terus menerus): tiap 1.25 ms sensor dibaca, hasilnya dimasukkan ke buffer.
3. **Pipeline Pemrosesan** (dipicu setiap 512 sampel terkumpul = tiap 640 ms): filter → proses → FFT → display.

---

## 3. Hardware & Wiring

```
ESP32-C3 Super Mini          ADXL345 / OLED SSD1306
─────────────────            ──────────────────────
GPIO 8 (SDA)  ─────────────  SDA  (kedua device, I2C shared bus)
GPIO 9 (SCL)  ─────────────  SCL  (kedua device, I2C shared bus)
3.3V          ─────────────  VCC  (kedua device)
GND           ─────────────  GND  (kedua device)
              ─────────────  ADXL345 SDO → 3.3V (I2C addr = 0x53)

I2C Address:
  ADXL345  → 0x53
  SSD1306  → 0x3C
  I2C Clock → 400 kHz (Fast Mode)
```

---

## 4. Struktur File Proyek

```
Vibration_Monitoring_ISO10816/
├── Vibration_Monitoring_ISO10816.ino  ← Main: setup(), loop(), pipeline orchestration
├── config.h                           ← Semua parameter konfigurasi (single source of truth)
├── sensor.h / sensor.cpp              ← ADXL345 driver (akuisisi + konversi unit)
├── calibration.h / calibration.cpp    ← Kalibrasi offset statis
├── filter.h / filter.cpp              ← DC removal + IIR HPF + IIR LPF
├── processing.h / processing.cpp      ← Integrasi numerik + RMS + Peak
├── fft.h / fft.cpp                    ← FFT analysis (frekuensi dominan)
├── display.h / display.cpp            ← OLED rendering
└── build/esp32.esp32.esp32c3/         ← Binary hasil kompilasi
    ├── main.ino.bin                   ← Firmware siap flash
    └── ...
```

---

## 5. Flow Data: Dari Sensor ke Display

Bagian ini menjelaskan perjalanan setiap variabel dari sensor fisik sampai ke angka yang tampil di layar, untuk ketiga parameter secara terpisah.

---

### 5.1 Flow Parameter: RMS Velocity (mm/s)

```
[ADXL345 Hardware]
    │
    │  6 byte raw I2C burst read @ 800 Hz
    ▼
[rawBuf[6]] — byte mentah dari register DATAX0..DATAZ1
    │
    │  Reconstruct 16-bit signed int:
    │    rawX = (rawBuf[1]<<8) | rawBuf[0]   (little-endian)
    │    rawY = (rawBuf[3]<<8) | rawBuf[2]
    │    rawZ = (rawBuf[5]<<8) | rawBuf[4]
    │
    │  Konversi ke fisik (m/s²):
    │    sample.x = rawX × ADXL345_SCALE_MS2
    │    sample.y = rawY × ADXL345_SCALE_MS2
    │    sample.z = rawZ × ADXL345_SCALE_MS2
    │    (ADXL345_SCALE_MS2 = 3.9e-3 × 9.80665 = 0.038246 m/s²/LSB)
    ▼
[AccelSample] — {x, y, z} dalam m/s² (raw, belum dikoreksi)
    │
    │  Kalibrasi: koreksi offset statis
    │    calibrated.x = raw.x - offsetX
    │    calibrated.y = raw.y - offsetY
    │    calibrated.z = raw.z - offsetZ
    │    (offsetX/Y/Z = rata-rata 512 sampel saat diam)
    ▼
[AccelSample] — {x, y, z} dalam m/s² (sudah dikoreksi, gravitasi dihilangkan)
    │
    │  Ekstraksi skalar (AXIS_SELECT=3, magnitude):
    │    scalar = sqrt(x² + y² + z²)
    ▼
[accelBuffer[bufferIndex]] ← append scalar
    │
    │  (Ulangi 512 kali @ 800 Hz = 640 ms)
    │
    ▼ Buffer penuh (512 sampel)
    │
    │  FILTER PIPELINE (in-place):
    │
    │  Stage 1 – DC Removal:
    │    mean = Σ(accelBuffer[i]) / 512
    │    accelBuffer[i] -= mean  (untuk semua i)
    │
    │  Stage 2 – High-Pass IIR:
    │    y[n] = α_hp × (y[n-1] + x[n] - x[n-1])
    │    α_hp = τ/(τ+dt) = 0.99999 (fc=2 Hz, fs=800 Hz)
    │
    │  Stage 3 – Low-Pass IIR:
    │    y[n] = α_lp × x[n] + (1-α_lp) × y[n-1]
    │    α_lp = dt/(τ+dt) ≈ 0.9997 (fc=390 Hz, fs=800 Hz)
    ▼
[accelBuffer[512]] — sinyal getaran bersih dalam m/s²
    │
    │  INTEGRASI TRAPESOID → kecepatan:
    │    v[n] = v[n-1] + (dt/2) × (a[n] + a[n-1])
    │    dt = 1/800 = 0.00125 s
    ▼
[_velBuf[512]] — kecepatan dalam m/s
    │
    │  Drift removal (hilangkan DC velocity):
    │    mean_v = Σ(v[i]) / 512
    │    v[i] -= mean_v
    │
    │  Scaling ke mm/s:
    │    v[i] *= 1000.0
    ▼
[_velBuf[512]] — kecepatan dalam mm/s
    │
    │  RMS Computation:
    │    rms = sqrt( Σ(v[i]²) / 512 )
    ▼
[ProcessingResult.rmsVelocity_mms]
    │
    │  Alarm check:
    │    alarmActive = (rms > 7.1 mm/s)   ← ISO 10816 Zone C/D boundary
    ▼
[Display Row 1] → "RMS:  2.34 mm/s"
```

---

### 5.2 Flow Parameter: Peak Acceleration (m/s²)

```
[accelBuffer[512]] — sinyal terfilter dalam m/s²
    │
    │  (Sama dengan flow RMS sampai tahap filter selesai)
    │
    │  Peak computation — langsung dari buffer akselerasi,
    │  SEBELUM integrasi ke kecepatan:
    │
    │    maxAbs = 0
    │    for i in 0..511:
    │        a = |accelBuffer[i]|
    │        if a > maxAbs: maxAbs = a
    ▼
[ProcessingResult.peakAccel_ms2]
    │
    ▼
[Display Row 2] → "Pk :  0.087 m/s^2"
```

> **Catatan penting:** Peak diambil dari buffer **akselerasi** (m/s²), bukan dari buffer kecepatan. Ini berarti Peak merepresentasikan lonjakan gaya terbesar yang dialami sensor dalam satu window 640 ms.

---

### 5.3 Flow Parameter: Dominant Frequency (Hz)

```
[accelBuffer[512]] — sinyal terfilter dalam m/s²
    │
    │  (Sama dengan flow RMS sampai tahap filter selesai)
    │
    │  Copy ke FFT array:
    │    _vReal[i] = accelBuffer[i]
    │    _vImag[i] = 0.0
    │
    │  Hanning Window (kurangi spectral leakage):
    │    w[n] = 0.5 × (1 - cos(2π·n/N))
    │    _vReal[i] *= w[i]
    │
    │  Forward FFT (Cooley-Tukey, radix-2, N=512):
    │    H[k] = Σ x[n] · e^(-j2πkn/N)    k = 0..511
    │
    │  Complex to Magnitude:
    │    |H[k]| = sqrt(_vReal[k]² + _vImag[k]²)
    │    (hasil disimpan kembali ke _vReal[k])
    │
    │  Frequency-to-bin mapping:
    │    f[k] = k × (fs/N) = k × (800/512) = k × 1.5625 Hz
    │
    │  Peak search dalam range [5 Hz, 400 Hz]:
    │    binMin = ceil(5.0 / 1.5625) = bin ke-4
    │    binMax = floor(400.0 / 1.5625) = bin ke-256
    │
    │    maxMag = -1; peakBin = binMin
    │    for k = binMin..binMax:
    │        if _vReal[k] > maxMag:
    │            maxMag = _vReal[k]
    │            peakBin = k
    │
    │    dominantFreq = peakBin × 1.5625 Hz
    ▼
[FFTResult.dominantFreq_Hz]
    │
    ▼
[Display Row 3] → "Fr :   49.22 Hz"
```

---

## 6. Penjelasan Matematis Lengkap

Bagian ini menjelaskan setiap langkah kalkulasi secara mendalam agar dapat dipahami oleh siapapun yang memahami dasar fisika dan matematika.

---

### 6.1 Tahap 1 – Akuisisi Sensor ADXL345

**Apa itu ADC dan LSB?**

Sensor ADXL345 adalah accelerometer digital. Di dalamnya terdapat ADC (Analog-to-Digital Converter) yang mengubah getaran fisik (gaya) menjadi angka digital. Dalam mode "full resolution" yang digunakan sistem ini, ADC menghasilkan angka 13-bit bertanda (signed), artinya nilainya bisa dari -4096 hingga +4095.

Satuan terkecil output sensor disebut **LSB (Least Significant Bit)**. Setiap LSB setara dengan:

```
3.9 mg/LSB  →  (3.9 × 10⁻³ g) per LSB
```

Di mana `g` adalah percepatan gravitasi = 9.80665 m/s².

**Konversi LSB ke m/s²:**

```
ADXL345_SCALE_MS2 = 3.9 × 10⁻³ × 9.80665
                  = 0.038246 m/s² per LSB

Contoh:
  rawX = 256 LSB
  accel_x = 256 × 0.038246 = 9.79 m/s²  (≈ 1g, arah X)
```

**Rekonstruksi nilai dari byte I2C:**

ADXL345 mengirim data dalam format Little-Endian (byte rendah dulu):
```
rawX = (int16_t)((rawBuf[1] << 8) | rawBuf[0])
```
Cast ke `int16_t` memastikan bit tanda (sign bit) ditafsirkan dengan benar untuk nilai negatif.

---

### 6.2 Tahap 2 – Kalibrasi Statis

**Masalah yang diselesaikan:**

Ketika sensor diam dan diletakkan datar, ia tetap membaca akselerasi non-nol karena dua alasan:
1. **Gravitasi bumi (9.81 m/s²):** Gravitasi selalu ada dan memengaruhi salah satu axis.
2. **Bias mekanik sensor:** Setiap chip ADXL345 punya sedikit offset bawaan dari pabrik.

Kedua hal ini harus dihilangkan agar yang tersisa hanya getaran dinamis dari mesin.

**Cara kerjanya:**

Saat startup, sistem mengambil 512 sampel dari sensor yang sedang diam, lalu menghitung rata-rata:

```
offsetX = (1/N) × Σ rawSample.x    N = 512
offsetY = (1/N) × Σ rawSample.y
offsetZ = (1/N) × Σ rawSample.z
```

Nilai rata-rata ini adalah **gambaran bias statis** (gravitasi + offset sensor). Lalu setiap sampel baru dikoreksi:

```
corrected.x = raw.x - offsetX
corrected.y = raw.y - offsetY
corrected.z = raw.z - offsetZ
```

**Hasilnya:** Saat mesin diam, output mendekati nol. Hanya getaran dinamis (AC) yang tersisa.

---

### 6.3 Tahap 3 – Ekstraksi Skalar 3-Axis

Sistem dikonfigurasi dengan `AXIS_SELECT = 3` (magnitude), yang berarti ketiga axis digabungkan menjadi satu nilai skalar menggunakan **norma Euclidean** (jarak 3D):

```
scalar = sqrt(x² + y² + z²)
```

**Mengapa magnitude?**

Mesin bergetar ke segala arah. Jika hanya memonitor satu axis, bisa terjadi kesalahan baca jika arah getaran tidak sejajar dengan axis tersebut. Magnitude tidak peduli arah — ia mengukur **besarnya total getaran** secara omnidireksional.

---

### 6.4 Tahap 4 – Filter Sinyal

Filter bekerja pada seluruh buffer 512 sampel secara berurutan (in-place).

#### Stage 1: DC Removal (Penghapusan Komponen DC)

Meskipun sudah dikalibrasi, masih bisa ada sisa offset kecil. DC Removal menghapusnya dengan cara mengurangi rata-rata buffer:

```
mean = (1/512) × Σ accelBuffer[i]
accelBuffer[i] -= mean    (untuk semua i)
```

Ini seperti "meluruskan" sinyal agar titik tengahnya berada di nol.

#### Stage 2: High-Pass Filter (HPF) — Hapus Frekuensi Rendah

HPF memblokir komponen frekuensi rendah (< 2 Hz) dari sinyal. Ini penting untuk menghilangkan **drift** (pergeseran baseline lambat) yang bisa merusak hasil integrasi.

Filter yang digunakan adalah IIR (Infinite Impulse Response) orde-1, yang merupakan aproksimasi digital dari RC HPF analog.

**Penurunan koefisien:**

Dari model analog RC HPF dengan fungsi transfer `H(s) = sτ/(1+sτ)` di mana `τ = 1/(2πfc)`, dilakukan transformasi bilinear (metode Tustin):

```
τ_hp = 1 / (2π × fc_hp)
     = 1 / (2π × 2.0)
     = 0.07958 s

dt = 1/fs = 1/800 = 0.00125 s

α_hp = τ_hp / (τ_hp + dt)
     = 0.07958 / (0.07958 + 0.00125)
     = 0.98454  (mendekati 1 → hampir semua frekuensi lewat kecuali DC)
```

**Persamaan rekursi (difference equation):**

```
y[n] = α_hp × (y[n-1] + x[n] - x[n-1])
```

Di mana `x[n]` adalah sampel input saat ini dan `y[n]` adalah output. Filter ini "mengingat" selisih antara sampel sekarang dan sebelumnya, sehingga sinyal yang berubah lambat (frekuensi rendah) diblokir.

**Intuisi:** Bayangkan HPF seperti filter "yang hanya peduli dengan perubahan cepat." Sinyal yang berubah pelan (getaran < 2 Hz) dianggap noise dan dibuang.

#### Stage 3: Low-Pass Filter (LPF) — Hapus Frekuensi Tinggi

LPF memblokir komponen frekuensi tinggi (> 390 Hz) — noise elektronik dan artefak di atas batas analisis.

```
τ_lp = 1 / (2π × fc_lp)
     = 1 / (2π × 390)
     = 0.000408 s

α_lp = dt / (τ_lp + dt)
     = 0.00125 / (0.000408 + 0.00125)
     = 0.7539

Difference equation:
  y[n] = α_lp × x[n] + (1 - α_lp) × y[n-1]
       = 0.754 × x[n] + 0.246 × y[n-1]
```

Ini adalah **Exponential Moving Average (EMA)** — output adalah campuran antara nilai baru (75.4%) dan nilai lama (24.6%). Nilai yang berubah terlalu cepat (frekuensi tinggi) "diperhalus."

---

### 6.5 Tahap 5 – Kalkulasi RMS Velocity

Ini adalah tahap paling kompleks dan paling penting dalam sistem. Prosesnya terdiri dari tiga sub-tahap.

#### Sub-tahap A: Integrasi Numerik (Akselerasi → Kecepatan)

**Mengapa harus diintegrasikan?**

Sensor menghasilkan **akselerasi** (m/s²), tetapi standar ISO 10816 menggunakan **kecepatan getaran** (mm/s) sebagai metrik utama. Secara matematis, kecepatan adalah integral dari akselerasi terhadap waktu:

```
v(t) = ∫ a(t) dt
```

Sistem menggunakan **metode trapezoidal** (lebih akurat dari Euler biasa):

```
v[n] = v[n-1] + (dt/2) × (a[n] + a[n-1])

di mana:
  dt = 0.00125 s (interval sampling)
  a[n] = akselerasi sampel ke-n (m/s²)
  v[n] = kecepatan sampel ke-n (m/s)
```

**Perbandingan Euler vs Trapesoid:**

```
Euler (kurang akurat):
  v[n] = v[n-1] + dt × a[n-1]
  → menggunakan akselerasi di ujung kiri interval saja

Trapesoid (lebih akurat):
  v[n] = v[n-1] + (dt/2) × (a[n] + a[n-1])
  → menggunakan rata-rata akselerasi di kedua ujung interval
  → error orde O(dt²) vs O(dt) untuk Euler
```

**Contoh numerik:**

```
Misal: a[0] = 0.05 m/s²,  a[1] = 0.08 m/s²,  dt = 0.00125 s
v[0] = 0 (kondisi awal)
v[1] = 0 + (0.00125/2) × (0.08 + 0.05) = 0.0000813 m/s
```

#### Sub-tahap B: Drift Removal dari Velocity

Integrasi numerik punya masalah: sisa offset kecil di sinyal akselerasi (walaupun sudah difilter) akan **terakumulasi** menjadi drift di sinyal kecepatan. Misalnya, offset 0.001 m/s² selama 640 ms menghasilkan drift kecepatan 0.64 mm/s — yang bisa mengacaukan RMS.

Solusinya: hapus rata-rata (mean) dari velocity buffer:

```
mean_v = (1/512) × Σ v[i]
v[i] -= mean_v
```

Setelah ini, kecepatan berpusat di nol — hanya komponen osilasi (vibrasi AC) yang tersisa.

#### Sub-tahap C: Scaling dan RMS

Konversi dari m/s ke mm/s:
```
v[i] *= 1000.0   → satuan sekarang mm/s
```

Lalu hitung **Root Mean Square (RMS)**:

```
RMS = sqrt( (1/N) × Σ v[i]² )
    = sqrt( (v[0]² + v[1]² + ... + v[511]²) / 512 )
```

**Apa arti RMS secara fisik?**

RMS adalah "nilai efektif" dari sinyal yang berubah-ubah. Untuk sinyal sinusoidal murni (misal getaran mesin yang sempurna pada 50 Hz):

```
v(t) = V_peak × sin(2π × 50 × t)

RMS = V_peak / √2 ≈ 0.707 × V_peak
```

Artinya: jika puncak getaran adalah 10 mm/s, maka RMS ≈ 7.07 mm/s. RMS mencerminkan **energi rata-rata** getaran, bukan hanya puncaknya.

---

### 6.6 Tahap 6 – Kalkulasi Peak Acceleration

Ini adalah kalkulasi paling sederhana — cari nilai absolut terbesar dari buffer akselerasi terfilter:

```
peakAccel = max(|accelBuffer[0]|, |accelBuffer[1]|, ..., |accelBuffer[511]|)
```

Dalam kode:
```cpp
float maxAbs = 0.0f;
for (uint16_t i = 0; i < len; i++) {
    float a = fabsf(buf[i]);
    if (a > maxAbs) maxAbs = a;
}
return maxAbs;
```

**Mengapa penting?**

Peak acceleration mengungkap **kejutan (shock) atau benturan** impulsif yang mungkin tidak terlihat dari RMS. Misalnya, ada bantalan bearing yang retak menghasilkan ketukan singkat — RMS mungkin masih rendah, tetapi Peak akan melonjak tajam.

---

### 6.7 Tahap 7 – Kalkulasi Dominant Frequency (FFT)

FFT (Fast Fourier Transform) adalah algoritma yang menganalisis sebuah sinyal di domain waktu dan membongkarnya menjadi komponen-komponen frekuensi penyusunnya.

**Analogi sederhana:** Bayangkan Anda mendengar sebuah akord musik. Telinga Anda bisa membedakan bahwa akord itu terdiri dari nada Do, Mi, dan Sol secara bersamaan. FFT melakukan hal yang sama pada sinyal getaran — ia mengidentifikasi "nada-nada" frekuensi apa yang ada dan seberapa kuat masing-masing.

#### Step 1: Hanning Window

Sebelum FFT, buffer dikalikan dengan fungsi jendela Hanning:

```
w[n] = 0.5 × (1 - cos(2π·n/N))    n = 0, 1, ..., N-1

_vReal[n] = accelBuffer[n] × w[n]
```

**Mengapa perlu window?**

FFT mengasumsikan sinyal yang dianalisis adalah **periodik dan tak terbatas**. Namun kita hanya punya potongan singkat (512 sampel). Jika frekuensi sinyal tidak pas kelipatan resolusi FFT, ujung-ujung buffer tidak tersambung mulus, menciptakan "lompatan" artifisial yang memunculkan frekuensi palsu (disebut spectral leakage).

Hanning window "melembutkan" kedua ujung buffer ke nol secara halus, sehingga efek ini diminimalkan. Tradeoff-nya: resolusi frekuensi sedikit berkurang, tetapi hasilnya jauh lebih bersih.

#### Step 2: Forward FFT

Menggunakan algoritma Cooley-Tukey radix-2 (jumlah sampel harus pangkat 2):

```
H[k] = Σ(n=0 to N-1) x[n] × e^(-j2πkn/N)    k = 0, 1, ..., N-1
```

Hasil H[k] adalah bilangan kompleks. Sistem ini menggunakan array `_vReal[]` dan `_vImag[]` untuk bagian real dan imajiner.

#### Step 3: Complex to Magnitude

Setelah FFT, setiap bin k memiliki komponen real dan imajiner. Magnitudonya:

```
|H[k]| = sqrt(_vReal[k]² + _vImag[k]²)
```

Nilai ini disimpan kembali ke `_vReal[k]`.

#### Step 4: Frequency Resolution dan Bin Mapping

**Resolusi frekuensi** (seberapa rapat bin-bin FFT):

```
Δf = fs / N = 800 / 512 = 1.5625 Hz/bin
```

Artinya, setiap bin merepresentasikan rentang 1.5625 Hz. Bin ke-k bersesuaian dengan frekuensi:

```
f[k] = k × Δf = k × 1.5625 Hz

Contoh:
  Bin 0  → 0 Hz    (DC)
  Bin 1  → 1.5625 Hz
  Bin 32 → 50.0 Hz
  Bin 64 → 100.0 Hz
  Bin 256 → 400.0 Hz  (Nyquist, batas atas)
```

#### Step 5: Peak Search

System mencari bin dengan magnitude tertinggi dalam range aman [5 Hz – 400 Hz]:

```
binMin = ceil(5.0 / 1.5625) = 4
binMax = floor(400.0 / 1.5625) = 256

Temukan peakBin = argmax { |H[k]| untuk k = 4..256 }

dominantFreq = peakBin × 1.5625 Hz
```

**Nyquist Theorem:** Frekuensi maksimum yang bisa dianalisis adalah setengah dari sampling rate:

```
f_max = fs / 2 = 800 / 2 = 400 Hz
```

Di atas 400 Hz, sinyal tidak bisa direpresentasikan dengan benar pada sampling rate 800 Hz.

---

## 7. Perbedaan RMS, Peak, dan Frequency

Ketiga parameter ini mengukur aspek berbeda dari fenomena getaran yang sama. Memahami perbedaannya penting untuk diagnosis mesin yang tepat.

### Analogi Dunia Nyata

Bayangkan Anda sedang mengamati laut:

| Parameter Laut | Analogi Getaran | Parameter Sistem |
|---|---|---|
| **Tinggi gelombang rata-rata** | Energi getaran rata-rata | **RMS Velocity** |
| **Gelombang tertinggi** dalam satu hari | Benturan terkuat yang terjadi | **Peak Acceleration** |
| **Frekuensi gelombang** (misal 0.1 Hz = gelombang tiap 10 detik) | Seberapa cepat mesin bergetar | **Dominant Frequency** |

---

### RMS Velocity (mm/s) — "Energi Rata-Rata Getaran"

**Definisi fisis:** RMS Velocity mengukur nilai efektif dari kecepatan getaran. Ia merepresentasikan "berapa banyak energi mekanik rata-rata yang dilepaskan mesin ke lingkungannya per satuan waktu."

**Karakteristik:**
- Mengintegrasikan waktu → sensitif terhadap getaran yang **berkelanjutan**
- Tidak terlalu terpengaruh oleh lonjakan sesaat
- **Standar industri** (ISO 10816): ini adalah parameter utama untuk menilai kondisi mesin
- Naik perlahan seiring keausan mesin, sehingga bagus untuk **trend monitoring**

**Kapan naik?** Ketika mesin tidak balance, bearing aus, misalignment poros, atau resonansi struktur.

---

### Peak Acceleration (m/s²) — "Benturan Terkuat"

**Definisi fisis:** Peak Acceleration adalah nilai absolut maksimum dari percepatan getaran yang terdeteksi dalam satu window pengukuran (640 ms). Karena F = m×a (Hukum Newton), akselerasi tinggi = gaya impak besar.

**Karakteristik:**
- Sangat sensitif terhadap kejadian **impulsif singkat** (shock, ketukan, benturan)
- Tidak merepresentasikan kondisi rata-rata — satu benturan keras akan membuat Peak tinggi meskipun mesin secara keseluruhan baik-baik saja
- Berguna untuk deteksi **spalling bearing** (bearing yang mulai terkelupas), gesekan gear, atau benturan mekanik

**Kapan naik?** Ketika ada ketidakmulusan mekanik: bearing crack, gear tooth breakage, atau benda asing di dalam mesin.

---

### Dominant Frequency (Hz) — "Ritme Getaran"

**Definisi fisis:** Dominant Frequency adalah frekuensi di mana getaran mesin paling kuat terjadi. Ini seperti "nada dasar" getaran mesin.

**Karakteristik:**
- Tidak mengukur intensitas, hanya **pola/ritme** getaran
- Memberikan **petunjuk sumber masalah** karena berbagai kerusakan menghasilkan frekuensi karakteristik
- Frekuensi relatif stabil selama kondisi mesin tidak berubah drastis

**Tabel diagnosis frekuensi:**

| Frekuensi | Kemungkinan Penyebab |
|---|---|
| 1× rpm mesin | Imbalance (ketidakseimbangan rotor) |
| 2× rpm | Misalignment poros |
| 3× rpm | Looseness mekanik |
| rpm × jumlah baling | Blade pass frequency (kipas/pompa) |
| Tinggi, acak | Bearing defect |

Contoh: mesin berputar 3000 rpm = 50 Hz. Jika dominantFreq ≈ 50 Hz → kemungkinan imbalance pada rotor.

---

### Ringkasan Perbandingan

```
┌─────────────────┬────────────────┬───────────────┬───────────────────┐
│ Parameter       │ Domain         │ Deteksi       │ Berfungsi untuk   │
├─────────────────┼────────────────┼───────────────┼───────────────────┤
│ RMS Velocity    │ Waktu (energy) │ Gradual wear  │ Overall severity  │
│ Peak Accel      │ Waktu (max)    │ Shock/impact  │ Fault detection   │
│ Dominant Freq   │ Frekuensi      │ Sumber masalah│ Fault diagnosis   │
└─────────────────┴────────────────┴───────────────┴───────────────────┘
```

---

## 8. Kenapa Satuan Berbeda-Beda?

Ini adalah pertanyaan yang paling sering membingungkan. Jawabannya adalah karena setiap parameter mengukur **besaran fisika yang berbeda**, dan masing-masing besaran punya satuan standar internasional (SI) sendiri.

---

### RMS Velocity → Satuan: mm/s

**Kenapa kecepatan (bukan akselerasi)?**

Standar ISO 10816 menetapkan kecepatan sebagai parameter utama karena kecepatan getaran memiliki korelasi terbaik dengan **kerusakan mekanik** dan **umur pakai** mesin. Ini berdasarkan penelitian empiris bertahun-tahun.

Alasannya secara fisis: kecepatan berkaitan langsung dengan **energi kinetik** (E = ½mv²) dan **stress mekanik** pada material. Gaya gesek dan keausan yang merusak mesin berbanding lurus dengan kecepatan, bukan akselerasi.

**Kenapa mm/s dan bukan m/s?**

Getaran mesin yang normal sangat kecil — umumnya dalam kisaran 0.5 – 10 mm/s. Jika dinyatakan dalam m/s, nilainya adalah 0.0005 – 0.01 m/s, yang tidak praktis untuk dibaca. Mengalikan dengan 1000 (mm/s) memberikan angka yang "ramah manusia."

```
Mesin baru (kondisi bagus): 0.5 – 2.3 mm/s RMS
Mesin perlu perhatian:      2.3 – 7.1 mm/s RMS
Mesin berbahaya:            > 11.2 mm/s RMS
```

**Kenapa RMS, bukan nilai rata-rata biasa?**

Sinyal getaran adalah gelombang yang bergantian positif-negatif. Jika dihitung rata-rata sederhana dari sinyal simetris, hasilnya mendekati nol — tidak informatif. RMS memastikan nilai positif dan negatif sama-sama berkontribusi (dikuadratkan terlebih dahulu):

```
Rata-rata biasa: (1 + 0 - 1 + 0 + 1 + 0 - 1 + 0) / 8 = 0  ← tidak informatif
RMS: sqrt((1² + 0² + 1² + 0² + 1² + 0² + 1² + 0²) / 8)
   = sqrt(4/8) = sqrt(0.5) = 0.707  ← informatif
```

---

### Peak Acceleration → Satuan: m/s²

**Kenapa akselerasi (bukan kecepatan)?**

Peak diukur dalam akselerasi karena tujuannya adalah mendeteksi **gaya impak** dan **shock**, dan gaya berbanding lurus dengan akselerasi (F = ma). Akselerasi yang tiba-tiba tinggi berarti ada gaya mekanik besar yang tiba-tiba — ini ciri khas kerusakan bearing atau benturan.

**Kenapa m/s² dan bukan mm/s²?**

Untuk Peak, nilai akselerasi bisa cukup besar (1 – 50 m/s² atau lebih untuk kondisi ekstrem), sehingga satuan m/s² lebih praktis. Berbeda dengan kecepatan yang sangat kecil dalam m/s.

**Kenapa bukan dinyatakan dalam "g"?**

`g` (gravitasi) = 9.80665 m/s² adalah satuan yang umum di industri aerospace. Namun untuk pemantauan mesin industri, m/s² lebih sering digunakan karena langsung dari SI tanpa konversi.

---

### Dominant Frequency → Satuan: Hz

**Kenapa Hz?**

Hz (Hertz) adalah satuan SI untuk frekuensi: jumlah siklus per detik. Ini adalah satuan universal untuk segala jenis fenomena periodik. Tidak ada satuan alternatif yang relevan di sini.

**Hubungan dengan RPM mesin:**

```
RPM (putaran per menit) → Hz (putaran per detik):

f [Hz] = RPM / 60

Contoh:
  Mesin 3000 RPM → f = 3000/60 = 50 Hz
  Mesin 1500 RPM → f = 1500/60 = 25 Hz
```

Jika `dominantFreq ≈ 50 Hz` pada mesin 3000 RPM, kemungkinan besar getaran disebabkan oleh **satu putaran penuh** mesin (1× order) — tanda klasik imbalance.

---

## 9. Standar ISO 10816 – Alarm Threshold

Sistem mengimplementasikan zona kondisi mesin sesuai **ISO 10816-3** (untuk mesin dengan motor > 15 kW):

```
Zone A: RMS < 2.3 mm/s   → Mesin baru, kondisi sempurna
Zone B: 2.3 – 4.5 mm/s   → Operasi normal jangka panjang, diterima
Zone C: 4.5 – 11.2 mm/s  → Investigasi! Getaran mulai tidak normal
Zone D: > 11.2 mm/s       → BAHAYA! Mesin harus segera dihentikan

Alarm sistem dikonfigurasi di: RMS_ALARM_THRESHOLD_MM_S = 7.1 mm/s
(batas tengah Zone C — aman untuk infrastruktur prototype)
```

Ketika `rmsVelocity_mms > 7.1 mm/s`:
- `alarmActive = true`
- Display menampilkan indikator `[!]` di pojok kanan atas
- Pesan peringatan dikirim via Serial

---

## 10. Parameter Konfigurasi

Semua parameter dapat diubah di `config.h` tanpa menyentuh kode logika.

| Parameter | Nilai Default | Keterangan |
|---|---|---|
| `SAMPLING_RATE_HZ` | 800 | Kecepatan sampling sensor (Hz) |
| `BUFFER_SIZE` | 512 | Jumlah sampel per window (harus pangkat 2) |
| `HPF_CUTOFF_HZ` | 2.0 | Cutoff High-Pass Filter (Hz) |
| `LPF_CUTOFF_HZ` | 390.0 | Cutoff Low-Pass Filter (Hz) |
| `CALIBRATION_SAMPLES` | 512 | Jumlah sampel kalibrasi saat startup |
| `GRAVITY_AXIS` | 2 (Z) | Axis yang sejajar gravitasi |
| `AXIS_SELECT` | 3 | 0=X, 1=Y, 2=Z, 3=Magnitude |
| `RMS_ALARM_THRESHOLD_MM_S` | 7.1 | Threshold alarm ISO 10816 (mm/s) |
| `VELOCITY_SCALE` | 1000.0 | Skala output: 1.0=m/s, 1000.0=mm/s |
| `DISPLAY_UPDATE_MS` | 500 | Interval refresh OLED (ms) |
| `FFT_FREQ_MIN_HZ` | 5.0 | Batas bawah pencarian peak FFT (Hz) |
| `FFT_FREQ_MAX_HZ` | 400.0 | Batas atas pencarian peak FFT (Hz) |

**Konstanta turunan (otomatis dihitung):**

```
SAMPLE_INTERVAL_US  = 1,000,000 / 800 = 1,250 µs per sampel
INTEGRATION_DT      = 1 / 800 = 0.00125 s
FFT_FREQ_RESOLUTION = 800 / 512 = 1.5625 Hz/bin
Buffer period       = 512 / 800 = 640 ms per window
```

---

## 11. Keterbatasan Sistem

Penting untuk memahami batas kemampuan sistem agar hasil tidak salah diinterpretasikan:

| Keterbatasan | Detail |
|---|---|
| **Frekuensi maksimum** | Andal sampai 400 Hz. Di atas 400 Hz, ADXL345 mulai meredam sinyal sendiri karena filter anti-alias internalnya. |
| **Resolusi FFT** | 1.5625 Hz/bin. Dua sumber getaran yang berdekatan (misal 49 Hz dan 50 Hz) tidak dapat dibedakan. |
| **Noise floor** | ADXL345: ~1.5 mg RMS noise pada 800 Hz. Setara ~0.02 mm/s kecepatan — ini batas deteksi minimum. |
| **ADC resolution** | 13-bit efektif = 3.9 mg/LSB = 0.038 m/s² per step. Getaran halus di bawah ini tidak terdeteksi. |
| **Drift integrasi** | Integrasi numerik mengakumulasi error. Diatasi dengan drift removal, tetapi tidak sempurna pada getaran sangat rendah frekuensi (< 5 Hz). |
| **ODR accuracy** | ADXL345 ODR toleransi ±1%. Frekuensi yang dihitung FFT bisa meleset ~1% dari nilai sebenarnya. |
| **Kalibrasi** | Hanya offset statis. Tidak ada kompensasi suhu atau koreksi scale factor. |
| **I2C latency** | Setiap burst read 6 byte membutuhkan ~15 µs. Pada 800 Hz, ini menggunakan ~1.2% waktu CPU. |

---

## 12. Library yang Digunakan

Install semua library berikut via **Arduino Library Manager** sebelum kompilasi:

| Library | Version | Fungsi |
|---|---|---|
| **Adafruit SSD1306** | ≥ 2.5.7 | Driver display OLED SSD1306 via I2C |
| **Adafruit GFX Library** | ≥ 1.11.9 | Primitif grafis (text, line, rect) |
| **arduinoFFT** (by kosme) | ≥ 1.9.2 | FFT dengan float support dan Hanning window |

**Board Package:**
- ESP32 by Espressif Systems v2.0.14
- Target board: `esp32 > ESP32C3 Dev Module`

---

*Dokumentasi ini dibuat berdasarkan analisis source code lengkap proyek Vibration_Monitoring_ISO10816 v1.0.0.*
