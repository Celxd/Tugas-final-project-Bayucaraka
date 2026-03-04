# Data Kelompok

| Nama | NRP | Prodi | Sub-divisi |
| --- | --- | --- | --- |
| Alif Gibran Muhammad Ervin | 5026251070 | Sistem Informasi | Programming |
| Kadek Rahayu Pradnyani | 2040251079 | Teknik Elektro Otomasi | Electrical |
| Ahmad Syahreza Nouval Siregar | | Matematika | Mekanik |

# Struktur Project
```
.
├── Electrical/
│   ├── Old/            (Demo Elektrikal)
│   ├── Revised/        (Revisi Elektrikal)
├── Mechanical/
│   ├── ...
├── Programming/
│   ├── revised_ros_ws/ (Revisi Programming)
│   └── OLD_final.py    (Demo Programming)
├── .gitignore
└── README.md
```

# Electrical
Beberapa peningkatan atau perbaikan yang dilakukan:  
- Ground plane ditempatkan di layer atas, sehingga distribusi ground lebih rapi dan stabil.
- Jalur sinyal dan jalur lainnya berada di layer bawah, untuk mempermudah routing dan meningkatkan connectivity antar komponen.
- Desain ini dibuat dengan mempertimbangkan kemudahan proses produksi di Toko Panut, sehingga layout lebih clean dan tidak terlalu kompleks untuk menyolder saat sudah dicetak.
- Clearance antar jalur diperlebar, sehingga proses penyolderan menjadi lebih mudah dan mengurangi risiko short circuit.

Secara desain dan layout, PCB sudah siap untuk tahap produksi dan assembly.

Namun, untuk saat ini pengujian penggerakan motor belum dapat dilakukan karena tidak dapat meminjam motor untuk dicoba.

# Programming
Logika awal programming:
- Cam detect tag0(payload)
- Send posisi tag0 ke mcu sampai tag0 dekat 0,0
- Send command "PAYLOAD" ke mcu agar mcu menggerakkan motor untuk mengambil payload
- Mcu mengirimkan command track drop lalu cam mendeteksi tag1
- Cam mengirimkan posisi tag1 ke mcu sampai tag1 dekat 0,0
- Mcu mengirimkan command "DROP" ke mcu agar mcu menggerakkan motor untuk menjatuhkan payload

Revisi (``Programming\revised_ros_ws``):
- Menggantikan koneksi langsung dengan serial menjadi koneksi melalui topic