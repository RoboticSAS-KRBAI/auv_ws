import numpy as np

# Variabel d adalah jarak dari tengah kapal ke titik di mana gaya dorongan diberikan
d = 1.0  # Contoh nilai, sesuaikan dengan konfigurasi kapal Anda
sqrt2 = np.sqrt(2)

# Matriks transformasi untuk 8 thruster
M = np.array([
    [-1/sqrt2, 1/sqrt2, 1/sqrt2, -1/sqrt2, 0, 0, 0, 0],   # Surge (maju/mundur)
    [1/sqrt2, 1/sqrt2, 1/sqrt2, 1/sqrt2, 0, 0, 0, 0],     # Sway (kanan/kiri)
    [d/2, -d/2, -d/2, d/2, 0, 0, 0, 0],                   # Yaw (putaran)
    [1, -1, 1, -1, 0, 0, 0, 0],                           # Kontribusi untuk momen yaw
    [0, 0, 0, 0, d/2, -d/2, -d/2, d/2],                   # Roll (miring ke samping)
    [0, 0, 0, 0, 1, -1, 1, -1],                           # Kontribusi untuk momen roll
    [0, 0, 0, 0, d/2, -d/2, -d/2, d/2],                   # Pitch (miring ke depan/belakang)
    [0, 0, 0, 0, 1, 1, -1, -1],                           # Kontribusi untuk momen pitch
    [0, 0, 0, 0, 1, 1, 1, 1],                             # Heave (naik/turun)
])

# Inputan gaya dan momen
fx = 0.0
fy = 0.0
fz = 0.0
tau_yaw = 0
tau_roll = 1
tau_pitch = 0.0

# Vektor input
input_vector = np.array([fx, fy, 0, tau_yaw, 0, tau_roll, 0, tau_pitch, fz])

# Menghitung pseudo-invers dari Matriks M
M_pseudo_inv = np.linalg.pinv(M)

# Menghitung gaya dorong dari thruster
F = M_pseudo_inv.dot(input_vector)

# Output nilai thruster dan nilai PWM yang dikonversi
print("Nilai Thruster dan PWM:")
for i in range(len(F)):
    pwm_value = 1500 - F[i] * 500
    print(f"F{i+1}: {F[i]:.2f}, PWM: {pwm_value:.2f}")
