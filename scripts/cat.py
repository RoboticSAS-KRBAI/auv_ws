import numpy as np

class ROVController:
    def __init__(self, d):
        self.d = d
        self.sqrt2 = np.sqrt(2)

        self.M = np.array([
            [-1/self.sqrt2, 1/self.sqrt2, 1/self.sqrt2, -1/self.sqrt2],
            [1/self.sqrt2, 1/self.sqrt2, 1/self.sqrt2, 1/self.sqrt2],
            [d/2, -d/2, -d/2, d/2],
            [1, -1, 1, -1]  # Kontribusi untuk momen yaw
        ])

    def control(self, Fx, Fy, tau):
        # Vektor gaya yang diinginkan
        F = np.array([Fx, Fy, 0, tau])  # tau dimasukkan ke koordinat keempat
        
        # Menghitung thrust yang diperlukan menggunakan pseudo-inverse matriks
        T = np.dot(np.linalg.pinv(self.M), F)

        return T


if __name__ == '__main__' :

    # Parameter ROV
    d = 1.0  # Jarak dari pusat ROV ke thruster

    # Buat controller
    controller = ROVController(d)

    # Gaya yang diinginkan (Fx, Fy, tau)
    Fx = 0  # Gaya maju
    Fy = 2.5  # Gaya ke samping
    tau = 0  # Momen yaw

    # Hitung thrust yang diperlukan untuk setiap thruster
    thrusts = controller.control(Fx, Fy ,tau)

    # Tampilkan thrust yang diperlukan untuk setiap thruster
    print(f'Thruster 1: {thrusts[0]:.2f}')
    print(f'Thruster 2: {thrusts[1]:.2f}')
    print(f'Thruster 3: {thrusts[3]:.2f}')
    print(f'Thruster 4: {thrusts[2]:.2f}')
    print(f'PWM Thruster 1: {1500 - thrusts[0]*500:.2f}')
    print(f'PWM Thruster 2: {1500 - thrusts[1]*500:.2f}')
    print(f'PWM Thruster 3: {1500 - thrusts[3]*500:.2f}')
    print(f'PWM Thruster 4: {1500 - thrusts[2]*500:.2f}')
