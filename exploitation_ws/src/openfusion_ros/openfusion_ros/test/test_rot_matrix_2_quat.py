import numpy as np
from tf_transformations import quaternion_from_matrix, euler_from_matrix

def main():
    # Deine 4x4-Transformationsmatrix
    matrix = np.array([
        [0.939048, 0.118417, -0.322748, 1.277212],
        [0.340776, -0.444599, 0.828375, 2.692177],
        [-0.045399, -0.887868, -0.457853, 1.372351],
        [0.0, 0.0, 0.0, 1.0]
    ], dtype=np.float64)

    # Translation extrahieren
    translation = matrix[:3, 3]

    # Quaternion berechnen (x, y, z, w)
    quat = quaternion_from_matrix(matrix)

    # Euler-Winkel berechnen (Roll, Pitch, Yaw)
    euler_rad = euler_from_matrix(matrix)
    euler_deg = np.degrees(euler_rad)

    # Ausgabe
    print("== Transformation ==")
    print("Translation (x, y, z):", translation)
    print("Quaternion (x, y, z, w):", quat)
    print("Euler angles [rad] (roll, pitch, yaw):", euler_rad)
    print("Euler angles [deg] (roll, pitch, yaw):", euler_deg)

if __name__ == "__main__":
    main()
