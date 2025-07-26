import matplotlib.pyplot as plt
import numpy as np

lut = (plt.get_cmap('inferno')(np.linspace(0, 1, 256))[:, :3] * 255).astype(int)
for r, g, b in lut:
    print(f'{{{r}, {g}, {b}}},')
