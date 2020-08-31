# Hotfix to select Times New Roman Regular instead of Bold:
# https://stackoverflow.com/a/44386835/3077540
import matplotlib
if 'roman' in matplotlib.font_manager.weight_dict:
    del matplotlib.font_manager.weight_dict['roman']
    matplotlib.font_manager._rebuild()

import matplotlib.pyplot as plt
plt.rcParams['font.family'] = 'serif'
plt.rcParams['font.serif'] = 'Times New Roman'
plt.rcParams['font.size'] = 12
