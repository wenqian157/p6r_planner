import sys;
sys.path.insert(0,'scripts'); 
import numpy as np; 
from jacobian_p6r import analyse; 
[print(q1, analyse(np.r_[0, np.deg2rad([q1,-60,90,-120,0,30])])['lost'].round(2)) for q1 in (0, 45, 90)]