'''
SLab Calibration Stage 4
calibrate4.py

Check calibration
'''

import slab
slab.setCalPrefix('Calibrations/')
print(slab.calprefix)
slab.connect()

slab.checkCalibration()

slab.disconnect()




    
