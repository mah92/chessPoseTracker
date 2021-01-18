# بسم اله الرحمن الرحیم - هست کلید در گنج حکیم

# chessPoseTracker
This routine calculates the path of a calibration pattern in front of camera.

Inputs: 
* calib.xml(copied from opencvCalib or filled manually)
* IMAGES ADDR macro (address of the selected calibration pattern images in the form of screenshot000xx.bmp)

outputs: path.csv (path of the calibration pattern)

A sample for the calib.xml is the following:

<!-- ox: 298.485405, oy:226.525456, fovX:92.107164, fovY:75.744182 -->
<idealParams resolutionScale="1." width="640" height="480" oxOffset="-21.014595" oyOffset="-12.974544" f="307.960654" desc = "opencv"/>
<staticParams  maxLensIterations="100" maxPixError="0.1" k1="-0.428129" k2="0.158059" t1="0.000245" t2="-0.002286" k3="-0.029194" k4="0.000000" k5="0.000000" k6="0.000000" desc = "opencv"/>


