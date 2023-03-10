"""This file is a test harness for the module VehicleClosedLoopControl and VehicleControlGains

It is meant to be run from the Testharnesses directory of the repo with:

python ./TestHarnesses/testChapter6.py (from the root directory) -or-
python testChapter6.py (from inside the TestHarnesses directory)

at which point it will execute various tests on the VehiclePerturbationModels module"""

#%% Initialization of test harness and helpers:

import math

import sys
sys.path.append("..") #python is horrible, no?

import ece163.Utilities.MatrixMath as mm
import ece163.Utilities.Rotations as Rotations
import ece163.Modeling.VehicleDynamicsModel as VDM
import ece163.Modeling.VehicleAerodynamicsModel as VAM
import ece163.Controls.VehiclePerturbationModels as VPM
import ece163.Modeling.WindModel as WM
import ece163.Controls.VehicleControlGains as VC
import ece163.Controls.VehicleClosedLoopControl as VCLC 
import ece163.Containers.Inputs as Inputs
import ece163.Containers.States as States
import ece163.Containers.Controls as controls
import ece163.Containers.Linearized as lin
import ece163.Sensors.SensorsModel as sensors

"""math.isclose doesn't work well for comparing things near 0 unless we 
use an absolute tolerance, so we make our own isclose:"""
isclose = lambda  a,b : math.isclose(a, b, abs_tol= 1e-6)

def compareVectors(a, b):
	"""A quick tool to compare two vectors"""
	el_close = [isclose(a[i][0], b[i][0]) for i in range(3)]
	return all(el_close)

#of course, you should test your testing tools too:
assert(compareVectors([[0], [0], [-1]],[[1e-13], [0], [-1+1e-9]]))
assert(not compareVectors([[0], [0], [-1]],[[1e-11], [0], [-1]]))
assert(not compareVectors([[1e8], [0], [-1]],[[1e8+1], [0], [-1]]))



failed = []
passed = []
def evaluateTest(test_name, boolean):
	"""evaluateTest prints the output of a test and adds it to one of two 
	global lists, passed and failed, which can be printed later"""
	if boolean:
		print(f"   passed {test_name}")
		passed.append(test_name)
	else:
		print(f"   failed {test_name}")
		failed.append(test_name)
	return boolean


#%% PUT A TEST HERE
vam = VAM.VehicleAerodynamicsModel()
sensorGausstest1 = sensors.GaussMarkov(2,5,10)
sensorGaussXYZtest1 = sensors.GaussMarkovXYZ(dT = 0.001, tauX =1000, etaX=10)
sensorGaussXYZtest2 = sensors.GaussMarkovXYZ(dT = 0.001, tauX = 100, etaX = 10, tauY = 1000, etaY = 100)
sensorGaussXYZtest3 = sensors.GaussMarkovXYZ()
sensorModel = sensors.SensorsModel(vam)

if sensorGausstest1.dT != 2:
	print("Guass 1d Init test failed")
else: print("Gauss 1d init test passed")

if sensorGaussXYZtest1.tauY != 1000 or sensorGaussXYZtest1.tauZ!= 1000:
	print("Guass 3d Init test failed")
else: print("Gauss 3d init test passed")


if sensorGaussXYZtest2.tauZ != 1000:
	print("Guass 3d Init test2 failed")
else: print("Gauss 3d init test2 passed")



print("Expected output of sensor model update accels true is:")
print(sensorModel.updateAccelsTrue(vam.getVehicleDynamicsModel(), vam.dynamicsModel.dot))

print("Expected output of sensor model update GPS true is:")
print(sensorModel.updateGPSTrue(vam.dynamicsModel.state, vam.dynamicsModel.dot))

print("Expected output of sensor model update gyros true is:")
print(sensorModel.updateGyrosTrue(vam.dynamicsModel.state))

print("Expected output of sensor model update Mags true is:")
print(sensorModel.updateMagsTrue(vam.dynamicsModel.state))

print("Expected output of sensor model update Pressure true is:")
print(sensorModel.updatePressureSensorsTrue(vam.dynamicsModel.state))






#TESTS FOR VEHICLECONTROLGAINS 

#%% Print results:

total = len(passed) + len(failed)
print(f"\n---\nPassed {len(passed)}/{total} tests")
[print("   " + test) for test in passed]

if failed:
	print(f"Failed {len(failed)}/{total} tests:")
	[print("   " + test) for test in failed]