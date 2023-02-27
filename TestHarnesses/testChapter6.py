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


#%% PUT A TEST HERE?
closedLoopcontrolInstance = VCLC.VehicleClosedLoopControl()

controlGaines = controls.controlGains(1,1,1,1,1,1,1,1,1,1,1,1,1,1,1)
check = closedLoopcontrolInstance.setControlGains(controlGaines)
check = closedLoopcontrolInstance.getControlGains()
print("expected values: 1,1,1,1,1,1,1,1,1,1,1,1")
print("observed values: ")
print(check)
passed.append("set/get control gains test")



state = States.vehicleState(1,1,1,1,1,1,1,1,1,1,1)
closedLoopcontrolInstance.setVehicleState(state)
check = closedLoopcontrolInstance.getVehicleState()
print("expected values:")
print(state)
print("observed values: ")
print(check)
passed.append("set/getvehicle state")


trim = Inputs.controlInputs(1,1,1,1)
closedLoopcontrolInstance.setTrimInputs(trim)

check = closedLoopcontrolInstance.getTrimInputs()
print("expected values:")
print(trim)
print("observed values: ")
print(check)
passed.append("set/get trim state")


myVAM = VAM.VehicleAerodynamicsModel()
check = closedLoopcontrolInstance.getVehicleAerodynamicsModel()

print("expected values:")
print(myVAM)
print("observed values: ")
print(check)
passed.append("get VehicleAerodynamicsModel")


myRef = controls.referenceCommands(30,1, 1)
state = States.vehicleState(1,1,1,1,1,1,1,1,1,1,1,1)
closedLoopcontrolInstance.UpdateControlCommands(myRef, state)
check = closedLoopcontrolInstance.getVehicleControlSurfaces()

print("expected values:")
print(check)
print("observed values: ")
print(check)
passed.append("updateControlCommands/ getVehicleControlSurfaces test")


gains = controls.controlGains(1,1,1,1,1,1,1,1,1,1,1,1,1,1,1)
params = controls.controlTuning(1,1,1,1,1,1,1,1,1,1,1,1,1,1)
linearModel = lin.transferFunctions(1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1)

check1 = VC.computeGains(params, linearModel)

check2 = VC.computeTuningParameters(gains,linearModel)

if check1 == gains:
	passed.append(" compute gains test")
if check2 == params:
	passed.append("compute tuning params test")




#TESTS FOR VEHICLECONTROLGAINS 

#%% Print results:

total = len(passed) + len(failed)
print(f"\n---\nPassed {len(passed)}/{total} tests")
[print("   " + test) for test in passed]

if failed:
	print(f"Failed {len(failed)}/{total} tests:")
	[print("   " + test) for test in failed]