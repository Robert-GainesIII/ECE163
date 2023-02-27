"""This file is a test harness for the module VehicleDynamicsModel. 

It is meant to be run from the Testharnesses directory of the repo with:

python ./TestHarnesses/testChapter3.py (from the root directory) -or-
python testChapter3.py (from inside the TestHarnesses directory)

at which point it will execute various tests on the VehicleDynamicsModel module"""

#%% Initialization of test harness and helpers:

import math

import sys
sys.path.append("..") #python is horrible, no?

import ece163.Utilities.MatrixMath as mm
import ece163.Utilities.Rotations as Rotations
import ece163.Modeling.VehicleDynamicsModel as VDM
import ece163.Modeling.VehicleAerodynamicsModel as MYNUTS
import ece163.Containers.Inputs
import ece163.Containers.States

"""math.isclose doesn't work well for comparing things near 0 unless we 
use an absolute tolerance, so we make our own isclose:"""
isclose = lambda  a,b : math.isclose(a, b, abs_tol= 1e-12)

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
	
	 
#%% Derivative():
print("Beginning testing of VDM.Derivative(), subtest of [pe,pn,pd]")

cur_test = "Derivative test p_dot x dir"

testVDM = VDM.VehicleDynamicsModel()
testState = States.vehicleState()
testFm = Inputs.forcesMoments()
testState.pitch = 30*math.pi/180
testState.R = Rotations.euler2DCM(0.0,testState.pitch,0.0)
testState.u = 10
testDot = testVDM.derivative(testState, testFm)

print("With a velocity of u = 10 m/s, and pitch = 30deg:\n")
resultPdot = [[testDot.pn],[testDot.pe],[testDot.pd]]
expectedPdot = [[10*math.sqrt(3)/2],[0],[-10/2]]

if compareVectors(resultPdot,expectedPdot):
	
 print("passed!")
else:
 print("failed :(")
"""
Students, add more tests here.  
You aren't required to use the testing framework we've started here, 
but it will work just fine.
"""

#%% Print results:

total = len(passed) + len(failed)
print(f"\n---\nPassed {len(passed)}/{total} tests")
[print("   " + test) for test in passed]

if failed:
	print(f"Failed {len(failed)}/{total} tests:")
	[print("   " + test) for test in failed]