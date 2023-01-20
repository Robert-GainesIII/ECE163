"""This file is a test harness for the module ece163.Utilities.Rotations,
and for the method ece163.Modeling.VehicleGeometry.getNewPoints(). 

It is meant to be run from the Testharnesses directory of the repo with:

python ./TestHarnesses/testChapter2.py (from the root directory) -or-
python testChapter2.py (from inside the TestHarnesses directory)

at which point it will execute various tests on the Rotations module"""

#%% Initialization of test harness and helpers:

import math

import sys
sys.path.append("..") #python is horrible, no?

import ece163.Utilities.MatrixMath as mm
import ece163.Utilities.Rotations as Rotations
import ece163.Modeling.VehicleGeometry as VG

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



#%% Euler2dcm():
print("Beginning testing of Rotations.Euler2dcm()")

cur_test = "Euler2dcm yaw test 1"
#we know that rotating [1,0,0] by 90 degrees about Z should produce [0,-1,0], so
R = Rotations.euler2DCM(90*math.pi/180, 0, 0)
orig_vec = [[1],[0],[0]]
expected_vec = [[0],[-1],[0]]
actual_vec = mm.multiply(R, orig_vec)
if not evaluateTest(cur_test, compareVectors(expected_vec, actual_vec) ):
	print(f"{expected_vec} != {actual_vec}")


#%%  

"""
Students, add more tests here.  
You aren't required to use the testing framework we've started here, 
but it will work just fine.
"""



cur_test = "Euler2dcm test 2"
R = Rotations.euler2DCM((-3/4) * math.pi, (1/3) * math.pi, (1/4) * math.pi)
orig_vec = [[0,0,0],[0,0,0],[0,0,0]]
expected_vec = [[1,0,0],[0,1,0],[0,0,1]]
actual_vec = mm.multiply(R, orig_vec)
if not evaluateTest(cur_test, compareVectors(expected_vec, actual_vec) ):
	print(f"{expected_vec} != {actual_vec}")


cur_test = "Euler2dcm test 3"
R = Rotations.euler2DCM(0, 0, 0)
orig_vec = [[0,0,0],[0,0,0],[0,0,0]]
expected_vec = [[-1*(math.sqrt(2))/4, -1*(math.sqrt(2))/4, -1*(math.sqrt(3))/2],
				[1/2 - ((math.sqrt(3))/4), (-1/2) - ((math.sqrt(3))/4),(math.sqrt(2))/4 ],
				[(-1/2) - ((math.sqrt(3))/4), 1/2 - ((math.sqrt(3))/4),(math.sqrt(2))/4]]
actual_vec = mm.multiply(R, orig_vec)
if not evaluateTest(cur_test, compareVectors(expected_vec, actual_vec) ):
	print(f"{expected_vec} != {actual_vec}")



print("Beginning testing of Rotations.Euler2dcm()")
cur_test = "dcm2Euler test 1"
R = Rotations.dcm2Euler([-1*(math.sqrt(2))/4, -1*(math.sqrt(2))/4, -1*(math.sqrt(3))/2],
				[1/2 - ((math.sqrt(3))/4), (-1/2) - ((math.sqrt(3))/4),(math.sqrt(2))/4 ],
				[(-1/2) - ((math.sqrt(3))/4), 1/2 - ((math.sqrt(3))/4),(math.sqrt(2))/4])
expected_ans = [(-3/4) * math.pi, (1/3) * math.pi, (1/4) * math.pi]

if not R == expected_ans:
	passed.append(cur_test)
else:
	failed.append(cur_test)



cur_test = "dcm2Euler test 2"
R = Rotations.dcm2Euler([[1,0,0],[0,1,0],[0,0,1]])
expected_ans = [0,0,0]
if not R == expected_ans:
	passed.append(cur_test)
else:
	failed.append(cur_test)


print("Beginning testing of Rotations.ned2enu()")
cur_test = "ned2enu test 1"
R = Rotations.ned2enu([[0,1,-1]])
expected_ans = [[1,0,1]]
if not R == expected_ans:
	passed.append(cur_test)
else:
	failed.append(cur_test)



cur_test = "ned2enu test 2"
R = Rotations.ned2enu([[1,2,3]])
expected_ans = [[2,1,-3]]
if not R == expected_ans:
	passed.append(cur_test)
else:
	failed.append(cur_test)




print("Beginning testing of VehicleGeometry.newPoints()")
cur_test = "newPoints test 1"
R = Rotations.euler2DCM(0, 0, 0)
orig_vec = [[0,0,0],[0,0,0],[0,0,0]]
expected_vec = [[-1*(math.sqrt(2))/4, -1*(math.sqrt(2))/4, -1*(math.sqrt(3))/2],
				[1/2 - ((math.sqrt(3))/4), (-1/2) - ((math.sqrt(3))/4),(math.sqrt(2))/4 ],
				[(-1/2) - ((math.sqrt(3))/4), 1/2 - ((math.sqrt(3))/4),(math.sqrt(2))/4]]
actual_vec = mm.multiply(R, orig_vec)
if not evaluateTest(cur_test, compareVectors(expected_vec, actual_vec) ):
	print(f"{expected_vec} != {actual_vec}")



cur_test = "newPoints test 2"
R = Rotations.euler2DCM(0, 0, 0)
orig_vec = [[0,0,0],[0,0,0],[0,0,0]]
expected_vec = [[-1*(math.sqrt(2))/4, -1*(math.sqrt(2))/4, -1*(math.sqrt(3))/2],
				[1/2 - ((math.sqrt(3))/4), (-1/2) - ((math.sqrt(3))/4),(math.sqrt(2))/4 ],
				[(-1/2) - ((math.sqrt(3))/4), 1/2 - ((math.sqrt(3))/4),(math.sqrt(2))/4]]
actual_vec = mm.multiply(R, orig_vec)
if not evaluateTest(cur_test, compareVectors(expected_vec, actual_vec) ):
	print(f"{expected_vec} != {actual_vec}")












#%% Print results:

total = len(passed) + len(failed)
print(f"\n---\nPassed {len(passed)}/{total} tests")
[print("   " + test) for test in passed]

if failed:
	print(f"Failed {len(failed)}/{total} tests:")
	[print("   " + test) for test in failed]
