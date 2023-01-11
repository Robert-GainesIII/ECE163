"""
.. modle:: matrixMathTestHarness.py
    :platform: MacOS, Unix, Windows,
    :synopsis: Compares output from matrixMathGeneration.py with students'  
    function implementations
    matrixMathTestHarness.py
.. moduleauthor:: Pavlo Vlastos <pvlastos@ucsc.edu>
"""
import os
import sys
sys.path.insert(0, os.path.abspath('..'))
import math

import ece163.Utilities.MatrixMath as MatrixMath
import argparse
import random
import pickle
import traceback

# Private methods
def matrixCompare(A, B):
	"""
	Compare the elements of two matrices

	:param A: matrix (list of lists) of [m x n]
	:param B: matrix (list of lists) of [n x r]
	:return: [True, expTot, resTot]: True or False if the matrices match, the
	number of expected matching elements, and the number of resulting matching
	elements between A and B
	"""
	[m, r] = MatrixMath.size(A)
	[m_c, r_c] = MatrixMath.size(B)

	expTot = (m_c * r_c)
	resTot = 0

	if (m == m_c) and (r == r_c):
		for row in range(m):
			for col in range(r):
				if math.isclose(A[row][col], B[row][col]) is not True:
					print("Element [{0},{1}] is incorrect".format(row, col))
					return [False, expTot, resTot]
				else:
					resTot += 1
		if expTot != resTot:
			print("\r\nResulting matrix dimensions match the expected matrix dimensions")
			return [True, expTot, resTot]
	else:
		print("Error: Resulting matrix dimensions do not match the expected matrix dimensions")
		return [False, expTot, resTot]

	return [True, expTot, resTot]

#  these two functions allow for more standardized output, they should be copied to each test harness and customized
def printTestBlockResult(function, testsPassed, testCount):
	if testsPassed != testCount:
		addendum = " (TESTS FAILED)"
	else:
		addendum = ""
	print("{}/{} tests passed for {}{}".format(testsPassed, testCount, function.__name__, addendum))

def printTestFailure(function, inputs, outputs, expectedoutputs):
	print("Test Failed for {}. Please find the corresponding inputs and expected output(s) below for testing".format(function.__name__))
	print("Inputs: {}".format(repr(inputs)))
	print("Outputs: {}".format(repr(outputs)))
	print("Expected Outputs: {}".format(repr(expectedoutputs)))


###############################################################################
parser = argparse.ArgumentParser()
parser.add_argument('-c','--continueMode', action='store_true', help='Runs all tests regardless of failures')
parser.add_argument('picklePath', nargs='?', default='MatrixMath_TestData.pickle', help='valid path to pickle for input')

arguments = parser.parse_args()

picklePath = arguments.picklePath
inContinueMode = arguments.continueMode

print("Beginning Test Harness for Matirx Math Library using file {}".format(picklePath))
try:
	with open(picklePath, 'rb') as f:
		allTests = pickle.load(f)
except FileNotFoundError:
	print('Test file not found, exiting')
	sys.exit(-1)

testBlocksPassed = 0  # we keep track of the number of test blocks passed

testBlockIterator = iter(allTests)  # we hard code the tests as well so we need an iterator

###############################################################################
#.multiply()
print("Comparing outputs for {}".format(MatrixMath.multiply.__name__))
curTestBlock = next(testBlockIterator)  # we now have all the tests for this block
testsPassed = 0
for A, B, expectedResult in curTestBlock:  # and now we can iterate through them
	try:
		result = MatrixMath.multiply(A, B)
		[matricesEqual, expTot, resTot] = matrixCompare(result, expectedResult)
		if matricesEqual is True:
			testsPassed += 1
		else:
			if not inContinueMode:
				printTestFailure(MatrixMath.multiply, (A, B), result, expectedResult)
				sys.exit(-1)
	except Exception as e:  # overly broad exception clause but we want it to catch everything
		print('Test harness failed with the exception below. It will not continue')
		print(traceback.format_exc())
		sys.exit(-1)

printTestBlockResult(MatrixMath.multiply, testsPassed, len(curTestBlock))
if testsPassed == len(curTestBlock):
	testBlocksPassed += 1

###############################################################################
# transpose()
print("Comparing outputs for {}".format(MatrixMath.transpose.__name__))
curTestBlock = next(testBlockIterator)  # we now have all the tests for this block
testsPassed = 0
for A, expectedResult in curTestBlock:  # and now we can iterate through them
	try:
		result = MatrixMath.transpose(A)
		[matricesEqual, expTot, resTot] = matrixCompare(result, expectedResult)
		if matricesEqual is True:
			testsPassed += 1
		else:
			if not inContinueMode:
				printTestFailure(MatrixMath.transpose, A, result, expectedResult)
				sys.exit(-1)
	except Exception as e:  # overly broad exception clause but we want it to catch everything
		print('Test harness failed with the exception below. It will not continue')
		print(traceback.format_exc())
		sys.exit(-1)

printTestBlockResult(MatrixMath.transpose, testsPassed, len(curTestBlock))
if testsPassed == len(curTestBlock):
	testBlocksPassed += 1

###############################################################################
# matrixAdd()
print("Comparing outputs for {}".format(MatrixMath.add.__name__))
curTestBlock = next(testBlockIterator)  # we now have all the tests for this block
testsPassed = 0
for A, B, expectedResult in curTestBlock:  # and now we can iterate through them
	try:
		result = MatrixMath.add(A, B)
		[matricesEqual, expTot, resTot] = matrixCompare(result, expectedResult)
		if matricesEqual is True:
			testsPassed += 1
		else:
			if not inContinueMode:
				printTestFailure(MatrixMath.add, (A, B), result, expectedResult)
				sys.exit(-1)
	except Exception as e:  # overly broad exception clause but we want it to catch everything
		print('Test harness failed with the exception below. It will not continue')
		print(traceback.format_exc())
		sys.exit(-1)

printTestBlockResult(MatrixMath.add, testsPassed, len(curTestBlock))
if testsPassed == len(curTestBlock):
	testBlocksPassed += 1

###############################################################################
#.subtract()
print("Comparing outputs for {}".format(MatrixMath.subtract.__name__))
curTestBlock = next(testBlockIterator)  # we now have all the tests for this block
testsPassed = 0
for A, B, expectedResult in curTestBlock:  # and now we can iterate through them
	try:
		result = MatrixMath.subtract(A, B)
		[matricesEqual, expTot, resTot] = matrixCompare(result, expectedResult)
		if matricesEqual is True:
			testsPassed += 1
		else:
			if not inContinueMode:
				printTestFailure(MatrixMath.subtract, (A, B), result, expectedResult)
				sys.exit(-1)
	except Exception as e:  # overly broad exception clause but we want it to catch everything
		print('Test harness failed with the exception below. It will not continue')
		print(traceback.format_exc())
		sys.exit(-1)

printTestBlockResult(MatrixMath.subtract, testsPassed, len(curTestBlock))
if testsPassed == len(curTestBlock):
	testBlocksPassed += 1

###############################################################################
# scalarMultiply()
print("Comparing outputs for {}".format(MatrixMath.scalarMultiply.__name__))
curTestBlock = next(testBlockIterator)  # we now have all the tests for this block
testsPassed = 0
for a, B, expectedResult in curTestBlock:  # and now we can iterate through them
	try:
		result = MatrixMath.scalarMultiply(a, B)
		[matricesEqual, expTot, resTot] = matrixCompare(result, expectedResult)
		if matricesEqual is True:
			testsPassed += 1
		else:
			if not inContinueMode:
				printTestFailure(MatrixMath.scalarMultiply, (a, B), result, expectedResult)
				sys.exit(-1)
	except Exception as e:  # overly broad exception clause but we want it to catch everything
		print('Test harness failed with the exception below. It will not continue')
		print(traceback.format_exc())
		sys.exit(-1)

printTestBlockResult(MatrixMath.scalarMultiply, testsPassed, len(curTestBlock))
if testsPassed == len(curTestBlock):
	testBlocksPassed += 1

###############################################################################
# dotProduct()
print("Comparing outputs for {}".format(MatrixMath.dotProduct.__name__))
curTestBlock = next(testBlockIterator)  # we now have all the tests for this block
testsPassed = 0
for a, B, expectedResult in curTestBlock:  # and now we can iterate through them
	try:
		result = MatrixMath.dotProduct(a, B)
		[matricesEqual, expTot, resTot] = matrixCompare(result, expectedResult)
		if matricesEqual is True:
			testsPassed += 1
		else:
			if not inContinueMode:
				printTestFailure(MatrixMath.dotProduct, (A, B), result, expectedResult)
				sys.exit(-1)
	except Exception as e:  # overly broad exception clause but we want it to catch everything
		print('Test harness failed with the exception below. It will not continue')
		print(traceback.format_exc())
		sys.exit(-1)

printTestBlockResult(MatrixMath.dotProduct, testsPassed, len(curTestBlock))
if testsPassed == len(curTestBlock):
	testBlocksPassed += 1

###############################################################################
# skew()
print("Comparing outputs for {}".format(MatrixMath.skew.__name__))
curTestBlock = next(testBlockIterator)  # we now have all the tests for this block
testsPassed = 0
for a, b, c, expectedResult in curTestBlock:  # and now we can iterate through them
	try:
		result = MatrixMath.skew(a, b, c)
		[matricesEqual, expTot, resTot] = matrixCompare(result, expectedResult)
		if matricesEqual is True:
			testsPassed += 1
		else:
			if not inContinueMode:
				printTestFailure(MatrixMath.skew, (a, b, c), result, expectedResult)
				sys.exit(-1)
	except Exception as e:  # overly broad exception clause but we want it to catch everything
		print('Test harness failed with the exception below. It will not continue')
		print(traceback.format_exc())
		sys.exit(-1)

printTestBlockResult(MatrixMath.skew, testsPassed, len(curTestBlock))
if testsPassed == len(curTestBlock):
	testBlocksPassed += 1

###############################################################################
# crossProduct()
print("Comparing outputs for {}".format(MatrixMath.crossProduct.__name__))
curTestBlock = next(testBlockIterator)  # we now have all the tests for this block
testsPassed = 0
for A, B, expectedResult in curTestBlock:  # and now we can iterate through them
	try:
		result = MatrixMath.crossProduct(A, B)
		[matricesEqual, expTot, resTot] = matrixCompare(result, expectedResult)
		if matricesEqual is True:
			testsPassed += 1
		else:
			if not inContinueMode:
				printTestFailure(MatrixMath.crossProduct, (A, B), result, expectedResult)
				sys.exit(-1)
	except Exception as e:  # overly broad exception clause but we want it to catch everything
		print('Test harness failed with the exception below. It will not continue')
		print(traceback.format_exc())
		sys.exit(-1)

printTestBlockResult(MatrixMath.crossProduct, testsPassed, len(curTestBlock))
if testsPassed == len(curTestBlock):
	testBlocksPassed += 1

###############################################################################
# offset()
print("Comparing outputs for {}".format(MatrixMath.offset.__name__))
curTestBlock = next(testBlockIterator)  # we now have all the tests for this block
testsPassed = 0
for A, x, y, z, expectedResult in curTestBlock:  # and now we can iterate through them
	try:
		result = MatrixMath.offset(A, x, y, z)
		[matricesEqual, expTot, resTot] = matrixCompare(result, expectedResult)
		if matricesEqual is True:
			testsPassed += 1
		else:
			if not inContinueMode:
				printTestFailure(MatrixMath.offset, (A, x, y, z), result, expectedResult)
				sys.exit(-1)
	except Exception as e:  # overly broad exception clause but we want it to catch everything
		print('Test harness failed with the exception below. It will not continue')
		print(traceback.format_exc())
		sys.exit(-1)

printTestBlockResult(MatrixMath.offset, testsPassed, len(curTestBlock))
if testsPassed == len(curTestBlock):
	testBlocksPassed += 1

###############################################################################
# size()
print("Comparing outputs for {}".format(MatrixMath.size.__name__))
curTestBlock = next(testBlockIterator)  # we now have all the tests for this block
testsPassed = 0
for A, expectedResult in curTestBlock:  # and now we can iterate through them
	try:
		result = MatrixMath.size(A)

		if (expectedResult[0] == result[0]) and (expectedResult[1] == result[1]):
			testsPassed += 1
		else:
			if not inContinueMode:
				printTestFailure(MatrixMath.size, A, result, expectedResult)
				sys.exit(-1)
	except Exception as e:  # overly broad exception clause but we want it to catch everything
		print('Test harness failed with the exception below. It will not continue')
		print(traceback.format_exc())
		sys.exit(-1)

printTestBlockResult(MatrixMath.size, testsPassed, len(curTestBlock))
if testsPassed == len(curTestBlock):
	testBlocksPassed += 1

###############################################################################
if testBlocksPassed == len(allTests):
	print("All tests Passed for Matrix Math")
else:
	print("{}/{} tests blocks passed for Matrix Math".format(testBlocksPassed, len(allTests)))