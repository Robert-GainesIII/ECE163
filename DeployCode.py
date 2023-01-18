import os
import pathlib
import zipfile
import pickle
import tempfile
import sys
from urllib import request
from urllib import error

sourceURL = "https://classes.soe.ucsc.edu/ece163/Winter23/ECE163.zip"
stubPicklePath = 'ECE163/stubFiles.pickle'
# print(stubPaths)
try:
	with request.urlopen(sourceURL) as f:
		zipData = f.read()
except error.URLError as e:
	print('Source File not found with error {}'.format(e))
	sys.exit(0)

with tempfile.TemporaryFile(mode='w+b') as zipDataFile:
	zipDataFile.write(zipData)
	with zipfile.ZipFile(zipDataFile) as sourceZip:
		with sourceZip.open(str(stubPicklePath)) as f:
			stubPaths = pickle.load(f)

		stubPaths = [pathlib.Path(x) for x in stubPaths]
		for index, curFilePath in enumerate(sourceZip.namelist()):
			if curFilePath == stubPicklePath:
				continue
			print('{} Processing {}'.format(index, curFilePath))
			zipCurFilePath = pathlib.Path(curFilePath)  # need a pathlib object for comparison
			destPath = pathlib.Path(*zipCurFilePath.parts[1:])  # and removes the ECE163 folder
			if destPath in stubPaths and destPath.exists():  # if it is a stubfile and already exists we are done
				continue

			destFolder = pathlib.Path(*destPath.parts[:-1])
			os.makedirs(destFolder, exist_ok=True)  # and make the directory structure
			with sourceZip.open(curFilePath) as sourceBytes:  #  need to use open here because zipfile extract is an idiot
				with open(destPath, 'wb') as destBytes:
					destBytes.write(sourceBytes.read())
