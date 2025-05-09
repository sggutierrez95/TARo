import glob
import pathlib
import os
files = []
for path in glob.glob('runs/**', recursive=True):
    if os.path.isfile(path):
        files.append(path)

print(files)