from distutils.core import setup
import py2exe
import os
import glob

def search(pattern, path = "."):
    files = glob.glob(path + "/" + pattern)
    for subdir in filter(os.path.isdir,glob.glob(path + "/*")):
        files.extend(search(pattern,subdir))
    return files

modules = [i[:-3].lstrip('.\\').replace('\\','.') for i in search("*.py")]

opts = {
    'py2exe': {
        'packages': ['pygame','OpenGL','pymunk'] + modules
        }
    }

setup(console=['main.py'],options=opts)

def copyToDist(filename):
    os.system("copy %s dist" % filename)

import OpenGL
copyToDist("%s\DLLS\glut32.dll" % OpenGL.__path__[0])

import pymunk
copyToDist("%s\chipmunk.dll" % pymunk.__path__[0])
