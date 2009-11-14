import glob
import os

class FileSystem:
    def __init__(self):
        files = self.enumerateFiles()

        self.filesByName = dict(zip(map(os.path.basename,files),files))

    def enumerateFiles(self, path = "."):
        rawFiles = glob.glob("%s/*" % path)
        return sum(map(self.enumerateFiles,filter(os.path.isdir,rawFiles)), \
                   filter(os.path.isfile,rawFiles))

    def get(self, name):
        if os.path.isfile(name):
            return name
        else:
            assert name in self.filesByName, "couldn't find file %s" % name
            return self.filesByName[name]

fileSystem = FileSystem()
