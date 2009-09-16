from glob import glob
import os.path

class FileServer:
    def __init__(self):
        self.files = [filename.replace('\\','/') for filename in self.enumerateFiles('.')]
        
        self.filesByType = {}
        self.filesByName = {}
        
        for filename in self.files:
            self.filesByType.setdefault(self.getType(filename),[]).append(filename)
            self.filesByName[self.getName(filename)] = filename
        
    def enumerateFiles(self, path, pattern = None):
        files = filter(os.path.isfile,glob("%s/%s" % (path,pattern or "*")))
        for subdir in filter(os.path.isdir,glob("%s/*" % path)):
            files.extend(self.enumerateFiles(subdir))
        return files

    def getType(self, filename):
        if '.' in filename:
            return filename[filename.rfind('.')+1:]
        else:
            return ''
            
    def getName(self, filename):
        return filename[filename.rfind('/')+1:]
        
    def getFile(self, filename):
        if os.path.isfile(filename):
            return filename
        else:
            assert filename in self.filesByName, "unknown file %s" % filename
            return self.filesByName[filename]
        
    def isFile(self, filename):
        return os.path.isfile(filename) or filename in self.filesByName

fileServer = FileServer()

if __name__ == "__main__":
    assert fileServer.getFile("fileServer.py") == "./core/fileServer.py"
