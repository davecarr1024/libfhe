#ifndef FILESERVER_H
#define FILESERVER_H

#include "Var.h"

#include <map>
#include <vector>
#include <string>

#include "boost/filesystem.hpp"

///sdse_lib boost_filesystem

namespace sdse {

    class FileServer {
        private:
            static void enumerateFiles(boost::filesystem::path path);
            static void clear();

            static bool filesEnumerated;
            static void checkFilesEnumerated();
            
            static std::string getExtension(std::string filename);
            static std::string getName(std::string filename);
            
            static StringMap filesByName;
            static StringListMap filesByType;
            static StringList filePaths;
        
        public:
            static std::string getFile(std::string filename);
            static StringList getFilesOfType(std::string ext);
            static StringList getFilePaths();
    };

}

#endif
