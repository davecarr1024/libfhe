#ifndef FILESYSTEM_H
#define FILESYSTEM_H

#include <string>
#include <vector>
#include <map>

namespace fhe
{
    
    class FileSystem
    {
        private:
            std::vector<std::string> m_allDirs;
            std::map<std::string, std::string> m_filesByName;
            std::map<std::string, std::vector<std::string> > m_filesByExt;
            
            FileSystem();
            
            void enumerateFiles( const std::string& path );
            
            std::string getExt( const std::string& path );
            
        public:
            static FileSystem& instance();
            
            std::string getFile( const std::string& filename );
            
            std::vector<std::string> getAllDirs();
            
            std::vector<std::string> getFilesByExt( const std::string& ext );
    };
}

#endif
