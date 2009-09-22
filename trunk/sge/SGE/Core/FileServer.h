#ifndef FILESERVER_H
#define FILESERVER_H

#include <string>
#include <vector>
#include <map>

namespace SGE
{
    
    class FileServer
    {
        private:
            FileServer();
            
            std::map<std::string, std::string> m_filesByName;
            std::map<std::string, std::vector<std::string> > m_filesByType;
            
            void enumerateFiles(const std::string& path, std::vector<std::string>& files);
            
        public:
            virtual ~FileServer();
            
            static FileServer& instance();

            bool fileExists(const std::string& name) const;
            std::string getFile(const std::string& name) const;
            void getFilesOfType(const std::string& ext, std::vector<std::string>& files) const;
    };
    
}

#endif
