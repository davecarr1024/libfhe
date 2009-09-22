#include "FileServer.h"
#include <Poco/Glob.h>
#include <Poco/Path.h>
#include <Poco/File.h>

namespace SGE
{
    
    FileServer::FileServer()
    {
        std::vector<std::string> files;
        enumerateFiles(".",files);
        
        for (std::vector<std::string>::iterator i = files.begin(); i != files.end(); ++i)
        {
            Poco::Path p(*i);

            std::string ext = p.getExtension();
            if (m_filesByType.find(ext) == m_filesByType.end())
                m_filesByType[ext] = std::vector<std::string>();
            m_filesByType[ext].push_back(*i);
            
            std::string name = p.getFileName();
            m_filesByName[name] = *i;
        }
    }
    
    FileServer::~FileServer()
    {
    }
    
    
    FileServer& FileServer::instance()
    {
        static FileServer fileServer;
        return fileServer;
    }
    
    void FileServer::enumerateFiles(const std::string& path, std::vector<std::string>& files)
    {
        std::set<std::string> paths;
        Poco::Glob::glob(path + "/*", paths);
        
        for (std::set<std::string>::iterator i = paths.begin(); i != paths.end(); ++i)
        {
            Poco::Path p = Poco::Path(*i);
            if (p.isFile())
                files.push_back(p.toString());
            else if (p.isDirectory())
                enumerateFiles(p.toString(),files);
        }
    }
    
    std::string FileServer::getFile(const std::string& name) const
    {
        if (Poco::File(name).exists())
            return name;
        else if (m_filesByName.find(name) != m_filesByName.end())
            return const_cast<FileServer*>(this)->m_filesByName[name];
        else
            return "";
    }
    
    void FileServer::getFilesOfType(const std::string& ext, std::vector<std::string>& files) const
    {
        if (m_filesByType.find(ext) != m_filesByType.end())
            for (std::vector<std::string>::const_iterator i = const_cast<FileServer*>(this)->m_filesByType[ext].begin();
                 i != const_cast<FileServer*>(this)->m_filesByType[ext].end(); ++i)
                 files.push_back(*i);
    }
    
}
