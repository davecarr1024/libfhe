#include "FileSystem.h"
#include <boost/filesystem.hpp>
#include <cassert>

namespace fhe
{
    FileSystem::FileSystem()
    {
        enumerateFiles(".");
    }
    
    void FileSystem::enumerateFiles( const std::string& path )
    {
        m_allDirs.push_back( path );
        boost::filesystem::directory_iterator end;
        for ( boost::filesystem::directory_iterator i( path ); i != end; ++i )
        {
            if ( i->leaf()[0] != '.' ) 
            {
                if ( boost::filesystem::is_directory( i->status() ) )
                {
                    enumerateFiles( i->string() );
                }
                else
                {
                    std::string filepath = i->string(), filename = i->leaf(), ext = getExt( filename );
                    m_filesByName[filename] = filepath;
                    if ( m_filesByExt.find(ext) == m_filesByExt.end() )
                    {
                        m_filesByExt[ext] = std::vector<std::string>();
                    }
                    m_filesByExt[ext].push_back(filepath);
                }
            }
        }
    }
    
    std::string FileSystem::getExt( const std::string& path )
    {
        size_t i = path.rfind(".");
        if ( i == std::string::npos )
        {
            return "";
        }
        else
        {
            return path.substr(i+1);
        }
    }
    
    FileSystem& FileSystem::instance()
    {
        static FileSystem sys;
        return sys;
    }
    
    std::string FileSystem::getFile( const std::string& filename )
    {
        if ( boost::filesystem::exists( filename ) ) return filename;
        assert( m_filesByName.find( filename ) != m_filesByName.end() );
        return m_filesByName[filename];
    }
    
    std::vector<std::string> FileSystem::getFilesByExt( const std::string& ext )
    {
        assert( m_filesByExt.find( ext ) != m_filesByExt.end() );
        return m_filesByExt[ext];
    }
    
    std::vector<std::string> FileSystem::getAllDirs()
    {
        return m_allDirs;
    }
}
