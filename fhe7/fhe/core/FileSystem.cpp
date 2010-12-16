#include <fhe/core/FileSystem.h>
#include <fhe/core/Util.h>
#include <boost/filesystem.hpp>

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
    
    std::string FileSystem::getExt( const std::string& path ) const
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
    
    std::string FileSystem::getFile( const std::string& filename ) const
    {
        if ( boost::filesystem::exists( filename ) ) return filename;
        std::map< std::string, std::string >::const_iterator i = m_filesByName.find( filename );
        FHE_ASSERT_MSG( i != m_filesByName.end(), "unable to find file %s", filename.c_str() );
        return i->second;
    }
    
    std::vector<std::string> FileSystem::getFilesByExt( const std::string& ext ) const
    {
        std::map< std::string, std::vector< std::string > >::const_iterator i = m_filesByExt.find( ext );
        FHE_ASSERT_MSG( i != m_filesByExt.end(), "no files with ext %s", ext.c_str() );
        return i->second;
    }
    
    std::vector<std::string> FileSystem::getAllDirs() const
    {
        return m_allDirs;
    }
}
