#include "FileServer.h"

using namespace sdse;

bool FileServer::filesEnumerated = false;
StringMap FileServer::filesByName;
StringListMap FileServer::filesByType;
StringList FileServer::filePaths;

void FileServer::checkFilesEnumerated() {
    if (!filesEnumerated) {
        filesEnumerated = true;
        clear();
        enumerateFiles(boost::filesystem::path("."));
    }
}

void FileServer::clear() {
    filesByName.clear();
    filesByType.clear();
    filePaths.clear();
}

void FileServer::enumerateFiles(boost::filesystem::path path) {
    boost::filesystem::directory_iterator end;
    filePaths.push_back(path.string());
    for (boost::filesystem::directory_iterator i(path); i != end; ++i) {
        if (boost::filesystem::is_directory(i->status())) {
            enumerateFiles(i->path());
        } else {
            std::string filename = i->string(),
                ext = getExtension(filename),
                name = getName(filename);
                
            filesByName[name] = filename;
            
            if (filesByType.find(ext) != filesByType.end()) filesByType[ext] = StringList();
            filesByType[ext].push_back(filename);
        }
    }
}

std::string FileServer::getFile(std::string filename) {
    if (boost::filesystem::exists(boost::filesystem::path(filename)))
        return filename;
    checkFilesEnumerated();
    StringMap::iterator i = filesByName.find(filename);
    if (i != filesByName.end())
        return i->second;
    else
        return "";
}

StringList FileServer::getFilesOfType(std::string ext) {
    checkFilesEnumerated();
    StringListMap::iterator i = filesByType.find(ext);
    if (i != filesByType.end())
        return filesByType[ext];
    else
        return StringList();
}

StringList FileServer::getFilePaths() {
    return filePaths;
}

std::string FileServer::getExtension(std::string filename) {
    int pos = filename.rfind(".");
    if (pos != std::string::npos)
        return filename.substr(pos+1);
    else
        return "";
}

std::string FileServer::getName(std::string filename) {
    return filename.substr(filename.rfind("/")+1);
}
