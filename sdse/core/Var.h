#ifndef VAR_H
#define VAR_H

#include <boost/any.hpp>
#include <string>
#include <map>
#include <vector>

namespace sdse {

    typedef std::map<std::string, std::string> StringMap;
    typedef std::vector<std::string> StringList;
    typedef std::map<std::string, StringList> StringListMap;
    
    typedef boost::any Var;
    
    class VarUtil {
        public:
            static Var stringToVar(std::string s);
            static std::string varToString(Var var);
    };
    
    typedef std::vector<Var> VarList;
    typedef std::map<std::string, Var> VarMap;

}

#endif
