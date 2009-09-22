#ifndef VAR_H
#define VAR_H

#include <vector>
#include <string>
#include <map>

#include <Poco/AutoPtr.h>
#include <Poco/RefCountedObject.h>
#include <Poco/DOM/Element.h>

namespace SGE
{
    class Var;
    
    typedef Poco::AutoPtr<Var> VarPtr;
    typedef std::vector<VarPtr> VarList;
    typedef std::map<std::string, VarPtr> VarMap;
    
    class Var : public Poco::RefCountedObject
    {
        public:
            enum VarType { NONE, BOOL, INT, FLOAT, STRING, LIST, DICT };
            
        private:
            VarType m_type;
            
        protected:
            Var( VarType type );
        
        public:
            Var();
            virtual ~Var() {}
            
            VarType getType() const;
            
            static VarPtr load( Poco::XML::Element* element );
    };

}

#endif
