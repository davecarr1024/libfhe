#include "Var.h"
#include "VarMap.h"
#include <sstream>

namespace fhe
{
    
    Var::Var() :
        m_data(0)
    {
    }
    
    Var::Var( const Var& var ) :
        m_data(var.m_data ? var.m_data->clone() : 0)
    {
    }
    
    Var& Var::operator=( const Var& var )
    {
        clear();
        m_data = var.m_data ? var.m_data->clone() : 0;
        return *this;
    }
    
    Var::~Var()
    {
        clear();
    }
    
    bool Var::empty() const
    {
        return !m_data;
    }
    
    void Var::clear()
    {
        if ( m_data )
        {
            delete m_data;
            m_data = 0;
        }
    }
    
    Var Var::load( TiXmlHandle h )
    {
        TiXmlElement* e = h.ToElement();
        assert(e);
        const char* ctype = e->Attribute("type");
        assert(ctype);
        const char* cvalue = e->GetText();
        std::string type(ctype), value(cvalue ? cvalue : "");
        std::istringstream ins(value);
        
        if ( type == "bool" )
        {
            bool b;
            ins >> b;
            return Var::build<bool>(b);
        }
        else if ( type == "int" )
        {
            int i;
            ins >> i;
            return Var::build<int>(i);
        }
        else if ( type == "float" )
        {
            float f;
            ins >> f;
            return Var::build<float>(f);
        }
        else if ( type == "string" )
        {
            return Var::build<std::string>(value);
        }
        else if ( type == "dict" )
        {
            VarMap vm;
            for ( TiXmlElement* i = e->FirstChildElement("var"); i; i = i->NextSiblingElement("var") )
            {
                const char* name = i->Attribute("name");
                assert(name);
                vm.setRawVar(name,Var::load(i));
            }
            return Var::build<VarMap>(vm);
        }
        else
        {
            throw std::runtime_error( "can't load unknown var type " + type );
        }
    }
}
