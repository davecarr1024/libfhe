#include "Var.h"
#include "VarMap.h"
#include "VarList.h"

#include "math/Vec2.h"
#include "math/Vec3.h"
#include "math/Rot.h"
#include "math/Quat.h"
#include "math/Color.h"

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
        else if ( type == "vec2" )
        {
            float x, y;
            ins >> x >> y;
            return Var::build<Vec2>(Vec2(x,y));
        }
        else if ( type == "vec3" )
        {
            float x, y, z;
            ins >> x >> y >> z;
            return Var::build<Vec3>(Vec3(x,y,z));
        }
        else if ( type == "rot" )
        {
            float a;
            ins >> a;
            return Var::build<Rot>(Rot(Math::radians(a)));
        }
        else if ( type == "quat" )
        {
            float x, y, z, a;
            ins >> x >> y >> z >> a;
            return Var::build<Quat>(Quat(Vec3(x,y,z),Math::radians(a)));
        }
        else if ( type == "color" )
        {
            float r, g, b, a = 1;
            ins >> r >> g >> b >> a;
            return Var::build<Color>(Color(r,g,b,a));
        }
        else if ( type == "dict" )
        {
            VarMap vm;
            for ( TiXmlElement* i = e->FirstChildElement("var"); i; i = i->NextSiblingElement("var") )
            {
                const char* name = i->Attribute("name");
                assert(name);
                vm._setVar(name,Var::load(i));
            }
            return Var::build<VarMap>(vm);
        }
        else if ( type == "list" )
        {
            VarList list;
            for ( TiXmlElement* i = e->FirstChildElement("var"); i; i = i->NextSiblingElement("var") )
            {
                list.appendRaw(Var::load(i));
            }
            return Var::build<VarList>(list);
        }
        else
        {
            throw std::runtime_error( "can't load unknown var type " + type );
        }
    }
    
    TiXmlElement* Var::save()
    {
        TiXmlElement* e = 0;
        std::ostringstream outs;
        if ( is<bool>() )
        {
            e = new TiXmlElement("var");
            e->SetAttribute("type","bool");
            outs << get<bool>();
            e->LinkEndChild(new TiXmlText(outs.str().c_str()));
        }
        else if ( is<int>() )
        {
            e = new TiXmlElement("var");
            e->SetAttribute("type","int");
            outs << get<int>();
            e->LinkEndChild(new TiXmlText(outs.str().c_str()));
        }
        else if ( is<float>() )
        {
            e = new TiXmlElement("var");
            e->SetAttribute("type","float");
            outs << get<float>();
            e->LinkEndChild(new TiXmlText(outs.str().c_str()));
        }
        else if ( is<std::string>() )
        {
            e = new TiXmlElement("var");
            e->SetAttribute("type","string");
            e->LinkEndChild(new TiXmlText(get<std::string>().c_str()));
        }
        else if ( is<VarList>() )
        {
            e = new TiXmlElement("var");
            e->SetAttribute("type","list");
            VarList vl = get<VarList>();
            for ( int i = 0; i < vl.length(); ++i )
            {
                TiXmlElement* var = vl.getRawVar(i).save();
                if ( var )
                {
                    e->LinkEndChild(var);
                }
            }
        }
        else if ( is<VarMap>() )
        {
            e = new TiXmlElement("var");
            e->SetAttribute("type","dict");
            VarMap vm = get<VarMap>();
            std::vector<std::string> varNames = vm.getVarNames();
            for ( std::vector<std::string>::iterator i = varNames.begin(); i != varNames.end(); ++i )
            {
                TiXmlElement* var = vm._getVar(*i).save();
                if ( var )
                {
                    var->SetAttribute("name",i->c_str());
                    e->LinkEndChild(var);
                }
            }
        }
        return e;
    }
}
