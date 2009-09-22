#include "Var.h"

#include "BoolVar.h"
#include "IntVar.h"
#include "FloatVar.h"
#include "StringVar.h"
#include "ListVar.h"
#include "DictVar.h"

#include "StringUtil.h"

#include <Poco/DOM/Node.h>

#include <cstdio>

namespace SGE
{
    
    Var::Var() :
        m_type( NONE )
    {
    }
    
    Var::Var( VarType type ) :
        m_type( type )
    {
    }
    
    Var::VarType Var::getType() const
    {
        return m_type;
    }
    
    VarPtr Var::load( Poco::XML::Element* element )
    {
        if ( element )
        {
            std::string tagName = element->tagName();
            
            if ( tagName == "bool" )
            {
                bool val;
                if ( StringUtil::tryParse( element->innerText(), val ) )
                    return new BoolVar( val );
            }
            else if ( tagName == "int" )
            {
                int val;
                if ( StringUtil::tryParse( element->innerText(), val ) )
                    return new IntVar( val );
            }
            else if ( tagName == "float" )
            {
                float val;
                if ( StringUtil::tryParse( element->innerText(), val ) )
                    return new FloatVar( val );
            }
            else if ( tagName == "string" )
            {
                std::string val;
                if ( StringUtil::tryParse( element->innerText(), val ) )
                    return new StringVar( val );
            }
            else if ( tagName == "list" )
            {
                ListVar* list( new ListVar );
                for ( Poco::XML::Node* node = element->firstChild(); node; node = node->nextSibling() )
                {
                    Poco::XML::Element* child = dynamic_cast<Poco::XML::Element*>(node);
                    if ( child )
                    {
                        VarPtr childVar = load( child );
                        if ( childVar )
                        {
                            list->append( childVar );
                        }
                    }
                }
                return list;
            }
            else if ( tagName == "dict" )
            {
                DictVar* dict( new DictVar );
                for ( Poco::XML::Node* node = element->firstChild(); node; node = node->nextSibling() )
                {
                    Poco::XML::Element* child = dynamic_cast<Poco::XML::Element*>(node);
                    if ( child && child->hasAttribute( "name" ) )
                    {
                        VarPtr childVar = load( child );
                        if ( childVar )
                        {
                            dict->set( child->getAttribute( "name" ), childVar );
                        }
                    }
                }
                return dict;
            }
        }
        return 0;
    }
    
}
