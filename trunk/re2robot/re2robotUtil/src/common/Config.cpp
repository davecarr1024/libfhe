#include <re2robotUtil/Config.h>
#include <ros/ros.h>
#include <tinyxml/tinyxml.h>

namespace re2
{
    namespace robot
    {
        
        Config::Config( const std::string& path ) :
            m_doc( new TiXmlDocument )
        {
            std::string xml;
            ROS_ASSERT_MSG( ros::param::get( path, xml ), "unable to load robot description from %s", path.c_str() );
            m_doc->Parse( xml.c_str() );
            ROS_ASSERT_MSG( !m_doc->Error(), "unable to parse robot description %s: %s", path.c_str(), m_doc->ErrorDesc() );
        }
        
        Config::Config( const Config& parent, size_t index ) :
            m_doc( parent.m_doc ),
            m_references( parent.m_references )
        {
            m_references.push_back( ReferencePtr( new Reference( index ) ) );
        }
        
        Config::Config( const Config& parent, const std::string& name ) :
            m_doc( parent.m_doc ),
            m_references( parent.m_references )
        {
            m_references.push_back( ReferencePtr( new Reference( name ) ) );
        }
        
        Config::~Config()
        {
        }
        
        Config Config::operator[]( size_t index ) const
        {
            return Config( *this, index );
        }
        
        Config Config::operator[]( const std::string& name ) const
        {
            return Config( *this, name );
        }
        
        Config::Reference::Reference( size_t index ) :
            m_type( TYPE_INDEX ),
            m_index( index )
        {
        }
        
        Config::Reference::Reference( const std::string& name ) :
            m_type( TYPE_NAME ),
            m_name( name )
        {
        }
        
        Config::Reference::Type Config::Reference::type() const
        {
            return m_type;
        }
        
        size_t Config::Reference::index() const
        {
            ROS_ASSERT_MSG( type() == TYPE_INDEX, "reference type mismatch" );
            return m_index;
        }
        
        std::string Config::Reference::name() const
        {
            ROS_ASSERT_MSG( type() == TYPE_NAME, "reference type mismatch" );
            return m_name;
        }
        
        TiXmlHandle Config::toHandle( size_t offset ) const
        {
            TiXmlHandle handle( m_doc.get() );
            
            for ( size_t i = 0; i < m_references.size() - offset; ++i )
            {
                if ( m_references[i]->type() == Reference::TYPE_NAME )
                {
                    if( i + 1 < m_references.size() && m_references[ i + 1 ]->type() == Reference::TYPE_INDEX )
                    {
                        handle = handle.Child( m_references[ i ]->name().c_str(), m_references[ i + 1 ]->index() );
                        ++i;
                    }
                    else
                    {
                        handle = handle.FirstChild( m_references[ i ]->name().c_str() );
                    }
                }
                else
                {
                    ROS_WARN( "an index reference must be preceded by a name reference" );
                    handle = TiXmlHandle( 0 );
                }
            }
            
            return handle;
        }
        
        std::string Config::toString() const
        {
            TiXmlHandle handle = toHandle();
            TiXmlElement* element = handle.ToElement();
            if ( element )
            {
                return element->GetText();
            }

            if ( m_references.size() && m_references[ m_references.size() - 1 ]->type() == Reference::TYPE_NAME )
            {
                std::string attr = m_references[ m_references.size() - 1 ]->name();
                handle = toHandle( 1 );
                element = handle.ToElement();
                if ( element )
                {
                    const char* val = element->Attribute( attr.c_str() );
                    if ( val )
                    {
                        return val;
                    }
                }
            }
            
            return std::string();
        }
        
        size_t Config::size() const
        {
            if ( m_references.size() && m_references[ m_references.size() - 1 ]->type() == Reference::TYPE_NAME )
            {
                std::string name = m_references[ m_references.size() - 1 ]->name();
                TiXmlHandle handle = toHandle();
                TiXmlElement* element = handle.ToElement();
                if ( element )
                {
                    size_t size;
                    for ( size = 0; element; element = element->NextSiblingElement( name ), ++size );
                    return size;
                }
            }
            return 0;
        }
        
        std::set< std::string > Config::children() const
        {
            TiXmlHandle handle = toHandle();
            std::set< std::string > children;
            for ( TiXmlElement* element = handle.ToElement(); element; element = element->NextSiblingElement() )
            {
                children.insert( element->Value() );
            }
            return children;
        }
        
    }
}
