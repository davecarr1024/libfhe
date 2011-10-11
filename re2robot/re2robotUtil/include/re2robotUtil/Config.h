#ifndef RE2_ROBOT_UTIL_CONFIG_H
#define RE2_ROBOT_UTIL_CONFIG_H

#include <tinyxml/tinyxml.h>
#include <boost/noncopyable.hpp>
#include <boost/shared_ptr.hpp>
#include <string>
#include <vector>
#include <set>

namespace re2
{
    namespace robot
    {
        
        class Config
        {
            public:
                Config( const std::string& path = "/robot_description" );
                virtual ~Config();
                
                Config operator[]( size_t index ) const;
                Config operator[]( const std::string& name ) const;
                
                size_t size() const;
                std::set< std::string > children() const;
                
                template <typename T>
                operator T() const
                {
                    return to<T>( T() );
                }
                
                template <typename T> 
                T get( const std::string& name, const T& def ) const
                {
                    return operator[]( name ).to<T>( def );
                }
                
                template <typename T>
                bool tryGet( const std::string& name, T& t ) const
                {
                    return operator[]( name ).tryTo<T>( t );
                }
                
                template <typename T>
                T to( const T& def ) const
                {
                    T t;
                    if ( tryTo<T>( t ) )
                    {
                        return t;
                    }
                    else
                    {
                        return def;
                    }
                }

                template <typename T>
                bool tryTo( T& t ) const
                {
                    std::istringstream is( toString() );
                    return is >> t;
                }
                
            private:
                class Reference : public boost::noncopyable
                {
                    public:
                        enum Type
                        {
                            TYPE_INDEX,
                            TYPE_NAME
                        };
                        
                        Reference( size_t index );
                        Reference( const std::string& name );
                        
                        Type type() const;
                        size_t index() const;
                        std::string name() const;
                        
                    private:
                        Type m_type;
                        size_t m_index;
                        std::string m_name;
                };
                
                typedef boost::shared_ptr< Reference > ReferencePtr;
                
                Config( const Config& parent, size_t index );
                Config( const Config& parent, const std::string& name );
                
                boost::shared_ptr< TiXmlDocument > m_doc;
                std::vector< ReferencePtr > m_references;
                
                TiXmlHandle toHandle( size_t offset = 0 ) const;
                std::string toString() const;
        };
        
    }
}

#endif
