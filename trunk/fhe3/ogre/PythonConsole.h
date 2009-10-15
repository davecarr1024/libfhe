#ifndef PYTHON_CONSOLE_H
#define PYTHON_CONSOLE_H

#include "Console.h"

namespace fhe
{
    
    class PythonConsole : public Console
    {
        private:
            class Redirect
            {
                private:
                    boost::python::object m_stream;
                    PythonConsole* m_console;
                
                public:
                    Redirect( PythonConsole* console, boost::python::object stream );
                    
                    void write( const std::string& s );
                    
                    boost::python::object getStream();
                    
                    static boost::python::object defineClass();
            };
            
            boost::python::dict m_ns;
            Redirect* m_stdout;
            Redirect* m_stderr;
            
        public:
            
            PythonConsole();
            ~PythonConsole();
            
            void on_text( const std::string& text );
    };
    
}

#endif
