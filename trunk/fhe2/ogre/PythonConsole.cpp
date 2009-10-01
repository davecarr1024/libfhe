#include "PythonConsole.h"

namespace fhe
{
    FHE_NODE_IMPL(PythonConsole);
    
    PythonConsole::PythonConsole()
    {
        m_ns = defaultNamespace();
        
        Redirect::defineClass();
        
        boost::python::object sys = boost::python::import("sys");
        m_stdout = new Redirect(this,sys.attr("stdout"));
        m_stderr = new Redirect(this,sys.attr("stderr"));
        sys.attr("stdout") = boost::python::object(boost::python::ptr(m_stdout));
        sys.attr("stderr") = boost::python::object(boost::python::ptr(m_stderr));
    }
    
    PythonConsole::~PythonConsole()
    {
        boost::python::object sys = boost::python::import("sys");
        sys.attr("stdout") = m_stdout->getStream();
        sys.attr("stderr") = m_stderr->getStream();
        delete m_stdout;
        delete m_stderr;
    }
    
    void PythonConsole::on_text( const std::string& text )
    {
        appendOutput( "> " + text );
        std::string input = text.substr(0,5) == "print" || text.find(" = ") != std::string::npos ? text : "print " + text;
        try
        {
            execScript(input,m_ns);
        }
        catch ( boost::python::error_already_set const& )
        {
            PyErr_Print();
            PyErr_Clear();
        }
    }
    
    PythonConsole::Redirect::Redirect( PythonConsole* console, boost::python::object stream ) :
        m_console(console),
        m_stream(stream)
    {
    }
    
    void PythonConsole::Redirect::write( const std::string& s )
    {
        m_stream.attr("write")(s);
        if ( s != "\n" )
        {
            m_console->appendOutput(s);
        }
    }
    
    boost::python::object PythonConsole::Redirect::getStream()
    {
        return m_stream;
    }
    
    boost::python::object PythonConsole::Redirect::defineClass()
    {
        static bool defined = false;
        if ( !defined )
        {
            defined = true;
            return boost::python::class_<PythonConsole::Redirect>("Redirect",boost::python::no_init)
                .def("write",&PythonConsole::Redirect::write)
            ;
        }
        else
        {
            return boost::python::object();
        }
    }
}
