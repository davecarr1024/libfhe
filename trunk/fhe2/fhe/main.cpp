#include "App.h"

using namespace fhe;

int main( int argc, char** argv )
{
    AppPtr app( (App*)NodeFactory::instance().buildNode("App","root") );
    
    for ( int i = 1; i < argc; ++i )
    {
        app->loadChild(argv[i]);
    }
    
    app->run();
    
    return 0;
}
