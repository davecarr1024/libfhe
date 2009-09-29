#include "App.h"

using namespace fhe;

int main( int argc, char** argv )
{
    AppPtr app = boost::dynamic_pointer_cast<App,Node>(NodeFactory::instance().buildNode("App","root"));
    
    for ( int i = 1; i < argc; ++i )
    {
        app->loadChild(argv[i]);
    }
    
    app->run();
    
    return 0;
}
