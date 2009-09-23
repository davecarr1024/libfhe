#include "App.h"
using namespace fhe;

int main( int argc, char** argv )
{
    AppPtr app( new App("App","App"));
    
    for ( int i = 1; i < argc; ++i )
    {
        app->addChild( Node::load( argv[i] ) );
    }
    
    app->run();
    
    return 0;
}
