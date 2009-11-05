#include "Entity.h"
using namespace gge;

int main( int argc, char** argv )
{
    EntityPtr app(new Entity("app") );
    for ( int i = 1; i < argc; ++i )
    {
        app->loadChild(argv[i]);
    }
//     app.run();
    return 0;
}
