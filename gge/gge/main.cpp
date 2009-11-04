#include "App.h"
using namespace gge;

int main( int argc, char** argv )
{
    App app;
    for ( int i = 1; i < argc; ++i )
    {
        app.load(argv[i]);
    }
//     app.run();
    return 0;
}
