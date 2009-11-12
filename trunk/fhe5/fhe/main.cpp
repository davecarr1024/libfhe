#include "App.h"
using namespace fhe;

int main( int argc, char **argv )
{
    EntityPtr root( new Entity("root") );
    AutoPtr<App> app( root->buildAspect("fhe/App").cast<App>() );
    for ( int i = 1; i < argc; ++i )
    {
        root->loadChild(argv[i]);
    }
    root->callNoRetNoArg("run");
    return 0;
}
