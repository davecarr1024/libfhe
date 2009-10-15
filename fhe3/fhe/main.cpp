#include "App.h"

using namespace fhe;

int main( int argc, char** argv )
{
    Poco::AutoPtr<App> app( Entity::root->buildChild("App")->addAspect("App").cast<App>() );
    for ( int i = 1; i < argc; ++i )
    {
        app->getEntity()->loadChild( argv[i] );
    }
    app->run();
    return 0;
}
