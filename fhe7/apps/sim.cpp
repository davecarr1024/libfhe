#include <fhe/sim/Sim.h>
using namespace fhe;
using namespace sim;

int main( int argc, char** argv )
{
    if ( argc < 2 )
    {
        fprintf( stderr, "usage: %s <filename>\n", argv[0] );
        return 1;
    }
    else
    {
        NodePtr sim( new Sim );
        sim->attachChild( Node::load( argv[1] ) );
        sim->call( &Sim::run );
        return 0;
    }
}
