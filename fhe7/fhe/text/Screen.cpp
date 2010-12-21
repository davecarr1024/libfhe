#include <fhe/text/Screen.h>
#include <ncurses.h>

namespace fhe
{
    namespace text
    {
        
        FHE_NODE( Screen );
        FHE_DEP( Screen, sim, IUpdate );
        
        Screen::Screen()
        {
            initscr();
            cbreak();
            keypad( stdscr, 1 );
        }
        
        Screen::~Screen()
        {
            nocbreak();
            keypad( stdscr, 0 );
            echo();
            endwin();
            printf( "~Screen\n" );
        }
        
        void Screen::update( double time, double dtime )
        {
            mvprintw( 10, 10, "update %f", time );
            refresh();
        }
        
    }
}
