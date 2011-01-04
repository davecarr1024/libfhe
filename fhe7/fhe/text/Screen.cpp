#include <fhe/text/Screen.h>
#include <fhe/text/SceneNode.h>
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
        }
        
        void Screen::update( double time, double dtime )
        {
            m_rc.clear();
            publish( &SceneNode::render, &m_rc );
            int w, h, i, j;
            getmaxyx( stdscr, h, w );
            char c;
            for ( i = 0; i < w; ++i )
            {
                for ( j = 0; j < h; ++j )
                {
                    mvaddch( j, i, m_rc.get( i, j, c ) ? c : ' ' );
                }
            }
            refresh();
        }
        
    }
}
