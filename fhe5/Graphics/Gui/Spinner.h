#ifndef SPINNER_H
#define SPINNER_H

#include "Widget.h"

namespace fhe
{
    namespace Graphics
    {
        class Spinner : public Widget
        {
            private:
                EntityPtr m_left, m_text, m_right;
                
            public:
                FHE_FUNC_DECL(on_attach);
                FHE_FUNC_DECL(msg_buttonClicked);
                FHE_FUNC_DECL(set_selection);
        };
    }
}

#endif
