#include "Widget.h"
#include <fhe/math/Color.h>

namespace fhe
{
    namespace Graphics
    {
        FHE_ASPECT(Widget,SceneNode2);
        
        FHE_FUNC_IMPL(Widget,on_attach)
        {
            VarMap defFill, defStroke, defFocusFill, defFocusStroke;
            
            Color defFillColor = getEntity()->getAncestorVar<Color>("guiFillColor",Color(0,0,1,0.5)), 
                defStrokeColor = getEntity()->getAncestorVar<Color>("guiStrokeColor",Color(1,1,1,1));
            
            defFill = getEntity()->getAncestorVar<VarMap>("guiFill",VarMap());
            defFocusFill = defFill;
            defFill.defaultVar<Color>("color",defFillColor);
            defFocusFill.defaultVar<Color>("color",defFillColor.darken());
            
            defStroke = getEntity()->getAncestorVar<VarMap>("guiStroke",VarMap());
            defFocusStroke = defStroke;
            defStroke.defaultVar<Color>("color",defStrokeColor);
            defFocusStroke.defaultVar<Color>("color",defStrokeColor.darken());
                
            getEntity()->defaultVar<VarMap>("fill",defFill);
            getEntity()->defaultVar<VarMap>("focusFill",defFocusFill);
            getEntity()->defaultVar<VarMap>("stroke",defStroke);
            getEntity()->defaultVar<VarMap>("focusStroke",defFocusStroke);
            
            return Var();
        }
    }
}
