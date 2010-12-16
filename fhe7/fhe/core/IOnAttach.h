#ifndef FHE_I_ON_ATTACH_H
#define FHE_I_ON_ATTACH_H

namespace fhe
{
    
    class IOnAttach
    {
        public:
            virtual void onAttach() = 0;
            virtual void onDetach() = 0;
    };
    
}

#endif
