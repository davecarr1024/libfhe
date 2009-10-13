///sdse_aspect TestAspect

#ifndef TESTASPECT_H
#define TESTASPECT_H

#include "core/Aspect.h"

using namespace sdse;

class TestAspect : public Aspect {
    public:
        TestAspect(std::string _type, std::string _name) : Aspect("TestAspect",_type,_name) {}
        
        void onAttach(Entity* ent) {
            printf("testAspect attached %s %s\n",getName().c_str(),ent->getName().c_str());
            ent->setVar("testVar",212);
        }

        void onDetach(Entity* ent) {
            printf("testAspect detached %s %s\n",getName().c_str(),ent->getName().c_str());
        }
};

#endif
