#include "Entity.h"
#include "Var.h"
#include "Aspect.h"
#include "App.h"
#include "FileServer.h"

#include <cstdio>

using namespace sdse;

class MainAspect : public Aspect {
    public:
        MainAspect(std::string _type, std::string _name) : Aspect("MainAspect",_type,_name) {
            printf("build %s\n",getName().c_str());
        }
        
        void onAttach(Entity* attachedTo) {
            printf("%s attached to %s\n",getName().c_str(),attachedTo->getName().c_str());
        }

        void onDetach(Entity* detachedFrom) {
            printf("%s detached from %s\n",getName().c_str(),detachedFrom->getName().c_str());
        }
        
        void onReceiveMessage(std::string cmd, VarList args) {
            int i = boost::any_cast<int>(args[0]);
            printf("%s receive %s %d\n",getName().c_str(),cmd.c_str(),i);
        }
        
        void onSetVar(std::string varName, Var val) {
            float f = boost::any_cast<float>(val);
            printf("%s got set var %s %f\n",getName().c_str(),varName.c_str(),f);
        }
        
        void onUpdate(float time, float dtime) {
            //printf("%s update %f %f\n",getName().c_str(),time,dtime);
        }
};

int main() {
/*    printf("bool %s\n",VarUtil::varToString(VarUtil::stringToVar(VarUtil::varToString(false))).c_str());
    printf("int %s\n",VarUtil::varToString(VarUtil::stringToVar(VarUtil::varToString(12))).c_str());
    printf("float %s\n",VarUtil::varToString(VarUtil::stringToVar(VarUtil::varToString(4.16f))).c_str());
    printf("string %s\n",VarUtil::varToString(VarUtil::stringToVar(VarUtil::varToString(std::string("test")))).c_str());
    
    printf("file %s\n",FileServer::getFile("test.py").c_str());*/
    
    App* app = new App();
    
/*    Entity* e1 = new Entity("ent");
    Entity* e2 = new Entity("ent");
    MainAspect* a1 = new MainAspect("MainAspect","asp");
    
    app->addEntity(e1);
    app->addEntity(e2);
    e1->addAspect(a1);
    e2->addAspect(a1);
    
    e2->subscribe("mainMsg");
    VarList args;
    args.push_back(4);
    e1->publish("mainMsg",args);
    
    e2->setVar("testFloat",3.14f);*/
    
    app->loadFile("test/test.sim");

//     Entity* e3 = app->getEntity("fileEnt");
//     
//     printf("%i %i %f %s\n",
//             boost::any_cast<bool>(e3->getVar("boolVar")),
//             boost::any_cast<int>(e3->getVar("intVar")),
//             boost::any_cast<float>(e3->getVar("floatVar")),
//             boost::any_cast<std::string>(e3->getVar("stringVar")).c_str());

    app->saveFile("test.xml");
            
    app->run();
    
    delete app;
    
    return 0;
}
