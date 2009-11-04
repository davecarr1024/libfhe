#ifndef TEST_ASPECT_H
#define TEST_ASPECT_H

#include <gge/Aspect.h>

namespace gge
{
    namespace Test
    {
        class TestAspect : public Aspect
        {
            public:
                TestAspect() :
                    on_detach_called(false)
                {
                    addFunc("on_attach",&TestAspect::on_attach,this);
                    addFunc("on_detach",&TestAspect::on_detach,this);
                    addFunc("set_setTest",&TestAspect::set_setTest,this);
                    addFunc("get_getTest",&TestAspect::get_getTest,this);
                    addFunc("msg_msgTest",&TestAspect::msg_msgTest,this);
                    addFunc("load_loadTest",&TestAspect::load_loadTest,this);
                }
                
                Var on_attach( const Var& arg )
                {
                    getEntity()->setVar<bool>("on_attach_called",true);
                    return Var();
                }
                
                bool on_detach_called;
                
                Var on_detach( const Var& arg )
                {
                    on_detach_called = true;
                    return Var();
                }
                
                Var set_setTest( const Var& arg )
                {
                    getEntity()->setVar<int>("setTestVal",arg.get<int>(0));
                    return Var();
                }
                
                Var get_getTest( const Var& arg )
                {
                    Var val;
                    val.set<int>(35);
                    return val;
                }
                
                Var msg_msgTest( const Var& arg )
                {
                    getEntity()->setVar("msgTest",arg.get<int>(0));
                    return Var();
                }
                
                Var load_loadTest( const Var& arg )
                {
                    getEntity()->setVar<bool>("loadTest_called",true);
                    return Var();
                }
        };
    }
}

#endif
