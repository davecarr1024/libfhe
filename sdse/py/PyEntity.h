#ifndef PYENTITY_H
#define PYENTITY_H

#include "core/Entity.h"
#include "boost/python.hpp"

namespace sdse {
    
    class PyEntity {
        private:
            Entity* entity;
            
        public: 
            PyEntity(Entity* _entity);
            
            boost::python::object getVar(std::string varName);
            boost::python::object getVarWithDefault(std::string varName, boost::python::object defaultVal);
            void setVar(std::string varName, boost::python::object val);
            bool hasVar(std::string varName);
            
            boost::python::object addAspect(std::string aspectType, std::string aspectName);
            bool hasAspect(std::string aspectName);
            void deleteAspect(std::string aspectName);
            
            void publish(std::string cmd, boost::python::object pyArgs);
            void subscribe(std::string cmd);
            void unsubscribe(std::string cmd);
            
            boost::python::object getEntity(std::string name);
            bool hasEntity(std::string name);
            void deleteEntity(std::string name);

            std::string getName();
    };
    
}

#endif
