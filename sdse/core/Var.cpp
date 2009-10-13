#include "Var.h"

#include "core/math/Vector2.h"
#include "core/math/Vector3.h"
#include "core/math/Quaternion.h"

#include <cstdio>
#include <sstream>

using namespace sdse;

Var VarUtil::stringToVar(std::string s) {
    float f, x, y, z, a;
    int i;
    if (sscanf(s.c_str(),"%f%f%f%f",&a,&x,&y,&z) == 4)
        return Quaternion(Degree(a),Vector3(x,y,z));
    else if (sscanf(s.c_str(),"%f%f%f",&x,&y,&z) == 3)
        return Vector3(x,y,z);
    else if (sscanf(s.c_str(),"%f%f",&x,&y) == 2)
        return Vector2(x,y);
    else if (s.find(".") != std::string::npos && sscanf(s.c_str(),"%f",&f))
        return f;
    else if (sscanf(s.c_str(),"%i",&i))
        return i;
    else {
        if (s == "false")
            return false;
        else if (s == "true")
            return true;
        else if (s == "<null>")
            return Var();
        else
            return s;
    }
}

std::string VarUtil::varToString(Var var) {
    std::ostringstream sout;
    if (boost::any_cast<bool>(&var)) {
        if (boost::any_cast<bool>(var))
            return "true";
        else
            return "false";
    } else if (boost::any_cast<int>(&var)) {
        sout << boost::any_cast<int>(var);
        return sout.str();
    } else if (boost::any_cast<float>(&var)) {
        sout.precision(10);
        sout << boost::any_cast<float>(var);
        return sout.str();
    } else if (boost::any_cast<std::string>(&var))
        return boost::any_cast<std::string>(var);
    else if (boost::any_cast<Vector2>(&var)) {
        sout.precision(10);
        Vector2 v = boost::any_cast<Vector2>(var);
        sout << v.x << " " << v.y;
        return sout.str();
    } else if (boost::any_cast<Vector3>(&var)) {
        sout.precision(10);
        Vector3 v = boost::any_cast<Vector3>(var);
        sout << v.x << " " << v.y << " " << v.z;
        return sout.str();
    } else if (boost::any_cast<Quaternion>(&var)) {
        sout.precision(10);
        Radian angle;
        Vector3 axis;
        Quaternion q = boost::any_cast<Quaternion>(var);
        q.ToAngleAxis(angle,axis);
        sout << angle.valueDegrees() << " " << axis.x << " " << axis.y << " " << axis.z;
        return sout.str();
    }
    else
        return "<null>";
}

