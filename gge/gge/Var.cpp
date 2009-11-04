#include "Var.h"
#include "math/Vec3.h"
#include "math/Quat.h"
#include "math/Vec2.h"
#include "math/Rot.h"

namespace gge
{
    
    Var::Var() :
        m_data(0)
    {
    }
    
    Var::Var( const Var& var ) :
        m_data(var.m_data ? var.m_data->clone() : 0)
    {
    }
    
    Var& Var::operator=( const Var& var )
    {
        clear();
        m_data = var.m_data ? var.m_data->clone() : 0;
        return *this;
    }
    
    Var::~Var()
    {
        clear();
    }
    
    bool Var::empty() const
    {
        return !m_data;
    }
    
    void Var::clear()
    {
        if ( m_data )
        {
            delete m_data;
            m_data = 0;
        }
    }
}
