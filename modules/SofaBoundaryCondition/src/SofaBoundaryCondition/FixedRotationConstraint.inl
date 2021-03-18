/******************************************************************************
*                 SOFA, Simulation Open-Framework Architecture                *
*                    (c) 2006 INRIA, USTL, UJF, CNRS, MGH                     *
*                                                                             *
* This program is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this program. If not, see <http://www.gnu.org/licenses/>.        *
*******************************************************************************
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#pragma once

#include <SofaBoundaryCondition/FixedRotationConstraint.h>
#include <sofa/core/visual/VisualParams.h>
#include <utility>

using sofa::core::objectmodel::ComponentState;


namespace sofa::component::projectiveconstraintset
{


template <class DataTypes>
FixedRotationConstraint<DataTypes>::FixedRotationConstraint()
    : core::behavior::ProjectiveConstraintSet<DataTypes>(nullptr),
      d_fixedXRotation( initData( &d_fixedXRotation, false, "FixedXRotation", "Prevent Rotation around X axis")),
      d_fixedYRotation( initData( &d_fixedYRotation, false, "FixedYRotation", "Prevent Rotation around Y axis")),
      d_fixedZRotation( initData( &d_fixedZRotation, false, "FixedZRotation", "Prevent Rotation around Z axis")),
      d_indices( initData( &d_indices, "indices", "Indices to be fixed (if none, all nodes are considered)"))
{
    this->addUpdateCallback("updateIndices", { &d_indices}, [this](const core::DataTracker& t)
    {
        SOFA_UNUSED(t);
        checkIndices();
        return sofa::core::objectmodel::ComponentState::Valid;
    }, {});
}


template <class DataTypes>
FixedRotationConstraint<DataTypes>::~FixedRotationConstraint()
{
}


template <class DataTypes>
void FixedRotationConstraint<DataTypes>::init()
{
    this->d_componentState.setValue(ComponentState::Invalid);
    this->core::behavior::ProjectiveConstraintSet<DataTypes>::init();

    // Retrieves mechanical state
    VecCoord x = this->mstate->read(core::ConstVecCoordId::position())->getValue();

    // Stores initial orientation for each vertex
    m_previousOrientation.resize(x.size());
    for (Size i=0; i<m_previousOrientation.size(); i++)
    {
        m_previousOrientation[i] = x[i].getOrientation();
    }

    this->d_componentState.setValue(ComponentState::Valid);

    m_indices.clear();

    if(d_indices.getValue().size() != 0)
        checkIndices();
    else
    {
        m_indices.resize(x.size());
        for (Size i=0; i<x.size(); i++)
            m_indices[i] = i;
    }
}

template <class DataTypes>
void FixedRotationConstraint<DataTypes>::checkIndices()
{
    // Check value of given indices
    Index maxIndex=this->mstate->getSize();

    const SetIndexArray & indices = d_indices.getValue();
    m_indices.clear();
    m_indices.resize(indices.size());

    for (unsigned int i=0; i<indices.size(); ++i)
    {
        const Index index=indices[i];

        if (index >= maxIndex)
        {
            msg_warning() << "Index " << index << " not valid, should be [0,"<< maxIndex <<"]. Constraint will be removed.";
            this->d_componentState.setValue(ComponentState::Invalid);
            return;
        }

        m_indices[i] = index;
    }
}

template <class DataTypes>
void FixedRotationConstraint<DataTypes>::projectResponse(const core::MechanicalParams* /*mparams*/, DataVecDeriv& /*res*/)
{

}

template <class DataTypes>
void FixedRotationConstraint<DataTypes>::projectJacobianMatrix(const core::MechanicalParams* /*mparams*/, DataMatrixDeriv& /*res*/)
{

}

template <class DataTypes>
void FixedRotationConstraint<DataTypes>::projectVelocity(const core::MechanicalParams* /*mparams*/, DataVecDeriv& /*dx*/)
{

}

template <class DataTypes>
void FixedRotationConstraint<DataTypes>::projectPosition(const core::MechanicalParams* /*mparams*/, DataVecCoord& xData)
{
    if (this->d_componentState.getValue() != ComponentState::Valid) return;

    helper::WriteAccessor<DataVecCoord> x = xData;

    for (Size i = 0; i < m_indices.size(); ++i)
    {
        // Current orientations
        sofa::defaulttype::Quat Q = x[m_indices[i]].getOrientation();
        // Previous orientations
        sofa::defaulttype::Quat Q_prev = m_previousOrientation[m_indices[i]];

        auto project = [](const Vec3 a, const Vec3 b) -> Vec3 {
            return (a * b) * b;
        };
        auto decompose_ts = [&](const sofa::defaulttype::Quat q, const Vec3 twistAxis) {
            Vec3 vec3_part(q[0], q[1], q[2]);
            Vec3 projected = project(vec3_part, twistAxis);
            sofa::defaulttype::Quat twist(projected[0], projected[1], projected[2], q[3]);
            // Singularity : A perpendicular angle would give you quaternion (0, 0, 0, 0)
            if(std::none_of(twist.ptr(), twist.ptr() + 4 * sizeof(double), [](double x) {return x != 0. ;})) {
                twist = sofa::defaulttype::Quat::identity();
            }
            twist.normalize();
            sofa::defaulttype::Quat swing = q * twist.inverse();
            swing.normalize();
            return std::make_pair(twist, swing);
        };
        const Vec3 vx(1, 0, 0), vy(0, 1, 0), vz(0, 0, 1);

        sofa::defaulttype::Quat Q_remaining = Q;
        sofa::defaulttype::Quat Qp_remaining = Q_prev;
        sofa::defaulttype::Quat to_keep = sofa::defaulttype::Quat::identity();

        auto remove_rotation = [&](const Vec3 axis) {
            Q_remaining = decompose_ts(Q_remaining, axis).second;
            sofa::defaulttype::Quat twist;
            std::tie(twist, Qp_remaining) = decompose_ts(Qp_remaining, axis);
            to_keep = twist * to_keep;
        };

        if (d_fixedXRotation.getValue() == true){
            remove_rotation(vx);
        }
        if (d_fixedYRotation.getValue() == true){
            remove_rotation(vy);
        }
        if (d_fixedZRotation.getValue() == true){
            remove_rotation(vz);
        }
        x[m_indices[i]].getOrientation() = Q_remaining * to_keep;
    }

}


template <class DataTypes>
void FixedRotationConstraint<DataTypes>::draw(const core::visual::VisualParams* )
{
}

} // namespace sofa::component::projectiveconstraintset
