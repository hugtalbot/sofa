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
#include <SofaBoundaryCondition/config.h>

#include <sofa/core/behavior/ProjectiveConstraintSet.h>
#include <sofa/defaulttype/Quat.h>
#include <SofaBaseTopology/TopologySubsetData.h>

namespace sofa::component::projectiveconstraintset
{

/**
 * Prevents rotation around X or Y or Z axis
 */
template <class DataTypes>
class FixedRotationConstraint : public core::behavior::ProjectiveConstraintSet<DataTypes>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(FixedRotationConstraint,DataTypes),SOFA_TEMPLATE(sofa::core::behavior::ProjectiveConstraintSet, DataTypes));

    typedef typename DataTypes::VecCoord VecCoord;
    typedef typename DataTypes::VecDeriv VecDeriv;
    typedef typename DataTypes::MatrixDeriv MatrixDeriv;
    typedef typename DataTypes::MatrixDeriv::RowType MatrixDerivRowType;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::Deriv Deriv;
    typedef typename DataTypes::Real Real;
    typedef Data<VecCoord> DataVecCoord;
    typedef Data<VecDeriv> DataVecDeriv;
    typedef Data<MatrixDeriv> DataMatrixDeriv;
    typedef defaulttype::Vec<3,Real> Vec3;
    typedef helper::vector<Index> SetIndexArray;

protected:
    FixedRotationConstraint();
    ~FixedRotationConstraint() override;

    void checkIndices();

public:
    void init() override;

    void projectResponse(const core::MechanicalParams* mparams, DataVecDeriv& dx) override;
    void projectVelocity(const core::MechanicalParams* mparams, DataVecDeriv& dx) override;
    void projectPosition(const core::MechanicalParams* mparams, DataVecCoord& x) override;
    void projectJacobianMatrix(const core::MechanicalParams* mparams, DataMatrixDeriv& c) override;

    void draw(const core::visual::VisualParams* vparams) override;


protected :
    Data< bool > d_fixedXRotation; ///< Prevent Rotation around X axis
    Data< bool > d_fixedYRotation; ///< Prevent Rotation around Y axis
    Data< bool > d_fixedZRotation; ///< Prevent Rotation around Z axis
    Data< SetIndexArray > d_indices; ///< Indices to be fixed

    SetIndexArray m_indices; ///< Indices to be fixed
    helper::vector<defaulttype::Quat> m_previousOrientation;
};


#if  !defined(SOFA_COMPONENT_PROJECTIVECONSTRAINTSET_FIXEDROTATIONCONSTRAINT_CPP)
extern template class FixedRotationConstraint<defaulttype::Rigid3Types>;

#endif

} // namespace sofa::component::projectiveconstraintset
