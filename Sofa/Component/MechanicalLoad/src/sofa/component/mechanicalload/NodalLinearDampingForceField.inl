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

#include <sofa/component/mechanicalload/NodalLinearDampingForceField.h>
#include <sofa/core/MechanicalParams.h>
#include <sofa/core/behavior/BaseLocalForceFieldMatrix.h>

namespace sofa::component::mechanicalload
{

template <class DataTypes>
NodalLinearDampingForceField<DataTypes>::NodalLinearDampingForceField()
    : d_dampingCoefficients(initData(&d_dampingCoefficients, "dampingCoefficients", "Velocity damping coefficients (by cinematic dof and by node)"))
{
    sofa::core::objectmodel::Base::addUpdateCallback(
        "updateFromDampingCoefficientVector", {&d_dampingCoefficients},
        [this](const core::DataTracker&)
        {
            msg_info() << "call back update: from dampingCoefficients";
            return updateFromDampingCoefficients();
        },
        {});
}

template <class DataTypes>
sofa::core::objectmodel::ComponentState
NodalLinearDampingForceField<DataTypes>::updateFromDampingCoefficients()
{
    // Check if the coefficients has a null size, in that case we cannot do anything.
    auto coefficients = helper::getWriteAccessor(d_dampingCoefficients);
    auto coeffSize = coefficients.size();
    if (coeffSize == 0)
    {
        msg_error() << "Size of the \'dampingCoefficients\' vector is null";
        return sofa::core::objectmodel::ComponentState::Invalid;
    }

    // Recover the mstate size
    auto mstateSize = this->mstate->getSize();

    // If size is smaller than the one of the mstate, we resize it duplicating the last entry.
    if (coeffSize < mstateSize)
    {
        Deriv constantCoef = coefficients[coeffSize-1];
        coefficients.resize(mstateSize);
        for (unsigned i = coeffSize-1; i < mstateSize; ++i)
        {
            coefficients[i] = constantCoef;
        }
    }

    // At that point, the size of coefficient vector is the same as the one from mechanical object.
    assert(mstateSize==coefficients.size());

    // So we check of there is invalid entries, in that case we exit with an error.
    for (unsigned i = 0; i < mstateSize; ++i)
    {
        for (unsigned j = 0; j < Deriv::total_size; ++j)
        {
            if (coefficients[i][j] < 0.)
            {
                msg_error() << "Negative value at the " << i
                            << "th entry of the \'dampingCoefficients\' vector";
                return sofa::core::objectmodel::ComponentState::Invalid;
            }
        }
    }

    msg_info() << "Update from dampingCoefficients successfully done";
    return sofa::core::objectmodel::ComponentState::Valid;
}

template <class DataTypes>
void NodalLinearDampingForceField<DataTypes>::init()
{
    Inherit::init();

    // Case no input is given
    if (!d_dampingCoefficients.isSet())
    {
        msg_error() << "A least one damping coefficient should be specified.";
        this->d_componentState.setValue(sofa::core::objectmodel::ComponentState::Invalid);
        return;
    }

    this->d_componentState.setValue(sofa::core::objectmodel::ComponentState::Valid);
}

template <class DataTypes>
void NodalLinearDampingForceField<DataTypes>::addForce(const core::MechanicalParams*,
                                                       DataVecDeriv& _f, const DataVecCoord& _x,
                                                       const DataVecDeriv& _v)
{
    if (!this->isComponentStateValid()) return;

    SOFA_UNUSED(_x);

    auto f = sofa::helper::getWriteOnlyAccessor(_f);
    const auto v = sofa::helper::getReadAccessor(_v);
    const auto coefs = sofa::helper::getReadAccessor(d_dampingCoefficients);

    for (std::size_t i = 0; i < coefs.size(); ++i )
    {
        for (unsigned j = 0; j < Deriv::total_size; ++j)
        {
            f[i][j] -= v[i][j] * coefs[i][j];
        }
    }
}

template <class DataTypes>
void NodalLinearDampingForceField<DataTypes>::addDForce(const core::MechanicalParams* mparams,
                                                        DataVecDeriv& d_df,
                                                        const DataVecDeriv& d_dx)
{
    if (!this->isComponentStateValid()) return;

    const Real bfactor = (Real)mparams->bFactor();
    if(!bfactor) return;

    auto df = helper::getWriteOnlyAccessor(d_df);
    const auto dx = helper::getReadAccessor(d_dx);
    const auto& coefs = helper::getReadAccessor(d_dampingCoefficients);

    for (size_t i = 0; i < coefs.size(); ++i)
    {
        for (size_t j = 0; j < Deriv::total_size; ++j)
        {
            df[i][j] -= dx[i][j] * coefs[i][j] * bfactor;
        }
    }
}

template <class DataTypes>
void NodalLinearDampingForceField<DataTypes>::buildStiffnessMatrix(core::behavior::StiffnessMatrix*)
{
    // NodalLinearDampingForceField is a pure damping component
    // No stiffness is computed
}

template <class DataTypes>
void NodalLinearDampingForceField<DataTypes>::addBToMatrix(sofa::linearalgebra::BaseMatrix* mat,
                                                           SReal bFact, unsigned int& offset)
{
    if (bFact) return;
    if (!this->isComponentStateValid()) return;

    const auto& coefs = helper::getReadAccessor(d_dampingCoefficients);
    const unsigned int size = this->mstate->getSize();

    for (std::size_t i = 0; i < size; i++)
    {
        const unsigned blockrow = offset + i * Deriv::total_size;
        for (unsigned j = 0; j < Deriv::total_size; j++)
        {
            unsigned row = blockrow + j;
            mat->add(row, row, -coefs[i][j] * bFact);
        }
    }

}

template <class DataTypes>
void NodalLinearDampingForceField<DataTypes>::buildDampingMatrix(
    core::behavior::DampingMatrix* matrix)
{
    if (!this->isComponentStateValid()) return;

    auto dfdv = matrix->getForceDerivativeIn(this->mstate).withRespectToVelocityIn(this->mstate);
    const auto& coefs = d_dampingCoefficients.getValue();

    const unsigned int size = coefs.size();
    for (std::size_t i = 0; i < size; ++i)
    {
        const unsigned blockrow = i * Deriv::total_size;
        for (unsigned j = 0; j < Deriv::total_size; ++j)
        {
            const unsigned row = blockrow + j;
            dfdv(row, row) += -coefs[i][j];
        }
    }
}

template <class DataTypes>
SReal NodalLinearDampingForceField<DataTypes>::getPotentialEnergy(const core::MechanicalParams*,
                                                                  const DataVecCoord&) const
{
    return 0;
}

}  // namespace sofa::component::mechanicalload
