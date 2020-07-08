/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
*                (c) 2006-2020 MGH, INRIA, USTL, UJF, CNRS                    *
*                                                                             *
* This library is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This library is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this library; if not, write to the Free Software Foundation,     *
* Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.          *
*******************************************************************************
*                               SOFA :: Modules                               *
*                                                                             *
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#pragma once

#include "config.h"

#include <sofa/core/behavior/ForceField.h>
#include <sofa/core/objectmodel/Data.h>
#include <sofa/helper/vector.h>
#include <sofa/core/BaseMapping.h>
#include <sofa/defaulttype/RGBAColor.h>


namespace sofa::core::behavior
{
template< class T > class MechanicalState;

} // namespace sofa::core::behavior


namespace sofa::component::forcefield
{

/**
* @brief This class describes a polynomial elastic springs ForceField between DOFs positions and rest positions.
*
* Springs are applied to given degrees of freedom between their current positions and their rest shape positions.
* An external MechanicalState reference can also be passed to the ForceField as rest shape position.
*/
template<class DataTypes>
class PolynomialRestShapeSpringsForceField : public core::behavior::ForceField<DataTypes>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(PolynomialRestShapeSpringsForceField, DataTypes), SOFA_TEMPLATE(core::behavior::ForceField, DataTypes));

    typedef core::behavior::ForceField<DataTypes> Inherit;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef typename DataTypes::VecDeriv VecDeriv;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::CPos CPos;
    typedef typename DataTypes::Deriv Deriv;
    typedef typename DataTypes::Real Real;
    typedef helper::vector<unsigned int> VecIndex;
    typedef helper::vector<Real> VecReal;

    typedef core::objectmodel::Data<VecCoord> DataVecCoord;
    typedef core::objectmodel::Data<VecDeriv> DataVecDeriv;


    Data< helper::vector<unsigned int> > d_points;
    Data< helper::vector<unsigned int> > d_external_points;

    /// polynomial data
    /// Describe set of polynomial coefficients combines in one array.
    /// The coefficients are put from smaller degree to bigger one, and the free polynomial parameter is also zero
    /// (for zero strain we have zero stress)
    /// For examples the coeffiencts for polynomials with degrees [3, 2, 4] will be put as [ a1, a2, a3, b1, b2, c1, c2, c3, c4]
    Data< VecReal > d_polynomialStiffness;
    /// Describe set of polynomial degrees fro every spring
    Data< helper::vector<unsigned int> > d_polynomialDegree;


    Data<bool> d_recomputeIndices;
    Data<bool> d_drawSpring;                      ///< draw Spring
    Data<defaulttype::RGBAColor> d_springColor;
    Data<float> d_showIndicesScale;

    Data<VecReal> d_zeroLength;       /// Springs initial lengths
    Data<double> d_smoothShift;
    Data<double> d_smoothScale;

    SingleLink<PolynomialRestShapeSpringsForceField<DataTypes>, sofa::core::behavior::MechanicalState<DataTypes>,
    BaseLink::FLAG_STOREPATH|BaseLink::FLAG_STRONGLINK> l_restMState;

    // data to compute spring derivatives
    typedef defaulttype::Vec<Coord::total_size, Real> JacobianVector;


protected:
    PolynomialRestShapeSpringsForceField();

    void recomputeIndices();

    VecIndex m_indices;
    VecIndex m_ext_indices;

    helper::vector<JacobianVector> m_differential;

    VecReal m_directionSpringLength;
    VecReal m_strainValue;
    VecCoord m_weightedCoordinateDifference;
    VecReal m_coordinateSquaredNorm;

    helper::vector<helper::vector<unsigned int>> m_polynomialsMap;

    bool m_useRestMState; /// Indicator whether an external MechanicalState is used as rest reference.


    void ComputeJacobian(unsigned int stiffnessIndex, unsigned int springIndex);
    double PolynomialValue(unsigned int springIndex, double strainValue);
    double PolynomialDerivativeValue(unsigned int springIndex, double strainValue);

public:
    void bwdInit() override;

    /// Add the forces.
    virtual void addForce(const core::MechanicalParams* mparams, DataVecDeriv& f, const DataVecCoord& x, const DataVecDeriv& v) override;

    virtual void addDForce(const core::MechanicalParams* mparams, DataVecDeriv& df, const DataVecDeriv& dx) override;

    /// Brings ForceField contribution to the global system stiffness matrix.
    virtual void addKToMatrix(const core::MechanicalParams* mparams, const sofa::core::behavior::MultiMatrixAccessor* matrix ) override;

    virtual void addSubKToMatrix(const core::MechanicalParams* mparams,
                                 const sofa::core::behavior::MultiMatrixAccessor* matrix,
                                 const helper::vector<unsigned> & addSubIndex ) override;

    virtual void draw(const core::visual::VisualParams* vparams) override;

    virtual SReal getPotentialEnergy(const core::MechanicalParams* /*mparams*/, const DataVecCoord& /* x */) const override
    {
        serr << "Get potentialEnergy not implemented" << sendl;
        return 0.0;
    }

    void addMBKToMatrix(const core::MechanicalParams* mparams, const sofa::core::behavior::MultiMatrixAccessor* matrix) override
    {
        sofa::core::BaseMapping *bmapping;
        this->getContext()->get(bmapping);
        if (bmapping ) /// do not call addKToMatrix since the object is mapped
            return;
        if (mparams->kFactorIncludingRayleighDamping(this->rayleighStiffness.getValue()) != 0.0 )
            this->addKToMatrix(mparams, matrix);
        if (mparams->bFactor() != 0.0)
            this->addBToMatrix(mparams, matrix);
    }


    const DataVecCoord* getExtPosition() const;
    const VecIndex& getIndices() const { return m_indices; }
    const VecIndex& getExtIndices() const { return (m_useRestMState ? m_ext_indices : m_indices); }
};

#if defined(SOFA_EXTERN_TEMPLATE) && !defined(SOFA_COMPONENT_FORCEFIELD_POLYNOMIAL_RESTSHAPESPRINGSFORCEFIELD_CPP)

using namespace sofa::defaulttype;

extern template class SOFA_DEFORMABLE_API PolynomialRestShapeSpringsForceField<Vec3Types>;

#endif // defined(SOFA_EXTERN_TEMPLATE) && !defined(SOFA_COMPONENT_FORCEFIELD_POLYNOMIAL_RESTSHAPESPRINGFORCEFIELD_CPP)

} // namespace sofa::component::forcefield
