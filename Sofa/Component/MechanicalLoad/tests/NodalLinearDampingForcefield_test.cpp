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

//Force field
#include <sofa/component/mechanicalload/NodalLinearDampingForceField.h>
#include <sofa/component/solidmechanics/testing/ForceFieldTestCreation.h>

namespace sofa {

using sofa::component::mechanicalload::NodalLinearDampingForceField;

template<class T>
class MyNodalDamping : public NodalLinearDampingForceField<T> {
public:
    using NodalLinearDampingForceField<T>::d_dampingCoefficients;
};

/**  Test TrianglePressureForceField.
  */
template <typename _NodalLinearDampingForceField>
struct NodalLinearDampingForceField_test : public ForceField_test<_NodalLinearDampingForceField>
{
    typedef ForceField_test<_NodalLinearDampingForceField> Inherited;
    typedef _NodalLinearDampingForceField ForceType;
    typedef typename ForceType::DataTypes DataTypes;

    typedef typename ForceType::VecCoord VecCoord;
    typedef typename ForceType::VecDeriv VecDeriv;
    typedef typename ForceType::Coord Coord;
    typedef typename ForceType::Deriv Deriv;
    typedef typename Coord::value_type Real;
    typedef type::Vec<3,Real> Vec3;

    VecCoord x;
    VecDeriv v,f;

    NodalLinearDampingForceField_test():
        Inherited::ForceField_test(std::string(SOFA_COMPONENT_MECHANICALLOAD_TEST_SCENES_DIR) + "/" + "NodalLinearDampingForceField.scn")
    {}

    void test_requiredInputs()
    {
        EXPECT_MSG_EMIT(Error);
        sofa::simulation::node::initRoot(Inherited::node.get());
        ASSERT_FALSE(this->force->isComponentStateValid());
    }

    void test_setInvalidCoeficients()
    {
        EXPECT_MSG_EMIT(Error);
        this->force->findData("dampingCoefficients")->read("2.0 -3.0 4.0");
        sofa::simulation::node::initRoot(Inherited::node.get());
        ASSERT_FALSE(this->force->isComponentStateValid());
    }

    void test_setCoeficientsSizeSameAsMState()
    {
        this->force->findData("dampingCoefficients")->read("2.0 3.0 4.0");
        sofa::simulation::node::initRoot(Inherited::node.get());

        ASSERT_TRUE(this->force->isComponentStateValid());
        ASSERT_EQ(this->dof->getSize(), this->force->d_dampingCoefficients.getValue().size());
        auto size = this->dof->getSize();

        auto coefficients = helper::getReadAccessor(this->force->d_dampingCoefficients);
        for(auto i = 0; i < size ; ++i )
        {
            auto d = coefficients[i];
            decltype(d) v{2.0,3.0,4.0};
            ASSERT_EQ(d, v);
        }
    }

    void test_setCoeficientsReproduceLastValue()
    {
        this->force->findData("dampingCoefficients")->read("2.0 3.0 4.0 5.0 6.0 7.0");
        sofa::simulation::node::initRoot(Inherited::node.get());

        ASSERT_TRUE(this->force->isComponentStateValid());
        ASSERT_EQ(this->dof->getSize(), this->force->d_dampingCoefficients.getValue().size());
        auto size = this->dof->getSize();

        auto coefficients = helper::getReadAccessor(this->force->d_dampingCoefficients);
        for(auto i = 1; i < size ; ++i )
        {
            auto d = coefficients[i];
            decltype(d) v{5.0,6.0,7.0};
            ASSERT_EQ(d, v);
        }
    }

    void test_setConstantDamping()
    {
        this->force->findData("printLog")->read("1");
        this->force->findData("dampingCoefficients")->read("2.0 3.0 4.0");
        sofa::simulation::node::initRoot(Inherited::node.get());

        type::vector<Vec3> ones = {{2.0,3.0,4.0}};
        type::vector<Vec3> res = this->force->d_dampingCoefficients.getValue();
        ASSERT_EQ(res, ones);
    }

    void test_valueForce()
    {
        // run the forcefield_test
        Inherited::run_test( x, v, f );
    }

    // Test that the force value is constant
    void test_constantForce()
    {
        sofa::simulation::node::initRoot(Inherited::node.get());

        // Do a few animation steps
        for(int k=0;k<10;k++)
        {
            sofa::simulation::node::animate(Inherited::node.get(), 0.5_sreal);
        }

        // run the forcefield_test
        Inherited::run_test( x, v, f, false );
    }
};

// Types to instantiate.
typedef ::testing::Types<
    MyNodalDamping<defaulttype::Vec3Types>
> TestTypes;


// Tests to run for each instantiated type
TYPED_TEST_SUITE(NodalLinearDampingForceField_test, TestTypes);

TYPED_TEST( NodalLinearDampingForceField_test , requiredInput)
{
    this->test_requiredInputs();
}

TYPED_TEST( NodalLinearDampingForceField_test , test_setInvalidCoeficients)
{
    this->test_setInvalidCoeficients();
}

TYPED_TEST( NodalLinearDampingForceField_test , setConstantDamping)
{
    this->test_setConstantDamping();
}

TYPED_TEST( NodalLinearDampingForceField_test , setCoeficientsSizeSameAsMState)
{
    this->test_setCoeficientsSizeSameAsMState();
}

TYPED_TEST( NodalLinearDampingForceField_test , setCoeficientsReproduceLastValue)
{
    this->test_setCoeficientsReproduceLastValue();
}

}// namespace sofa
