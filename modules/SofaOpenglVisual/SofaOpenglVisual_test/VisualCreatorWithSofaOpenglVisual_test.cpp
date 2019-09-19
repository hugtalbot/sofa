/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, development version     *
*                (c) 2006-2019 INRIA, USTL, UJF, CNRS, MGH                    *
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

#include <gtest/gtest.h>
#include <SofaBaseVisual/VisualCreator.h>
#include <SofaOpenglVisual/OglModel.h>

using namespace sofa::component::visualmodel;
using namespace sofa::core::objectmodel;

namespace sofa {

struct VisualCreatorWithSofaOpenglVisual_test : public ::testing::Test
{
    std::shared_ptr<VisualCreator> m_creator;

    VisualCreatorWithSofaOpenglVisual_test()
    {
        m_creator = VisualCreator::getInstance();
        sofa::component::visualmodel::VisualCreator::getInstance()->registerVisualModel("OglModel");
    }

};

TEST_F( VisualCreatorWithSofaOpenglVisual_test , testOglAlreadyRegistered )
{
    BaseContext::SPtr dummyContext = New<BaseContext>();

    auto resultVM = m_creator->instanciateVisualModel(dummyContext);
    ASSERT_TRUE(resultVM != nullptr);
    ASSERT_TRUE(dynamic_cast<OglModel*>(resultVM.get()));
}

} //sofa
