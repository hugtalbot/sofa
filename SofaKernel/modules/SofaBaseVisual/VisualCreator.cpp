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

#include <SofaBaseVisual/VisualCreator.h>
#include <SofaSimulationGraph/DAGSimulation.h>

using sofa::component::visualmodel::VisualModelImpl;
using namespace sofa::core;

namespace sofa
{

namespace component
{

namespace visualmodel
{

std::shared_ptr<VisualCreator> VisualCreator::m_instance = nullptr;

std::shared_ptr<VisualCreator> VisualCreator::getInstance()
{
    if(m_instance == nullptr)
    {
        m_instance.reset(new VisualCreator());
    }

    return m_instance;
}

VisualCreator::VisualCreator()
{
    m_map.clear();
}

VisualCreator::~VisualCreator()
{
}

bool VisualCreator::registerVisualModel(const std::string& visualModelClassName)
{
    // if it exists in the ObjectFactory (instanciable)
    // and if it is really a VisualModel(Impl)
    sofa::core::objectmodel::BaseContext::SPtr dummyContext = sofa::core::objectmodel::New<objectmodel::BaseContext>();

    bool res = ObjectFactory::getInstance()->hasCreator(visualModelClassName)
             && instanciate<VisualModelImpl>(visualModelClassName, dummyContext);

    if(res)
    {
        m_map.emplace(VisualCreatorItem::VISUALMODEL,visualModelClassName);
    }

    return res;
}


VisualModelImpl::SPtr VisualCreator::instanciateVisualModel(sofa::core::objectmodel::BaseContext::SPtr node)
{
    auto mapVisualModel = m_map.find(VisualCreatorItem::VISUALMODEL);

    if(mapVisualModel != m_map.end())
    {
        return instanciate<VisualModelImpl>(mapVisualModel->second, node);
    }

    return nullptr;
}

} // namespace visualmodel

} //namespace component

} //namespace sofa

