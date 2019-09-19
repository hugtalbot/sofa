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
#pragma once

#include "config.h"

#include <SofaBaseVisual/VisualModelImpl.h>
#include <sofa/core/ObjectFactory.h>
#include <sofa/simulation/Node.h>

namespace sofa
{

namespace component
{

namespace visualmodel
{

class SOFA_BASE_VISUAL_API VisualCreator
{
public:
    using BaseObject = sofa::core::objectmodel::BaseObject;

    enum class VisualCreatorItem
    {
        VISUALMODEL,
        CAMERA,
        LIGHT
    };

    virtual ~VisualCreator();

    bool registerVisualModel(const std::string& visualModelClassName);
    VisualModelImpl::SPtr instanciateVisualModel( sofa::core::objectmodel::BaseContext::SPtr node);
    void clearRegisters() { m_map.clear(); }

    static std::shared_ptr<VisualCreator> getInstance();

private:
    VisualCreator();

    template <class T>
    typename T::SPtr instanciate(const std::string& className, sofa::core::objectmodel::BaseContext::SPtr node)
    {
        using namespace sofa::core::objectmodel;
        typename T::SPtr res;

        BaseObjectDescription desc(className.c_str(),className.c_str());
        BaseObject::SPtr obj = sofa::core::ObjectFactory::getInstance()->createObject(node.get(), &desc);

        res.reset(dynamic_cast<T*>(obj.get()));

        return res;
    }

    static std::shared_ptr<VisualCreator> m_instance;
    std::map<VisualCreatorItem, const std::string> m_map;
};



} // namespace visualmodel

} // namespace component

} // namespace sofa
