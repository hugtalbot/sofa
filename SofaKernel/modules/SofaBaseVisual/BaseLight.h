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

#include <sofa/core/visual/VisualModel.h>

namespace sofa
{

namespace component
{

namespace visualmodel
{

class SOFA_BASE_VISUAL_API BaseLight : public sofa::core::visual::VisualModel
{
public:
    SOFA_CLASS(BaseLight, core::objectmodel::BaseObject);

    enum LightType { DIRECTIONAL = 0, POSITIONAL = 1, SPOTLIGHT = 2 };

protected:
    BaseLight();
    virtual ~BaseLight() override;

public:
    /// Draw the light source from an external point of view.
    virtual void drawSource(const sofa::core::visual::VisualParams*) = 0;

    virtual LightType getLightType() = 0;

};



} // namespace visualmodel

} // namespace component

} // namespace sofa
