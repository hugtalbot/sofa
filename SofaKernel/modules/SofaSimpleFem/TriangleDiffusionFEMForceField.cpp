/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 beta 4      *
*                (c) 2006-2009 MGH, INRIA, USTL, UJF, CNRS                    *
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

#define SOFA_COMPONENT_FORCEFIELD_TRIANGLEDIFFUSIONFEMFORCEFIELD_CPP

#include <sofa/core/ObjectFactory.h>
#include <sofa/core/topology/BaseMeshTopology.h>
#include <fstream> // for reading the file
#include <iostream> //for debugging
#include <vector>
#include <sofa/defaulttype/Vec3Types.h>
#include "TriangleDiffusionFEMForceField.inl"


// #define DEBUG_TRIANGLEFEM

namespace sofa
{

namespace component
{

namespace forcefield
{

using namespace sofa::defaulttype;

SOFA_DECL_CLASS(TriangleDiffusionFEMForceField)

using namespace sofa::defaulttype;


// Register in the Factory
int TriangleDiffusionFEMForceFieldClass = core::RegisterObject("Diffusion on a Triangular Mesh")
#ifndef SOFA_FLOAT
  .add< TriangleDiffusionFEMForceField<Vec1dTypes> >()
  .add< TriangleDiffusionFEMForceField<Vec2dTypes> >()
  .add< TriangleDiffusionFEMForceField<Vec3dTypes> >()
#endif
#ifndef SOFA_DOUBLE
  .add< TriangleDiffusionFEMForceField<Vec1fTypes> >()
  .add< TriangleDiffusionFEMForceField<Vec2fTypes> >()
  .add< TriangleDiffusionFEMForceField<Vec3fTypes> >()
#endif
;

#ifndef SOFA_FLOAT
  template class  TriangleDiffusionFEMForceField<Vec1dTypes>;
  template class  TriangleDiffusionFEMForceField<Vec2dTypes>;
  template class  TriangleDiffusionFEMForceField<Vec3dTypes>;
#endif
#ifndef SOFA_DOUBLE
  template class  TriangleDiffusionFEMForceField<Vec1fTypes>;
  template class  TriangleDiffusionFEMForceField<Vec2fTypes>;
  template class  TriangleDiffusionFEMForceField<Vec3fTypes>;
#endif



} // namespace forcefield

} // namespace Components

} // namespace Sofa

