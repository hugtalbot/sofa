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
#ifndef SOFA_COMPONENT_FORCEFIELD_TRIANGLEDIFFUSIONFEMFORCEFIELD_H
#define SOFA_COMPONENT_FORCEFIELD_TRIANGLEDIFFUSIONFEMFORCEFIELD_H

#if !defined(__GNUC__) || (__GNUC__ > 3 || (_GNUC__ == 3 && __GNUC_MINOR__ > 3))
#pragma once
#endif

#include <sofa/core/behavior/ForceField.h>
#include <sofa/core/topology/BaseMeshTopology.h>
#include <sofa/defaulttype/Vec.h>
#include <sofa/defaulttype/VecTypes.h>
#include <SofaBaseTopology/TopologyData.h>
#include <SofaBaseMechanics/MechanicalObject.h>

namespace sofa
{

namespace component
{


namespace forcefield
{
using namespace sofa::helper;
using namespace sofa::defaulttype;
using namespace sofa::component::topology;
using namespace sofa::component::container;

template<class DataTypes>
class TriangleDiffusionFEMForceField : public core::behavior::ForceField<DataTypes>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(TriangleDiffusionFEMForceField, DataTypes), SOFA_TEMPLATE(core::behavior::ForceField, DataTypes));

    typedef core::behavior::ForceField<DataTypes> Inherited;
    typedef typename DataTypes::VecCoord           VecCoord;
    typedef typename DataTypes::VecDeriv           VecDeriv;
    typedef typename DataTypes::Coord              Coord;
    typedef typename DataTypes::Deriv              Deriv;
    typedef typename Coord::value_type             Real;
    typedef core::objectmodel::Data<VecCoord>      DataVecCoord;
    typedef core::objectmodel::Data<VecDeriv>      DataVecDeriv;

    /// assumes the mechanical object type (3D)
    typedef Vec<3,Real>                            Vec3;
    typedef StdVectorTypes< Vec3, Vec3, Real >     MechanicalTypes ;
    typedef MechanicalObject<MechanicalTypes>      MechObject;

    typedef sofa::core::topology::Topology::Tetrahedron Tetrahedron;
    typedef sofa::core::topology::Topology::Triangle Triangle;
    typedef sofa::core::topology::Topology::Point Point;
    typedef sofa::core::topology::Topology::TetraID TetraID;
    typedef sofa::core::topology::Topology::Tetra Tetra;
    typedef sofa::core::topology::Topology::Edge Edge;
    typedef sofa::core::topology::BaseMeshTopology::EdgesInTriangle EdgesInTriangle;
    typedef sofa::core::topology::BaseMeshTopology::EdgesInTetrahedron EdgesInTetrahedron;
    typedef sofa::core::topology::BaseMeshTopology::TrianglesInTetrahedron TrianglesInTetrahedron;

protected:
    class EdgeInformation
    {
    public:
          Real diffusionScalar; /// the edge stiffness matrix
          /// Output stream
          inline friend std::ostream& operator<< ( std::ostream& os, const EdgeInformation& /*ei*/ )
          {return os;}

          /// Input stream
          inline friend std::istream& operator>> ( std::istream& in, EdgeInformation& /*ei*/ )
          {return in;}
    };

    EdgeData<helper::vector<EdgeInformation> > d_edgeInfo;

    class TriangularTMEdgeHandler : public TopologyDataHandler<Edge,vector<EdgeInformation> >
    {
    public:
      typedef typename TriangleDiffusionFEMForceField<DataTypes>::EdgeInformation EdgeInformation;
      TriangularTMEdgeHandler(TriangleDiffusionFEMForceField<DataTypes>* _ff, EdgeData<sofa::helper::vector<EdgeInformation> >* _data) : TopologyDataHandler<Edge, sofa::helper::vector<EdgeInformation> >(_data), ff(_ff) {}

      void applyCreateFunction(unsigned int edgeIndex,
                               EdgeInformation &ei,
                               const Edge& ,  const sofa::helper::vector< unsigned int > &,
                               const sofa::helper::vector< double >&);

      void applyTriangleCreation(const sofa::helper::vector<unsigned int> &triangleAdded,
                                 const sofa::helper::vector<Triangle> & ,
                                 const sofa::helper::vector<sofa::helper::vector<unsigned int> > & ,
                                 const sofa::helper::vector<sofa::helper::vector<double> > &);

      void applyTriangleDestruction(const sofa::helper::vector<unsigned int> &triangleRemoved);

    protected:
      TriangleDiffusionFEMForceField<DataTypes>* ff;
    };

public:
    //Constructor
    TriangleDiffusionFEMForceField();
    //Destructor
    virtual ~TriangleDiffusionFEMForceField();
    //Other functions
    void init() override;
    void reinit() override;
    void draw(const core::visual::VisualParams* vparams) override;

    /// Forcefield functions for Matrix system. Adding force to global forcefield vector.
    virtual void addForce(const core::MechanicalParams* mparams /* PARAMS FIRST */, DataVecDeriv& f, const DataVecCoord& x, const DataVecDeriv& v) override;

    /// Forcefield functions for Matrix system. Adding derivate force to global forcefield vector.
    virtual void addDForce(const core::MechanicalParams* mparams, DataVecDeriv& df, const DataVecDeriv& dx ) override;

    /// Return Potential energy of the mesh.
    virtual SReal getPotentialEnergy(const core::MechanicalParams* /*mparams*/, const DataVecCoord& x) const override;

    /// Return diffusivity coefficient.
    virtual Real getDiffusivity() const { return d_diffusivity.getValue();}

    /// Set diffusivity scalar.
    void setDiffusivity(const double val)
    {
      d_diffusivity.setValue(val);
    }

    /// handle topological changes.
    virtual void handleTopologyChange() override;




    /// Diffusivity scalar.
    Data<Real> d_diffusivity;
    /// bool used to specify 1D diffusion
    Data<bool> d_1DDiffusion;
    /// bool for drawing potential values.
    Data<bool> d_drawPotentiel;
    /// Mechanic xml tags of the system.
    Data<std::string> m_tagMeshMechanics;

    TriangularTMEdgeHandler* edgeHandler;



protected:
    /// Pointer to mechanical mechanicalObject
    MechanicalObject<MechanicalTypes> *mechanicalObject;
    /// Pointer to topology
    sofa::core::topology::BaseMeshTopology* m_topology;
    /// Link to 3D mechanical object (mechanics)
    SingleLink<TriangleDiffusionFEMForceField<DataTypes>, MechanicalObject< MechanicalTypes >, BaseLink::FLAG_STOREPATH|BaseLink::FLAG_STRONGLINK> l_mechanicalObjectLink;
    /// Link to the topology
    SingleLink<TriangleDiffusionFEMForceField<DataTypes>, sofa::core::topology::BaseMeshTopology, BaseLink::FLAG_STOREPATH|BaseLink::FLAG_STRONGLINK> l_topologyLink;

public:
    /// the intial potentials of the points.
    VecCoord  m_initialPoints;
    /// Update matrix??
    bool m_updateMatrix;


};


#if defined(WIN32) && !defined(SOFA_COMPONENT_FORCEFIELD_TRIANGLEDIFFUSIONFEMFORCEFIELD_CPP)
#pragma warning(disable : 4231)
#ifndef SOFA_FLOAT
template class  TriangleDiffusionFEMForceField<Vec1dTypes>;
template class  TriangleDiffusionFEMForceField<Vec2dTypes>;
template class  TriangleDiffusionFEMForceField<Vec3dTypes>;
#endif //SOFA_FLOAT
#ifndef SOFA_DOUBLE
template class  TriangleDiffusionFEMForceField<Vec1fTypes>;
template class  TriangleDiffusionFEMForceField<Vec2fTypes>;
template class  TriangleDiffusionFEMForceField<Vec3fTypes>;
#endif //SOFA_DOUBLE
#endif

} //namespace forcefield

} // namespace Components

} // namespace Sofa

#endif /* SOFA_COMPONENT_FORCEFIELD_TRIANGLEDIFFUSIONFEMFORCEFIELD_H */
