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

#include "TriangleDiffusionFEMForceField.h"
#include <fstream> // for reading the file
#include <iostream> //for debugging
#include <sofa/helper/gl/template.h>
#include <SofaBaseTopology/TopologyData.inl>
#include <sofa/core/behavior/ForceField.inl>

namespace sofa
{

namespace component
{

namespace forcefield
{

   using namespace sofa::defaulttype;
   using namespace sofa::component::topology;
   using namespace core::topology;
   using namespace sofa::core::objectmodel;

   using core::topology::BaseMeshTopology;
   typedef BaseMeshTopology::EdgesInTriangle EdgesInTriangle;



// --------------------------------------------------------------------------------------
// ---  Topology Creation/Destruction functions
// --------------------------------------------------------------------------------------
template< class DataTypes>
void TriangleDiffusionFEMForceField<DataTypes>::TriangularTMEdgeHandler::applyCreateFunction(unsigned int , EdgeInformation &ei, const Edge &, const sofa::helper::vector<unsigned int> &, const sofa::helper::vector<double> &)
{
    if (ff)
        ei.diffusionScalar=0;
}



template< class DataTypes>
void TriangleDiffusionFEMForceField<DataTypes>::TriangularTMEdgeHandler::applyTriangleCreation(const sofa::helper::vector<unsigned int> &triangleAdded, const sofa::helper::vector<Triangle> &, const sofa::helper::vector<sofa::helper::vector<unsigned int> > &, const sofa::helper::vector<sofa::helper::vector<double> > &)
{
    if (ff)
    {
             size_t i;
             unsigned int j;

             typename DataTypes::Real area,restSquareLength[3],cotangent[3];
             typename DataTypes::Real diff;
             Vec3 point[3];

             const typename TriangleDiffusionFEMForceField<DataTypes>::MechanicalTypes::VecCoord restPosition =
                     ff->mechanicalObject->read(core::ConstVecCoordId::restPosition())->getValue();
             helper::vector<EdgeInformation>& edgeData = *(ff->d_edgeInfo.beginEdit());

             for (i=0;i<triangleAdded.size();++i)
             {
                /// describe the jth edge index of triangle no i
                const EdgesInTriangle &te= ff->m_topology->getEdgesInTriangle(triangleAdded[i]);
                /// describe the jth vertex index of triangle no i
                const Triangle &t= ff->m_topology->getTriangle(triangleAdded[i]);

                // store points
                for(j=0;j<3;++j)
                   point[j]= restPosition[t[j]];
                // store square rest length
                for(j=0;j<3;++j)
                {
                   restSquareLength[j]= (point[(j+1)%3] -point[(j+2)%3]).norm2();
                }
                // compute rest area based on Heron's formula
                area=0;

                for(j=0;j<3;++j)
                {
                   area+=restSquareLength[j]*(restSquareLength[(j+1)%3] +restSquareLength[(j+2)%3]-restSquareLength[j]);
                }
                area=sqrt(area)/4;
                diff=ff->d_diffusivity.getValue()/(4*area);

                for(j=0;j<3;++j)
                {
                   cotangent[j]=(restSquareLength[(j+1)%3] +restSquareLength[(j+2)%3]-restSquareLength[j])/(4*area);
                   if (ff->f_printLog.getValue())
                   {
                      if (cotangent[j]<0)
                         std::cerr<<"negative cotangent["<<triangleAdded[i]<<"]["<<j<<"]"<<std::endl;
                   }
                }

                std::cout<<"cotangent = "<<cotangent[0]<<","<<cotangent[1]<<","<<cotangent[2]<<std::endl;

                for(j=0;j<3;++j)
                {
                   EdgeInformation &eInfo = edgeData[te[j]];
                   eInfo.diffusionScalar += -cotangent[j]*diff/2;
                }
             }
             ff->d_edgeInfo.endEdit();
          }

}


template< class DataTypes>
void TriangleDiffusionFEMForceField<DataTypes>::TriangularTMEdgeHandler::applyTriangleDestruction(const sofa::helper::vector<unsigned int> &triangleRemoved)
{
    if (ff)
    {

        size_t i;
        unsigned int j;

        typename DataTypes::Real area,restSquareLength[3],cotangent[3];
        typename DataTypes::Real diff;
        Vec3 point[3];

        const typename TriangleDiffusionFEMForceField<DataTypes>::MechanicalTypes::VecCoord  restPosition=
                ff->mechanicalObject->read(core::ConstVecCoordId::restPosition())->getValue();
        helper::vector<EdgeInformation>& edgeData = *(ff->d_edgeInfo.beginEdit());

        for (i=0;i<triangleRemoved.size();++i)
        {

           /// describe the jth edge index of triangle no i
           const EdgesInTriangle &te= ff->m_topology->getEdgesInTriangle(triangleRemoved[i]);
           /// describe the jth vertex index of triangle no i
           const Triangle &t= ff->m_topology->getTriangle(triangleRemoved[i]);
           // store points
           for(j=0;j<3;++j)
              point[j]= restPosition[t[j]];
           // store square rest length
           for(j=0;j<3;++j)
           {
              restSquareLength[j]= (point[(j+1)%3] -point[(j+2)%3]).norm2();
           }
           // compute rest area based on Heron's formula
           area=0;
           for(j=0;j<3;++j)
           {
              area+=restSquareLength[j]*(restSquareLength[(j+1)%3] +restSquareLength[(j+2)%3]-restSquareLength[j]);
           }
           area=sqrt(area)/4;
           diff=ff->d_diffusivity.getValue()/(4*area);

           for(j=0;j<3;++j)
           {
              cotangent[j]=(restSquareLength[(j+1)%3] +restSquareLength[(j+2)%3]-restSquareLength[j])/(4*area);
              if (ff->f_printLog.getValue())
              {
                 if (cotangent[j]<0)
                    std::cerr<<"negative cotangent["<<triangleRemoved[i]<<"]["<<j<<"]"<<std::endl;
              }
           }
           for(j=0;j<3;++j)
           {
              EdgeInformation &eInfo = edgeData[te[j]];
              eInfo.diffusionScalar-= -cotangent[j]*diff/2;
           }
        }
        ff->d_edgeInfo.endEdit();
    }

}


// --------------------------------------------------------------------------------------
// --- constructor
// --------------------------------------------------------------------------------------
   template <class DataTypes>
   TriangleDiffusionFEMForceField<DataTypes>::TriangleDiffusionFEMForceField()
      :  d_edgeInfo(initData(&d_edgeInfo,"edgeInfo","Data to handle topology on edges"))
      , d_diffusivity(initData(&d_diffusivity, (Real)1.0,"diffusivity","Diffusion Coefficient"))
      , d_1DDiffusion(initData(&d_1DDiffusion, false, "scalarDiffusion","if true, diffuse only on the first dimension."))
      , d_drawPotentiel(initData(&d_drawPotentiel, false, "drawPotentiels","if true, draw the potentiels in the mesh"))
      , m_tagMeshMechanics(initData(&m_tagMeshMechanics,std::string("meca"),"tagMechanics","Tag of the Mechanical Object"))
      , l_mechanicalObjectLink(initLink("mechanicalObject3d", "MechanicalObject storing the 3D DOFs"))
      , l_topologyLink(initLink("topology", "Topology storing the 3D geometry"))
      , m_initialPoints(0)
      , m_updateMatrix(true)
   {
       edgeHandler = new TriangularTMEdgeHandler(this, &d_edgeInfo);
   }


   template <class DataTypes>
   TriangleDiffusionFEMForceField<DataTypes>::~TriangleDiffusionFEMForceField()
   {
       if(edgeHandler) delete edgeHandler;
   }


// --------------------------------------------------------------------------------------
// --- Initialization stage
// --------------------------------------------------------------------------------------
   template <class DataTypes>
   void TriangleDiffusionFEMForceField<DataTypes>::init()
   {
      this->Inherited::init();

       m_topology=NULL;

      /// Test if topology is present and is a appropriate
      if(!l_topologyLink.empty()) {
          m_topology = l_topologyLink.get();
          if (m_topology!=NULL) {
              msg_info()<<"Topology found (link)";
          }
      }
      else
      {
          m_topology = this->getContext()->getMeshTopology();
          if (m_topology!=NULL) {
              msg_info()<<"Topology found (context)";
          }
      }

      /// Error if topology is not found
      if (m_topology==NULL) {
          msg_error() << "Topology is not found";
          return;
      }

      /// Check that topology contains triangles
      if (m_topology->getNbTriangles()==0) {
         msg_error() << "Object must have a Triangular Set Topology not empty.";
         return;
      }

      /// Get the mechanical object containing the mesh position in 3D
      if(m_tagMeshMechanics.isSet()) {
          Tag mechanicalTag(m_tagMeshMechanics.getValue());
          this->getContext()->get(mechanicalObject, mechanicalTag,sofa::core::objectmodel::BaseContext::SearchUp);
          if (mechanicalObject!=NULL) {
              msg_info() << "mechanicalObject found (using the tag information \'"<<m_tagMeshMechanics.getValue()<<"\')";
          }
      }
      else {
          mechanicalObject = l_mechanicalObjectLink.get();
          if (mechanicalObject!=NULL) {
              msg_info() << "mechanicalObject found (link)";
          }
      }

      /// Check if pointer to mechanicalObject is valid
      if (mechanicalObject==NULL) {
         msg_error() << "Cannot find the mechanical object .";
         return;
      }

      /// Get rest Value of potential mechanicalObject
      m_initialPoints = this->mstate->read(core::ConstVecCoordId::restPosition())->getValue();

      /// prepare to store info in the edge array
      d_edgeInfo.createTopologicalEngine(m_topology,edgeHandler);
      d_edgeInfo.registerTopologicalData();

      /// Check rest position is not empty
      const typename MechanicalTypes::VecCoord restPosition = mechanicalObject->read(core::ConstVecCoordId::restPosition())->getValue();
      if(restPosition.size()==0) {
          msg_error() << "Rest position of the Vec3 MechanicalObject is empty.";
      }

      this->reinit();
   }

   template <class DataTypes>
   void TriangleDiffusionFEMForceField<DataTypes>::reinit()
   {
       helper::vector<typename TriangleDiffusionFEMForceField<DataTypes>::EdgeInformation>& edgeInf = *(d_edgeInfo.beginEdit());
       edgeInf.resize(m_topology->getNbEdges());

       size_t i;
       // set edge tensor to 0
       for (i=0;i<m_topology->getNbEdges();++i)
          edgeHandler->applyCreateFunction(i, edgeInf[i], m_topology->getEdge(i), (const sofa::helper::vector< unsigned int > )0, (const sofa::helper::vector< double >)0);

       // create edge tensor by calling the triangle creation function on the entire mesh
       sofa::helper::vector<unsigned int> triangleAdded;
       for (i=0;i<m_topology->getNbTriangles();++i)
          triangleAdded.push_back(i);

       edgeHandler->applyTriangleCreation(triangleAdded, (const sofa::helper::vector<Triangle>)0, (const sofa::helper::vector<sofa::helper::vector<unsigned int> >)0, (const sofa::helper::vector<sofa::helper::vector<double> >)0);

       d_edgeInfo.endEdit();
   }



   template <class DataTypes>
   void TriangleDiffusionFEMForceField<DataTypes>::handleTopologyChange()
   {
   }

   template <class DataTypes>
   SReal TriangleDiffusionFEMForceField<DataTypes>::getPotentialEnergy(const core::MechanicalParams*, const DataVecCoord&) const
   {
      msg_error()<<"TriangleDiffusionFEMForceField::getPotentialEnergy-not-implemented !!!";
      return 0;
   }


   template <class DataTypes>
   void TriangleDiffusionFEMForceField<DataTypes>::addForce(const core::MechanicalParams*, DataVecDeriv& dataf, const DataVecCoord& datax, const DataVecDeriv&)
   {
      VecDeriv& f = *dataf.beginEdit() ;
      const VecCoord& x = datax.getValue();

      size_t i,v0,v1;
      size_t nbEdges=m_topology->getNbEdges();

      helper::vector<typename TriangleDiffusionFEMForceField<DataTypes>::EdgeInformation>& edgeInf = *(d_edgeInfo.beginEdit());
      EdgeInformation *einfo;

      Coord dp;

      for(i=0; i<nbEdges; i++ )
      {
         v0=m_topology->getEdge(i)[0];
         v1=m_topology->getEdge(i)[1];
         einfo = &edgeInf[i];

         if (d_1DDiffusion.getValue())
         {
            dp[0] = x[v1][0]-x[v0][0];
            dp[0]*= einfo->diffusionScalar;

            f[v1][0]+=dp[0];
            f[v0][0]-=dp[0];
         }
         else
         {
            dp = x[v1]-x[v0];
            dp*= einfo->diffusionScalar;
            f[v1]+=dp;
            f[v0]-=dp;
         }
      }
      d_edgeInfo.endEdit();
      dataf.endEdit();
   }


   template <class DataTypes>
   void TriangleDiffusionFEMForceField<DataTypes>::addDForce(const core::MechanicalParams*, DataVecDeriv& dataf, const DataVecDeriv& datax )
   {
      VecDeriv& df = *dataf.beginEdit() ;
      const VecCoord& dx = datax.getValue();

      size_t v0,v1;
      size_t nbEdges=m_topology->getNbEdges();

      helper::vector<typename TriangleDiffusionFEMForceField<DataTypes>::EdgeInformation>& edgeInf = *(d_edgeInfo.beginEdit());
      EdgeInformation *einfo;

      Coord dp;

      for(size_t i=0; i<nbEdges; i++ )
      {
         v0=m_topology->getEdge(i)[0];
         v1=m_topology->getEdge(i)[1];
         einfo = &edgeInf[i];


         if (d_1DDiffusion.getValue())
         {
            dp[0] = dx[v1][0]-dx[v0][0];
            dp[0]*= einfo->diffusionScalar;

            df[v1][0]+=dp[0];
            df[v0][0]-=dp[0];
         }
         else
         {
            dp = dx[v1]-dx[v0];
            dp*= einfo->diffusionScalar;

            df[v1]+=dp;
            df[v0]-=dp;
         }
      }
      d_edgeInfo.endEdit();
      dataf.endEdit();
   }


   template<class DataTypes>
   void TriangleDiffusionFEMForceField<DataTypes>::draw(const core::visual::VisualParams* /*vparams*/)
   {
      if (d_drawPotentiel.getValue())
      {
         size_t i;

         const VecCoord& x = this->mstate->read(core::ConstVecCoordId::position())->getValue();
         size_t nbTriangles=m_topology->getNbTriangles();

         glDisable(GL_LIGHTING);
         glColor3f((float)0.2,(float)1.0,(float)1.0);
         glBegin(GL_TRIANGLES);

         for(i=0;i<nbTriangles; ++i)
         {

            size_t a = m_topology->getTriangle(i)[0];
            size_t b = m_topology->getTriangle(i)[1];
            size_t c = m_topology->getTriangle(i)[2];

            glColor4f(0,1,0,1);
            helper::gl::glVertexT(x[a]);
            glColor4f(0,0.5,0.5,1);
            helper::gl::glVertexT(x[b]);
            glColor4f(0,0,1,1);
            helper::gl::glVertexT(x[c]);
         }
         glEnd();
      }
   }

} // namespace forcefield

} // namespace Components

} // namespace Sofa
