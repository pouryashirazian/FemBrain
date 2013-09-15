FemBrain
=========================================================================================

Finite Element Simulation of Brain Tissues modeled with BlobTree implicit surface models.


Our system is capable of real-time animation and rendering of deformable models in the format of implicit surfaces.
The models are designed using skeletal implicit surface primitives, blending, warping and affine transformation operators
organized in a scene graph data structure called the BlobTree. When augmented with viscoelastic material properties the
BlobTree model can demonstrate interesting reactions to external forces applied to it by a haptic 
input device. The forces create reactions and deformations or in mechanical engineering terms: stresses and strains. 

By evaluating the stresses and strains of the BlobTree, a deformable model is resulted which can be
used in many applications. One such application is surgical simulation of Brain tissues in the Hyrocephalus
Brain operation.

For Rendering the BlobTree is discretized in its domain for mesh extraction, a process called "Polygonization". 
A by-product of Polygonization is a set of tethedral elements embedded inside the model. Interpolating the material
properties of the implicit primitives, the tetrahedral elements are associated with viscoelastic material properties.

The connected tetrahedra mesh is solved for stresses and strains using finite element procedures and the resulting
deformations are applied to the main output of Polygonization which is the triangle mesh.

We believe that our system is superior to the previous attempts in deformable tissue modeling due to its strong modeling
capabilities and the ability to simulate complex topologies in real-time. 



Video:
http://www.youtube.com/watch?v=CE7xb50RWX0
