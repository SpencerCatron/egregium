
#include <egregium/mesh/surface_mesh.h>

#include <map>
#include<unordered_map>
#include <utility>
#include <algorithm>

namespace egregium{


    SurfaceMesh::SurfaceMesh(const std::vector<std::array<int, 3>>& faces){

        std::map< std::pair<int,int>, int > halfedgeIndex;
        int nEdges=0;
        const int nFaces=static_cast<int> (faces.size());
        const int originalSize=3* nFaces;
        int maxVertexIndex=0;

        next_.resize(originalSize);
        twin_.resize(originalSize, INVALID);
        tail_.resize(originalSize);
        edge_.resize(originalSize);
        face_.resize(originalSize);

        //run through faces. 
        //mostly, use hashmap to map vertices to half edge. 
        //keep track of next_, tail_, face_, and edge_
        for(int i=0; i< nFaces; ++i){

            const auto & f=faces[i];

            maxVertexIndex=std::max({maxVertexIndex, f[0], f[1], f[2]});

            faceHalfedge_.push_back(3*i);

            //iterate vertices 
            for(int j=0; j<3; j++){

                const int tail = f[j];
                const int head =  f[(j+1)%3];
                const int heIdx = 3*i+j; // indexing for the current halfedge

                halfedgeIndex[{tail, head}]=heIdx;

                next_[heIdx]=3*i+(j+1)%3;
                tail_[heIdx]=tail;
                face_[heIdx]=i;

                //twin element hasn't been seen yet
                //count edge
                if( halfedgeIndex.count(  { head, tail} )==0){
                    edge_[heIdx]=nEdges;
                    edgeHalfedge_.push_back(heIdx);
                    ++nEdges;
                }
                //twin has been seen.
                //same edge. declare twins
                else{
                    int twinIdx=halfedgeIndex[{ head, tail} ];

                    edge_[heIdx] = edge_[twinIdx ];

                    twin_[heIdx] = twinIdx;
                    twin_[twinIdx]=heIdx;
                }
            }
        }

        //rerun twin_ , declare twins for boundary
        int boundaryCount = 0;
        std::unordered_map<int, int> heFromHead;
        for(int i= 0; i<originalSize; ++i){

            if(twin_[i]==INVALID){
                const int twinIdx=originalSize+boundaryCount;

                twin_[i]=twinIdx;
                heFromHead[tail_[next_[i]]]=i;

                face_.push_back(INVALID);
                twin_.push_back(i);
                edge_.push_back(edge_[i]);
                tail_.push_back(tail_[next_[i]]);
                next_.push_back(INVALID);

                boundaryCount++;
            }
        }
        for(const auto& [head, halfedgeIdx]: heFromHead){
            const int twinIdx=twin_[halfedgeIdx];
            //todo: check for non manifold input. will throw error here
            next_[twinIdx]= twin_[heFromHead[tail_[halfedgeIdx]]];
        }      
        
        // write vertexHalfedge_
        vertexHalfedge_.resize(maxVertexIndex+1, INVALID);
        const int nHalfedges= static_cast<int>(tail_.size());
        for( int i =0; i< nHalfedges; ++i){
            vertexHalfedge_[tail_[i]]=i;
        }
    }
}