#pragma once

#include <array>
#include <vector>

namespace egregium{


    constexpr int INVALID =-1;

    /*
    Half edge structure that stores connectivity.

    each edge has two half edges pointing in opposite directions. 
    */
    class SurfaceMesh{

    public:
        
        //build structure from list of triangles. each triangle has 3 vertices listed in counterclockwise order
        explicit SurfaceMesh(const std::vector<std::array<int, 3>>& faces);

        //element counts
        int nVertices() const;
        int nEdges() const;
        int nHalfedges() const;

        //half-edge navigation
        int next(int h) const; //next half edge on same face, counterclockwise
        int twin(int h) const; // other half edge along a given edge
        int tailVertex(int h) const; //tail vertex of half edge
        int headVertex(int h) const; //head of half edge
        int edge(int h) const; //edge given half edge belongs to
        int face(int h) const; // face half edge belongs to (face on leftside of arrow)


        //elements to half edge
        //choice tbd
        int vertexHalfedge(int v) const;
        int edgeHalfedge(int e) const;
        int faceHalfedge(int f) const;

        //derived
        bool isBoundaryHalfedge(int h) const;
        bool isBoundaryVertex(int e) const;
        int degree(int v) const; //number edges at vertex v
        int eulerCharaceristic() const; //v-E+F, tological invariant

    private:
    
        void buildBoundaryHalfedges();
        void assignVertexHalfedges();

        std::vector<int> next_;
        std::vector<int> twin_;
        std::vector<int> tail_;
        std::vector<int> edge_;
        std::vector<int> face_;

        //halfedge for given vertex, edge,face 
        std::vector<int> vertexHalfedge_;
        std::vector<int> edgeHalfedge_;
        std::vector<int> faceHalfedge_;
    };
}