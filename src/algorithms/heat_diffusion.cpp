#include <egregium/algorithms/heat_diffusion.h>
#include <stdexcept>
#include <egregium/operators/laplacian.h>

namespace egregium {

HeatDiffusion::HeatDiffusion(const Geometry& geom, const HeatParams& params = {}){
    size_ = geom.mesh().nVertices();
    if(params.diffusivity.size()==0){
        diffusivity_=Eigen::VectorXd::Ones(size_);
    }
    else if( params.diffusivity.size()!= size_){
        throw std::invalid_argument("HeatDiffusion: Diffusivity must be same size as vertices on mesh.");
    }
    else{
        diffusivity_=params.diffusivity;
    }
    timeStep_ = params.timeStep;
    ambient_=params.ambient;
    coupling_=params.coupling;
    L_=cotanLaplacian(geom);
    M_=massMatrix(geom);
}

void HeatDiffusion::setState(const Eigen::VectorXd& u){
    u_=u;
}

void HeatDiffusion::setTimeStep(double timeStep){
    timeStep_=timeStep;
}

void HeatDiffusion::setAmbientTemp(double ambient, double coupling){
    ambient_=ambient;
    coupling_=coupling;
}

void HeatDiffusion::setDiffusivity(Eigen::VectorXd& diffusivity){
    if (diffusivity.size() !=size_){
        throw std::invalid_argument("HeatDiffusion: Diffusivity must be same size as vertices on mesh.");
    }
}

void HeatDiffusion::step(){
    timeTotal_+=timeStep_;
}

double HeatDiffusion::elapsedTime() const{
    return timeTotal_;
}

Eigen::Vector3d HeatDiffusion::state() const{
    return u_;
}
}  // namespace egregium