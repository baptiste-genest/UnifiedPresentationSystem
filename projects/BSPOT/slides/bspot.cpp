#include "bspot.h"


polyscope::CurveNetwork* plotBijection(const pts2D &X, const pts2D &Y, const plan &T)
{
    pts2D P;
    P.insert(P.end(),X.begin(),X.end());
    P.insert(P.end(),Y.begin(),Y.end());

    std::vector<std::vector<size_t>> edges;
    for (size_t i=0;i<T.size();i++){
        edges.push_back({size_t(i),size_t(T[i]+X.size())});
    }
    auto* curve = polyscope::registerCurveNetwork("Bijection",P,edges);

}
