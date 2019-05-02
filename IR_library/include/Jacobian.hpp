#ifndef JACOBIAN
#define JACOBIAN

#include <vector>
#include "TypeDefs.hpp"

namespace IRlibrary {

      JacobianMat JacobianSpace(  std::vector <ScrewAxis>, std::vector <double> );
        
      JacobianMat JacobianBody(  std::vector <ScrewAxis>, std::vector <double> );  
        


} /* IRlibrary */
#endif /* ifndef JACOBIAN */
