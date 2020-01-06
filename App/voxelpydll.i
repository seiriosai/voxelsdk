%module voxelpydll
%{
#include "voxelpydll.h"
#include <vector>
%}

%include stl.i
namespace std{
%template(VectorOfFloat) vector<float>;
 
}

%include "voxelpydll.h"
