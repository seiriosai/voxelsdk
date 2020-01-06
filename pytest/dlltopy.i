%module(directors="1") dlltopy
%{
#include "Dlltopy.h"
%}

%feature("director") dlltopy;
%include "Dlltopy.h"
