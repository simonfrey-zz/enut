#include "Shared/enut_models.h"


std::string enut::Attitude_to_string(enut::Attitude a){
    if( a == relax )
        return "relax";
    else if( a == standing )
        return "standing";
    else
        return "relax";
}

enut::Attitude enut::Attitude_from_string(std::string a){
    if( a == "relax" )
        return relax;
    else if( a == "standing" )
        return standing;
    else
        return relax;
}