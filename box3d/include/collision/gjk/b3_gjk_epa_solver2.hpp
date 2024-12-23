
#ifndef B3_GJK_EPA2_HPP
#define B3_GJK_EPA2_HPP

#include "geometry/b3_shape.hpp"
#include "dynamics/b3_transform.hpp"

struct b3GjkEpaSolver2
{
    struct sResults
    {
        enum eStatus
        {
            Separated,   /* Shapes doesnt penetrate												*/
            Penetrating, /* Shapes are penetrating												*/
            GJK_Failed,  /* GJK phase fail, no big issue, shapes are probably just 'touching'	*/
            EPA_Failed   /* EPA phase fail, bigger problem, need to save parameters, and debug	*/
        } status;
        b3Vec3r witnesses[2];
        b3Vec3r normal;
        real distance;
    };

    static int StackSizeRequirement();

    static bool Distance(const b3Shape* shape0, const b3Transformr& wtrs0,
                         const b3Shape* shape1, const b3Transformr& wtrs1,
                         const b3Vec3r& guess,
                         sResults& results);

    static bool Penetration(const b3Shape* shape0, const b3Transformr& wtrs0,
                            const b3Shape* shape1, const b3Transformr& wtrs1,
                            const b3Vec3r& guess,
                            sResults& results,
                            bool usemargins = true);
    
    static real SignedDistance(const b3Vec3r& position,
                                   real margin,
                                   const b3Shape* shape,
                                   const b3Transformr& wtrs,
                                   sResults& results);

    static bool SignedDistance(const b3Shape* shape0, const b3Transformr& wtrs0,
                               const b3Shape* shape1, const b3Transformr& wtrs1,
                               const b3Vec3r& guess,
                               sResults& results);

};

#endif  
