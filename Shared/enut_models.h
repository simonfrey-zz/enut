#ifndef ENUT_MODELS_H
#define ENUT_MODELS_H

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#define FL 0
#define HL 1
#define FR 2
#define HR 3

#define FOOT_LEN 0.064
#define LEG_LEN 0.055
#define SHOULDER_LEN 0.024

#define ENUT_MIN_HEIGHT 0.06
#define ENUT_MAX_HEIGHT 0.14

namespace enut {

enum Attitude {
    relax = 0,
    standing = 10,
};


struct Angles {
    double angles[13] = {M_PI_2,M_PI_2,M_PI_2,
                         M_PI_2,M_PI_2,M_PI_2,
                         M_PI_2,M_PI_2,M_PI_2,
                         M_PI_2,M_PI_2,M_PI_2,
                         M_PI_2};

    double & operator [] ( int index ){
        return angles[index];
    }

    //double s_fl, double s_fr, double s_hl, double s_hr,
    //double l_fl, double l_fr, double l_hl, double l_hr,
    //double f_fl, double f_fr, double f_hl, double f_hr,
    //double h
};

struct Foot {
    Eigen::Vector3d p{10,0,0};
    double angle=M_PI;
    double len = FOOT_LEN;
    void set_angles( double f ){
        angle = f;
        Eigen::AngleAxisd aa(angle, Eigen::Vector3d(0,0,1));
        p = aa._transformVector(Eigen::Vector3d(len,0,0)); // (1)
    }
};

struct Leg
{
    Eigen::Vector3d p{10,0,0};
    double angle=M_PI;
    double len = LEG_LEN;
    Foot foot;
    void set_angles( double l, double f ){
        foot.set_angles( f );
        angle = l;
        Eigen::AngleAxisd aa(M_PI - angle - M_PI_2, Eigen::Vector3d(0,0,1));
        p = aa._transformVector(Eigen::Vector3d(len,0,0) ); // (2)
        foot.p = aa._transformVector( foot.p ); // (3)

    }
};
struct Shoulder
{
    Eigen::Vector3d tr={0,0,0};
    Eigen::Vector3d p;
    double angle = M_PI;
    double len = SHOULDER_LEN;
    Leg leg;
    bool mirror=false;

    void set_angles( double s, double l, double f ){
        leg.set_angles( l, f );
        angle = s;
        const double mod_angle = mirror ? M_PI - angle : angle;
        Eigen::AngleAxisd aa( mod_angle, Eigen::Vector3d(0,1,0));
        p = aa._transformVector(Eigen::Vector3d(len,0,0) ) + tr; // (4)
        leg.p = aa._transformVector( leg.p ) + p; // (5) + (6)
        leg.foot.p = aa._transformVector( leg.foot.p ) + leg.p; // (7) + (8)
    }
};

struct Head {
    Eigen::Vector3d tr={0,0.1,0};
    Eigen::Quaterniond q{1,0,0,0};
    double angle;
    void set_angles( double head ){
        angle = head;
        Eigen::AngleAxisd aa(angle-M_PI, Eigen::Vector3d(1,0,0));
        q = aa;
    }
};

struct Body {
    Eigen::Vector3d tr={0,0,0};
    Eigen::Quaterniond q{1,0,0,0};

    std::map<int,Shoulder> shoulders;
    Head head;

    void set_angles( Angles & a ){
        shoulders[FL].set_angles( a.angles[0], a.angles[4], a.angles[8] );
        shoulders[FR].set_angles( a.angles[1], a.angles[5], a.angles[9] );
        shoulders[HL].set_angles( a.angles[2], a.angles[6], a.angles[10] );
        shoulders[HR].set_angles( a.angles[3], a.angles[7], a.angles[11] );
        head.set_angles( a.angles[12] );
    }
};


class Enut_Pate_model {
public:

    Enut_Pate_model( const Eigen::Vector3d foot_final ) : m_foot_final( foot_final ){

    }

    void set_foot_final( Eigen::Vector3d p ){
        m_foot_final = p;
    }

    // return residuals
    template< typename T >
    bool operator ()( const T * const shoulder,
                      const T * const leg,
                      const T * const foot,
                      T * residuals
                      ) const {

        // Foot angle
        T p_foot[3] = {T(FOOT_LEN), T(0), T(0)};
        T aa_foot[3]={T(0),T(0),*foot };
        T p_foot2[3];
        ceres::AngleAxisRotatePoint( aa_foot, p_foot, p_foot2 ); // (1)

        // leg angle
        T aa_leg[3]={T(0),T(0), T(M_PI) - *leg - T(M_PI_2) };
        T p_foot3[3];
        T p_leg[3] = {T(LEG_LEN), T(0), T(0)};
        T p_leg2[3];
        ceres::AngleAxisRotatePoint( aa_leg, p_foot2, p_foot3 );  // (3)
        ceres::AngleAxisRotatePoint( aa_leg, p_leg, p_leg2 ); // (2)

        // shoulder angle
        T aa_shoulder[3]={T(0),*shoulder, T(0) };
        T p_shoulder[3] = {T(SHOULDER_LEN), T(0), T(0)};
        T p_shoulder2[3];
        ceres::AngleAxisRotatePoint( aa_shoulder, p_shoulder, p_shoulder2 ); // (4)

        T p_leg3[3];
        T p_leg4[3];
        ceres::AngleAxisRotatePoint( aa_shoulder, p_leg2, p_leg3 ); // (5)

        p_leg4[0] = p_leg3[0] + p_shoulder2[0]; // (6)
        p_leg4[1] = p_leg3[1] + p_shoulder2[1]; // (6)
        p_leg4[2] = p_leg3[2] + p_shoulder2[2]; // (6)

        T p_foot4[3];
        ceres::AngleAxisRotatePoint( aa_shoulder, p_foot3, p_foot4 ); // (7)

        T p_foot_end[3];
        p_foot_end[0] = p_foot4[0] + p_leg4[0]; // (8)
        p_foot_end[1] = p_foot4[1] + p_leg4[1]; // (8)
        p_foot_end[2] = p_foot4[2] + p_leg4[2]; // (8)

        residuals[0] = p_foot_end[0] - T(m_foot_final[0]);
        residuals[1] = p_foot_end[1] - T(m_foot_final[1]);
        residuals[2] = p_foot_end[2] - T(m_foot_final[2]);

        return true;
    }

private:
    Eigen::Vector3d m_foot_final;
};


}

#endif // ENUT_MODELS_H
