#include "stdio.h"
#include "math.h"
#include "stdlib.h"
#include "time.h"
#include <stdbool.h>
#include "trilateration.h"
#include "instance.h"



#if defined(TREK1000_TRILATERATION)
/* Largest nonnegative number still considered zero */
#define   MAXZERO  0.001

#define		ERR_TRIL_CONCENTRIC						-1
#define		ERR_TRIL_COLINEAR_2SOLUTIONS			-2
#define		ERR_TRIL_SQRTNEGNUMB					-3
#define		ERR_TRIL_NOINTERSECTION_SPHERE4			-4
#define		ERR_TRIL_NEEDMORESPHERE					-5

#define CM_ERR_ADDED (10) //was 5

/* Return the difference of two vectors, (vector1 - vector2). */
vec3d vdiff(const vec3d vector1, const vec3d vector2)
{
    vec3d v;
    v.x = vector1.x - vector2.x;
    v.y = vector1.y - vector2.y;
    v.z = vector1.z - vector2.z;
    return v;
}

/* Return the sum of two vectors. */
vec3d vsum(const vec3d vector1, const vec3d vector2)
{
    vec3d v;
    v.x = vector1.x + vector2.x;
    v.y = vector1.y + vector2.y;
    v.z = vector1.z + vector2.z;
    return v;
}

/* Multiply vector by a number. */
vec3d vmul(const vec3d vector, const double n)
{
    vec3d v;
    v.x = vector.x * n;
    v.y = vector.y * n;
    v.z = vector.z * n;
    return v;
}

/* Divide vector by a number. */
vec3d vdiv(const vec3d vector, const double n)
{
    vec3d v;
    v.x = vector.x / n;
    v.y = vector.y / n;
    v.z = vector.z / n;
    return v;
}

/* Return the Euclidean norm. */
double vdist(const vec3d v1, const vec3d v2)
{
    double xd = v1.x - v2.x;
    double yd = v1.y - v2.y;
    double zd = v1.z - v2.z;
    return sqrt(xd * xd + yd * yd + zd * zd);
}

/* Return the Euclidean norm. */
double vnorm(const vec3d vector)
{
    return sqrt(vector.x * vector.x + vector.y * vector.y + vector.z * vector.z);
}

/* Return the dot product of two vectors. */
double dot(const vec3d vector1, const vec3d vector2)
{
    return vector1.x * vector2.x + vector1.y * vector2.y + vector1.z * vector2.z;
}

/* Replace vector with its cross product with another vector. */
vec3d cross(const vec3d vector1, const vec3d vector2)
{
    vec3d v;
    v.x = vector1.y * vector2.z - vector1.z * vector2.y;
    v.y = vector1.z * vector2.x - vector1.x * vector2.z;
    v.z = vector1.x * vector2.y - vector1.y * vector2.x;
    return v;
}

/* Return the GDOP (Geometric Dilution of Precision) rate between 0-1.
 * Lower GDOP rate means better precision of intersection.
 */
double gdoprate(const vec3d tag, const vec3d p1, const vec3d p2, const vec3d p3)
{
    vec3d ex, t1, t2, t3;
    double h, gdop1, gdop2, gdop3, result;

    ex = vdiff(p1, tag);
    h = vnorm(ex);
    t1 = vdiv(ex, h);

    ex = vdiff(p2, tag);
    h = vnorm(ex);
    t2 = vdiv(ex, h);

    ex = vdiff(p3, tag);
    h = vnorm(ex);
    t3 = vdiv(ex, h);

    gdop1 = fabs(dot(t1, t2));
    gdop2 = fabs(dot(t2, t3));
    gdop3 = fabs(dot(t3, t1));

    if (gdop1 < gdop2) result = gdop2; else result = gdop1;
    if (result < gdop3) result = gdop3;

    return result;
}

/* Intersecting a sphere sc with radius of r, with a line p1-p2.
 * Return zero if successful, negative error otherwise.
 * mu1 & mu2 are constant to find points of intersection.
*/
int sphereline(const vec3d p1, const vec3d p2, const vec3d sc, double r, double *const mu1, double *const mu2)
{
   double a,b,c;
   double bb4ac;
   vec3d dp;

   dp.x = p2.x - p1.x;
   dp.y = p2.y - p1.y;
   dp.z = p2.z - p1.z;

   a = dp.x * dp.x + dp.y * dp.y + dp.z * dp.z;

   b = 2 * (dp.x * (p1.x - sc.x) + dp.y * (p1.y - sc.y) + dp.z * (p1.z - sc.z));

   c = sc.x * sc.x + sc.y * sc.y + sc.z * sc.z;
   c += p1.x * p1.x + p1.y * p1.y + p1.z * p1.z;
   c -= 2 * (sc.x * p1.x + sc.y * p1.y + sc.z * p1.z);
   c -= r * r;

   bb4ac = b * b - 4 * a * c;

   if (fabs(a) == 0 || bb4ac < 0) {
      *mu1 = 0;
      *mu2 = 0;
      return -1;
   }

   *mu1 = (-b + sqrt(bb4ac)) / (2 * a);
   *mu2 = (-b - sqrt(bb4ac)) / (2 * a);

   return 0;
}

/* Return TRIL_3SPHERES if it is performed using 3 spheres and return
 * TRIL_4SPHERES if it is performed using 4 spheres
 * For TRIL_3SPHERES, there are two solutions: result1 and result2
 * For TRIL_4SPHERES, there is only one solution: best_solution
 *
 * Return negative number for other errors
 *
 * To force the function to work with only 3 spheres, provide a duplicate of
 * any sphere at any place among p1, p2, p3 or p4.
 *
 * The last parameter is the largest nonnegative number considered zero;
 * it is somewhat analogous to machine epsilon (but inclusive).
*/
int trilateration(vec3d *const result1,
                  vec3d *const result2,
                  vec3d *const best_solution,
                  const vec3d p1, const double r1,
                  const vec3d p2, const double r2,
                  const vec3d p3, const double r3,
                  const vec3d p4, const double r4,
                  const double maxzero)
{
    vec3d	ex, ey, ez, t1, t2, t3;
    double	h, i, j, x, y, z, t;
    double	mu1, mu2, mu;
    int result;

    /*********** FINDING TWO POINTS FROM THE FIRST THREE SPHERES **********/

    // if there are at least 2 concentric spheres within the first 3 spheres
    // then the calculation may not continue, drop it with error -1

    /* h = |p3 - p1|, ex = (p3 - p1) / |p3 - p1| */
    ex = vdiff(p3, p1); // vector p13
    h = vnorm(ex); // scalar p13
    if (h <= maxzero) {
        /* p1 and p3 are concentric, not good to obtain a precise intersection point */
        //printf("concentric13 return -1\n");
        return ERR_TRIL_CONCENTRIC;
    }

    /* h = |p3 - p2|, ex = (p3 - p2) / |p3 - p2| */
    ex = vdiff(p3, p2); // vector p23
    h = vnorm(ex); // scalar p23
    if (h <= maxzero) {
        /* p2 and p3 are concentric, not good to obtain a precise intersection point */
        //printf("concentric23 return -1\n");
        return ERR_TRIL_CONCENTRIC;
    }

    /* h = |p2 - p1|, ex = (p2 - p1) / |p2 - p1| */
    ex = vdiff(p2, p1); // vector p12
    h = vnorm(ex); // scalar p12
    if (h <= maxzero) {
        /* p1 and p2 are concentric, not good to obtain a precise intersection point */
        //printf("concentric12 return -1\n");
        return ERR_TRIL_CONCENTRIC;
    }
    ex = vdiv(ex, h); // unit vector ex with respect to p1 (new coordinate system)

    /* t1 = p3 - p1, t2 = ex (ex . (p3 - p1)) */
    t1 = vdiff(p3, p1); // vector p13
    i = dot(ex, t1); // the scalar of t1 on the ex direction
    t2 = vmul(ex, i); // colinear vector to p13 with the length of i

    /* ey = (t1 - t2), t = |t1 - t2| */
    ey = vdiff(t1, t2); // vector t21 perpendicular to t1
    t = vnorm(ey); // scalar t21
    if (t > maxzero) {
        /* ey = (t1 - t2) / |t1 - t2| */
        ey = vdiv(ey, t); // unit vector ey with respect to p1 (new coordinate system)

        /* j = ey . (p3 - p1) */
        j = dot(ey, t1); // scalar t1 on the ey direction
    } else
        j = 0.0;

    /* Note: t <= maxzero implies j = 0.0. */
    if (fabs(j) <= maxzero) {

        /* Is point p1 + (r1 along the axis) the intersection? */
        t2 = vsum(p1, vmul(ex, r1));
        if (fabs(vnorm(vdiff(p2, t2)) - r2) <= maxzero &&
            fabs(vnorm(vdiff(p3, t2)) - r3) <= maxzero) {
            /* Yes, t2 is the only intersection point. */
            if (result1)
                *result1 = t2;
            if (result2)
                *result2 = t2;
            return TRIL_3SPHERES;
        }

        /* Is point p1 - (r1 along the axis) the intersection? */
        t2 = vsum(p1, vmul(ex, -r1));
        if (fabs(vnorm(vdiff(p2, t2)) - r2) <= maxzero &&
            fabs(vnorm(vdiff(p3, t2)) - r3) <= maxzero) {
            /* Yes, t2 is the only intersection point. */
            if (result1)
                *result1 = t2;
            if (result2)
                *result2 = t2;
            return TRIL_3SPHERES;
        }
        /* p1, p2 and p3 are colinear with more than one solution */
        return ERR_TRIL_COLINEAR_2SOLUTIONS;
    }

    /* ez = ex x ey */
    ez = cross(ex, ey); // unit vector ez with respect to p1 (new coordinate system)

    x = (r1*r1 - r2*r2) / (2*h) + h / 2;
    y = (r1*r1 - r3*r3 + i*i) / (2*j) + j / 2 - x * i / j;
    z = r1*r1 - x*x - y*y;
    if (z < -maxzero) {
        /* The solution is invalid, square root of negative number */
        return ERR_TRIL_SQRTNEGNUMB;
    } else
    if (z > 0.0)
        z = sqrt(z);
    else
        z = 0.0;

    /* t2 = p1 + x ex + y ey */
    t2 = vsum(p1, vmul(ex, x));
    t2 = vsum(t2, vmul(ey, y));

    /* result1 = p1 + x ex + y ey + z ez */
    if (result1)
        *result1 = vsum(t2, vmul(ez, z));

    /* result1 = p1 + x ex + y ey - z ez */
    if (result2)
        *result2 = vsum(t2, vmul(ez, -z));

    /*********** END OF FINDING TWO POINTS FROM THE FIRST THREE SPHERES **********/
    /********* RESULT1 AND RESULT2 ARE SOLUTIONS, OTHERWISE RETURN ERROR *********/


    /************* FINDING ONE SOLUTION BY INTRODUCING ONE MORE SPHERE ***********/

    // check for concentricness of sphere 4 to sphere 1, 2 and 3
    // if it is concentric to one of them, then sphere 4 cannot be used
    // to determine the best solution and return -1

    /* h = |p4 - p1|, ex = (p4 - p1) / |p4 - p1| */
    ex = vdiff(p4, p1); // vector p14
    h = vnorm(ex); // scalar p14
    if (h <= maxzero) {
        /* p1 and p4 are concentric, not good to obtain a precise intersection point */
        //printf("concentric14 return 0\n");
        return TRIL_3SPHERES;
    }
    /* h = |p4 - p2|, ex = (p4 - p2) / |p4 - p2| */
    ex = vdiff(p4, p2); // vector p24
    h = vnorm(ex); // scalar p24
    if (h <= maxzero) {
        /* p2 and p4 are concentric, not good to obtain a precise intersection point */
        //printf("concentric24 return 0\n");
        return TRIL_3SPHERES;
    }
    /* h = |p4 - p3|, ex = (p4 - p3) / |p4 - p3| */
    ex = vdiff(p4, p3); // vector p34
    h = vnorm(ex); // scalar p34
    if (h <= maxzero) {
        /* p3 and p4 are concentric, not good to obtain a precise intersection point */
        //printf("concentric34 return 0\n");
        return TRIL_3SPHERES;
    }

    // if sphere 4 is not concentric to any sphere, then best solution can be obtained
    /* find i as the distance of result1 to p4 */
    t3 = vdiff(*result1, p4);
    i = vnorm(t3);
    /* find h as the distance of result2 to p4 */
    t3 = vdiff(*result2, p4);
    h = vnorm(t3);

    /* pick the result1 as the nearest point to the center of sphere 4 */
    if (i > h) {
        *best_solution = *result1;
        *result1 = *result2;
        *result2 = *best_solution;
    }

    int count4 = 0;
    double rr4 = r4;
    result = 1;
    /* intersect result1-result2 vector with sphere 4 */
    while(result && count4 < 10)
    {
        result=sphereline(*result1, *result2, p4, rr4, &mu1, &mu2);
        rr4+=0.1;
        count4++;
    }

    if (result) {

        /* No intersection between sphere 4 and the line with the gradient of result1-result2! */
        *best_solution = *result1; // result1 is the closer solution to sphere 4
        //return ERR_TRIL_NOINTERSECTION_SPHERE4;

    } else {

        if (mu1 < 0 && mu2 < 0) {

            /* if both mu1 and mu2 are less than 0 */
            /* result1-result2 line segment is outside sphere 4 with no intersection */
            if (fabs(mu1) <= fabs(mu2)) mu = mu1; else mu = mu2;
            /* h = |result2 - result1|, ex = (result2 - result1) / |result2 - result1| */
            ex = vdiff(*result2, *result1); // vector result1-result2
            h = vnorm(ex); // scalar result1-result2
            ex = vdiv(ex, h); // unit vector ex with respect to result1 (new coordinate system)
            /* 50-50 error correction for mu */
            mu = 0.5*mu;
            /* t2 points to the intersection */
            t2 = vmul(ex, mu*h);
            t2 = vsum(*result1, t2);
            /* the best solution = t2 */
            *best_solution = t2;

        } else if ((mu1 < 0 && mu2 > 1) || (mu2 < 0 && mu1 > 1)) {

            /* if mu1 is less than zero and mu2 is greater than 1, or the other way around */
            /* result1-result2 line segment is inside sphere 4 with no intersection */
            if (mu1 > mu2) mu = mu1; else mu = mu2;
            /* h = |result2 - result1|, ex = (result2 - result1) / |result2 - result1| */
            ex = vdiff(*result2, *result1); // vector result1-result2
            h = vnorm(ex); // scalar result1-result2
            ex = vdiv(ex, h); // unit vector ex with respect to result1 (new coordinate system)
            /* t2 points to the intersection */
            t2 = vmul(ex, mu*h);
            t2 = vsum(*result1, t2);
            /* vector t2-result2 with 50-50 error correction on the length of t3 */
            t3 = vmul(vdiff(*result2, t2),0.5);
            /* the best solution = t2 + t3 */
            *best_solution = vsum(t2, t3);

        } else if (((mu1 > 0 && mu1 < 1) && (mu2 < 0 || mu2 > 1))
                || ((mu2 > 0 && mu2 < 1) && (mu1 < 0 || mu1 > 1))) {

            /* if one mu is between 0 to 1 and the other is not */
            /* result1-result2 line segment intersects sphere 4 at one point */
            if (mu1 >= 0 && mu1 <= 1) mu = mu1; else mu = mu2;
            /* add or subtract with 0.5*mu to distribute error equally onto every sphere */
            if (mu <= 0.5) mu-=0.5*mu; else mu-=0.5*(1-mu);
            /* h = |result2 - result1|, ex = (result2 - result1) / |result2 - result1| */
            ex = vdiff(*result2, *result1); // vector result1-result2
            h = vnorm(ex); // scalar result1-result2
            ex = vdiv(ex, h); // unit vector ex with respect to result1 (new coordinate system)
            /* t2 points to the intersection */
            t2 = vmul(ex, mu*h);
            t2 = vsum(*result1, t2);
            /* the best solution = t2 */
            *best_solution = t2;

        } else if (mu1 == mu2) {

            /* if both mu1 and mu2 are between 0 and 1, and mu1 = mu2 */
            /* result1-result2 line segment is tangential to sphere 4 at one point */
            mu = mu1;
            /* add or subtract with 0.5*mu to distribute error equally onto every sphere */
            if (mu <= 0.25) mu-=0.5*mu;
            else if (mu <=0.5) mu-=0.5*(0.5-mu);
            else if (mu <=0.75) mu-=0.5*(mu-0.5);
            else mu-=0.5*(1-mu);
            /* h = |result2 - result1|, ex = (result2 - result1) / |result2 - result1| */
            ex = vdiff(*result2, *result1); // vector result1-result2
            h = vnorm(ex); // scalar result1-result2
            ex = vdiv(ex, h); // unit vector ex with respect to result1 (new coordinate system)
            /* t2 points to the intersection */
            t2 = vmul(ex, mu*h);
            t2 = vsum(*result1, t2);
            /* the best solution = t2 */
            *best_solution = t2;

        } else {

            /* if both mu1 and mu2 are between 0 and 1 */
            /* result1-result2 line segment intersects sphere 4 at two points */

            //return ERR_TRIL_NEEDMORESPHERE;

            mu = mu1 + mu2;
            /* h = |result2 - result1|, ex = (result2 - result1) / |result2 - result1| */
            ex = vdiff(*result2, *result1); // vector result1-result2
            h = vnorm(ex); // scalar result1-result2
            ex = vdiv(ex, h); // unit vector ex with respect to result1 (new coordinate system)
            /* 50-50 error correction for mu */
            mu = 0.5*mu;
            /* t2 points to the intersection */
            t2 = vmul(ex, mu*h);
            t2 = vsum(*result1, t2);
            /* the best solution = t2 */
            *best_solution = t2;

        }

    }

    return TRIL_4SPHERES;

    /******** END OF FINDING ONE SOLUTION BY INTRODUCING ONE MORE SPHERE *********/
}


/* This function calls trilateration to get the best solution.
 *
 * If any three spheres does not produce valid solution,
 * then each distance is increased to ensure intersection to happens.
 *
 * Return the selected trilateration mode between TRIL_3SPHERES or TRIL_4SPHERES
 * For TRIL_3SPHERES, there are two solutions: solution1 and solution2
 * For TRIL_4SPHERES, there is only one solution: best_solution
 *
 * nosolution_count = the number of failed attempt before intersection is found
 * by increasing the sphere diameter.
*/
int deca_3dlocate (	vec3d	*const solution1,
                    vec3d	*const solution2,
                    vec3d	*const best_solution,
                    int		*const nosolution_count,
                    double	*const best_3derror,
                    double	*const best_gdoprate,
                    vec3d p1, double r1,
                    vec3d p2, double r2,
                    vec3d p3, double r3,
                    vec3d p4, double r4,
                    int *combination)
{
    vec3d	o1, o2, solution, ptemp;
    //vec3d	solution_compare1, solution_compare2;
    double	/*error_3dcompare1, error_3dcompare2,*/ rtemp;
    double	gdoprate_compare1, gdoprate_compare2;
    double	ovr_r1, ovr_r2, ovr_r3, ovr_r4;
    int		overlook_count, combination_counter;
    int		trilateration_errcounter, trilateration_mode34;
    int		success, concentric, result;

    trilateration_errcounter = 0;
    trilateration_mode34 = 0;

    combination_counter = 1; /* four spheres combination */

    *best_gdoprate = 1; /* put the worst gdoprate init */
    gdoprate_compare1 = 1; 
    gdoprate_compare2 = 1;
    //solution_compare1.x = 0; solution_compare1.y = 0; solution_compare1.z = 0;
    //error_3dcompare1 = 0;

    do {
        success = 0;
        concentric = 0;
        overlook_count = 0;
        ovr_r1 = r1; ovr_r2 = r2; ovr_r3 = r3; ovr_r4 = r4;

        do {

            result = trilateration(&o1, &o2, &solution, p1, ovr_r1, p2, ovr_r2, p3, ovr_r3, p4, ovr_r4, MAXZERO);

            switch (result)
            {
                case TRIL_3SPHERES: // 3 spheres are used to get the result
                    trilateration_mode34 = TRIL_3SPHERES;
                    success = 1;
                    break;

                case TRIL_4SPHERES: // 4 spheres are used to get the result
                    trilateration_mode34 = TRIL_4SPHERES;
                    success = 1;
                    break;

                case ERR_TRIL_CONCENTRIC:
                    concentric = 1;
                    break;

                default: // any other return value goes here
                    ovr_r1 += 0.10;
                    ovr_r2 += 0.10;
                    ovr_r3 += 0.10;
                    ovr_r4 += 0.10;
                    overlook_count++;
                    break;
            }


        } while (!success && (overlook_count <= CM_ERR_ADDED) && !concentric);

        if (success)
        {
            switch (result)
            {
            case TRIL_3SPHERES:
                *solution1 = o1;
                *solution2 = o2;
                *nosolution_count = overlook_count;

                combination_counter = 0;
                break;

            case TRIL_4SPHERES:

                /* calculate the new gdop */
                gdoprate_compare1	= gdoprate(solution, p1, p2, p3);

                /* compare and swap with the better result */
                if (gdoprate_compare1 <= gdoprate_compare2)
                {

                    *solution1 = o1;
                    *solution2 = o2;
                    *best_solution	= solution;
                    *nosolution_count = overlook_count;
                    *best_3derror	= sqrt((vnorm(vdiff(solution, p1))-r1)*(vnorm(vdiff(solution, p1))-r1) +
                                        (vnorm(vdiff(solution, p2))-r2)*(vnorm(vdiff(solution, p2))-r2) +
                                        (vnorm(vdiff(solution, p3))-r3)*(vnorm(vdiff(solution, p3))-r3) +
                                        (vnorm(vdiff(solution, p4))-r4)*(vnorm(vdiff(solution, p4))-r4));
                    *best_gdoprate	= gdoprate_compare1;

                    /* save the previous result */
                    //solution_compare2 = solution_compare1;
                    //error_3dcompare2 = error_3dcompare1;
                    gdoprate_compare2 = gdoprate_compare1;

                    *combination = 5 - combination_counter;

//                    ptemp = p1; p1 = p2; p2 = p3; p3 = p4; p4 = ptemp;
//                    rtemp = r1; r1 = r2; r2 = r3; r3 = r4; r4 = rtemp;
//                    combination_counter--;

                }

                ptemp = p1; p1 = p2; p2 = p3; p3 = p4; p4 = ptemp;
                rtemp = r1; r1 = r2; r2 = r3; r3 = r4; r4 = rtemp;
                combination_counter--;
                if(combination_counter<0)
                {
                    combination_counter=0;
                }
                break;

            default:
                break;
            }
        }
        else
        {
            trilateration_errcounter++;
            //trilateration_errcounter = 4;
            //combination_counter = 0;
            combination_counter--;
            if(combination_counter<0)
            {
               combination_counter=0;
            }
        }


    } while (combination_counter);

    // if it gives error for all 4 sphere combinations then no valid result is given
    // otherwise return the trilateration mode used
    if (trilateration_errcounter >= 4)
        return -1;
    else
        return trilateration_mode34;

}
struct num
{
    int anc_ID;
    int distance;
}valid_anc_num[(MAX_AHCHOR_NUMBER+1)]; 

int cmp(const void *m,const void *n) //定义返回值返回方式
{
    return ((struct num*)m) ->distance - ((struct num*)n) ->distance;
}


int GetLocation(vec3d *best_solution, vec3d* anchorArray, int *distanceArray)
{

    vec3d	o1, o2, p1, p2, p3, p4;
    double	r1 = 0, r2 = 0, r3 = 0, r4 = 0, best_3derror, best_gdoprate;
    int		result;
    int     error, combination;
    int     valid_anc_count=0;
    int     j=0;
    int     use3anc=0;

    for(int i=0;i<(MAX_AHCHOR_NUMBER+1);i++)//清空结构体数组
    {
        valid_anc_num[i].anc_ID=0;
        valid_anc_num[i].distance=0;
    }

    for(int i=0;i<MAX_AHCHOR_NUMBER;i++)//验证几个有效距离值
    {
        if(distanceArray[i]>0)
        {
            valid_anc_count++;
            valid_anc_num[j].anc_ID=i;//记录有效基站编号
            valid_anc_num[j].distance=distanceArray[i];//记录有效基站距离
            j++;
        }
    }

    if(valid_anc_count<3)
    {
        return -1;
        //puts("err1");
    }

    else if(valid_anc_count==3)//直接执行三基站定位
    {
        use3anc=1;
        /* Anchors coordinate */
        p1.x = anchorArray[valid_anc_num[0].anc_ID].x;		p1.y = anchorArray[valid_anc_num[0].anc_ID].y;	    p1.z = anchorArray[valid_anc_num[0].anc_ID].z;
        p2.x = anchorArray[valid_anc_num[1].anc_ID].x;		p2.y = anchorArray[valid_anc_num[1].anc_ID].y;	    p2.z = anchorArray[valid_anc_num[1].anc_ID].z;
        p3.x = anchorArray[valid_anc_num[2].anc_ID].x;		p3.y = anchorArray[valid_anc_num[2].anc_ID].y;	    p3.z = anchorArray[valid_anc_num[2].anc_ID].z;
        p4.x = p1.x;		                                p4.y = p1.y;	                                    p4.z = p1.z;

        r1 = (double) distanceArray[valid_anc_num[0].anc_ID] / 1000.0;
        r2 = (double) distanceArray[valid_anc_num[1].anc_ID] / 1000.0;
        r3 = (double) distanceArray[valid_anc_num[2].anc_ID] / 1000.0;
        r4 = r1;

        // printf("anc1=%d,anc2=%d,anc3=%d\n",valid_anc_num[0],valid_anc_num[1],valid_anc_num[2]);
        // printf("dis1=%f,dis2=%f,dis3=%f\n",r1,r2,r3);
        // printf("P1X=%f,P1Y=%f,P1Z=%f\n",p1.x,p1.y,p1.z);
    }

    else if(valid_anc_count==4)//直接执行4基站定位
    {
        /* Anchors coordinate */
        p1.x = anchorArray[valid_anc_num[0].anc_ID].x;		p1.y = anchorArray[valid_anc_num[0].anc_ID].y;	    p1.z = anchorArray[valid_anc_num[0].anc_ID].z;
        p2.x = anchorArray[valid_anc_num[1].anc_ID].x;		p2.y = anchorArray[valid_anc_num[1].anc_ID].y;	    p2.z = anchorArray[valid_anc_num[1].anc_ID].z;
        p3.x = anchorArray[valid_anc_num[2].anc_ID].x;		p3.y = anchorArray[valid_anc_num[2].anc_ID].y;	    p3.z = anchorArray[valid_anc_num[2].anc_ID].z;
        p4.x = anchorArray[valid_anc_num[3].anc_ID].x;		p4.y = anchorArray[valid_anc_num[3].anc_ID].y;	    p4.z = anchorArray[valid_anc_num[3].anc_ID].z;


        r1 = (double) distanceArray[valid_anc_num[0].anc_ID] / 1000.0;
        r2 = (double) distanceArray[valid_anc_num[1].anc_ID] / 1000.0;
        r3 = (double) distanceArray[valid_anc_num[2].anc_ID] / 1000.0;
        r4 = (double) distanceArray[valid_anc_num[3].anc_ID] / 1000.0;

    }

    //valid_anc_count 有效基站个数
    //valid_anc_num[0] 有效基站编号
    else if(valid_anc_count>4)//执行基站选取机制，选取最近的4个基站1234进行计算
    {
         qsort(valid_anc_num, (valid_anc_count+1), sizeof(valid_anc_num[0]), cmp);  //将有效距离值进行从小到大排序
        //  for(int i=1;i<=valid_anc_count;i++) //输出结果
        //     printf("No%d DIS=%d,ID=A%d\n",i,valid_anc_num[i].distance,valid_anc_num[i].anc_ID);
        
        p1.x = anchorArray[valid_anc_num[1].anc_ID].x;		p1.y = anchorArray[valid_anc_num[1].anc_ID].y;	    p1.z = anchorArray[valid_anc_num[1].anc_ID].z;
        p2.x = anchorArray[valid_anc_num[2].anc_ID].x;		p2.y = anchorArray[valid_anc_num[2].anc_ID].y;	    p2.z = anchorArray[valid_anc_num[2].anc_ID].z;
        p3.x = anchorArray[valid_anc_num[3].anc_ID].x;		p3.y = anchorArray[valid_anc_num[3].anc_ID].y;	    p3.z = anchorArray[valid_anc_num[3].anc_ID].z;
        p4.x = anchorArray[valid_anc_num[4].anc_ID].x;		p4.y = anchorArray[valid_anc_num[4].anc_ID].y;	    p4.z = anchorArray[valid_anc_num[4].anc_ID].z;

        r1 = (double) distanceArray[valid_anc_num[1].anc_ID] / 1000.0;
        r2 = (double) distanceArray[valid_anc_num[2].anc_ID] / 1000.0;
        r3 = (double) distanceArray[valid_anc_num[3].anc_ID] / 1000.0;
        r4 = (double) distanceArray[valid_anc_num[4].anc_ID] / 1000.0;

        //printf("use1=A%d,use2=A%d,use3=A%d,use4=A%d\n",valid_anc_num[1].anc_ID,valid_anc_num[2].anc_ID,valid_anc_num[3].anc_ID,valid_anc_num[4].anc_ID);
    }
    

    result = deca_3dlocate (&o1, &o2, best_solution, &error, &best_3derror, &best_gdoprate,
                            p1, r1, p2, r2, p3, r3, p4, r4, &combination);

    
    if((result==0)&&(valid_anc_count>4))//多于4基站选取后计算失败，把第1舍掉用第2345计算
    {
        //puts("Second calculation");
        p1.x = anchorArray[valid_anc_num[2].anc_ID].x;		p1.y = anchorArray[valid_anc_num[2].anc_ID].y;	    p1.z = anchorArray[valid_anc_num[2].anc_ID].z;
        p2.x = anchorArray[valid_anc_num[3].anc_ID].x;		p2.y = anchorArray[valid_anc_num[3].anc_ID].y;	    p2.z = anchorArray[valid_anc_num[3].anc_ID].z;
        p3.x = anchorArray[valid_anc_num[4].anc_ID].x;		p3.y = anchorArray[valid_anc_num[4].anc_ID].y;	    p3.z = anchorArray[valid_anc_num[4].anc_ID].z;
        p4.x = anchorArray[valid_anc_num[5].anc_ID].x;		p4.y = anchorArray[valid_anc_num[5].anc_ID].y;	    p4.z = anchorArray[valid_anc_num[5].anc_ID].z;

        r1 = (double) distanceArray[valid_anc_num[2].anc_ID] / 1000.0;
        r2 = (double) distanceArray[valid_anc_num[3].anc_ID] / 1000.0;
        r3 = (double) distanceArray[valid_anc_num[4].anc_ID] / 1000.0;
        r4 = (double) distanceArray[valid_anc_num[5].anc_ID] / 1000.0;

        //printf("use1=A%d,use2=A%d,use3=A%d,use4=A%d\n",valid_anc_num[2].anc_ID,valid_anc_num[3].anc_ID,valid_anc_num[4].anc_ID,valid_anc_num[5].anc_ID);

        result = deca_3dlocate (&o1, &o2, best_solution, &error, &best_3derror, &best_gdoprate,
                        p1, r1, p2, r2, p3, r3, p4, r4, &combination);
    }


    if (result >= 0)
    {
        if(o1.z <= o2.z) best_solution->z = o1.z; else best_solution->z = o2.z;
        if (use3anc == 1 || result == TRIL_3SPHERES)
        {
            if(o1.z < p1.z) *best_solution = o1; else *best_solution = o2; //assume tag is below the anchors (1, 2, and 3)
        }

        return result;
    }
    return -1;
}

#elif defined(SR150_3D)


//UWB_POS_ERROR_CODES localization()
int GetLocation(vec3d *best_solution, vec3d* anchorArray, int *distanceArray)
{
	/*!@brief: This function calculates the 3D position of the initiator from the anchor distances and positions using least squared errors.
	 *	 	   The function expects more than 4 anchors. The used equation system looks like follows:\n
	 \verbatim
	  		    -					-
	  		   | M_11	M_12	M_13 |	 x	  b[0]
	  		   | M_12	M_22	M_23 | * y	= b[1]
	  		   | M_23	M_13	M_33 |	 z	  b[2]
	  		    -					-
	 \endverbatim
	 * @param distances_cm_in_pt: 			Pointer to array that contains the distances to the anchors in cm (including invalid results)
	 * @param no_distances: 				Number of valid distances in distance array (it's not the size of the array)
	 * @param anchor_pos: 	                Pointer to array that contains anchor positions in cm (including positions related to invalid results)
	 * @param no_anc_positions: 			Number of valid anchor positions in the position array (it's not the size of the array)
	 * @param position_result_pt: 			Pointer toposition. position_t variable that holds the result of this calculation
	 * @return: The function returns a status code. */

	/* 		Algorithm used:
	 *		Linear Least Sqaures to solve Multilateration
	 * 		with a Special case if there are only 3 Anchors.
	 * 		Output is the Coordinates of the Initiator in relation to Anchor 0 in NEU (North-East-Up) Framing
	 * 		In cm
	 */

	/* Resulting Position Vector*/
	double x_pos = 0;
	double y_pos = 0;
	double z_pos = 0;
	/* Matrix components (3*3 Matrix resulting from least square error method) [cm^2] */
	double M_11 = 0;
	double M_12 = 0;																						// = M_21
	double M_13 = 0;																						// = M_31
	double M_22 = 0;
	double M_23 = 0;																						// = M_23
	double M_33 = 0;

	/* Vector components (3*1 Vector resulting from least square error method) [cm^3] */
	double b[3] = {0};

	/* Miscellaneous variables */
	double temp = 0;
	double temp2 = 0;
	double nominator = 0;
	double denominator = 0;
	bool   anchors_on_x_y_plane = true;																		// Is true, if all anchors are on the same height => x-y-plane
	bool   lin_dep = true;																						// All vectors are linear dependent, if this variable is true
	uint8_t ind_y_indi = 0;	//numberr of independet vectors																					// First anchor index, for which the second row entry of the matrix [(x_1 - x_0) (x_2 - x_0) ... ; (y_1 - x_0) (y_2 - x_0) ...] is non-zero => linear independent

	/* Arrays for used distances and anchor positions (without rejected ones) */
	uint8_t 	no_distances = MAX_AHCHOR_NUMBER;
	int 	distances_cm[no_distances];
	position_t 	anchor_pos[no_distances]; //position in CM
	uint8_t		no_valid_distances = 0;

	/* Reject invalid distances (including related anchor position) */
	for (int i = 0; i < no_distances; i++) 
    {
		if (distanceArray[i] > 0) 
        {
			//excludes any distance that is 0xFFFFU (int16 Maximum Value)
			distances_cm[no_valid_distances] = distanceArray[i]/10;
			anchor_pos[no_valid_distances].x = anchorArray[i].x*100;
            anchor_pos[no_valid_distances].y = anchorArray[i].y*100;
            anchor_pos[no_valid_distances].z = anchorArray[i].z*100;
            no_valid_distances++;
		}
		// if (_dis[i] != -1) 
        // {
		// 	//excludes any distance that is 0xFFFFU (int16 Maximum Value)
		// 	distances_cm[no_valid_distances] = _dis[i];
		// 	anchor_pos[no_valid_distances] = _ancarray[i];
        //     no_valid_distances++;
		// }
        else
        {
            //printf("%d = -1\n", i);
        }
	}

    //printf("no_valid_distances = %d\n", no_valid_distances);

	/* Check, if there are enough valid results for doing the localization at all */
	if (no_valid_distances < 3) {
		return UWB_ANC_BELOW_THREE;
	}

	/* Check, if anchors are on the same x-y plane */
	for (int i = 1; i < no_valid_distances; i++) 
    {
		if (anchor_pos[i].z != anchor_pos[0].z) 
        {
			anchors_on_x_y_plane = false;
			break;
		}
	}

	/**** Check, if there are enough linear independent anchor positions ****/

	/* Check, if the matrix |(x_1 - x_0) (x_2 - x_0) ... | has rank 2
	 * 			|(y_1 - y_0) (y_2 - y_0) ... | 				*/

	for (ind_y_indi = 2; ((ind_y_indi < no_valid_distances) && (lin_dep == true)); ind_y_indi++) {
		temp = ((int64_t)anchor_pos[ind_y_indi].y - (int64_t)anchor_pos[0].y) * ((int64_t)anchor_pos[1].x -
				(int64_t)anchor_pos[0].x);
		temp2 = ((int64_t)anchor_pos[1].y - (int64_t)anchor_pos[0].y) * ((int64_t)anchor_pos[ind_y_indi].x -
				(int64_t)anchor_pos[0].x);

		if ((temp - temp2) != 0) {
			lin_dep = false;
			break;
		}
	}

	/* Leave function, if rank is below 2 */
	if (lin_dep == true) 
    {
		return UWB_LIN_DEP_FOR_THREE;
	}

	/* If the anchors are not on the same plane, three vectors must be independent => check */
	if (!anchors_on_x_y_plane) 
    {
		/* Check, if there are enough valid results for doing the localization */
		if (no_valid_distances < 4) 
        {
			return UWB_ANC_ON_ONE_LEVEL;
		}

		/* Check, if the matrix |(x_1 - x_0) (x_2 - x_0) (x_3 - x_0) ... | has rank 3 (Rank y, y already checked)
		 * 			|(y_1 - y_0) (y_2 - y_0) (y_3 - y_0) ... |
		 * 			|(z_1 - z_0) (z_2 - z_0) (z_3 - z_0) ... |											*/
		lin_dep = true;

		for (int i = 2; ((i < no_valid_distances) && (lin_dep == true)); i++) {
			if (i != ind_y_indi) {
				/* (x_1 - x_0)*[(y_2 - y_0)(z_n - z_0) - (y_n - y_0)(z_2 - z_0)] */
				temp 	= ((int64_t)anchor_pos[ind_y_indi].y - (int64_t)anchor_pos[0].y) * ((int64_t)anchor_pos[i].z -
						(int64_t)anchor_pos[0].z);
				temp 	-= ((int64_t)anchor_pos[i].y - (int64_t)anchor_pos[0].y) * ((int64_t)anchor_pos[ind_y_indi].z -
						(int64_t)anchor_pos[0].z);
				temp2 	= ((int64_t)anchor_pos[1].x - (int64_t)anchor_pos[0].x) * temp;

				/* Add (x_2 - x_0)*[(y_n - y_0)(z_1 - z_0) - (y_1 - y_0)(z_n - z_0)] */
				temp 	= ((int64_t)anchor_pos[i].y - (int64_t)anchor_pos[0].y) * ((int64_t)anchor_pos[1].z - (int64_t)anchor_pos[0].z);
				temp 	-= ((int64_t)anchor_pos[1].y - (int64_t)anchor_pos[0].y) * ((int64_t)anchor_pos[i].z - (int64_t)anchor_pos[0].z);
				temp2 	+= ((int64_t)anchor_pos[ind_y_indi].x - (int64_t)anchor_pos[0].x) * temp;

				/* Add (x_n - x_0)*[(y_1 - y_0)(z_2 - z_0) - (y_2 - y_0)(z_1 - z_0)] */
				temp 	= ((int64_t)anchor_pos[1].y - (int64_t)anchor_pos[0].y) * ((int64_t)anchor_pos[ind_y_indi].z -
						(int64_t)anchor_pos[0].z);
				temp 	-= ((int64_t)anchor_pos[ind_y_indi].y - (int64_t)anchor_pos[0].y) * ((int64_t)anchor_pos[1].z -
						(int64_t)anchor_pos[0].z);
				temp2 	+= ((int64_t)anchor_pos[i].x - (int64_t)anchor_pos[0].x) * temp;

				if (temp2 != 0) { lin_dep = false; }
			}
		}

		/* Leave function, if rank is below 3 */
		if (lin_dep == true) {
			return UWB_LIN_DEP_FOR_FOUR;
		}
	}

	/************************************************** Algorithm ***********************************************************************/

	/* Writing values resulting from least square error method (A_trans*A*x = A_trans*r; row 0 was used to remove x^2,y^2,z^2 entries => index starts at 1) */
	for (int i = 1; i < no_valid_distances; i++) {
		/* Matrix (needed to be multiplied with 2, afterwards) */
		M_11 += (int64_t)pow((int64_t)(anchor_pos[i].x - anchor_pos[0].x), 2);
		M_12 += (int64_t)((int64_t)(anchor_pos[i].x - anchor_pos[0].x) * (int64_t)(anchor_pos[i].y - anchor_pos[0].y));
		M_13 += (int64_t)((int64_t)(anchor_pos[i].x - anchor_pos[0].x) * (int64_t)(anchor_pos[i].z - anchor_pos[0].z));
		M_22 += (int64_t)pow((int64_t)(anchor_pos[i].y - anchor_pos[0].y), 2);
		M_23 += (int64_t)((int64_t)(anchor_pos[i].y - anchor_pos[0].y) * (int64_t)(anchor_pos[i].z - anchor_pos[0].z));
		M_33 += (int64_t)pow((int64_t)(anchor_pos[i].z - anchor_pos[0].z), 2);

		/* Vector */
		temp = (int64_t)((int64_t)pow(distances_cm[0], 2) - (int64_t)pow(distances_cm[i], 2)
				 + (int64_t)pow(anchor_pos[i].x, 2) + (int64_t)pow(anchor_pos[i].y, 2)
				 + (int64_t)pow(anchor_pos[i].z, 2) - (int64_t)pow(anchor_pos[0].x, 2)
				 - (int64_t)pow(anchor_pos[0].y, 2) - (int64_t)pow(anchor_pos[0].z, 2));

		b[0] += (int64_t)((int64_t)(anchor_pos[i].x - anchor_pos[0].x) * temp);
		b[1] += (int64_t)((int64_t)(anchor_pos[i].y - anchor_pos[0].y) * temp);
		b[2] += (int64_t)((int64_t)(anchor_pos[i].z - anchor_pos[0].z) * temp);
	}

	M_11 = 2 * M_11;
	M_12 = 2 * M_12;
	M_13 = 2 * M_13;
	M_22 = 2 * M_22;
	M_23 = 2 * M_23;
	M_33 = 2 * M_33;

	/* Calculating the z-position, if calculation is possible (at least one anchor at z != 0) */
	if (anchors_on_x_y_plane == false) {
		nominator = b[0] * (M_12 * M_23 - M_13 * M_22) + b[1] * (M_12 * M_13 - M_11 * M_23) + b[2] *
			    (M_11 * M_22 - M_12 * M_12);			// [cm^7]
		denominator = M_11 * (M_33 * M_22 - M_23 * M_23) + 2 * M_12 * M_13 * M_23 - M_33 * M_12 * M_12 - M_22 * M_13 *
			      M_13;				// [cm^6]

		/* Check, if denominator is zero (Rank of matrix not high enough) */
		if (denominator == 0) {
			return UWB_RANK_ZERO;
		}

		z_pos = ((nominator * 10) / denominator + 5) / 10;	// [cm]
	}

	/* Else prepare for different calculation approach (after x and y were calculated) */
	else {
		z_pos = 0;
	}

	/* Calculating the y-position */
	nominator = b[1] * M_11 - b[0] * M_12 - (z_pos * (M_11 * M_23 - M_12 * M_13));	// [cm^5]
	denominator = M_11 * M_22 - M_12 * M_12;// [cm^4]

	/* Check, if denominator is zero (Rank of matrix not high enough) */
	if (denominator == 0) {
		return UWB_RANK_ZERO;
	}

	y_pos = ((nominator * 10) / denominator + 5) / 10;	// [cm]

	/* Calculating the x-position */
	nominator = b[0] - z_pos * M_13 - y_pos * M_12;	// [cm^3]
	denominator = M_11;	// [cm^2]

	x_pos = ((nominator * 10) / denominator + 5) / 10;// [cm]

	/* Calculate z-position form x and y coordinates, if z can't be determined by previous steps (All anchors at z_n = 0) */
	if (anchors_on_x_y_plane == true) 
    {
		/* Calculate z-positon relative to the anchor grid's height */
		//for (int i = 0; i < no_distances; i++) {
        for (int i = 0; i < no_valid_distances; i++) {
			/* z² = dis_meas_n² - (x - x_anc_n)² - (y - y_anc_n)² */
			temp = (int64_t)((int64_t)pow(distances_cm[i], 2)
					 - (int64_t)pow((x_pos - (int64_t)anchor_pos[i].x), 2)
					 - (int64_t)pow((y_pos - (int64_t)anchor_pos[i].y), 2));

			/* z² must be positive, else x and y must be wrong => calculate positive sqrt and sum up all calculated heights, if positive */
			if (temp >= 0) {
				z_pos += (int64_t)sqrt(temp);

			} else {
				z_pos = 0;
			}
		}

        //printf("z_pos1 = %ld\n", z_pos);

		//z_pos = z_pos / no_distances;	// Divide sum by number of distances to get the average
        z_pos = z_pos / no_valid_distances;	// Divide sum by number of distances to get the average

        ///printf("z_pos2 = %ld\n", z_pos);

		/* Add height of the anchor grid's height */
		//z_pos += anchor_pos[0].z;
	}

    best_solution->x = (float)x_pos/100.0;
    best_solution->y = (float)y_pos/100.0;
    best_solution->z = (float)z_pos/100.0;

    //printf("x=%f, y=%f, z=%f\n",x_pos, y_pos, z_pos);

	return UWB_OK;
}

#endif
