#ifndef aabb_minmax_box_h
#define aabb_minmax_box_h

#include "utilities_package.h"

class aabb_minmax_box {

public:

	coordinate minimum;
	coordinate maximum;

	aabb_minmax_box() {}
	aabb_minmax_box(const coordinate& a, const coordinate& b) { minimum = a; maximum = b; }

	coordinate min() const { return minimum; }
	coordinate max() const { return maximum; }

    bool hit(const ray& r, double t_min, double t_max) const {

        for (int a = 0; a < 3; a++) {

            auto t0 = fmin((minimum[a] - r.origin()[a]) / r.direction()[a],
                (maximum[a] - r.origin()[a]) / r.direction()[a]);

            auto t1 = fmax((minimum[a] - r.origin()[a]) / r.direction()[a],
                (maximum[a] - r.origin()[a]) / r.direction()[a]);

            t_min = fmax(t0, t_min);
            t_max = fmin(t1, t_max);

            if (t_max <= t_min)
                return false;
        }

        return true;

    }

    double area() const {

        auto a = maximum.x() - minimum.x();
        auto b = maximum.y() - minimum.y();
        auto c = maximum.z() - minimum.z();

        return 2 * (a * b + b * c + c * a);

    }

    int longest_axis() const {

        auto a = maximum.x() - minimum.x();
        auto b = maximum.y() - minimum.y();
        auto c = maximum.z() - minimum.z();

        if (a > b && a > c)
            return 0;
        else if (b > c)
            return 1;
        else
            return 2;
    }

};


aabb_minmax_box surrounding_box(aabb_minmax_box box0, aabb_minmax_box box1) {

    vec3 small(fmin(box0.min().x(), box1.min().x()),
        fmin(box0.min().y(), box1.min().y()),
        fmin(box0.min().z(), box1.min().z()));

    vec3 big(fmax(box0.max().x(), box1.max().x()),
        fmax(box0.max().y(), box1.max().y()),
        fmax(box0.max().z(), box1.max().z()));

    return aabb_minmax_box(small, big);
}


#endif

