/* Code for the Computational Methods for Engineering Applications class
 * Copyright CSElab, ETH Zürich, 2015
 */

#include <cmath>
#include <vector>

int solve_thomas(
    // input: matrix & rhs
    std::vector<double> const & lower,
    std::vector<double> const & diag,
    std::vector<double> const & upper,
    std::vector<double> const & rhs,
    // output
    std::vector<double> & solution)
{
    int N = rhs.size();
    std::vector<double> c(N), d(N);
    double t;
    
    // forward sweep
    t = diag[0];
    if(fabs(t) < 1e-16)
        return -1;

    c[0] = upper[0]/t;
    d[0] = rhs[0]/t;

    for(int i=1; i<N-1; ++i)
    {
        t = diag[i]-lower[i-1]*c[i-1];
        if(fabs(t) < 1e-16)
            return -1;

        c[i] = upper[i]/t;
        d[i] = (rhs[i]-lower[i-1]*d[i-1])/t;
    }

    t = diag[N-1]-lower[N-2]*c[N-2];
    if(fabs(t) < 1e-16)
        return -1;

    d[N-1] = (rhs[N-1]-lower[N-2]*d[N-2])/t;

    // backward substitution
    solution[N-1] = d[N-1];
    for(int i=N-2; i>=0; --i)
        solution[i] = d[i]-c[i]*solution[i+1];

    return 0;
}

double get_l2_error(
    std::vector<double> const & lower,
    std::vector<double> const & diag,
    std::vector<double> const & upper,
    std::vector<double> const & rhs,
    std::vector<double> const & solution)
{
    int N = rhs.size();
    double t, res = 0.0;

    t = rhs[0] - (diag[0]*solution[0] + upper[0]*solution[1]);
    res += t*t;
    for(int i=1; i<N-1; ++i)
    {
        t = rhs[i] - (lower[i-1]*solution[i-1] + diag[i]*solution[i] + upper[i]*solution[i+1]);
        res += t*t;
    }
    t = rhs[N-1] - (lower[N-2]*solution[N-2] + diag[N-1]*solution[N-1]);
    res += t*t;

    return sqrt(res);
}
