#include <Eigen/Core>
#include <Eigen/Dense>

namespace tyco
{
namespace P3
{

template<typename T, typename Cs>
struct row_vector
{
    Eigen::Matrix<T, 1, 4> raw_;
};

template<typename T, typename Cs>
struct column_vector
{
    Eigen::Matrix<T, 4, 1> raw_;
};

template<typename T, typename CsLeft, typename CsRight>
struct homography
{
    Eigen::Matrix<T, 4, 4> raw_;
};

// M^-1
template<typename T, typename CsLeftIn, typename CsRightIn>
homography<T, CsRightIn, CsLeftIn> inverse(const homography<T, CsLeftIn, CsRightIn>& M)
{
    return {M.raw_.inverse()};
}

// M * p
template<typename T, typename CsLeftIn, typename CsRightIn>
column_vector<T, CsLeftIn> operator*(
    const homography<T, CsLeftIn, CsRightIn>& M,
    const column_vector<T, CsRightIn>& p)
{
    return {M.raw_ * p.raw_};
}

// p^T * M
template<typename T, typename CsLeftIn, typename CsRightIn>
row_vector<T, CsRightIn> operator*(
    const row_vector<T, CsLeftIn>& p,
    const homography<T, CsLeftIn, CsRightIn>& M)
{
    return {p.raw_ * M.raw_};
}

} // namespace P3
} // namespace tyco
