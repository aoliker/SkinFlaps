// #pragma once
#include <Common/KernelCommon.h>

#ifdef FORCE_INLINE
#include <Kernels/Matrix_Times_Matrix/Matrix_Times_Matrix.h>
#include <Kernels/Matrix_Times_Transpose/Matrix_Times_Transpose.h>
#include <Kernels/Singular_Value_Decomposition/Singular_Value_Decomposition.h>
#else

#define SUBROUTINE_Matrix_Times_Transpose
#include <Kernels/Matrix_Times_Transpose/Matrix_Times_Transpose.cpp>
#undef SUBROUTINE_Matrix_Times_Transpose

#define SUBROUTINE_Singular_Value_Decomposition
#include <Kernels/Singular_Value_Decomposition/Singular_Value_Decomposition.cpp>
#undef SUBROUTINE_Singular_Value_Decomposition

#define SUBROUTINE_Matrix_Times_Matrix
#include <Kernels/Matrix_Times_Matrix/Matrix_Times_Matrix.cpp>
#undef SUBROUTINE_Matrix_Times_Matrix
#endif

template<class Tarch,class T_DATA>
void Add_Force(const T_DATA (&x_Blocked)[4][3],
               const T_DATA (&DmInverse_Blocked)[9],
               const T_DATA &restVolume,
               const T_DATA &muLow,
               const T_DATA &muHigh,
               const T_DATA &strainMin,
               const T_DATA &strainMax,
               const T_DATA &activation,
               const T_DATA &fiberX,
               const T_DATA &fiberY,
               const T_DATA &fiberZ,
               T_DATA (&f_Blocked)[4][3])
{
    using namespace SIMD_Numeric_Kernel;
    constexpr int d = 3;

    using WideNumberType = Number<Tarch>;
    using WideVectorType = Vector3<WideNumberType>;

    using T = typename Tarch::Scalar;
    T TWO[Tarch::Width]{};
    for (int i=0; i<Tarch::Width; i++) TWO[i] = 2;
    T NEGONE[Tarch::Width]{};
    for (int i=0; i<Tarch::Width; i++) NEGONE[i] = -1;

    alignas(sizeof(T_DATA)) T_DATA  F_Blocked[d * d]{};
    alignas(sizeof(T_DATA)) T_DATA  R_Blocked[d * d]{};
   // alignas(sizeof(T_DATA)) T_DATA  P_Blocked[d*d];
    alignas(sizeof(T_DATA)) T_DATA  U_Blocked[d * d]{};
    alignas(sizeof(T_DATA)) T_DATA  V_Blocked[d * d]{};
    alignas(sizeof(T_DATA)) T_DATA  S_Blocked[d]{};

    WideVectorType v0, v1, v2, v3, v4, v5, v6;
    WideNumberType s0, s1;

    v0.Load_Aligned(x_Blocked[0]);
    v1.Load_Aligned(x_Blocked[1]);
    v2.Load_Aligned(x_Blocked[2]);
    v3.Load_Aligned(x_Blocked[3]);
    v1 = v1-v0;
    v2 = v2-v0;
    v3 = v3-v0;

    v1.Store(reinterpret_cast<T_DATA(&)[3]>(F_Blocked[0][0]));
    v2.Store(reinterpret_cast<T_DATA(&)[3]>(F_Blocked[3][0]));
    v3.Store(reinterpret_cast<T_DATA(&)[3]>(F_Blocked[6][0]));

    Matrix_Times_Matrix<Tarch, T_DATA>(F_Blocked,
                                       DmInverse_Blocked,
                                       F_Blocked);

    Singular_Value_Decomposition<Tarch, T_DATA>(F_Blocked, U_Blocked, S_Blocked, V_Blocked);

    //Sigma[v] = std::min( std::max( Sigma[v], strainMin[eee] ), strainMax[eee] );
    v0.Load_Aligned(S_Blocked);

    s0.Load_Aligned(strainMin);
    s1.Load_Aligned(strainMax);

    v0.x = min(max(v0.x, s0), s1);
    v0.y = min(max(v0.y, s0), s1);
    v0.z = min(max(v0.z, s0), s1);

    //Sigma[v] = muLow[eee] + muHigh[eee] * Sigma[v];
    s0.Load_Aligned(muLow);
    s1.Load_Aligned(muHigh);

    v0 *= s1;
    v0 += s0;

    v1.Load_Aligned(reinterpret_cast<T_DATA(&)[3]>(V_Blocked[0][0]));
    v2.Load_Aligned(reinterpret_cast<T_DATA(&)[3]>(V_Blocked[3][0]));
    v3.Load_Aligned(reinterpret_cast<T_DATA(&)[3]>(V_Blocked[6][0]));

    v1 *= v0.x;
    v2 *= v0.y;
    v3 *= v0.z;

    // R = U * Sigma.asDiagonal() * V.transpose();
    v1.Store(reinterpret_cast<T_DATA(&)[3]>(V_Blocked[0][0]));
    v2.Store(reinterpret_cast<T_DATA(&)[3]>(V_Blocked[3][0]));
    v3.Store(reinterpret_cast<T_DATA(&)[3]>(V_Blocked[6][0]));

    Matrix_Times_Transpose<Tarch, T_DATA>(U_Blocked,
                                          V_Blocked,
                                          R_Blocked);

    // --- active fiber contraction (COURT/agent): R <- R + (lambda-1)(R a) a^T ---
    // a = per-tet material-space unit fiber; lambda = activation (1 passive, 0.85 = 15% shortening
    // along the fiber). Shortens the projection target along the fiber only, so wall thickening and
    // twist emerge from the mechanics. RHS-only: the prefactored global matrix never sees a or lambda.
    {
        WideVectorType col0, col1, col2, Ra, Rl, tt;
        WideNumberType ax, ay, az, lam, lm1, negone;
        col0.Load_Aligned(reinterpret_cast<T_DATA(&)[3]>(R_Blocked[0][0]));
        col1.Load_Aligned(reinterpret_cast<T_DATA(&)[3]>(R_Blocked[3][0]));
        col2.Load_Aligned(reinterpret_cast<T_DATA(&)[3]>(R_Blocked[6][0]));
        ax.Load_Aligned(fiberX);
        ay.Load_Aligned(fiberY);
        az.Load_Aligned(fiberZ);
        // Ra = ax*col0 + ay*col1 + az*col2
        Ra = col0; Ra *= ax;
        tt = col1; tt *= ay; Ra = Ra + tt;
        tt = col2; tt *= az; Ra = Ra + tt;
        lam.Load_Aligned(activation);
        negone.Load_Aligned(NEGONE);
        lm1 = lam; lm1 = lm1 + negone;   // lambda - 1
        Rl = Ra; Rl *= lm1;              // (lambda-1) * Ra
        // col_j += a_j * Rl
        tt = Rl; tt *= ax; col0 = col0 + tt;
        tt = Rl; tt *= ay; col1 = col1 + tt;
        tt = Rl; tt *= az; col2 = col2 + tt;
        col0.Store(reinterpret_cast<T_DATA(&)[3]>(R_Blocked[0][0]));
        col1.Store(reinterpret_cast<T_DATA(&)[3]>(R_Blocked[3][0]));
        col2.Store(reinterpret_cast<T_DATA(&)[3]>(R_Blocked[6][0]));
    }

// MatrixType P =  -2. * ((muHigh[eee] + muLow[eee]) * F - R);
    v0.Load_Aligned(reinterpret_cast<T_DATA(&)[3]>(F_Blocked[0][0]));
    v1.Load_Aligned(reinterpret_cast<T_DATA(&)[3]>(F_Blocked[3][0]));
    v2.Load_Aligned(reinterpret_cast<T_DATA(&)[3]>(F_Blocked[6][0]));

    v3.Load_Aligned(reinterpret_cast<T_DATA(&)[3]>(R_Blocked[0][0]));
    v4.Load_Aligned(reinterpret_cast<T_DATA(&)[3]>(R_Blocked[3][0]));
    v5.Load_Aligned(reinterpret_cast<T_DATA(&)[3]>(R_Blocked[6][0]));

    s0.Load_Aligned(muLow);  // s0 was clobbered by the activation load above
    s0 = s0 + s1;

    v0 *= s0;
    v1 *= s0;
    v2 *= s0;

    // v0 = v0 - v3;
    // v1 = v1 - v4;
    // v2 = v2 - v5;

    v0 = v3 - v0;
    v1 = v4 - v1;
    v2 = v5 - v2;

    s0.Load_Aligned(restVolume);
    s1.Load_Aligned(TWO);

    v0 *= s0;
    v1 *= s0;
    v2 *= s0;

    v0 *= s1;
    v1 *= s1;
    v2 *= s1;

    //v0.Store(S_Blocked);
    v0.Store(reinterpret_cast<T_DATA(&)[3]>(F_Blocked[0][0]));
    v1.Store(reinterpret_cast<T_DATA(&)[3]>(F_Blocked[3][0]));
    v2.Store(reinterpret_cast<T_DATA(&)[3]>(F_Blocked[6][0]));

    //MatrixType H = -restVolume[eee] * P * DmInverse.transpose();
    Matrix_Times_Transpose<Tarch, T_DATA>(F_Blocked,
                                          DmInverse_Blocked,
                                          F_Blocked);

    v0.Load_Aligned(f_Blocked[0]);
    v1.Load_Aligned(f_Blocked[1]);
    v2.Load_Aligned(f_Blocked[2]);
    v3.Load_Aligned(f_Blocked[3]);

    v4.Load_Aligned(reinterpret_cast<T_DATA(&)[3]>(F_Blocked[0][0]));
    v5.Load_Aligned(reinterpret_cast<T_DATA(&)[3]>(F_Blocked[3][0]));
    v6.Load_Aligned(reinterpret_cast<T_DATA(&)[3]>(F_Blocked[6][0]));

    v0 = v0 - v4;
    v0 = v0 - v5;
    v0 = v0 - v6;

    v1 = v1 + v4;
    v2 = v2 + v5;
    v3 = v3 + v6;

    v0.Store(f_Blocked[0]);
    v1.Store(f_Blocked[1]);
    v2.Store(f_Blocked[2]);
    v3.Store(f_Blocked[3]);
}

#define INSTANCE_KERNEL_Add_Force(WIDTH,TYPE)               \
    const WIDETYPE(TYPE,WIDTH) (&x_Blocked)[4][3],          \
        const WIDETYPE(TYPE,WIDTH) (&DmInverse_Blocked)[9], \
        const WIDETYPE(TYPE,WIDTH) &restVolume,             \
        const WIDETYPE(TYPE,WIDTH) &muLow,                  \
        const WIDETYPE(TYPE,WIDTH) &muHigh,                 \
        const WIDETYPE(TYPE,WIDTH) &strainMin,              \
        const WIDETYPE(TYPE,WIDTH) &strainMax,              \
        const WIDETYPE(TYPE,WIDTH) &activation,             \
        const WIDETYPE(TYPE,WIDTH) &fiberX,                 \
        const WIDETYPE(TYPE,WIDTH) &fiberY,                 \
        const WIDETYPE(TYPE,WIDTH) &fiberZ,                 \
        WIDETYPE(TYPE,WIDTH) (&f_Blocked)[4][3]

INSTANCE_KERNEL_SIMD_AVX_FLOAT( Add_Force, 16)
INSTANCE_KERNEL_SIMD_MIC_FLOAT( Add_Force, 16)
#undef INSTANCE_KERNEL_Add_Force
