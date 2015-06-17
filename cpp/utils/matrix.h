#ifndef MATRIX_H_INCLUDED
#define MATRIX_H_INCLUDED

#include <limits>

#define INIT_NAN(matrix, W, H) do { \
    matrix.resize((H),(W)); \
    for(unsigned int r = 0; r < (H); r++) { \
        for(unsigned int c = 0; c < (W); c++) { \
            (matrix)(r,c) = std::numeric_limits<float>::quiet_NaN(); \
        } \
    } \
} while (0)

#endif /* end of include guard: MATRIX_H_INCLUDED */
