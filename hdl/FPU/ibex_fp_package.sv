package ibex_fp_pkg;

typedef enum logic[5:0]{
    
    FPU_ADD,
    FPU_SUB,
    FPU_MUL,
    FPU_DIV,
    FPU_SQRT,
    FPU_MIN,
    FPU_MAX,
    
    FPU_MADD,
    FPU_MSUB,
    FPU_NMADD,
    FPU_NMSUB,
    
    FPU_INT2FLOAT,
    FPU_FLOAT2INT,
    FPU_INT2FLOAT_U,
    FPU_FLOAT2INT_U,
    
    FPU_SGNJ,
    FPU_SGNJ_N, //negated sign-injection
    FPU_SGNJ_X, //xor sign-injection

    FPU_MOVE_INT2FLOAT, 
    FPU_MOVE_FLOAT2INT,
    
    FPU_CMP_EQ,
    FPU_CMP_LT,
    FPU_CMP_LE,

    FCLASS,
    
    FPU_NOP
}fpu_op_e;

endpackage