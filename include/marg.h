#ifndef _MARG_H_
#define _MARG_H_

typedef enum {
    MARG_NOERROR = 0,
    
    N_MARG_RET
} marg_ret_t;

marg_ret_t marg_init(void);

#endif /* _MARG_H_ */