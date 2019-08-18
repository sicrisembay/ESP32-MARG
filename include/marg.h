#ifndef _MARG_H_
#define _MARG_H_

typedef enum {
    MARG_NOERROR = 0,
    MARG_ERROR_INIT,
    
    N_MARG_RET
} marg_ret_t;

typedef struct {
#ifdef CONFIG_MARG_STRICT_FIX16
#error "To be implemented!"
#else
    float x;
    float y;
    float z;
#endif /* #ifdef CONFIG_MARG_STRICT_FIX16 */
} dataXYZf_t;


marg_ret_t marg_init(void);

#endif /* _MARG_H_ */
