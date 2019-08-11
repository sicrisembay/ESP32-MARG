#ifndef _MARG_SENSOR_H_
#define _MARG_SENSOR_H_

typedef struct {
#ifdef CONFIG_MARG_STRICT_FIX16
#else
    float x;
    float y;
    float z;
#endif /* #ifdef CONFIG_MARG_STRICT_FIX16 */
} dataXYZ_t;

int marg_sensor_init(void);

#endif /* _MARG_SENSOR_H_ */
