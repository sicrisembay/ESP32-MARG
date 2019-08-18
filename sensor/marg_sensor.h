#ifndef _MARG_SENSOR_H_
#define _MARG_SENSOR_H_

int marg_sensor_init(void);
int marg_sensor_reg_notification(void * TskHdl);
int marg_sensor_unreg_notification(void * TskHdl);
#ifdef CONFIG_MARG_STRICT_FIX16
#error "To be implemented!"
#else
int marg_sensor_get_data(dataXYZf_t *pAccelData, dataXYZf_t *pGyroData, dataXYZf_t *pMagData);
#endif /* #ifdef CONFIG_MARG_STRICT_FIX16 */

#endif /* _MARG_SENSOR_H_ */
