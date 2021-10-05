#ifndef __MYLIFE_AC_ZC_H__
#define __MYLIFE_AC_ZC_H__

#define AC_ZC_STATUS_ENTER (1 << 0)
#define AC_ZC_STATUS_LEAVE (1 << 1)

typedef void (*ac_zc_callback)(int status, void *data);

// return : id > 0 on success (to unregister), error < 0 on failure
int ac_zc_register(int status, ac_zc_callback cb, void *cb_data);

// return : 0 on success, error < 0 on failure
int ac_zc_unregister(int id);

int ac_zc_freq(void);

#endif // __MYLIFE_AC_ZC_H__
