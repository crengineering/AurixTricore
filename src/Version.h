#ifndef VERSION_H_
#define VERSION_H_

/* Software version, transported via XCP (Xcp_Data struct + GET_ID string).
 * No integer suffixes here — the values are also stringified for GET_ID. */
#define SW_VERSION_MAJOR    1
#define SW_VERSION_MINOR    4
#define SW_VERSION_STEP     1

#define SW_VERSION_STR_(x)  #x
#define SW_VERSION_STR(x)   SW_VERSION_STR_(x)
#define SW_VERSION_STRING   SW_VERSION_STR(SW_VERSION_MAJOR) "." \
                            SW_VERSION_STR(SW_VERSION_MINOR) "." \
                            SW_VERSION_STR(SW_VERSION_STEP)

#endif /* VERSION_H_ */
