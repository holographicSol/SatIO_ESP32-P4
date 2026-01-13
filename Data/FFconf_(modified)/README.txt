Required to replace existing ffconf.h in order to enable exFAT support which
is not enabled by default due to licensing.

configuration default except:

#define FF_LBA64        1

#define FF_FS_EXFAT     1

#define FF_USE_LFN      2

#define FF_FS_NOFSINFO  1
