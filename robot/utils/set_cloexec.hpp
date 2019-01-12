#pragma once

#include <fcntl.h>


/* Set the FD_CLOEXEC flag of file_descriptor.
Return 0 on success, or -1 on error with errno set. */
inline int set_cloexec_flag(int file_descriptor)
{
    int flags = fcntl(file_descriptor, F_GETFD, 0);
    /* If reading the flags failed, return error indication now. */
    if (flags < 0) {
        return flags;
    }
    /* Set just the flag we want to set. */
    flags |= FD_CLOEXEC;

    /* Store modified flag word in the file_descriptor. */
    return fcntl(file_descriptor, F_SETFD, flags);
}
