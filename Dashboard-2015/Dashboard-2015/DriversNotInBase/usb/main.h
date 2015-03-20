#ifndef _MAIN_H_
#define _MAIN_H_
#include <stdbool.h>

/*! \brief Called by MSC interface
 * Callback running when USB Host enable MSC interface
 *
 * \retval true if MSC startup is ok
 */
bool main_msc_enable(void);

/*! \brief Called by MSC interface
 * Callback running when USB Host disable MSC interface
 */
void main_msc_disable(void);

/*! \brief Called when a data transfer on MSC must be executed
 * Thus, the udi_msc_process_trans() function must be called
 */
void main_msc_notify_trans(void);

/*! \brief Called when a start of frame is received on USB line
 */
void main_sof_action(void);

/*! \brief Called by UDD when a suspend is received
 * Callback running when USB Host set USB line in suspend state
 */
void main_suspend_action(void);

/*! \brief Called by UDD when the USB line exit of suspend state
 */
void main_resume_action(void);

/*! \brief Initialize the memories used by examples
 */
void memories_initialization(void);

#endif // _MAIN_H_
