#ifndef SM_LOGGING_H
#define SM_LOGGING_H
#define write_log true
#include <string>
#include <plog/Log.h>
#include <csm/csm_all.h>
extern const char * sm_program_name;

void sm_set_program_name(const char*);

void sm_debug(const char *msg, ...);
void sm_error(const char *msg, ...);
void sm_info(const char *msg, ...);

namespace plog {
    Record &operator<<(Record &record, const LDP &ldp);
}
template <class  T>
void plog_write(const std::string key, const T &data){
    LOGI<<"source:"<<key<<";"<<data;
}

/* Optional context handling for hyerarchical display */
void sm_log_push(const char*);
void sm_log_pop();

/* Enable/disable writing of debug information */
void sm_debug_write(int enabled);



/* Private interface */
void sm_write_context();
void check_for_xterm_color();
	
#endif
