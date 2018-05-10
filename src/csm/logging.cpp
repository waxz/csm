#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <assert.h>

#include "logging.h"
#include "csm_all.h"
#include "utils.h"
#include <string>
#include <csm/csm.h>

int plog_write_flag = 0;

int sm_debug_write_flag = 0;

namespace plog
{
	Record& operator<<(Record& record, const LDP& ldp) // Implement a stream operator for our type.
	{

		int n = ldp->nrays;
		// for each field , create a string
		// combine at last

		//data to save
		/*
		 * point_w.p0
		 * point_w.p1
		 * point_w.rho
		 * point_w.phi
		 * valid
		 * corr.j1
		 * corr.j2
		 * corr.distj1
		 * */
		std::string readings, theta,j1,j2,valid,readings_w, theta_w;
		char readins_c[10000], thetas_c[10000];
		char fmt[] = "%.3f,";
        char fmt2[] = "%d,";
		std::string temp_s;
		char temp[6];

		for (int i = 1;i < n; i++){
			// reading
			sprintf(temp, fmt, ldp->readings[i]);
			readings += temp;
			// theta
			sprintf(temp, fmt, ldp->theta[i]);
			theta += temp;
			// corr j1
			sprintf(temp, fmt2, ldp->corr[i].j1);
			j1 += temp;
			// corr j2
			sprintf(temp, fmt2, ldp->corr[i].j2);
			j2 += temp;
			// valid
			sprintf(temp, fmt2, ldp->corr[i].valid);
			valid += temp;
            // reading_w
            sprintf(temp, fmt, ldp->points_w[i].rho);
            readings_w += temp;

            // theta_w
            sprintf(temp, fmt, ldp->points_w[i].phi);
            theta_w += temp;
		}

		std::string out = "readings:(" + readings + ");" +
						  "theta:("    + theta    + ");" +
						  "j1:("       + j1       + ");" +
						  "j2:("       + j2       + ");" +
				          "valid:("    + valid    + ");" +
                          "readings_w:(" + readings_w + ");" +
                          "theta_w:("    + theta_w    + ");" +
				"END";

		record<<out;

		return  record;
	}
}

const char * sm_program_name = 0;

void sm_debug_write(int flag) {
	sm_debug_write_flag = flag;
}

char sm_program_name_temp[256];
void sm_set_program_name(const char*name) {
	my_basename_no_suffix(name, sm_program_name_temp);
	sm_program_name = sm_program_name_temp;
}

int checked_for_xterm_color = 0;
int xterm_color_available = 0;


void check_for_xterm_color() {
	if(checked_for_xterm_color) return;
	checked_for_xterm_color = 1;
	
	const char * term = getenv("TERM");
	if(!term) term = "unavailable";
	xterm_color_available = !strcmp(term, "xterm-color") || !strcmp(term, "xterm") 
		|| !strcmp(term, "rxvt");
/*	sm_info("Terminal type: '%s', colors: %d\n", term, xterm_color_available); */
}

#define XTERM_COLOR_RED "\e[1;37;41m"
#define XTERM_COLOR_RESET "\e[0m"

#define XTERM_ERROR XTERM_COLOR_RED
#define XTERM_DEBUG "\e[1;35;40m"

void sm_write_context();

void sm_error(const char *msg, ...)
{
	check_for_xterm_color();
	if(xterm_color_available)
		fprintf(stderr, XTERM_ERROR);
		
	if(sm_program_name) 
		fprintf(stderr, "%s: ", sm_program_name);
	
	sm_write_context();
	
	if(!xterm_color_available) 
		fputs(":err: ", stderr);

	va_list ap;
	va_start(ap, msg);
	vfprintf(stderr, msg, ap);
	
	if(xterm_color_available)
		fprintf(stderr, XTERM_COLOR_RESET);
}

void sm_info(const char *msg, ...)
{
	check_for_xterm_color();
	
	if(sm_program_name) 
		fprintf(stderr, "%s: ", sm_program_name);
	
	sm_write_context();
	
	if(!xterm_color_available) 
		fputs(":inf: ", stderr);
	
	va_list ap;
	va_start(ap, msg);
	vfprintf(stderr, msg, ap);
}

void sm_debug(const char *msg, ...)
{
	if(!sm_debug_write_flag) return;
	
	check_for_xterm_color();
	
	if(xterm_color_available)
		fprintf(stderr, XTERM_DEBUG);
	
	if(sm_program_name) 
		fprintf(stderr, "%s: ", sm_program_name);

	sm_write_context();

	if(!xterm_color_available) 
		fputs(":dbg: ", stderr);
	
	va_list ap;
	va_start(ap, msg);
	vfprintf(stderr, msg, ap);
	
	
	if(xterm_color_available)
		fprintf(stderr, XTERM_COLOR_RESET);
}

#define MAX_CONTEXTS 1000
const char * sm_log_context_name[MAX_CONTEXTS] = {""};
int sm_log_context = 0;

void sm_log_push(const char*cname) {
	if(sm_debug_write_flag) {
		char message[1024]; sprintf(message, "  ___ %s \n", cname);
		sm_debug(message);
	}

	assert(sm_log_context<MAX_CONTEXTS-1);
	sm_log_context++;
	sm_log_context_name[sm_log_context] = cname;
}


void sm_log_pop() {
	assert(sm_log_context>0);
	sm_log_context--;
}

void sm_write_context() {
	for(int i=0;i<sm_log_context;i++)
		fprintf(stderr, "   ");
	
}

