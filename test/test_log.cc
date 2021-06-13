#include  "cyber/common/log.h"
#include <glog/logging.h>

int  main(){

    // ADEBUG  << "debug log " ;
    AINFO  << "Info log" ;
    AWARN << "Warn log";
    AERROR << "Error log";
    AFATAL << "Fatal log" ;
    
    return 0;
}