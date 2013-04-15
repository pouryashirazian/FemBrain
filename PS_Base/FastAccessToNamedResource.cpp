#include "FastAccessToNamedResource.h"
#include "PS_Logger.h"

namespace PS {

void Logging::LogArg1(const char *message, const char *arg1) {
    LogErrorArg1(message, arg1);
}

}
