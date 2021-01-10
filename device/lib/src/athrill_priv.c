#include "athrill_libs.h"
#include "athrill_priv.h"

std_rettype athrill_get_devcfg_value(const char* key, std_uint32 *value)
{
    return athrill_ex_devop->param.get_devcfg_value(key, value);
}
std_rettype athrill_get_devcfg_value_hex(const char* key, std_uint32 *value)
{
    return athrill_ex_devop->param.get_devcfg_value_hex(key, value);
}
std_rettype athrill_get_devcfg_string(const char* key, char **value)
{
    return athrill_ex_devop->param.get_devcfg_string(key, value);
}

std_rettype athrill_add_intr(const char* name, std_uint32 intno, std_uint32 priority)
{
    return athrill_ex_devop->intr.add_intr(name, intno, priority);
}

void athrill_raise_intr(std_uint32 intno)
{
    return athrill_ex_devop->intr.raise_intr(intno);
}
