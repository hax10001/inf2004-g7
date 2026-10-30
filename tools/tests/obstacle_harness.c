#include <tk/tkernel.h>
static UINT g_pattern = 0U;
ID tk_cre_sem(const T_CSEM *config) { (void)config; return 1; }
ER tk_wai_sem(ID sem, INT count, TMO timeout) { (void)sem; (void)count; (void)timeout; return E_OK; }
ER tk_sig_sem(ID sem, INT count) { (void)sem; (void)count; return E_OK; }
ID tk_cre_flg(const T_CFLG *config) { g_pattern = config->iflgptn; return 1; }
ER tk_ref_flg(ID flag, T_RFLG *info) { (void)flag; info->flgptn = g_pattern; return E_OK; }
ER tk_set_flg(ID flag, UINT bits) { (void)flag; g_pattern |= bits; return E_OK; }
ER tk_clr_flg(ID flag, UINT mask) { (void)flag; g_pattern &= mask; return E_OK; }