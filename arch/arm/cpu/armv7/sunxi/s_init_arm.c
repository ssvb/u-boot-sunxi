/* FEL starts execution in ARM mode, so we need this small adapter function */
void s_init(void); void s_init_arm(void) { s_init(); }
