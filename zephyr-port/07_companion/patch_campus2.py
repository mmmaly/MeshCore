p="src/serial_ble_interface.cpp"; s=open(p).read()
old="""static struct bt_conn_auth_cb auth_cb = {
	.passkey_display = auth_passkey_display,
	.cancel = auth_cancel,
};"""
new="""/* Zephyr >= 4.3: the fixed passkey comes from this callback (CONFIG_BT_APP_PASSKEY)
 * instead of the removed bt_passkey_set()/CONFIG_BT_FIXED_PASSKEY. */
static uint32_t s_fixed_passkey = 123456;
static uint32_t auth_app_passkey(struct bt_conn *conn) { return s_fixed_passkey; }
static struct bt_conn_auth_cb auth_cb = {
	.passkey_display = auth_passkey_display,
	.cancel = auth_cancel,
	.app_passkey = auth_app_passkey,
};"""
assert old in s; s=s.replace(old,new,1)
old2="\tbt_passkey_set(pin_code);\n"
assert old2 in s; s=s.replace(old2,"\ts_fixed_passkey = pin_code;\n",1)
open(p,"w").write(s)
c="prj.conf"; t=open(c).read()
t=t.replace("CONFIG_BT_FIXED_PASSKEY=y","CONFIG_BT_APP_PASSKEY=y")
open(c,"w").write(t); print("patched2")
