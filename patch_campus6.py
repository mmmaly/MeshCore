p="zephyr-port/07_companion/src/serial_ble_interface.cpp"; s=open(p).read()
old='''	/* return one received frame */
	size_t out = 0;
	k_mutex_lock(&_lock, K_FOREVER);
	if (_recv_len > 0) {
		out = _recv[0].len;
		memcpy(dest, _recv[0].buf, out);
		_recv_len--;
		for (uint8_t i = 0; i < _recv_len; i++) _recv[i] = _recv[i + 1];
	}
	k_mutex_unlock(&_lock);
'''
new='''	/* return one received frame; private codes (>= 0xF0, e.g. ranging) are kept
	 * aside for main() rather than handed to MyMesh as an unsupported command */
	size_t out = 0;
	k_mutex_lock(&_lock, K_FOREVER);
	while (_recv_len > 0) {
		Frame f = _recv[0];
		_recv_len--;
		for (uint8_t i = 0; i < _recv_len; i++) _recv[i] = _recv[i + 1];
		if (f.len > 0 && f.buf[0] >= 0xF0) {
			if (!_priv_pending) { _priv = f; _priv_pending = true; }
			continue;
		}
		out = f.len;
		memcpy(dest, f.buf, out);
		break;
	}
	k_mutex_unlock(&_lock);
'''
assert old in s; s=s.replace(old,new,1)
s += '''
size_t SerialBLEInterface::takePrivateFrame(uint8_t dest[], size_t max)
{
	size_t n = 0;
	k_mutex_lock(&_lock, K_FOREVER);
	if (_priv_pending) {
		n = _priv.len < max ? _priv.len : max;
		memcpy(dest, _priv.buf, n);
		_priv_pending = false;
	}
	k_mutex_unlock(&_lock);
	return n;
}
'''
open(p,"w").write(s)
h="zephyr-port/07_companion/src/serial_ble_interface.h"; t=open(h).read()
t=t.replace('''	size_t checkRecvFrame(uint8_t dest[]) override;
''','''	size_t checkRecvFrame(uint8_t dest[]) override;
	size_t takePrivateFrame(uint8_t dest[], size_t max);   /* frames with code >= 0xF0 (RangingControl.h) */
''')
t=t.replace('''	Frame _recv[QSZ];
	uint8_t _recv_len = 0;          /* filled in BT ctx, read in main -> guarded by _lock */''','''	Frame _recv[QSZ];
	uint8_t _recv_len = 0;          /* filled in BT ctx, read in main -> guarded by _lock */
	Frame _priv;                    /* one pending private (>= 0xF0) frame for main() */
	bool _priv_pending = false;''')
open(h,"w").write(t); print("patched6")
