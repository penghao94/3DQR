#include "Strategy.h"

void qrcode::control_strategy(GLOBAL & global)
{
	int border = global.info.border;
	int scale = global.info.scale;
	int size = (2 * border + global.info.pixels.size())*scale + 2 * scale + 1;
	global.under_control.setOnes(size,size);
}
