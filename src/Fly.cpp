
#include "state.h"

bool wasFlying = false;
void sendMotorSignal(ControllerState controller_out)
{
  if ( (int)(refControl.button8) %2 == 1 ) {
    copyPacket(XN297L_payloadIn[XN297L_goodPayloadIn], XN297L_payloadOut[!XN297L_goodPayloadOut]);
  } else {
    float newRx[] = {
		     controller_out.rightStickXPosition,
		     controller_out.rightStickYPosition,
		     controller_out.leftStickXPosition,
		     controller_out.throttle
		    };

    if ( ! okayToFly() ) {
      if ( wasFlying ) {
	newRx[3] = 0.3;
	newRx[0] = 0.0;
	newRx[1] = 0.0;
	newRx[2] = 0.0;
	if (( refSensor.rangeDown < 0.1 ) || (refSensor.rangeUp < 0.1)){
	  wasFlying = false;
	}
      } else {
	newRx[3] = 0.0;
      }
    } else if ( refSensor.rangeDown > 0.3 && controller_out.throttle > 0.3 )  {
      wasFlying = true;
    }
    replaceRx(newRx, XN297L_payloadIn[XN297L_goodPayloadIn], XN297L_payloadOut[!XN297L_goodPayloadOut]);
  }
  updateChecksum(XN297L_payloadOut[!XN297L_goodPayloadOut]);
  XN297L_goodPayloadOut = !XN297L_goodPayloadOut;
}
