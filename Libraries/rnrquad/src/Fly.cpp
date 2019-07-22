
#include "state.h"

void sendMotorSignal(ControllerState controller_out)
{
  float newRx[] = {
                   controller_out.rightStickXPosition,
                   controller_out.rightStickYPosition,
                   controller_out.leftStickXPosition,
                   controller_out.throttle
                  };

  replaceRx(newRx, XN297L_payloadIn[XN297L_goodPayloadIn], XN297L_payloadOut[!XN297L_goodPayloadOut]);
  updateChecksum(XN297L_payloadOut[!XN297L_goodPayloadOut]);

  XN297L_goodPayloadOut = !XN297L_goodPayloadOut;
}