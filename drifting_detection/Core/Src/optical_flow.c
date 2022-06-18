#include "optical_flow.h"
#include <string.h>
#include "Coarse2FineFlowWrapper.h"

int optical_flow_init(optical_flow_t *optflw, UART_HandleTypeDef *uart) {
  return 0;
}

void optical_flow_update(optical_flow_t *optflw, uint8_t *frame) {
  memcpy(optflw->prev_frame, optflw->frame, IMG_SIZE);
  memcpy(optflw->frame, frame, IMG_SIZE);

  Coarse2FineFlowWrapper(
      optflw->vx, optflw->vy, optflw->warpI2,
      optflw->prev_frame, optflw->frame,
      0.0012, 1.0, 30,
      2, 1, 1, 1,
      IMG_H, IMG_W, 1);
}
