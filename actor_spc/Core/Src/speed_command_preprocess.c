#include "speed_command_preprocess.h"

#include <stddef.h>

void SpeedCommandPreprocess_Apply(SpeedCommand *command)
{
  if (command == NULL)
  {
    return;
  }

  command->v_boom *= -1000.0f;
  command->v_stick *= -1000.0f;
  command->v_bucket *= -1000.0f;
  if (command->yaw_rate > 0.34f)
  {
    command->yaw_rate = 0.34f;
  }
  else if (command->yaw_rate < -0.34f)
  {
    command->yaw_rate = -0.34f;
  }

  command->yaw_rate *= 57.2957795f;
}
