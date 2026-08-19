#ifndef MANUAL_ACTION_H
#define MANUAL_ACTION_H

#define MANUAL_ACTION_DEAD_ZONE 0.15f

typedef struct
{
  float boom;
  float stick;
  float bucket;
  float swing;
} ManualAction;

void ManualAction_SetZero(ManualAction *action);
void ManualAction_FromStick(float x1, float x2, float y1, float y2,
                            ManualAction *action);

#endif /* MANUAL_ACTION_H */
