# Action assigned by the VLM to a single tracked pedestrian.
#
# action constants:
uint8 ACTION_NOT_CONSIDER = 0
uint8 ACTION_AVOID        = 1
uint8 ACTION_OVERTAKE     = 2
uint8 ACTION_FOLLOW       = 3
uint8 ACTION_YIELD_TO     = 4

int32   track_id                    # matches hdl_people_tracking Track.id
uint8   action                      # one of the ACTION_* constants above
float32 confidence                  # VLM confidence in [0, 1]

geometry_msgs/Point   position      # current estimated position (map frame)
geometry_msgs/Vector3 velocity      # current estimated velocity

string reason                       # optional short VLM explanation
