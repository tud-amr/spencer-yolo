# Message containing the velocity for each tracked person in the local frame (aligned to the person's heading)
# the position is still in global frame!!

uint64      track_id        # unique identifier of the target, consistent over time

geometry_msgs/PoseWithCovarianceStamped     globalpose      # pose of the track
geometry_msgs/TwistStamped                  localtwist      # velocity of the track in the local person frame
