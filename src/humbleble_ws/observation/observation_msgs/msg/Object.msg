std_msgs/Header header  #   最後に観測した時刻を記録
geometry_msgs/Pose pose #   msg_typeがPOINTの場合はpositionだけ
moveit_msgs/CollisionObject collision_object
moveit_msgs/AttachedCollisionObject attached_collision_object
string id   #   識別用の名前
string obj_group # botlle, ar_marker, locationなど
int32 status   
int32 msg_type 
float64 weight  #   重量

#   status
int32 UNKNOWN=0 #   観測したことがなく場所も不明
int32 KNOWN=1   #   観測したことがあり場所も既知
int32 LOST=2    #   観測したことがあるが場所は不明
int32 POSE_KNOWN=3    #   場所は既知だが観測したことがない（人が指示するとき用）
int32 ATTACHED=4     #   ロボットが所有

#   msg_type
int32 POINT=0 #   位置
int32 POSE=1 #   位置姿勢
int32 COLLISION=2   #   moveit_msgs
int32 ATTACHED_COLLISION=3    #   moveit_msgs