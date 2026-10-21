std_msgs/Header header
bool success
string query_text
string semantic_name
string detector_label
string object_id
geometry_msgs/Point position
string frame_id
float32 confidence
string status_message
# Search-link fields. Only meaningful when success == false:
# the query recognized a navigation command whose target has no landmark
# in memory yet, and suggests the coordinator run a vocabulary-injection
# search. search_label is the exact string to inject into the detect
# vocabulary — it equals the detector_label the landmark will carry, so
# re-running the same query after promotion resolves directly.
bool search_suggested
string search_label
