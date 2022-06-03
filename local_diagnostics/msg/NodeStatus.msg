# Possible levels of operations
byte OK=0
byte WARN=1
byte ERROR=2
byte STALE=3

byte level # level of operation enumerated above 
string name # a description of the test/component reporting
string message # a description of the status
string id # a hardware unique string
float64 last_updated
float32 age
diagnostic_msgs/KeyValue[] values # an array of values associated with the status
