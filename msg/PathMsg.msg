
Header header

Vertex[] path
uint32 sender_id

# id of the target node. If -1 (or negative), means sending to all nodes/agents
int32 target_id 

bool set_new_plan

# "physical" clock
uint64 clock

# it's own part of vector clock.
uint64 agent_vector_clk

## This for simulation of delay, not part of the protocol:
## when the network model delays a message, it would set this var
## to true so that it won't delayed twice - network model will skip 
## a message that already was delayed and not drop or delay it further.
bool delayed
