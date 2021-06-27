
Header header

Vertex[] path
uint32 sender_id

# id of the target node. If -1 (or negative), means sending to all nodes/agents
int32 target_id 

bool set_new_plan

# "physical" clock
uint64 clock

uint64 agent_vector_clk
# vector clock (to be added later)
#VectorClk[] vector_clk