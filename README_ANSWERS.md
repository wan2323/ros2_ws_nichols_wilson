1. Describe a robotics task where an Action is superior to a Service. Explain why.

An action can be superior to a service in situations where we want a robot to do something over a period of time. In a case where the request is not completed instantanously we may want to recieve data from the robot as it is performing the task. This is where an action would be superior as it could give us continuous feedback during said task. Services on the other hand are blocking, and will only provide us with information when the server updates.   

2. What are the three main components of an Action (goal, result, feedback)? Describe the
purpose of each.

The three main componenets of an action as, as stated, a goal, a result, and feedback. A goal is our initially sent request, say for a robotic manipulator to move to a certain pose. Our result of course, is a failure or sucess result that is sent back to the user when the goal is accomplished, or it has determined failure. The feedback occurs between goal and result, and updates the user on progress or some other metric. 

3. Why is the ability to preempt (cancel) a goal crucial in robotics?

It's important to be able to preemptively cancel goals in robotics because most things don't always go perfectly. Often times robots will have to perform a series of tasks, and sometimes certain tasks cannot be completed if a prior task is failed. For example if my robot is navigating through a hallway, and is told to go to the end of the hallway and then turn, but it accidentally only goes halfway and then tries to turn and go into a wall. It's important to be able to cancel goals when things do not go as expected. 