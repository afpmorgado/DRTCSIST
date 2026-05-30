
Additional features implemented:


-> Can add and remove nodes dynamically (any nodes from the network). Nodes constantly listen to heartbeats to determine if other nodes in the network are alive or not.

-> A node removal is always successful as long as time is given to the other nodes to update their list, before connecting another node. If the required time is not given then, for instance, in case node 1 is removed the next node in the list won't update it-self to node 1 before another node attempts to join the network and therefore this new node will be lost. In case any other node is removed it will still work.

-> When booting up for the first time we assign id’s based on a random time seeded with the luxes computed by each luminaire. This ensures that, as long as we do not add too many luminaires (it works for 2, 3 and 4 luminaires) at the same time during the first moment, they will obtain different ids. Right after booting up, before proceeding to any further steps, the user must wait for at least one heartbeat to be sent and received by every node, assuring that each node knows the existance of every other.

-> If there is already a node in the network and we are adding extra nodes, we should be able to add as many as we want at the same time (this was obviously not tested for more than 3 nodes joining to the first).

-> If any node is removed we do a “shift left” off all the nodes at the right of the disconnected node: If we remove node 2 then 3 will become 2 and 4 will become 3 (this is valid for the removal of any other node including node 1).

-> Due to the features above calibration does not start automaticaly, but after the command by the user "calib".
The user must make sure that all nodes are aware of each other before doing calib (wait 1 heartbeat), as mentioned above.

-> Heartbeat makes sure all are aware of each other by regularly sending "heartbeats" messages (LV id).


Note: The group recognizes that the code quality could be improved by having a better pattern / design and being more modular, making debug easier and a having a better readibility, but due to time constraints some shortcuts were taken, such as using a lot of global variables and repeting certain conditions more times than needed.
 

DONE BY:

	Diogo Miguel de Melo Nunes (96192)

	André Filipe Pereira Morgado (99888)