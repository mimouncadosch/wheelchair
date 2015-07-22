#Wheel Encoding Using Arduino, Photointerrupters and IR Sensor circuits

-> 20 degree (34 revolution) error when changing direction.
-> Large, but only changing direction in curves, so not that bad. 
-> If this causes problems, increase resolution on gray code wheels to something higher.

-> Drive very slowly, very very slowly.



Some math:
514 counts / rev

If 1 rev / sec:
	1/514 = 1.94 ~= 2ms per count

Mode 3: 1 rev / sec

Can Arduino count all the 514:
- single thread:
	* loop, no delay: counts all when rotating very fast.
	* loop, delay(0.1): 

