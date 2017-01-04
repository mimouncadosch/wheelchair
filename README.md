### Wheel Encoding Using Arduino, Photointerrupters and IR Sensor circuits

-> 20 degree (34 revolution) error when changing direction.
-> Large, but only changing direction in curves, so not that bad. 
-> If this causes problems, increase resolution on gray code wheels to something higher.

-> Drive very slowly, very very slowly.


### Some math:

514 counts / rev

If 1 rev / sec:
	1/514 = 1.94 ms ~= 2 ms per count
	==> i.e. new rev count every 2ms, processing needs to take < 2ms

If 2 rev / sec:
	2/514 ~= 4 ms per count
Mode 3: 1 rev / sec

### Can Arduino count all the 514:
- single thread:
	* loop, no delay: counts all when rotating very fast.
	* loop, delay(0.1): counts all when rotating very fast.

- multi thread:
	* loop, no direction encoder, speed encoder alone: counts all when rotating slowly. Also works when rotating very fast, but not as reliably. Better to drive slowly. 
	* loop, direction and speed encoders, one wheel: counts all when rotating slowly.


# TIME FUNCTIONS

* Photointerrupter response time: max 15 micros 
(https://www.sparkfun.com/datasheets/Components/GP1A57HRJ00F.pdf)
* Object orientation does not take longer (method = same time as function)

- Time for speed sensor 1 wheel: 144 micros = 0.144 ms
- Time for speed sensor 2 wheels: 272 micros = 0.280 ms
- Time for speed & direction sensor 2 wheels: 552 micros = 0.552 ms





