The goal of this project is to make a Soft Bank Robotic’s NAO into a bull fighter and a robot car controlled by Raspberry Pi into a bull. 
In this setup, both contenders are inside a ring drawn on the floor and the bull is replaced by a Raspberry Pi controlled robot car that charges for the humanoid’s cape.
The NAO must track the movement of the bull, decipher its intention, juke when aproppiate and exclaim the outcome of every round.

You can find the demo video of the final project hier:
https://drive.google.com/file/d/1gqRFt0L-AS6ZP8H6Qs4cHQvhVNnuVhXQ/view?usp=sharing


This is the main routine of every round:

1) Both contenders (NAO and bull) should track each other while the bull is circling a ring drawn on the floor. 
The bull-car should move to its ready position and aim at the NAO’s red cape and both should communicate using LED signals.

2) NAO should inform an external operator when he detects that the bull-car is ready. On the go signal (“¡Adelante!”) from the external operator, the Matador should send a
killer look (LED pattern) to the bull to provoke him.

3) Then the bull-car should charge along a linear collision trajectory with the cape.

4) The brave NAO Matador should in return move his body aside and let the robot-bull go past its cape at the very last moment, proudly exclaiming “¡Ole´e´e´e!”. 

5) In the unfortunate case the Matador is not fast enough to avoid the bull, it should detect the collision with its bumpers, 
shouting “¡Ay Caramba!”. After each round, both the bull and the matador should re-position and prepare for the next run.
