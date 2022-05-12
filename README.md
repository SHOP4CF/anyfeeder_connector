# AnyFeeder Connector
For ROS2

## Requirements
* ROS2 (eloquent)
* AnyFeeder SXM100/140 or SX240/340   
  You can find the manual here: 
  [link](https://assets.omron.eu/downloads/manual/en/v5/18876-0_anyfeeder_users_manual_en.pdf),
  [link](https://assets.omron.com/m/5e8428ff03caab0f/original/AnyFeeder-User-Guide.pdf), 
  [link](https://www.flexfactory.com/fileadmin/user_upload/Downloads/Schnittstellenbeschreibung_SXM/SerialProtocol_anyfeedSXM-DD_1.3.pdf)


## Services
A list of the available services:
* `/init`: move the AnyFeeder into its home position and clear all possible errors.  
  Required after power-up and before any motion command can be executed.
* `/feed_forward (s, r)`; Feed parts forward
* `/feed_backward (s, r)`: Feed parts backward 
* `/flip_forward (s, r)`: Flip parts forward 
* `/flipbackward (s, r)`: Flip parts backward
* `/flip (s, r)`: Flip parts without moving forward or backward
* `/dispense (s, r)`: Move parts from the bulk container onto the feed surface
* `/purge (s, r)`: Feed parts out backward, purge gate must be opened manually
* `/stop`: NOT IMPLEMENTED
* `/reset`: Reset error status and move AnyFeeder to home position
* `/restart (reset)`: if reset: Restart AnyFeeder firmware; resets all parameters to default values,  
  else: Start AnyFeeder firmware (also stops active motions)
* `/list_values`: output to the log the current (speed/repetition) values.
* `/status`: send the current status and output it to the log.

Note: `(s, r) = (speed, repetitions)`, where `speed ∈ {1, 2, ..., 10}` and `repetitions ∈ {1, 2, ..., 10}`.  
The command `/purge` is special, since: `repetitions ∈ {1, 2, ..., 127}`.


## USB troubleshoot
...